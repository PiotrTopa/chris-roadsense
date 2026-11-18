// camerad: rear camera -> NV12 shared-memory frame ring + JSON metadata socket
// + JPEG preview. See docs/ARCHITECTURE.md for the contract.
//
// Threads:
//   main     : setup, signal wait (SIGINT/SIGTERM), shutdown
//   libcamera: requestCompleted -> pushes the request into a queue (no work)
//   worker   : XRGB8888 -> NV12 straight into the ring slot, commit, JSON
//              publish, hand every Nth frame (2x downscaled) to the preview
//              thread, requeue the request
//   convert helpers (convert_threads-1): row bands of the conversion
//   preview  : JPEG encode 960x540 + write-to-temp + rename
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <sched.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <libcamera/libcamera.h>

#include "jpeg_nv12.h"
#include "governor.h"
#include "jsonsock.h"
#include "nv12.h"
#include "ring.h"

using namespace roadsense;
using Clock = std::chrono::steady_clock;

namespace {

constexpr int kWidth = 1920, kHeight = 1080;
constexpr int kBufferCount = 4;

// ------------------------------------------------------------------ config ---

struct Config {
    // Pinhole intrinsics at 1920x1080 (published in the ring header).
    // IMX363 on the Poco F1: f = 4.3 mm (nominal, 1/2.55" sensor), active
    // array 4032 px wide over ~5.64 mm (1.4 um pixels). The 1920x1080 mode
    // reads the full sensor width, scaled by 4032/1920 = 2.1, so
    //   focal_px = 4.3 mm / 5.64 mm * 1920 px = 1464 px  (~1470).
    // Treat it as a starting value; replace with a calibrated one in
    // /etc/roadsense/camera.json.
    float focal_px = 1470.0f;
    float cx = kWidth / 2.0f, cy = kHeight / 2.0f;
    int preview_every = 3;      // JPEG every Nth frame (0 = off)
    int preview_quality = 80;
    bool little_cores = true;   // pin the whole process to cpu0-3 (A55)
    int convert_threads = 1;    // threads for XRGB->NV12
    int fps = 20;               // sensor frame rate via VBLANK; 0 = leave the sensor alone
    // Empirical IMX363 frame timing in the 1920x1080 mode (see set_sensor_fps()).
    double sensor_line_ns = 10468.0;
    double sensor_base_lines = 3045.0;
};

std::string read_file(const std::string &path) {
    std::ifstream f(path);
    if (!f) return {};
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// Minimal flat-JSON lookup: finds "key" : <number|true|false>. Enough for a
// config file of scalars; nested objects/strings are not supported.
bool json_number(const std::string &s, const char *key, double *out) {
    const std::string k = std::string("\"") + key + "\"";
    size_t p = s.find(k);
    if (p == std::string::npos) return false;
    p = s.find(':', p + k.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < s.size() && isspace(uint8_t(s[p]))) ++p;
    if (s.compare(p, 4, "true") == 0) return *out = 1, true;
    if (s.compare(p, 5, "false") == 0) return *out = 0, true;
    char *end = nullptr;
    double v = strtod(s.c_str() + p, &end);
    if (end == s.c_str() + p) return false;
    *out = v;
    return true;
}

std::string config_path() {
    const char *d = getenv("ROADSENSE_CONFIG_DIR");
    return std::string(d && *d ? d : "/etc/roadsense") + "/camera.json";
}

void load_config(Config &c, bool verbose) {
    const std::string path = config_path();
    const std::string s = read_file(path);
    if (s.empty()) {
        if (verbose) fprintf(stderr, "camerad: no config at %s, using defaults\n", path.c_str());
        return;
    }
    double v;
    if (json_number(s, "focal_px", &v)) c.focal_px = float(v);
    if (json_number(s, "cx", &v)) c.cx = float(v);
    if (json_number(s, "cy", &v)) c.cy = float(v);
    if (json_number(s, "preview_every", &v)) c.preview_every = int(v);
    if (json_number(s, "preview_quality", &v)) c.preview_quality = int(v);
    if (json_number(s, "little_cores", &v)) c.little_cores = v != 0;
    if (json_number(s, "convert_threads", &v)) c.convert_threads = std::max(1, std::min(4, int(v)));
    if (json_number(s, "fps", &v)) c.fps = int(v);
    if (json_number(s, "sensor_line_ns", &v)) c.sensor_line_ns = v;
    if (json_number(s, "sensor_base_lines", &v)) c.sensor_base_lines = v;
    if (verbose) fprintf(stderr, "camerad: config %s loaded\n", path.c_str());
}

std::string rundir() {
    const char *d = getenv("ROADSENSE_RUNDIR");
    return d && *d ? d : "/run/roadsense";
}

bool mkdir_p(const std::string &path) {
    std::string cur;
    std::stringstream ss(path);
    std::string part;
    if (!path.empty() && path[0] == '/') cur = "/";
    while (std::getline(ss, part, '/')) {
        if (part.empty()) continue;
        cur += part + "/";
        if (mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST) return false;
    }
    return true;
}

int64_t clock_ns(clockid_t id) {
    timespec ts;
    clock_gettime(id, &ts);
    return int64_t(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}

// CLOCK_REALTIME - CLOCK_MONOTONIC, sampled with the tightest bracket of 5.
int64_t realtime_offset_ns() {
    int64_t best_gap = INT64_MAX, off = 0;
    for (int i = 0; i < 5; ++i) {
        int64_t m0 = clock_ns(CLOCK_MONOTONIC), r = clock_ns(CLOCK_REALTIME), m1 = clock_ns(CLOCK_MONOTONIC);
        if (m1 - m0 < best_gap) {
            best_gap = m1 - m0;
            off = r - (m0 + (m1 - m0) / 2);
        }
    }
    return off;
}

// ------------------------------------------------------------ sensor fps ---

// Finds the V4L2 subdev of the IMX363 by entity name (sysfs name attribute,
// identical to the media-controller entity name in /dev/media0).
std::string find_sensor_subdev() {
    DIR *d = opendir("/sys/class/video4linux");
    if (!d) return {};
    std::string found;
    while (dirent *e = readdir(d)) {
        if (strncmp(e->d_name, "v4l-subdev", 10) != 0) continue;
        std::string name = read_file(std::string("/sys/class/video4linux/") + e->d_name + "/name");
        while (!name.empty() && isspace(uint8_t(name.back()))) name.pop_back();
        if (name == "imx363 16-0010") {
            found = std::string("/dev/") + e->d_name;
            break;
        }
    }
    closedir(d);
    return found;
}

bool get_ctrl(int fd, uint32_t id, int64_t *v) {
    v4l2_ext_control c{};
    c.id = id;
    v4l2_ext_controls cs{};
    cs.which = V4L2_CTRL_WHICH_CUR_VAL;
    cs.count = 1;
    cs.controls = &c;
    if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &cs) != 0) return false;
    *v = id == V4L2_CID_PIXEL_RATE ? c.value64 : c.value;
    return true;
}

struct SensorTiming {
    std::string dev;
    int64_t vblank = 0, vblank_min = 0, vblank_max = 0, hblank = 0, pixel_rate = 0;
};

// Frame period model.
// Nominal V4L2 model: T = (height + vblank) * (width + hblank) / pixel_rate.
// It does NOT hold for this driver mode: libcamera's configure switches the
// sensor to hblank 4800 / pixel_rate 508.8 MHz (13.21 us/line nominal) and
// sets vblank 112, which by the nominal model would be 1192 lines = 63 fps,
// yet the measured rate is 30.26 fps. Sweeping vblank 112..4000 (bench,
// 2026-10-21, sensor timestamps) gives a straight line
//     T = (3045 + vblank) * 10.468 us        (max error < 0.3 %)
// i.e. the mode reads ~3045 lines (full sensor height, scaled to 1080) at a
// real line time of 10.468 us. Both constants are config values
// (sensor_base_lines, sensor_line_ns).
// raw_vblank >= 0 bypasses the model and writes that VBLANK (calibration).
bool set_sensor_fps(int fps, const Config &cfg, SensorTiming *t, bool verbose, int raw_vblank = -1) {
    t->dev = find_sensor_subdev();
    if (t->dev.empty()) {
        fprintf(stderr, "camerad: imx363 subdev not found, --fps ignored\n");
        return false;
    }
    int fd = open(t->dev.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        fprintf(stderr, "camerad: open %s: %s, --fps ignored\n", t->dev.c_str(), strerror(errno));
        return false;
    }
    v4l2_queryctrl q{};
    q.id = V4L2_CID_VBLANK;
    if (ioctl(fd, VIDIOC_QUERYCTRL, &q) == 0) {
        t->vblank_min = q.minimum;
        t->vblank_max = q.maximum;
    }
    get_ctrl(fd, V4L2_CID_HBLANK, &t->hblank);
    get_ctrl(fd, V4L2_CID_PIXEL_RATE, &t->pixel_rate);
    if (t->pixel_rate <= 0 || t->vblank_max <= 0) {
        close(fd);
        fprintf(stderr, "camerad: sensor timing controls unavailable, --fps ignored\n");
        return false;
    }
    const double line_s = cfg.sensor_line_ns * 1e-9;
    int64_t vb = raw_vblank >= 0 ? raw_vblank
                                 : int64_t(std::llround(1.0 / (double(fps) * line_s) - cfg.sensor_base_lines));
    vb = std::max(t->vblank_min, std::min(t->vblank_max, vb));
    v4l2_control c{};
    c.id = V4L2_CID_VBLANK;
    c.value = int32_t(vb);
    bool ok = ioctl(fd, VIDIOC_S_CTRL, &c) == 0;
    if (!ok) fprintf(stderr, "camerad: set vblank on %s: %s\n", t->dev.c_str(), strerror(errno));
    get_ctrl(fd, V4L2_CID_VBLANK, &t->vblank);
    close(fd);
    if (verbose)
        fprintf(stderr,
                "camerad: %s vblank=%" PRId64 " (req fps %d, model %.2f fps; driver hblank %" PRId64
                " pixel_rate %" PRId64 ")\n",
                t->dev.c_str(), t->vblank, fps, 1.0 / ((cfg.sensor_base_lines + t->vblank) * line_s), t->hblank,
                t->pixel_rate);
    return ok;
}

int64_t read_vblank(const std::string &dev) {
    int fd = open(dev.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    int64_t v = -1;
    get_ctrl(fd, V4L2_CID_VBLANK, &v);
    close(fd);
    return v;
}

// --------------------------------------------------------- conversion pool ---

class ConvertPool {
public:
    explicit ConvertPool(int n) : n_(std::max(1, n)), tmp_(size_t(n_) * 3 * kWidth) {
        for (int i = 1; i < n_; ++i) threads_.emplace_back([this, i] { run(i); });
    }
    ~ConvertPool() {
        {
            std::lock_guard<std::mutex> l(m_);
            quit_ = true;
        }
        cv_.notify_all();
        for (auto &t : threads_) t.join();
    }

    void convert(const uint8_t *src, size_t src_stride, uint8_t *y, uint8_t *uv) {
        src_ = src;
        src_stride_ = src_stride;
        y_ = y;
        uv_ = uv;
        if (n_ > 1) {
            std::lock_guard<std::mutex> l(m_);
            pending_ = n_ - 1;
            ++gen_;
        }
        cv_.notify_all();
        band(0);
        if (n_ > 1) {
            std::unique_lock<std::mutex> l(m_);
            done_cv_.wait(l, [this] { return pending_ == 0; });
        }
    }

private:
    void band(int i) {
        const int rows = (kHeight / 2 + n_ - 1) / n_ * 2;
        const int r0 = i * rows, r1 = std::min(kHeight, r0 + rows);
        if (r0 < r1)
            xrgb_to_nv12_rows(src_, src_stride_, kWidth, y_, kWidth, uv_, kWidth, r0, r1,
                              tmp_.data() + size_t(i) * 3 * kWidth);
    }
    void run(int i) {
        uint64_t seen = 0;
        for (;;) {
            {
                std::unique_lock<std::mutex> l(m_);
                cv_.wait(l, [&] { return quit_ || gen_ != seen; });
                if (quit_) return;
                seen = gen_;
            }
            band(i);
            {
                std::lock_guard<std::mutex> l(m_);
                if (--pending_ == 0) done_cv_.notify_one();
            }
        }
    }

    int n_;
    std::vector<uint16_t> tmp_;
    std::vector<std::thread> threads_;
    std::mutex m_;
    std::condition_variable cv_, done_cv_;
    uint64_t gen_ = 0;
    int pending_ = 0;
    bool quit_ = false;
    const uint8_t *src_ = nullptr;
    size_t src_stride_ = 0;
    uint8_t *y_ = nullptr, *uv_ = nullptr;
};

// ------------------------------------------------------------- preview ---

// Preview pacing: every Nth frame as configured, slowed or stopped by the load governor's `preview_hz` cap.
struct PreviewGate {
    roadsense::GovernorReader gov;
    int64_t last_ns = 0;
    bool pass(uint64_t seq, int every, int64_t now_ns) {
        if (every <= 0 || seq % uint64_t(every) != 0) return false;
        const double cap = gov.cap("preview_hz");
        if (cap == 0) return false;
        if (cap > 0 && last_ns && double(now_ns - last_ns) < 1e9 / cap) return false;
        last_ns = now_ns;
        return true;
    }
};

class PreviewWriter {
public:
    static constexpr int kW = kWidth / 2, kH = kHeight / 2;

    PreviewWriter(std::string path, int quality) : path_(std::move(path)), enc_(quality), buf_(size_t(kW) * kH * 3 / 2) {
        thread_ = std::thread([this] { run(); });
    }
    ~PreviewWriter() {
        {
            std::lock_guard<std::mutex> l(m_);
            quit_ = true;
        }
        cv_.notify_all();
        thread_.join();
    }

    bool busy() const { return busy_.load(std::memory_order_acquire); }

    // Downscales into the preview buffer (on the caller's thread) and wakes
    // the encoder. Call only when !busy(). Returns the downscale time in ms.
    double submit(const uint8_t *y, const uint8_t *uv) {
        auto t0 = Clock::now();
        nv12_half(y, kWidth, uv, kWidth, kWidth, kHeight, buf_.data(), buf_.data() + size_t(kW) * kH);
        double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        busy_.store(true, std::memory_order_release);
        {
            std::lock_guard<std::mutex> l(m_);
            pending_ = true;
        }
        cv_.notify_one();
        return ms;
    }

    // Stats since last call: count, mean encode ms, mean write ms, bytes of last JPEG.
    void take_stats(int *n, double *enc_ms, double *write_ms, size_t *bytes) {
        std::lock_guard<std::mutex> l(sm_);
        *n = n_;
        *enc_ms = n_ ? enc_ms_ / n_ : 0;
        *write_ms = n_ ? write_ms_ / n_ : 0;
        *bytes = last_bytes_;
        n_ = 0;
        enc_ms_ = write_ms_ = 0;
    }

private:
    void run() {
        const std::string tmp = path_ + ".tmp";
        for (;;) {
            {
                std::unique_lock<std::mutex> l(m_);
                cv_.wait(l, [this] { return quit_ || pending_; });
                if (quit_) return;
                pending_ = false;
            }
            auto t0 = Clock::now();
            const auto &jpg = enc_.encode(buf_.data(), kW, buf_.data() + size_t(kW) * kH, kW, kW, kH);
            auto t1 = Clock::now();
            busy_.store(false, std::memory_order_release);  // buf_ no longer needed
            int fd = open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
            bool ok = fd >= 0;
            size_t off = 0;
            while (ok && off < jpg.size()) {
                ssize_t w = write(fd, jpg.data() + off, jpg.size() - off);
                if (w <= 0) ok = false;
                else off += size_t(w);
            }
            if (fd >= 0) close(fd);
            if (ok) ok = rename(tmp.c_str(), path_.c_str()) == 0;
            if (!ok) unlink(tmp.c_str());
            auto t2 = Clock::now();
            std::lock_guard<std::mutex> l(sm_);
            ++n_;
            enc_ms_ += std::chrono::duration<double, std::milli>(t1 - t0).count();
            write_ms_ += std::chrono::duration<double, std::milli>(t2 - t1).count();
            last_bytes_ = jpg.size();
        }
    }

    std::string path_;
    JpegNV12Encoder enc_;
    std::vector<uint8_t> buf_;
    std::thread thread_;
    std::mutex m_;
    std::condition_variable cv_;
    bool pending_ = false, quit_ = false;
    std::atomic<bool> busy_{false};
    std::mutex sm_;
    int n_ = 0;
    double enc_ms_ = 0, write_ms_ = 0;
    size_t last_bytes_ = 0;
};

// -------------------------------------------------------------- camerad ---

std::atomic<bool> g_done{false};  // --frames reached or fatal error in worker

struct Options {
    bool verbose = false;
    bool no_preview = false;
    uint64_t frames = 0;
    int fps = -1;  // -1 = from config
    int vblank = -1;  // raw VBLANK override (calibration), -1 = off
};

class Camerad {
public:
    Camerad(const Config &cfg, const Options &opt) : cfg_(cfg), opt_(opt) {}

    int run_setup() {
        pool_ = std::make_unique<ConvertPool>(cfg_.convert_threads);

        cm_ = std::make_unique<libcamera::CameraManager>();
        if (cm_->start() != 0) {
            fprintf(stderr, "camerad: CameraManager start failed\n");
            return 1;
        }
        for (const auto &c : cm_->cameras()) {
            auto loc = c->properties().get(libcamera::properties::Location);
            if (loc && *loc == libcamera::properties::CameraLocationBack) {
                camera_ = c;
                break;
            }
        }
        if (!camera_ && !cm_->cameras().empty()) camera_ = cm_->cameras()[0];
        if (!camera_) {
            fprintf(stderr, "camerad: no camera found (held by another process? check `fuser /dev/media0 /dev/video*`)\n");
            return 1;
        }
        if (camera_->acquire() != 0) {
            fprintf(stderr, "camerad: cannot acquire %s (busy? check `fuser /dev/video*`)\n", camera_->id().c_str());
            camera_.reset();
            return 1;
        }
        acquired_ = true;

        // Ring, socket and preview only once we own the camera, so a second
        // instance never replaces the ring of a running one.
        const std::string rd = rundir();
        if (!mkdir_p(rd)) {
            fprintf(stderr, "camerad: cannot create %s: %s\n", rd.c_str(), strerror(errno));
            return 1;
        }
        std::string err;
        if (!ring_.create(ring_path(), kWidth, kHeight, cfg_.focal_px, cfg_.cx, cfg_.cy, &err)) {
            fprintf(stderr, "camerad: ring: %s\n", err.c_str());
            return 1;
        }
        if (!server_.listen(rd + "/camerad.sock", &err)) {
            fprintf(stderr, "camerad: socket: %s\n", err.c_str());
            return 1;
        }
        if (!opt_.no_preview && cfg_.preview_every > 0)
            preview_ = std::make_unique<PreviewWriter>(rd + "/preview.jpg", cfg_.preview_quality);

        config_ = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
        if (!config_) {
            fprintf(stderr, "camerad: generateConfiguration failed\n");
            return 1;
        }
        libcamera::StreamConfiguration &sc = config_->at(0);
        sc.pixelFormat = libcamera::formats::XRGB8888;
        sc.size = libcamera::Size(kWidth, kHeight);
        sc.bufferCount = kBufferCount;
        if (config_->validate() == libcamera::CameraConfiguration::Invalid) {
            fprintf(stderr, "camerad: configuration invalid\n");
            return 1;
        }
        if (sc.pixelFormat != libcamera::formats::XRGB8888 || sc.size != libcamera::Size(kWidth, kHeight)) {
            fprintf(stderr, "camerad: camera adjusted the stream to %s, need XRGB8888 1920x1080\n",
                    sc.toString().c_str());
            return 1;
        }
        if (camera_->configure(config_.get()) != 0) {
            fprintf(stderr, "camerad: configure failed\n");
            return 1;
        }
        stream_ = sc.stream();
        src_stride_ = sc.stride;
        if (opt_.verbose)
            fprintf(stderr, "camerad: camera %s stream %s stride %u buffers %u\n", camera_->id().c_str(),
                    sc.toString().c_str(), sc.stride, sc.bufferCount);

        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        if (allocator_->allocate(stream_) < 0) {
            fprintf(stderr, "camerad: buffer allocation failed\n");
            return 1;
        }
        for (const auto &buf : allocator_->buffers(stream_)) {
            const auto &p = buf->planes()[0];
            const size_t map_len = p.offset + p.length;
            void *m = mmap(nullptr, map_len, PROT_READ, MAP_SHARED, p.fd.get(), 0);
            if (m == MAP_FAILED) {
                fprintf(stderr, "camerad: mmap buffer: %s\n", strerror(errno));
                return 1;
            }
            maps_[buf.get()] = Mapping{static_cast<uint8_t *>(m), map_len, p.offset, p.fd.get()};
            auto req = camera_->createRequest();
            if (!req || req->addBuffer(stream_, buf.get()) != 0) {
                fprintf(stderr, "camerad: request setup failed\n");
                return 1;
            }
            requests_.push_back(std::move(req));
        }

        // Sensor frame rate (VBLANK) must be set before streaming starts.
        const int fps = opt_.fps >= 0 ? opt_.fps : cfg_.fps;
        if (fps > 0 || opt_.vblank >= 0)
            fps_set_ = set_sensor_fps(fps, cfg_, &timing_, opt_.verbose, opt_.vblank);

        camera_->requestCompleted.connect(this, &Camerad::on_request);
        rt_offset_ = realtime_offset_ns();
        worker_ = std::thread([this] { worker(); });
        if (camera_->start() != 0) {
            fprintf(stderr, "camerad: camera start failed\n");
            return 1;
        }
        started_ = true;
        if (fps_set_) {
            int64_t vb = read_vblank(timing_.dev);
            if (vb != timing_.vblank) {
                // The pipeline reset it on start: apply again.
                if (opt_.verbose)
                    fprintf(stderr, "camerad: vblank reset to %" PRId64 " by start, re-applying\n", vb);
                set_sensor_fps(fps, cfg_, &timing_, opt_.verbose, opt_.vblank);
            }
        }
        for (auto &r : requests_) camera_->queueRequest(r.get());
        return 0;
    }

    void shutdown() {
        running_.store(false);
        if (started_) camera_->stop();
        started_ = false;
        {
            std::lock_guard<std::mutex> l(qm_);
            quit_ = true;
        }
        qcv_.notify_all();
        if (worker_.joinable()) worker_.join();
        if (camera_) camera_->requestCompleted.disconnect(this);
        requests_.clear();
        for (auto &kv : maps_) munmap(kv.second.base, kv.second.len);
        maps_.clear();
        allocator_.reset();
        if (acquired_) camera_->release();
        acquired_ = false;
        camera_.reset();
        config_.reset();
        if (cm_) cm_->stop();
        cm_.reset();
        preview_.reset();
        pool_.reset();
        server_.close();  // also unlinks the socket
        if (opt_.verbose) fprintf(stderr, "camerad: stopped after %" PRIu64 " frames\n", seq_);
    }

private:
    struct Mapping {
        uint8_t *base;
        size_t len;
        size_t offset;
        int fd;
    };

    // libcamera thread: hand off only.
    void on_request(libcamera::Request *req) {
        {
            std::lock_guard<std::mutex> l(qm_);
            queue_.push_back(req);
        }
        qcv_.notify_one();
    }

    void worker() {
        auto stats_t0 = Clock::now();
        int n_frames = 0;
        double conv_sum = 0, conv_max = 0, down_sum = 0;
        int n_down = 0;
        for (;;) {
            libcamera::Request *req = nullptr;
            {
                std::unique_lock<std::mutex> l(qm_);
                qcv_.wait_for(l, std::chrono::milliseconds(50), [this] { return quit_ || !queue_.empty(); });
                if (!queue_.empty()) {
                    req = queue_.front();
                    queue_.pop_front();
                } else if (quit_) {
                    return;
                }
            }
            if (!req) {
                server_.service();
                continue;
            }
            if (req->status() == libcamera::Request::RequestCancelled || !running_.load()) continue;

            double conv_ms = 0, down_ms = -1;
            process(req, &conv_ms, &down_ms);
            ++n_frames;
            conv_sum += conv_ms;
            conv_max = std::max(conv_max, conv_ms);
            if (down_ms >= 0) {
                down_sum += down_ms;
                ++n_down;
            }

            if (running_.load()) {
                req->reuse(libcamera::Request::ReuseBuffers);
                camera_->queueRequest(req);
            }

            auto now = Clock::now();
            double el = std::chrono::duration<double>(now - stats_t0).count();
            if (el >= 1.0) {
                rt_offset_ = realtime_offset_ns();  // follow NTP steps/slews
                if (opt_.verbose) {
                    int np = 0;
                    double enc = 0, wr = 0;
                    size_t bytes = 0;
                    if (preview_) preview_->take_stats(&np, &enc, &wr, &bytes);
                    fprintf(stderr,
                            "camerad: seq %" PRIu64 " fps %.1f (sensor %.2f) conv %.2f ms avg %.2f max | preview "
                            "%d/s down %.2f enc %.2f write %.2f ms %zu B | exp %u us gain %.2f | clients %zu "
                            "sensor_drops %" PRIu64 "\n",
                            seq_, n_frames / el, fps_, conv_sum / std::max(1, n_frames), conv_max, np,
                            n_down ? down_sum / n_down : 0.0, enc, wr, bytes, last_exp_, last_gain_,
                            server_.client_count(), sensor_drops_);
                }
                stats_t0 = now;
                n_frames = 0;
                conv_sum = conv_max = down_sum = 0;
                n_down = 0;
            }
            if (opt_.frames && seq_ >= opt_.frames) {
                running_.store(false);
                g_done.store(true);
                kill(getpid(), SIGTERM);  // wake the main thread's sigwait
            }
        }
    }

    void process(libcamera::Request *req, double *conv_ms, double *down_ms) {
        libcamera::FrameBuffer *fb = req->findBuffer(stream_);
        if (!fb) return;
        const auto &bm = fb->metadata();
        if (bm.status != libcamera::FrameMetadata::FrameSuccess) return;
        if (have_cam_seq_ && bm.sequence > last_cam_seq_ + 1) sensor_drops_ += bm.sequence - last_cam_seq_ - 1;
        last_cam_seq_ = bm.sequence;
        have_cam_seq_ = true;

        const libcamera::ControlList &md = req->metadata();
        int64_t ts = 0;
        if (auto v = md.get(libcamera::controls::SensorTimestamp)) ts = *v;
        else ts = int64_t(bm.timestamp);
        uint32_t exp = 0;
        if (auto v = md.get(libcamera::controls::ExposureTime)) exp = uint32_t(std::max<int32_t>(0, *v));
        float gain = 0, gr = 0, gb = 0;
        if (auto v = md.get(libcamera::controls::AnalogueGain)) gain = *v;
        if (auto v = md.get(libcamera::controls::ColourGains)) {
            gr = (*v)[0];
            gb = (*v)[1];
        }
        last_exp_ = exp;
        last_gain_ = gain;

        // rolling 1 s fps from sensor timestamps
        ts_hist_.push_back(ts);
        while (ts_hist_.size() > 2 && ts - ts_hist_.front() > 1000000000) ts_hist_.pop_front();
        if (ts_hist_.size() >= 2 && ts_hist_.back() > ts_hist_.front())
            fps_ = double(ts_hist_.size() - 1) * 1e9 / double(ts_hist_.back() - ts_hist_.front());

        const Mapping &m = maps_.at(fb);
        dma_buf_sync sync{DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ};
        ioctl(m.fd, DMA_BUF_IOCTL_SYNC, &sync);

        const uint64_t seq = ++seq_;
        auto t0 = Clock::now();
        uint8_t *dst = ring_.begin(seq);
        uint8_t *dst_uv = dst + size_t(kWidth) * kHeight;
        pool_->convert(m.base + m.offset, src_stride_, dst, dst_uv);
        SlotHeader *s = ring_.slot_for(seq);
        s->ts_mono_ns = ts;
        s->ts_real_ns = ts + rt_offset_;
        s->exposure_us = exp;
        s->analogue_gain = gain;
        s->colour_gain_r = gr;
        s->colour_gain_b = gb;
        ring_.commit(seq);
        *conv_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
        ioctl(m.fd, DMA_BUF_IOCTL_SYNC, &sync);

        char line[512];
        snprintf(line, sizeof(line),
                 "{\"type\":\"camera\",\"seq\":%" PRIu64 ",\"ts_mono_ns\":%" PRId64 ",\"ts_real_ns\":%" PRId64
                 ",\"exposure_us\":%u,\"analogue_gain\":%.3f,\"colour_gains\":[%.4f,%.4f],\"fps\":%.2f,"
                 "\"dropped\":0,\"sensor_drops\":%" PRIu64 ",\"conv_ms\":%.2f}",
                 seq, ts, ts + rt_offset_, exp, gain, gr, gb, fps_, sensor_drops_, *conv_ms);
        server_.publish(line);

        // Preview from the slot just written (it stays intact for 5 more frames).
        if (preview_ && preview_gate_.pass(seq, cfg_.preview_every, ts) && !preview_->busy())
            *down_ms = preview_->submit(dst, dst_uv);
    }

    Config cfg_;
    Options opt_;
    RingWriter ring_;
    JsonLineServer server_;
    std::unique_ptr<PreviewWriter> preview_;
    PreviewGate preview_gate_;
    std::unique_ptr<ConvertPool> pool_;

    std::unique_ptr<libcamera::CameraManager> cm_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::map<const libcamera::FrameBuffer *, Mapping> maps_;
    libcamera::Stream *stream_ = nullptr;
    unsigned src_stride_ = 0;
    bool acquired_ = false, started_ = false, fps_set_ = false;
    SensorTiming timing_;

    std::thread worker_;
    std::mutex qm_;
    std::condition_variable qcv_;
    std::deque<libcamera::Request *> queue_;
    bool quit_ = false;
    std::atomic<bool> running_{true};

    // worker-thread state
    uint64_t seq_ = 0;
    std::atomic<int64_t> rt_offset_{0};
    std::deque<int64_t> ts_hist_;
    double fps_ = 0;
    uint32_t last_cam_seq_ = 0;
    bool have_cam_seq_ = false;
    uint64_t sensor_drops_ = 0;
    uint32_t last_exp_ = 0;
    float last_gain_ = 0;
};

void usage() {
    fprintf(stderr,
            "usage: camerad [--verbose] [--frames N] [--no-preview] [--fps N]\n"
            "  --fps N   sensor frame rate via IMX363 VBLANK (default 20 or config \"fps\"; 0 = leave sensor alone)\n"
            "env: ROADSENSE_RUNDIR (default /run/roadsense), ROADSENSE_CONFIG_DIR (default /etc/roadsense),\n"
            "     ROADSENSE_RING (default /dev/shm/roadsense-frames)\n");
}

}  // namespace

int main(int argc, char **argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--verbose" || a == "-v") opt.verbose = true;
        else if (a == "--no-preview") opt.no_preview = true;
        else if (a == "--frames" && i + 1 < argc) opt.frames = strtoull(argv[++i], nullptr, 10);
        else if (a == "--fps" && i + 1 < argc) opt.fps = atoi(argv[++i]);
        else if (a == "--vblank" && i + 1 < argc) opt.vblank = atoi(argv[++i]);
        else {
            usage();
            return a == "--help" || a == "-h" ? 0 : 2;
        }
    }
    Config cfg;
    load_config(cfg, opt.verbose);

    if (cfg.little_cores) {
        // Whole process (inherited by libcamera's threads too) on the A55s.
        cpu_set_t set;
        CPU_ZERO(&set);
        for (int c = 0; c < 4; ++c) CPU_SET(c, &set);
        if (sched_setaffinity(0, sizeof(set), &set) != 0 && opt.verbose)
            fprintf(stderr, "camerad: sched_setaffinity: %s\n", strerror(errno));
    }

    // Block the termination signals in every thread; main waits for them.
    sigset_t sigs;
    sigemptyset(&sigs);
    sigaddset(&sigs, SIGINT);
    sigaddset(&sigs, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &sigs, nullptr);
    signal(SIGPIPE, SIG_IGN);

    Camerad cd(cfg, opt);
    int rc = cd.run_setup();
    if (rc == 0) {
        int sig = 0;
        sigwait(&sigs, &sig);
        if (opt.verbose && !g_done.load()) fprintf(stderr, "camerad: signal %d, stopping\n", sig);
    }
    cd.shutdown();
    return rc;
}

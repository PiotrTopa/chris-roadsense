// camerad-replay: a recorded ride (concatenated baseline JPEGs, "MJPEG" file as written by
// `ffmpeg -f mjpeg`) -> NV12 frame ring + camerad.sock JSON + preview.jpg, paced at the
// source frame rate and looping. A drop-in alternative to camerad / camerad-dvr (same ring
// contract, same socket name), see docs/ARCHITECTURE.md ("Alternative camera sources") and
// docs/lanes/L5-ride-replay.md (how the file is made: tools/make_ride.sh).
//
// Single thread: mmap the file, index the frames once (SOI .. EOI), then per frame decode
// with libjpeg(-turbo) raw (YCbCr planes, no colour conversion, no upsampling) straight into
// the ring slot (Y) and interleave CbCr (NV12), commit, publish, and every Nth frame write the
// frame's own JPEG as preview.jpg (no re-encode). Pacing uses absolute CLOCK_MONOTONIC
// deadlines; a frame whose deadline has already passed by more than one period is skipped
// (counted in sensor_drops) so the ride stays in real time. At the end of the file it pauses
// --gap-ms (default 1000, > modeld's 500 ms pairing limit) and starts again.
#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <csetjmp>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <jpeglib.h>

#include "jsonsock.h"
#include "ring.h"

using namespace roadsense;

namespace {

volatile sig_atomic_t g_run = 1;
void on_signal(int) { g_run = 0; }

int64_t clock_ns(clockid_t id) {
    timespec ts;
    clock_gettime(id, &ts);
    return int64_t(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}
double ms_since(int64_t t0) { return double(clock_ns(CLOCK_MONOTONIC) - t0) * 1e-6; }

std::string read_file(const std::string &path) {
    std::ifstream f(path);
    std::stringstream ss;
    if (f) ss << f.rdbuf();
    return ss.str();
}

// ------------------------------------------------------------------ config ---

struct Config {
    float focal_px = 0, cx = -1, cy = -1;  // 0 / -1 = derive from the frame size (see load_config)
    double fps = 20;                       // source frame rate of the file
    int preview_every = 2;                 // frame JPEG copied to preview.jpg every Nth frame (0 = off)
};

bool json_number(const std::string &s, const char *key, double *out) {
    const std::string k = std::string("\"") + key + "\"";
    size_t p = s.find(k);
    if (p == std::string::npos || (p = s.find(':', p + k.size())) == std::string::npos) return false;
    char *end = nullptr;
    double v = strtod(s.c_str() + p + 1, &end);
    if (end == s.c_str() + p + 1) return false;
    *out = v;
    return true;
}

void load_config(Config &c, bool verbose) {
    const char *d = getenv("ROADSENSE_CONFIG_DIR");
    const std::string path = std::string(d && *d ? d : "/etc/roadsense") + "/camera-replay.json";
    const std::string s = read_file(path);
    if (s.empty()) {
        fprintf(stderr, "camerad-replay: no config at %s, defaults\n", path.c_str());
        return;
    }
    double v;
    if (json_number(s, "focal_px", &v)) c.focal_px = float(v);
    if (json_number(s, "cx", &v)) c.cx = float(v);
    if (json_number(s, "cy", &v)) c.cy = float(v);
    if (json_number(s, "fps", &v) && v > 0) c.fps = v;
    if (json_number(s, "preview_every", &v)) c.preview_every = int(v);
    if (verbose) fprintf(stderr, "camerad-replay: config %s loaded\n", path.c_str());
}

std::string rundir() {
    const char *d = getenv("ROADSENSE_RUNDIR");
    return d && *d ? d : "/run/roadsense";
}

bool mkdir_p(const std::string &path) {
    std::string cur = path[0] == '/' ? "/" : "", part;
    std::stringstream ss(path);
    while (std::getline(ss, part, '/')) {
        if (part.empty()) continue;
        cur += part + "/";
        if (mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST) return false;
    }
    return true;
}

// ------------------------------------------------------------- the file ---

// Length of the JPEG starting at d (FF D8) up to and including its EOI, or 0 if malformed.
// Marker segments are walked up to SOS; entropy-coded data never contains FF D9.
size_t jpeg_length(const uint8_t *d, size_t n) {
    if (n < 4 || d[0] != 0xFF || d[1] != 0xD8) return 0;
    for (size_t i = 2; i + 3 < n;) {
        if (d[i] != 0xFF) return 0;
        const uint8_t m = d[i + 1];
        if (m == 0xFF) { ++i; continue; }  // fill byte
        const size_t len = size_t(d[i + 2]) << 8 | d[i + 3];
        if (m == 0xDA) {
            const uint8_t *p = d + i + 2 + len, *e = d + n;
            while (p + 1 < e) {
                p = static_cast<const uint8_t *>(memchr(p, 0xFF, size_t(e - p - 1)));
                if (!p) return 0;
                if (p[1] == 0xD9) return size_t(p + 2 - d);
                ++p;
            }
            return 0;
        }
        i += 2 + len;
    }
    return 0;
}

struct Frame { size_t off, len; };

class Ride {
public:
    ~Ride() {
        if (base_) munmap(const_cast<uint8_t *>(base_), size_);
    }
    bool open(const std::string &path, std::string *err) {
        int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd < 0) return *err = path + ": " + strerror(errno), false;
        struct stat st;
        if (fstat(fd, &st) != 0 || st.st_size < 4) {
            ::close(fd);
            return *err = path + ": empty or unreadable", false;
        }
        size_ = size_t(st.st_size);
        void *m = mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd, 0);
        ::close(fd);
        if (m == MAP_FAILED) return *err = path + ": mmap: " + strerror(errno), false;
        base_ = static_cast<const uint8_t *>(m);
        madvise(m, size_, MADV_SEQUENTIAL);
        for (size_t o = 0; o + 4 <= size_;) {
            if (base_[o] != 0xFF || base_[o + 1] != 0xD8) {  // resync on the next SOI
                const void *p = memmem(base_ + o + 1, size_ - o - 1, "\xFF\xD8\xFF", 3);
                if (!p) break;
                ++skipped_;
                o = size_t(static_cast<const uint8_t *>(p) - base_);
                continue;
            }
            const size_t len = jpeg_length(base_ + o, size_ - o);
            if (!len) break;  // truncated tail
            frames_.push_back({o, len});
            o += len;
        }
        if (frames_.empty()) return *err = path + ": no JPEG frames found", false;
        return true;
    }
    size_t count() const { return frames_.size(); }
    size_t skipped() const { return skipped_; }
    const uint8_t *data(size_t i) const { return base_ + frames_[i].off; }
    size_t len(size_t i) const { return frames_[i].len; }
    double mean_bytes() const { return double(size_) / double(frames_.size()); }

private:
    const uint8_t *base_ = nullptr;
    size_t size_ = 0, skipped_ = 0;
    std::vector<Frame> frames_;
};

// ------------------------------------------------------------ JPEG decode ---

struct JpegErr {
    jpeg_error_mgr mgr;
    jmp_buf jb;
    char msg[JMSG_LENGTH_MAX];
    int warnings;
};
void jpeg_err_exit(j_common_ptr c) {
    JpegErr *e = reinterpret_cast<JpegErr *>(c->err);
    c->err->format_message(c, e->msg);
    longjmp(e->jb, 1);
}
void jpeg_err_emit(j_common_ptr c, int level) {
    if (level < 0) ++reinterpret_cast<JpegErr *>(c->err)->warnings;
}

class Decoder {
public:
    Decoder() {
        cinfo_.err = jpeg_std_error(&err_.mgr);
        err_.mgr.error_exit = jpeg_err_exit;
        err_.mgr.emit_message = jpeg_err_emit;
        jpeg_create_decompress(&cinfo_);
    }
    ~Decoder() { jpeg_destroy_decompress(&cinfo_); }

    // Frame size of a JPEG (header only). False if it is not YCbCr 4:2:0 with W, H multiples of 16.
    bool probe(const uint8_t *data, size_t size, int *w, int *h, std::string *err) {
        if (setjmp(err_.jb)) {
            jpeg_abort_decompress(&cinfo_);
            return *err = err_.msg, false;
        }
        jpeg_mem_src(&cinfo_, const_cast<unsigned char *>(data), (unsigned long)size);
        jpeg_read_header(&cinfo_, TRUE);
        const bool ok = check(err);
        *w = int(cinfo_.image_width), *h = int(cinfo_.image_height);
        jpeg_abort_decompress(&cinfo_);
        return ok;
    }

    // Decodes a W x H 4:2:0 YCbCr JPEG to NV12 (y: W x H, uv: W x H/2).
    bool decode(const uint8_t *data, size_t size, int W, int H, uint8_t *y, uint8_t *uv, std::string *err,
                int *warnings) {
        err_.warnings = 0;
        if (setjmp(err_.jb)) {
            jpeg_abort_decompress(&cinfo_);
            *err = err_.msg;
            return false;
        }
        jpeg_mem_src(&cinfo_, const_cast<unsigned char *>(data), (unsigned long)size);
        jpeg_read_header(&cinfo_, TRUE);
        if (!check(err) || int(cinfo_.image_width) != W || int(cinfo_.image_height) != H) {
            jpeg_abort_decompress(&cinfo_);
            if (err->empty()) *err = "frame size changed inside the file";
            return false;
        }
        cinfo_.raw_data_out = TRUE;
        cinfo_.out_color_space = JCS_YCbCr;
        cinfo_.do_fancy_upsampling = FALSE;
        cinfo_.dct_method = JDCT_ISLOW;
        jpeg_start_decompress(&cinfo_);
        if (cb_.size() < size_t(8 * W / 2)) cb_.resize(8 * W / 2), cr_.resize(8 * W / 2);
        JSAMPROW yr[16], cbr[8], crr[8];
        JSAMPARRAY planes[3] = {yr, cbr, crr};
        while (cinfo_.output_scanline < unsigned(H)) {
            const unsigned r0 = cinfo_.output_scanline;  // multiple of 16
            for (int i = 0; i < 16; ++i) yr[i] = y + size_t(r0 + i) * W;
            for (int i = 0; i < 8; ++i) {
                cbr[i] = cb_.data() + size_t(i) * (W / 2);
                crr[i] = cr_.data() + size_t(i) * (W / 2);
            }
            if (jpeg_read_raw_data(&cinfo_, planes, 16) != 16) {
                jpeg_abort_decompress(&cinfo_);
                *err = "short raw read";
                return false;
            }
            for (int i = 0; i < 8; ++i) {  // CbCr rows r0/2 .. r0/2+7, interleaved
                uint8_t *d = uv + size_t(r0 / 2 + i) * W;
                const uint8_t *b = cbr[i], *r = crr[i];
                for (int x = 0; x < W / 2; ++x) {
                    d[2 * x] = b[x];
                    d[2 * x + 1] = r[x];
                }
            }
        }
        jpeg_abort_decompress(&cinfo_);
        *warnings = err_.warnings;
        return true;
    }

private:
    bool check(std::string *err) {
        const jpeg_component_info *c = cinfo_.comp_info;
        if (cinfo_.num_components != 3 || cinfo_.jpeg_color_space != JCS_YCbCr || c[0].h_samp_factor != 2 ||
            c[0].v_samp_factor != 2 || c[1].h_samp_factor != 1 || c[1].v_samp_factor != 1 ||
            c[2].h_samp_factor != 1 || c[2].v_samp_factor != 1 || cinfo_.image_width % 16 ||
            cinfo_.image_height % 16) {
            char b[160];
            snprintf(b, sizeof(b), "unsupported JPEG %ux%u, %d comps (need YCbCr 4:2:0, W and H multiples of 16)",
                     cinfo_.image_width, cinfo_.image_height, cinfo_.num_components);
            *err = b;
            return false;
        }
        err->clear();
        return true;
    }
    jpeg_decompress_struct cinfo_{};
    JpegErr err_{};
    std::vector<uint8_t> cb_, cr_;
};

bool write_preview(const std::string &path, const uint8_t *jpg, size_t n) {
    const std::string tmp = path + ".tmp";
    int fd = open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) return false;
    bool ok = write(fd, jpg, n) == ssize_t(n);
    close(fd);
    if (ok) ok = rename(tmp.c_str(), path.c_str()) == 0;
    if (!ok) unlink(tmp.c_str());
    return ok;
}

// ----------------------------------------------------------------- daemon ---

struct Options {
    std::string file;
    bool verbose = false, no_preview = false, once = false;
    uint64_t frames = 0;
    double fps = 0, start_s = 0;
    int gap_ms = 1000;
};

void usage() {
    fprintf(stderr,
            "usage: camerad-replay --file RIDE.mjpeg [--fps N] [--start SECONDS] [--once] [--gap-ms MS]\n"
            "                      [--verbose] [--frames N] [--no-preview]\n"
            "  --file    concatenated baseline JPEGs, YCbCr 4:2:0, W and H multiples of 16\n"
            "            (ffmpeg ... -pix_fmt yuvj420p -f mjpeg RIDE.mjpeg, see tools/make_ride.sh)\n"
            "  --fps     source frame rate (default: camera-replay.json \"fps\", else 20)\n"
            "  --start   first frame at this time in the file (seconds)\n"
            "  --once    stop at the end of the file instead of looping\n"
            "  --gap-ms  pause at the loop seam (default 1000: > modeld's 500 ms pairing limit)\n"
            "env: ROADSENSE_RUNDIR (default /run/roadsense), ROADSENSE_CONFIG_DIR (default /etc/roadsense,\n"
            "     file camera-replay.json: focal_px, cx, cy, fps, preview_every), ROADSENSE_RING\n");
}

void sleep_until(int64_t t_ns) {
    timespec ts{time_t(t_ns / 1000000000), long(t_ns % 1000000000)};
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR && g_run) {
    }
}

int run(const Options &opt, Config cfg) {
    Ride ride;
    Decoder dec;
    std::string err;
    const int64_t t_open = clock_ns(CLOCK_MONOTONIC);
    if (!ride.open(opt.file, &err)) {
        fprintf(stderr, "camerad-replay: %s\n", err.c_str());
        return 1;
    }
    int W = 0, H = 0;
    if (!dec.probe(ride.data(0), ride.len(0), &W, &H, &err)) {
        fprintf(stderr, "camerad-replay: first frame: %s\n", err.c_str());
        return 1;
    }
    if (opt.fps > 0) cfg.fps = opt.fps;
    if (cfg.focal_px <= 0) cfg.focal_px = float(W) * 975.0f / 1280.0f;  // camerad-dvr's lens, scaled: a placeholder
    if (cfg.cx < 0) cfg.cx = W / 2.0f;
    if (cfg.cy < 0) cfg.cy = H / 2.0f;
    const size_t n = ride.count();
    fprintf(stderr,
            "camerad-replay: %s: %zu frames %dx%d (%.1f min at %.2f fps, %.0f KB/frame, %zu resyncs), "
            "indexed in %.0f ms; focal %.1f cx %.1f cy %.1f\n",
            opt.file.c_str(), n, W, H, double(n) / cfg.fps / 60.0, cfg.fps, ride.mean_bytes() / 1024.0,
            ride.skipped(), ms_since(t_open), cfg.focal_px, cfg.cx, cfg.cy);

    const std::string rd = rundir(), preview_path = rd + "/preview.jpg";
    RingWriter ring;
    JsonLineServer server;
    if (!mkdir_p(rd)) {
        fprintf(stderr, "camerad-replay: cannot create %s: %s\n", rd.c_str(), strerror(errno));
        return 1;
    }
    if (!ring.create(ring_path(), uint32_t(W), uint32_t(H), cfg.focal_px, cfg.cx, cfg.cy, &err) ||
        !server.listen(rd + "/camerad.sock", &err)) {
        fprintf(stderr, "camerad-replay: %s\n", err.c_str());
        return 1;
    }

    const int64_t period = int64_t(1e9 / cfg.fps + 0.5);
    size_t idx = std::min(n - 1, size_t(std::max(0.0, opt.start_s) * cfg.fps));
    uint64_t seq = 0, skipped = 0, decode_fail = 0, warnings_total = 0, loops = 0;
    int64_t deadline = clock_ns(CLOCK_MONOTONIC), rt_off = 0, next_rt = 0, next_stat = 0;
    int64_t fps_t0 = 0;
    uint64_t fps_seq0 = 0;
    double fps = 0;
    int st_n = 0;
    double st_dec = 0, st_dec_max = 0;

    while (g_run) {
        if (idx >= n) {  // end of the ride
            if (opt.once) break;
            idx = 0;
            ++loops;
            deadline += int64_t(opt.gap_ms) * 1000000;
            fprintf(stderr, "camerad-replay: loop %" PRIu64 " (pause %d ms)\n", loops, opt.gap_ms);
        }
        // wait for this frame's slot in time, serving clients meanwhile
        for (;;) {
            server.service();
            const int64_t now = clock_ns(CLOCK_MONOTONIC);
            if (now >= deadline || !g_run) break;
            sleep_until(std::min(deadline, now + 20000000));
        }
        if (!g_run) break;
        int64_t now = clock_ns(CLOCK_MONOTONIC);
        if (now - deadline > period) {  // fell behind by more than a frame: skip to stay real time
            const int64_t k = (now - deadline) / period;
            skipped += uint64_t(k), idx += size_t(k), deadline += k * period;
            if (idx >= n) continue;
        }
        if (now >= next_rt) {
            int64_t best = INT64_MAX;
            for (int i = 0; i < 5; ++i) {
                int64_t m0 = clock_ns(CLOCK_MONOTONIC), r = clock_ns(CLOCK_REALTIME), m1 = clock_ns(CLOCK_MONOTONIC);
                if (m1 - m0 < best) best = m1 - m0, rt_off = r - (m0 + (m1 - m0) / 2);
            }
            next_rt = now + 1000000000;
        }
        if (opt.verbose && now >= next_stat) {
            if (next_stat && st_n)
                fprintf(stderr,
                        "camerad-replay: seq %" PRIu64 " frame %zu/%zu fps %.1f decode+nv12 %.2f ms avg %.2f max, "
                        "skipped %" PRIu64 " decode_fail %" PRIu64 " loops %" PRIu64 "\n",
                        seq, idx, n, fps, st_dec / st_n, st_dec_max, skipped, decode_fail, loops);
            st_n = 0, st_dec = st_dec_max = 0;
            next_stat = now + 1000000000;
        }

        const int64_t t_frame = deadline;  // the frame's nominal capture time
        const int64_t t1 = clock_ns(CLOCK_MONOTONIC);
        uint8_t *dst = ring.begin(seq + 1);
        int warn = 0;
        const uint8_t *jpg = ride.data(idx);
        const size_t jlen = ride.len(idx);
        if (dec.decode(jpg, jlen, W, H, dst, dst + size_t(W) * H, &err, &warn)) {
            ++seq;
            SlotHeader *s = ring.slot_for(seq);
            s->ts_mono_ns = t_frame;
            s->ts_real_ns = t_frame + rt_off;
            s->exposure_us = 0;
            s->analogue_gain = s->colour_gain_r = s->colour_gain_b = 0;
            ring.commit(seq);
            const double dec_ms = ms_since(t1);
            warnings_total += uint64_t(warn);
            if (!fps_t0) fps_t0 = t_frame, fps_seq0 = seq;
            if (t_frame - fps_t0 >= 1000000000) {
                fps = double(seq - fps_seq0) * 1e9 / double(t_frame - fps_t0);
                fps_t0 = t_frame, fps_seq0 = seq;
            }
            ++st_n, st_dec += dec_ms, st_dec_max = std::max(st_dec_max, dec_ms);
            char line[512];
            snprintf(line, sizeof(line),
                     "{\"type\":\"camera\",\"seq\":%" PRIu64 ",\"ts_mono_ns\":%" PRId64 ",\"ts_real_ns\":%" PRId64
                     ",\"exposure_us\":0,\"analogue_gain\":0,\"colour_gains\":[0,0],\"fps\":%.2f,\"dropped\":0,"
                     "\"sensor_drops\":%" PRIu64 ",\"conv_ms\":%.2f,\"source\":\"replay\",\"file_frame\":%zu,"
                     "\"file_frames\":%zu,\"file_t_s\":%.2f,\"loops\":%" PRIu64 ",\"jpeg_bytes\":%zu,"
                     "\"decode_fail\":%" PRIu64 ",\"jpeg_warnings\":%" PRIu64 "}",
                     seq, t_frame, t_frame + rt_off, fps, skipped, dec_ms, idx, n, double(idx) / cfg.fps, loops, jlen,
                     decode_fail, warnings_total);
            server.publish(line);
            if (!opt.no_preview && cfg.preview_every > 0 && seq % uint64_t(cfg.preview_every) == 0)
                write_preview(preview_path, jpg, jlen);
            if (opt.frames && seq >= opt.frames) g_run = 0;
        } else {
            ++decode_fail;
            if (opt.verbose) fprintf(stderr, "camerad-replay: frame %zu: decode failed: %s\n", idx, err.c_str());
        }
        ++idx;
        deadline += period;
    }
    server.close();
    unlink((rd + "/camerad.sock").c_str());
    fprintf(stderr,
            "camerad-replay: stopped after %" PRIu64 " frames (skipped %" PRIu64 ", decode_fail %" PRIu64
            ", loops %" PRIu64 ")\n",
            seq, skipped, decode_fail, loops);
    return 0;
}

}  // namespace

int main(int argc, char **argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--file" && i + 1 < argc) opt.file = argv[++i];
        else if (a == "--fps" && i + 1 < argc) opt.fps = atof(argv[++i]);
        else if (a == "--start" && i + 1 < argc) opt.start_s = atof(argv[++i]);
        else if (a == "--gap-ms" && i + 1 < argc) opt.gap_ms = std::max(0, atoi(argv[++i]));
        else if (a == "--once") opt.once = true;
        else if (a == "--verbose" || a == "-v") opt.verbose = true;
        else if (a == "--no-preview") opt.no_preview = true;
        else if (a == "--frames" && i + 1 < argc) opt.frames = strtoull(argv[++i], nullptr, 10);
        else {
            usage();
            return a == "--help" || a == "-h" ? 0 : 2;
        }
    }
    if (opt.file.empty()) {
        usage();
        return 2;
    }
    Config cfg;
    load_config(cfg, opt.verbose);
    struct sigaction sa{};
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN);
    return run(opt, cfg);
}

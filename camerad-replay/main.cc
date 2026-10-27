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
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <dirent.h>
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

std::string config_dir() {
    const char *d = getenv("ROADSENSE_CONFIG_DIR");
    return d && *d ? d : "/etc/roadsense";
}

// Reads focal_px, cx, cy, fps, preview_every from a camera-replay.json; false if absent/empty.
bool load_config(Config &c, const std::string &path, bool verbose) {
    const std::string s = read_file(path);
    if (s.empty()) return false;
    double v;
    if (json_number(s, "focal_px", &v)) c.focal_px = float(v);
    if (json_number(s, "cx", &v)) c.cx = float(v);
    if (json_number(s, "cy", &v)) c.cy = float(v);
    if (json_number(s, "fps", &v) && v > 0) c.fps = v;
    if (json_number(s, "preview_every", &v)) c.preview_every = int(v);
    if (verbose) fprintf(stderr, "camerad-replay: config %s loaded\n", path.c_str());
    return true;
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
        if (!path_.empty()) {  // drop the played file from the page cache: it comes back in hours at the earliest
            int fd = ::open(path_.c_str(), O_RDONLY | O_CLOEXEC);
            if (fd >= 0) posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED), ::close(fd);
        }
    }
    bool open(const std::string &path, std::string *err) {
        path_ = path;
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
    std::string path_;
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

// --------------------------------------------------------------- playlist ---

struct RideEntry {
    std::string dir, file, slug, author, licence;
};

bool is_dir(const std::string &p) {
    struct stat st;
    return stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}
std::string dirname_of(const std::string &p) {
    const size_t i = p.rfind('/');
    return i == std::string::npos ? "." : i == 0 ? "/" : p.substr(0, i);
}
std::string basename_of(std::string p) {
    while (p.size() > 1 && p.back() == '/') p.pop_back();
    const size_t i = p.rfind('/');
    return i == std::string::npos ? p : p.substr(i + 1);
}
std::string trim(const std::string &s) {
    size_t a = 0, b = s.size();
    while (a < b && isspace((unsigned char)s[a])) ++a;
    while (b > a && isspace((unsigned char)s[b - 1])) --b;
    return s.substr(a, b - a);
}
std::vector<std::string> list_dir(const std::string &dir) {
    std::vector<std::string> out;
    if (DIR *d = opendir(dir.c_str())) {
        while (dirent *e = readdir(d))
            if (e->d_name[0] != '.') out.push_back(e->d_name);
        closedir(d);
    }
    std::sort(out.begin(), out.end());
    return out;
}
bool ends_with(const std::string &s, const char *suf) {
    const size_t n = strlen(suf);
    return s.size() >= n && s.compare(s.size() - n, n, suf) == 0;
}

// SOURCE.txt (docs/RIDES.md): line 1 "<title> -- <author> (<url>)"; the licence is the first "CC BY..." token.
void read_source(RideEntry &r) {
    std::ifstream f(r.dir + "/SOURCE.txt");
    std::string line;
    for (int ln = 0; std::getline(f, line); ++ln) {
        if (ln == 0) {
            const size_t p = line.rfind(" -- ");
            if (p != std::string::npos) {
                std::string a = line.substr(p + 4);
                const size_t q = a.find(" (");
                r.author = trim(q == std::string::npos ? a : a.substr(0, q));
            }
        }
        const size_t p = line.find("CC BY");
        if (r.licence.empty() && p != std::string::npos) {
            size_t e = p;
            while (e < line.size() && (isalnum((unsigned char)line[e]) || strchr(" .-", line[e]))) ++e;
            r.licence = trim(line.substr(p, e - p));
        }
    }
}

// A ride directory (its first *.mjpeg, sorted) or an .mjpeg file (its directory is the ride's).
bool make_entry(const std::string &path, RideEntry *r) {
    *r = RideEntry();
    if (is_dir(path)) {
        r->dir = path;
        for (const std::string &n : list_dir(path))
            if (ends_with(n, ".mjpeg")) {
                r->file = path + "/" + n;
                break;
            }
        if (r->file.empty()) return false;
    } else {
        r->file = path;
        r->dir = dirname_of(path);
    }
    r->slug = basename_of(r->dir);
    read_source(*r);
    return true;
}

// FILE: one ride directory or .mjpeg per line ('#' comments, relative to the list's directory);
// DIR: every subdirectory with an .mjpeg, in name order.
bool load_playlist(const std::string &src, std::vector<RideEntry> *out, bool verbose) {
    out->clear();
    std::vector<std::string> paths;
    if (is_dir(src)) {
        for (const std::string &n : list_dir(src)) paths.push_back(src + "/" + n);
    } else {
        std::ifstream f(src);
        if (!f) return false;
        std::string line;
        while (std::getline(f, line)) {
            line = trim(line.substr(0, line.find('#')));
            if (line.empty()) continue;
            paths.push_back(line[0] == '/' ? line : dirname_of(src) + "/" + line);
        }
    }
    for (const std::string &p : paths) {
        RideEntry e;
        if (make_entry(p, &e)) out->push_back(e);
        else if (verbose || !is_dir(src)) fprintf(stderr, "camerad-replay: playlist: %s: no .mjpeg, skipped\n", p.c_str());
    }
    return true;
}

std::string json_escape(const std::string &s) {
    std::string o;
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') o += '\\', o += char(c);
        else if (c < 0x20) {
            char b[8];
            snprintf(b, sizeof(b), "\\u%04x", c);
            o += b;
        } else o += char(c);
    }
    return o;
}

// Atomic copy (tmp + rename): modeld re-reads calib.json when its mtime changes.
bool install_file(const std::string &src, const std::string &dst, std::string *err) {
    const std::string s = read_file(src);
    if (s.empty()) return *err = src + ": missing or empty", false;
    const std::string tmp = dst + ".tmp";
    int fd = open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) return *err = tmp + ": " + strerror(errno), false;
    bool ok = write(fd, s.data(), s.size()) == ssize_t(s.size());
    fchmod(fd, 0644);
    close(fd);
    if (ok) ok = rename(tmp.c_str(), dst.c_str()) == 0;
    if (!ok) {
        unlink(tmp.c_str());
        return *err = dst + ": " + strerror(errno), false;
    }
    return true;
}

// ----------------------------------------------------------------- daemon ---

struct Options {
    std::string file, playlist, calib_out;
    bool verbose = false, no_preview = false, once = false, shuffle = false, no_calib = false;
    uint64_t frames = 0;
    double fps = 0, start_s = 0, max_s_per_ride = 0;
    int gap_ms = -1;  // default: 1000 (one file), 2500 (playlist)
};

void usage() {
    fprintf(stderr,
            "usage: camerad-replay --file RIDE.mjpeg | --playlist LIST|DIR  [--fps N] [--start SECONDS] [--once]\n"
            "                      [--gap-ms MS] [--shuffle] [--max-seconds-per-ride S] [--calib-out PATH | --no-calib]\n"
            "                      [--verbose] [--frames N] [--no-preview]\n"
            "  --file      concatenated baseline JPEGs, YCbCr 4:2:0, W and H multiples of 16\n"
            "              (ffmpeg ... -pix_fmt yuvj420p -f mjpeg RIDE.mjpeg, see tools/make_ride.sh); loops\n"
            "  --playlist  rides one after another, looping the list: LIST = one ride directory (or .mjpeg) per\n"
            "              line, '#' comments, relative to the list's directory; DIR = its subdirectories in name\n"
            "              order. Per ride: <dir>/*.mjpeg, camera-replay.json (focal_px, cx, cy, fps), calib.json\n"
            "              (copied to --calib-out, default $ROADSENSE_CONFIG_DIR/calib.json), SOURCE.txt\n"
            "              (author, licence). Every ride gets a new ring (seq restarts at 1). The list is re-read\n"
            "              at every wrap.\n"
            "  --shuffle   playlist in random order, reshuffled at every wrap\n"
            "  --max-seconds-per-ride  (testing) play at most S seconds of each ride\n"
            "  --fps       source frame rate (default: camera-replay.json \"fps\", else 20)\n"
            "  --start     first frame at this time in the (first) file (seconds)\n"
            "  --once      stop at the end of the file / of the playlist instead of looping\n"
            "  --gap-ms    pause at a loop seam or ride change (default 1000 for a file: > modeld's 500 ms pairing\n"
            "              limit; 2500 for a playlist: > modeld's 2 s calib.json check, so the first frame of a\n"
            "              ride already uses its calibration)\n"
            "env: ROADSENSE_RUNDIR (default /run/roadsense), ROADSENSE_CONFIG_DIR (default /etc/roadsense; with\n"
            "     --file its camera-replay.json: focal_px, cx, cy, fps, preview_every), ROADSENSE_RING\n");
}

void sleep_until(int64_t t_ns) {
    timespec ts{time_t(t_ns / 1000000000), long(t_ns % 1000000000)};
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR && g_run) {
    }
}

class Player {
public:
    Player(const Options &opt, const Config &base) : opt_(opt), base_(base) {}

    int run() {
        playlist_ = !opt_.playlist.empty();
        gap_ms_ = opt_.gap_ms >= 0 ? opt_.gap_ms : (playlist_ ? 2500 : 1000);
        calib_out_ = opt_.calib_out.empty() ? config_dir() + "/calib.json" : opt_.calib_out;
        if (!reload_list() || list_.empty()) {
            fprintf(stderr, "camerad-replay: %s: no rides\n", playlist_ ? opt_.playlist.c_str() : opt_.file.c_str());
            return 1;
        }
        rng_.seed(uint32_t(clock_ns(CLOCK_REALTIME)));
        make_order(false);
        rd_ = rundir();
        if (!mkdir_p(rd_)) {
            fprintf(stderr, "camerad-replay: cannot create %s: %s\n", rd_.c_str(), strerror(errno));
            return 1;
        }
        std::string err;
        if (!server_.listen(rd_ + "/camerad.sock", &err)) {
            fprintf(stderr, "camerad-replay: %s\n", err.c_str());
            return 1;
        }
        if (playlist_)
            fprintf(stderr, "camerad-replay: playlist %s: %zu rides%s, gap %d ms, calib -> %s\n", opt_.playlist.c_str(),
                    list_.size(), opt_.shuffle ? " (shuffled)" : "", gap_ms_, opt_.no_calib ? "(off)" : calib_out_.c_str());
        pos_ = 0;
        if (!start_ride(true)) return playlist_ ? 0 : 1;
        loop();
        server_.close();
        unlink((rd_ + "/camerad.sock").c_str());
        fprintf(stderr,
                "camerad-replay: stopped after %" PRIu64 " frames (skipped %" PRIu64 ", decode_fail %" PRIu64
                ", loops %" PRIu64 ", rides %" PRIu64 ")\n",
                total_, skipped_, decode_fail_, loops_, rides_started_);
        return 0;
    }

private:
    bool reload_list() {
        if (!playlist_) {
            RideEntry e;
            if (!make_entry(opt_.file, &e)) return false;
            list_ = {e};
            return true;
        }
        std::vector<RideEntry> l;
        if (!load_playlist(opt_.playlist, &l, opt_.verbose) || l.empty()) return !list_.empty();  // keep the old list
        list_.swap(l);
        return true;
    }
    void make_order(bool avoid_repeat) {
        order_.resize(list_.size());
        for (size_t i = 0; i < order_.size(); ++i) order_[i] = i;
        if (opt_.shuffle) {
            std::shuffle(order_.begin(), order_.end(), rng_);
            if (avoid_repeat && order_.size() > 1 && list_[order_[0]].slug == cur_slug_) std::swap(order_[0], order_[1]);
        }
    }

    void wait_serving(int64_t until) {
        for (;;) {
            server_.service();
            const int64_t now = clock_ns(CLOCK_MONOTONIC);
            if (now >= until || !g_run) return;
            sleep_until(std::min(until, now + 20000000));
        }
    }

    // Opens order_[pos_] (or the next playable one), writes its calib.json, makes a new ring.
    bool start_ride(bool first) {
        for (int attempt = 0; g_run; ++attempt) {
            if (pos_ >= order_.size()) {  // wrap: re-read the list
                if (opt_.once) return false;
                ++loops_;
                reload_list();
                make_order(true);
                pos_ = 0;
                fprintf(stderr, "camerad-replay: playlist loop %" PRIu64 " (%zu rides)\n", loops_, list_.size());
            }
            if (attempt > 0 && attempt % int(std::max<size_t>(1, list_.size())) == 0) {
                fprintf(stderr, "camerad-replay: no playable ride, retrying in 10 s\n");
                wait_serving(clock_ns(CLOCK_MONOTONIC) + 10000000000LL);
                if (!g_run) return false;
            }
            const size_t k = order_[pos_++];
            if (open_ride(k, first)) return true;
        }
        return false;
    }

    bool open_ride(size_t k, bool first) {
        const RideEntry &e = list_[k];
        const int64_t t0 = clock_ns(CLOCK_MONOTONIC);
        std::string err;
        // calibration first: modeld picks it up on the first frame after the gap (>= its 2 s check)
        if (playlist_ && !opt_.no_calib && !install_file(e.dir + "/calib.json", calib_out_, &err))
            fprintf(stderr, "camerad-replay: %s: calib not switched: %s\n", e.slug.c_str(), err.c_str());
        auto r = std::make_unique<Ride>();
        ride_.reset();  // unmap (and uncache) the previous file first
        int W = 0, H = 0;
        if (!r->open(e.file, &err) || !dec_.probe(r->data(0), r->len(0), &W, &H, &err)) {
            fprintf(stderr, "camerad-replay: %s: %s\n", e.slug.c_str(), err.c_str());
            return false;
        }
        Config c = base_;
        if (playlist_) {
            c = Config();
            if (!load_config(c, e.dir + "/camera-replay.json", opt_.verbose))
                fprintf(stderr, "camerad-replay: %s: no camera-replay.json, default intrinsics\n", e.slug.c_str());
        }
        if (opt_.fps > 0) c.fps = opt_.fps;
        if (c.focal_px <= 0) c.focal_px = float(W) * 975.0f / 1280.0f;  // camerad-dvr's lens, scaled: a placeholder
        if (c.cx < 0) c.cx = W / 2.0f;
        if (c.cy < 0) c.cy = H / 2.0f;
        // A new ring per ride: the intrinsics (and maybe the size) change; seq restarts at 1, which every
        // reader treats as a camera restart (modeld pairing, envd smoother, signd tracks reset).
        if (!ring_.create(ring_path(), uint32_t(W), uint32_t(H), c.focal_px, c.cx, c.cy, &err)) {
            fprintf(stderr, "camerad-replay: ring: %s\n", err.c_str());
            return false;
        }
        ride_ = std::move(r);
        cfg_ = c, W_ = W, H_ = H, n_ = ride_->count(), cur_ = k, cur_slug_ = e.slug;
        seq_ = 0, fps_t0_ = 0, ride_skipped0_ = skipped_;
        idx_ = first ? std::min(n_ - 1, size_t(std::max(0.0, opt_.start_s) * cfg_.fps)) : 0;
        end_idx_ = n_;
        if (opt_.max_s_per_ride > 0) end_idx_ = std::min(n_, idx_ + size_t(opt_.max_s_per_ride * cfg_.fps));
        period_ = int64_t(1e9 / cfg_.fps + 0.5);
        ++rides_started_;
        fprintf(stderr,
                "camerad-replay: ride %zu/%zu %s: %s: %zu frames %dx%d (%.1f min at %.2f fps, %.0f KB/frame, %zu "
                "resyncs), opened in %.0f ms; focal %.1f cx %.1f cy %.1f; %s, %s\n",
                pos_, order_.size(), e.slug.c_str(), e.file.c_str(), n_, W, H, double(n_) / cfg_.fps / 60.0,
                cfg_.fps, ride_->mean_bytes() / 1024.0, ride_->skipped(), ms_since(t0), cfg_.focal_px, cfg_.cx,
                cfg_.cy, e.author.empty() ? "author ?" : e.author.c_str(), e.licence.empty() ? "licence ?" : e.licence.c_str());
        // gap before the first frame (also on the very first ride of a playlist: a running modeld must see the
        // new calib.json before frames come)
        deadline_ = clock_ns(CLOCK_MONOTONIC) + (playlist_ ? int64_t(gap_ms_) * 1000000 : 0);
        return true;
    }

    void loop() {
        int64_t rt_off = 0, next_rt = 0, next_stat = 0;
        int st_n = 0;
        double st_dec = 0, st_dec_max = 0, fps = 0;
        uint64_t fps_seq0 = 0;
        while (g_run) {
            if (idx_ >= end_idx_) {  // end of the ride (or of its test window)
                if (!playlist_) {
                    if (opt_.once) break;
                    idx_ = 0, end_idx_ = n_;
                    ++loops_;
                    deadline_ += int64_t(gap_ms_) * 1000000;
                    fprintf(stderr, "camerad-replay: loop %" PRIu64 " (pause %d ms)\n", loops_, gap_ms_);
                } else {
                    if (!start_ride(false)) break;
                    fps = 0;
                }
            }
            wait_serving(deadline_);
            if (!g_run) break;
            int64_t now = clock_ns(CLOCK_MONOTONIC);
            if (now - deadline_ > period_) {  // fell behind by more than a frame: skip to stay real time
                const int64_t k = (now - deadline_) / period_;
                skipped_ += uint64_t(k), idx_ += size_t(k), deadline_ += k * period_;
                if (idx_ >= end_idx_) continue;
            }
            if (now >= next_rt) {
                int64_t best = INT64_MAX;
                for (int i = 0; i < 5; ++i) {
                    int64_t m0 = clock_ns(CLOCK_MONOTONIC), r = clock_ns(CLOCK_REALTIME), m1 = clock_ns(CLOCK_MONOTONIC);
                    if (m1 - m0 < best) best = m1 - m0, rt_off = r - (m0 + (m1 - m0) / 2);
                }
                next_rt = now + 1000000000;
            }
            if (opt_.verbose && now >= next_stat) {
                if (next_stat && st_n)
                    fprintf(stderr,
                            "camerad-replay: %s seq %" PRIu64 " frame %zu/%zu fps %.1f decode+nv12 %.2f ms avg %.2f max, "
                            "skipped %" PRIu64 " decode_fail %" PRIu64 " loops %" PRIu64 "\n",
                            cur_slug_.c_str(), seq_, idx_, n_, fps, st_dec / st_n, st_dec_max, skipped_, decode_fail_,
                            loops_);
                st_n = 0, st_dec = st_dec_max = 0;
                next_stat = now + 1000000000;
            }

            const int64_t t_frame = deadline_;  // the frame's nominal capture time
            const int64_t t1 = clock_ns(CLOCK_MONOTONIC);
            uint8_t *dst = ring_.begin(seq_ + 1);
            int warn = 0;
            std::string err;
            const uint8_t *jpg = ride_->data(idx_);
            const size_t jlen = ride_->len(idx_);
            if (dec_.decode(jpg, jlen, W_, H_, dst, dst + size_t(W_) * H_, &err, &warn)) {
                ++seq_, ++total_;
                SlotHeader *s = ring_.slot_for(seq_);
                s->ts_mono_ns = t_frame;
                s->ts_real_ns = t_frame + rt_off;
                s->exposure_us = 0;
                s->analogue_gain = s->colour_gain_r = s->colour_gain_b = 0;
                ring_.commit(seq_);
                const double dec_ms = ms_since(t1);
                warnings_total_ += uint64_t(warn);
                if (!fps_t0_) fps_t0_ = t_frame, fps_seq0 = seq_;
                if (t_frame - fps_t0_ >= 1000000000) {
                    fps = double(seq_ - fps_seq0) * 1e9 / double(t_frame - fps_t0_);
                    fps_t0_ = t_frame, fps_seq0 = seq_;
                }
                ++st_n, st_dec += dec_ms, st_dec_max = std::max(st_dec_max, dec_ms);
                server_.publish(message(t_frame, t_frame + rt_off, fps, dec_ms, jlen));
                if (!opt_.no_preview && cfg_.preview_every > 0 && seq_ % uint64_t(cfg_.preview_every) == 0)
                    write_preview(rd_ + "/preview.jpg", jpg, jlen);
                if (opt_.frames && total_ >= opt_.frames) g_run = 0;
            } else {
                ++decode_fail_;
                if (opt_.verbose) fprintf(stderr, "camerad-replay: %s frame %zu: decode failed: %s\n", cur_slug_.c_str(), idx_, err.c_str());
            }
            ++idx_;
            deadline_ += period_;
        }
    }

    std::string message(int64_t t_mono, int64_t t_real, double fps, double dec_ms, size_t jlen) {
        const RideEntry &e = list_[cur_];
        char line[640];
        snprintf(line, sizeof(line),
                 "{\"type\":\"camera\",\"seq\":%" PRIu64 ",\"ts_mono_ns\":%" PRId64 ",\"ts_real_ns\":%" PRId64
                 ",\"exposure_us\":0,\"analogue_gain\":0,\"colour_gains\":[0,0],\"fps\":%.2f,\"dropped\":0,"
                 "\"sensor_drops\":%" PRIu64 ",\"conv_ms\":%.2f,\"source\":\"replay\",\"file_frame\":%zu,"
                 "\"file_frames\":%zu,\"file_t_s\":%.2f,\"loops\":%" PRIu64 ",\"jpeg_bytes\":%zu,"
                 "\"decode_fail\":%" PRIu64 ",\"jpeg_warnings\":%" PRIu64 ",\"playlist\":%s,\"ride_index\":%zu,"
                 "\"ride_count\":%zu,\"ride_t_s\":%.2f,\"ride_duration_s\":%.2f,",
                 seq_, t_mono, t_real, fps, skipped_ - ride_skipped0_, dec_ms, idx_, n_, double(idx_) / cfg_.fps,
                 loops_, jlen, decode_fail_, warnings_total_, playlist_ ? "true" : "false", playlist_ ? pos_ - 1 : 0,
                 playlist_ ? order_.size() : 1, double(idx_) / cfg_.fps, double(n_) / cfg_.fps);
        std::string m = line;
        m += "\"ride\":\"" + json_escape(e.slug) + "\",\"author\":\"" + json_escape(e.author) + "\",\"licence\":\"" +
             json_escape(e.licence) + "\"}";
        return m;
    }

    const Options &opt_;
    const Config base_;
    bool playlist_ = false;
    int gap_ms_ = 1000;
    std::string calib_out_, rd_, cur_slug_;
    std::vector<RideEntry> list_;
    std::vector<size_t> order_;
    size_t pos_ = 0, cur_ = 0;
    std::mt19937 rng_;
    JsonLineServer server_;
    RingWriter ring_;
    Decoder dec_;
    std::unique_ptr<Ride> ride_;
    Config cfg_;
    int W_ = 0, H_ = 0;
    size_t n_ = 0, idx_ = 0, end_idx_ = 0;
    int64_t period_ = 50000000, deadline_ = 0, fps_t0_ = 0;
    uint64_t seq_ = 0, total_ = 0, skipped_ = 0, ride_skipped0_ = 0, decode_fail_ = 0, warnings_total_ = 0,
             loops_ = 0, rides_started_ = 0;
};

}  // namespace

int main(int argc, char **argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--file" && i + 1 < argc) opt.file = argv[++i];
        else if (a == "--playlist" && i + 1 < argc) opt.playlist = argv[++i];
        else if (a == "--fps" && i + 1 < argc) opt.fps = atof(argv[++i]);
        else if (a == "--start" && i + 1 < argc) opt.start_s = atof(argv[++i]);
        else if (a == "--gap-ms" && i + 1 < argc) opt.gap_ms = std::max(0, atoi(argv[++i]));
        else if (a == "--max-seconds-per-ride" && i + 1 < argc) opt.max_s_per_ride = atof(argv[++i]);
        else if (a == "--calib-out" && i + 1 < argc) opt.calib_out = argv[++i];
        else if (a == "--no-calib") opt.no_calib = true;
        else if (a == "--shuffle") opt.shuffle = true;
        else if (a == "--once") opt.once = true;
        else if (a == "--verbose" || a == "-v") opt.verbose = true;
        else if (a == "--no-preview") opt.no_preview = true;
        else if (a == "--frames" && i + 1 < argc) opt.frames = strtoull(argv[++i], nullptr, 10);
        else {
            usage();
            return a == "--help" || a == "-h" ? 0 : 2;
        }
    }
    if (opt.file.empty() == opt.playlist.empty()) {
        usage();
        return 2;
    }
    Config cfg;
    if (opt.file.size()) {
        const std::string p = config_dir() + "/camera-replay.json";
        if (!load_config(cfg, p, opt.verbose)) fprintf(stderr, "camerad-replay: no config at %s, defaults\n", p.c_str());
    }
    struct sigaction sa{};
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN);
    return Player(opt, cfg).run();
}

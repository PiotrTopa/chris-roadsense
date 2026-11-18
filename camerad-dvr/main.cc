// camerad-dvr: Huiying/Buildwin "USB DVR" head-unit camera (USB 1902:3241) -> NV12
// frame ring + camerad.sock JSON + preview.jpg. A drop-in alternative to camerad
// (same ring contract, same socket name), see docs/ARCHITECTURE.md
// ("Alternative camera sources") and docs/HUIYING_DVR_PROTOCOL.md.
//
// The camera is a USB mass-storage disk whose firmware serves the live MJPEG
// stream from fixed sectors inside the file VSFILE of its FAT volume: an info
// sector (frame size, frame sequence, check word) and the frame itself
// (1280x720 baseline JPEG, 4:2:0, ~20 fps). camerad-dvr reads through
// O_RDONLY|O_DIRECT|O_EXCL and writes exactly one sector per connection: the
// vendor app's time sync (see Device::set_time), which stops the camera from
// leaving the USB bus ~28 s after power-up. --no-set-time makes it read-only.
//
// Single thread: poll the info sector every poll_us (O_DIRECT, uncached), on a
// new sequence take CLOCK_MONOTONIC as the frame timestamp, read the frame in
// one O_DIRECT pread, verify the check word, decode with libjpeg(-turbo) raw
// (YCbCr planes, no colour conversion, no upsampling) straight into the ring
// slot (Y) and interleave CbCr (NV12), commit, publish, and every Nth frame
// write the camera's own JPEG as preview.jpg (no re-encode).
#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <csetjmp>
#include <climits>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <time.h>
#include <unistd.h>

#include <jpeglib.h>

#include "governor.h"
#include "jsonsock.h"
#include "ring.h"

using namespace roadsense;

namespace {

constexpr int kW = 1280, kH = 720;
constexpr uint16_t kVid = 0x1902, kPid = 0x3241;
constexpr uint32_t kSector = 512;
constexpr uint32_t kMaxFrame = 0x4B000;   // getDataThread's limit
constexpr uint32_t kAudioLen = 0x2000;
constexpr uint32_t kKickExtra = 0x1400;   // stall "kick" read length beyond the frame

struct Layout { uint32_t info, video, cmd_w; };
constexpr Layout kLayoutA{0x380, 0x391, 0x340};  // MBR partition 1 at LBA 8
constexpr Layout kLayoutB{0x300, 0x311, 0x2C0};  // anything else
constexpr uint8_t kOpSetTime = 11;        // CAM_SET_TIME, the vendor app's first command after init

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
uint16_t rd16(const uint8_t *p) { return uint16_t(p[0] | p[1] << 8); }
uint32_t rd32(const uint8_t *p) { return uint32_t(p[0] | p[1] << 8 | p[2] << 16 | uint32_t(p[3]) << 24); }

// ------------------------------------------------------------------ config ---

struct Config {
    // Wide dashcam lens, assumed ~110 deg HFOV: focal = 640 / tan(55 deg) = 448 px.
    // NOT calibrated: set the measured value in /etc/roadsense/camera-dvr.json.
    float focal_px = 448.0f;
    float cx = kW / 2.0f, cy = kH / 2.0f;
    int preview_every = 2;  // camera JPEG copied to preview.jpg every Nth frame (0 = off)
    int poll_us = 2000;     // info-sector poll interval
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
    const std::string path = std::string(d && *d ? d : "/etc/roadsense") + "/camera-dvr.json";
    const std::string s = read_file(path);
    if (s.empty()) {
        fprintf(stderr, "camerad-dvr: no config at %s, defaults (focal_px %.0f is a guess: calibrate)\n", path.c_str(),
                c.focal_px);
        return;
    }
    double v;
    if (json_number(s, "focal_px", &v)) c.focal_px = float(v);
    if (json_number(s, "cx", &v)) c.cx = float(v);
    if (json_number(s, "cy", &v)) c.cy = float(v);
    if (json_number(s, "preview_every", &v)) c.preview_every = int(v);
    if (json_number(s, "poll_us", &v)) c.poll_us = std::max(200, int(v));
    if (verbose) fprintf(stderr, "camerad-dvr: config %s loaded\n", path.c_str());
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

// --------------------------------------------------------- device lookup ---

// sysfs path of a block device number, or "".
std::string sysfs_block(dev_t rdev) {
    char buf[64], real[PATH_MAX];
    snprintf(buf, sizeof(buf), "/sys/dev/block/%u:%u", major(rdev), minor(rdev));
    return realpath(buf, real) ? real : "";
}

// Walks up from the block device to its USB device node; true if it is 1902:3241.
bool is_dvr_usb(const std::string &sys, std::string *why) {
    for (std::string p = sys; p.size() > 1; p = p.substr(0, p.rfind('/'))) {
        std::string v = read_file(p + "/idVendor"), d = read_file(p + "/idProduct");
        if (v.empty() || d.empty()) continue;
        unsigned vi = unsigned(strtoul(v.c_str(), nullptr, 16)), di = unsigned(strtoul(d.c_str(), nullptr, 16));
        if (vi == kVid && di == kPid) return true;
        char b[160];
        snprintf(b, sizeof(b), "belongs to USB %04x:%04x, not %04x:%04x", vi, di, kVid, kPid);
        *why = b;
        return false;
    }
    *why = "is not backed by a USB device";
    return false;
}

// First whole-disk block device whose USB parent is 1902:3241, or "".
std::string find_auto() {
    DIR *d = opendir("/sys/block");
    if (!d) return "";
    std::string found;
    while (dirent *e = readdir(d)) {
        if (e->d_name[0] == '.') continue;
        char real[PATH_MAX];
        std::string link = std::string("/sys/block/") + e->d_name, why;
        if (realpath(link.c_str(), real) && is_dvr_usb(real, &why)) {
            found = std::string("/dev/") + e->d_name;
            break;
        }
    }
    closedir(d);
    return found;
}

// ---------------------------------------------------------------- device ---

class Device {
public:
    ~Device() { close(); free(buf_); }

    // Opens and validates (USB id, whole disk, 512-byte sectors, MBR, FAT, VSFILE
    // covering the info/video sectors). On failure returns false with *err set.
    bool open(const std::string &path, std::string *err) {
        close();
        struct stat st;
        if (stat(path.c_str(), &st) != 0) return fail(err, path + ": " + strerror(errno));
        if (!S_ISBLK(st.st_mode)) return fail(err, path + " is not a block device");
        const std::string sys = sysfs_block(st.st_rdev);
        std::string why;
        if (sys.empty() || !is_dvr_usb(sys, &why)) return fail(err, path + " " + (sys.empty() ? "has no sysfs node" : why) + "; refusing");
        if (!read_file(sys + "/partition").empty()) return fail(err, path + " is a partition, give the whole disk; refusing");
        std::string lbs = read_file(sys + "/queue/logical_block_size");
        if (!lbs.empty() && atoi(lbs.c_str()) != int(kSector)) return fail(err, "logical block size is not 512");
        // Read-only, uncached; O_EXCL on a block device = no mount and no other O_EXCL opener.
        fd_ = ::open(path.c_str(), O_RDONLY | O_DIRECT | O_EXCL | O_CLOEXEC);
        if (fd_ < 0) return fail(err, "open " + path + ": " + strerror(errno));
        if (!buf_ && posix_memalign(reinterpret_cast<void **>(&buf_), 4096, kBufSize) != 0) {
            buf_ = nullptr;
            return fail(err, "posix_memalign");
        }
        path_ = path;
        if (!validate(err)) {
            close();
            return false;
        }
        return true;
    }

    void close() {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }
    bool is_open() const { return fd_ >= 0; }
    const std::string &path() const { return path_; }
    const std::string &describe() const { return desc_; }

    // Reads n bytes (rounded up to whole sectors) at lba into the aligned buffer;
    // only inside VSFILE once validated. Returns nullptr on I/O error.
    const uint8_t *read(uint32_t lba, uint32_t n) {
        n = (n + kSector - 1) / kSector * kSector;
        if (n > kBufSize) return nullptr;
        if (vs_end_ && (lba < vs_begin_ || uint64_t(lba) * kSector + n > uint64_t(vs_end_) * kSector)) return nullptr;
        ssize_t got = pread(fd_, buf_, n, off_t(lba) * kSector);
        if (got != ssize_t(n)) {
            io_errno_ = got < 0 ? errno : EIO;
            return nullptr;
        }
        return buf_;
    }
    int io_errno() const { return io_errno_; }
    Layout layout() const { return lay_; }
    bool cmd_ok() const { return cmd_ok_; }

    // The one write this daemon does: the vendor app's time sync (opcode 11), one 512-byte
    // sector to the command sector of the HWC file. Without it the camera leaves the USB bus
    // once, 23-28 s after power-up, and is back 4-5 s later (2026-11-17: 4 of 4 power-ups
    // without the command dropped at 28 s, 6 of 6 with it stayed for the 75-110 s watched).
    // Only when validate() found HWC in the FAT and the command sector inside it.
    // board_v1: info[0x34] == 0xE9 selects the 16-bit header.
    bool set_time(bool board_v1, std::string *err) {
        if (!cmd_ok_) return fail(err, "command sector not verified (HWC file); not writing");
        uint8_t *pkt = nullptr;
        if (posix_memalign(reinterpret_cast<void **>(&pkt), 4096, kSector) != 0) return fail(err, "posix_memalign");
        memset(pkt, 0, kSector);
        time_t now = time(nullptr);
        struct tm tm;
        localtime_r(&now, &tm);
        size_t o = 0;
        if (board_v1) {
            pkt[o++] = kOpSetTime, pkt[o++] = 0, pkt[o++] = 7, pkt[o++] = 0;
        } else {
            pkt[o++] = kOpSetTime, pkt[o++] = 7;
        }
        const uint16_t year = uint16_t(tm.tm_year + 1900);
        pkt[o++] = uint8_t(year & 0xFF), pkt[o++] = uint8_t(year >> 8);
        pkt[o++] = uint8_t(tm.tm_mon + 1), pkt[o++] = uint8_t(tm.tm_mday);
        pkt[o++] = uint8_t(tm.tm_hour), pkt[o++] = uint8_t(tm.tm_min), pkt[o++] = uint8_t(tm.tm_sec);
        // Second descriptor: our read fd holds O_EXCL, a plain open next to it is allowed.
        int wfd = ::open(path_.c_str(), O_WRONLY | O_DIRECT | O_SYNC | O_CLOEXEC);
        bool ok = wfd >= 0 && pwrite(wfd, pkt, kSector, off_t(lay_.cmd_w) * kSector) == ssize_t(kSector);
        const int e = errno;
        if (wfd >= 0) ::close(wfd);
        free(pkt);
        return ok ? true : fail(err, std::string("time sync write: ") + strerror(e));
    }

private:
    static constexpr uint32_t kBufSize = 1u << 20;
    static bool fail(std::string *err, const std::string &m) { return *err = m, false; }

    // ---- minimal read-only FAT12/16/32 walker (port of tools/dvr_probe.py) ----
    struct Fat {
        uint32_t part, spc, fat_lba, root_lba, data_lba, root_secs, root_clus;
        int kind;
    };
    bool sec(uint32_t lba, uint32_t n, std::vector<uint8_t> *out) {
        const uint8_t *p = read(lba, n);
        if (!p) return false;
        out->assign(p, p + n);
        return true;
    }
    int64_t next_clus(const Fat &f, uint32_t c) {
        uint32_t off = f.kind == 16 ? c * 2 : f.kind == 32 ? c * 4 : c + c / 2;
        std::vector<uint8_t> s;
        if (!sec(f.fat_lba + off / kSector, 2 * kSector, &s)) return -1;
        const uint8_t *p = s.data() + off % kSector;
        if (f.kind == 16) { uint32_t v = rd16(p); return v >= 0xFFF8 ? -1 : v; }
        if (f.kind == 32) { uint32_t v = rd32(p) & 0x0FFFFFFF; return v >= 0x0FFFFFF8 ? -1 : v; }
        uint32_t v = rd16(p);
        v = c & 1 ? v >> 4 : v & 0xFFF;
        return v >= 0xFF8 ? -1 : v;
    }
    std::vector<uint32_t> chain(const Fat &f, int64_t c, size_t limit) {
        std::vector<uint32_t> out;
        while (c >= 2 && out.size() < limit) {
            out.push_back(uint32_t(c));
            c = next_clus(f, uint32_t(c));
        }
        return out;
    }

    // All 32-byte entries of a directory (root of FAT12/16 by LBA, otherwise by cluster chain).
    bool read_dir(const Fat &f, int64_t clus, std::vector<uint8_t> *dir) {
        dir->clear();
        if (clus < 2) return f.kind != 32 && sec(f.root_lba, f.root_secs * kSector, dir);
        for (uint32_t c : chain(f, clus, 64)) {
            std::vector<uint8_t> s;
            if (!sec(f.data_lba + (c - 2) * f.spc, f.spc * kSector, &s)) return false;
            dir->insert(dir->end(), s.begin(), s.end());
        }
        return !dir->empty();
    }
    // Looks `name` up (long name or 8.3, ASCII, case-insensitive). Returns the first cluster, -1 if absent.
    int64_t lookup(const Fat &f, const std::vector<uint8_t> &dir, const std::string &name, bool *is_dir) {
        auto lower = [](std::string v) { for (char &c : v) c = char(tolower(uint8_t(c))); return v; };
        const std::string want = lower(name);
        std::string lfn;
        for (size_t i = 0; i + 32 <= dir.size() && dir[i]; i += 32) {
            const uint8_t *e = &dir[i];
            if (e[0] == 0xE5) { lfn.clear(); continue; }
            if (e[11] == 0x0F) {  // long-name part, stored last part first
                static const int pos[13] = {1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30};
                std::string part;
                for (int k : pos) {
                    const uint16_t ch = rd16(e + k);
                    if (ch == 0 || ch == 0xFFFF) break;
                    part += ch < 0x80 ? char(ch) : '?';
                }
                lfn = part + lfn;
                continue;
            }
            if (e[11] & 0x08) { lfn.clear(); continue; }  // volume label
            std::string base(reinterpret_cast<const char *>(e), 8), ext(reinterpret_cast<const char *>(e + 8), 3);
            while (!base.empty() && base.back() == ' ') base.pop_back();
            while (!ext.empty() && ext.back() == ' ') ext.pop_back();
            const std::string shortn = ext.empty() ? base : base + "." + ext;
            const bool hit = (!lfn.empty() && lower(lfn) == want) || lower(shortn) == want;
            lfn.clear();
            if (!hit) continue;
            *is_dir = e[11] & 0x10;
            return (f.kind == 32 ? uint32_t(rd16(e + 20)) << 16 : 0) | rd16(e + 26);
        }
        return -1;
    }
    // True when Android/data/com.fvision.camera/HWC exists and its clusters contain `lba`.
    bool hwc_covers(const Fat &f, uint32_t lba) {
        int64_t clus = f.kind == 32 ? int64_t(f.root_clus) : 0;
        std::vector<uint8_t> dir;
        bool is_dir = true;
        for (const char *comp : {"Android", "data", "com.fvision.camera", "HWC"}) {
            if (!is_dir || !read_dir(f, clus, &dir)) return false;
            clus = lookup(f, dir, comp, &is_dir);
            if (clus < 2) return false;
        }
        if (is_dir) return false;
        for (uint32_t c : chain(f, clus, 4096)) {
            const uint32_t l0 = f.data_lba + (c - 2) * f.spc;
            if (lba >= l0 && lba < l0 + f.spc) return true;
        }
        return false;
    }

    bool validate(std::string *err) {
        vs_begin_ = vs_end_ = 0;
        cmd_ok_ = false;
        std::vector<uint8_t> b;
        if (!sec(0, kSector, &b)) return fail(err, "read LBA 0 failed");
        if (b[510] != 0x55 || b[511] != 0xAA) return fail(err, "LBA 0 has no MBR signature");
        Fat f{};
        f.part = rd32(&b[0x1C6]);
        lay_ = f.part == 8 ? kLayoutA : kLayoutB;
        if (!sec(f.part, kSector, &b) || b[510] != 0x55 || b[511] != 0xAA)
            return fail(err, "no FAT boot sector at LBA " + std::to_string(f.part));
        const uint32_t bps = rd16(&b[11]), res = rd16(&b[14]), nf = b[16], rootent = rd16(&b[17]);
        const uint32_t ts = rd16(&b[19]) ? rd16(&b[19]) : rd32(&b[32]);
        const uint32_t fsz = rd16(&b[22]) ? rd16(&b[22]) : rd32(&b[36]);
        f.spc = b[13];
        if (bps != kSector || f.spc == 0) return fail(err, "unexpected FAT geometry");
        f.fat_lba = f.part + res;
        f.root_secs = (rootent * 32 + kSector - 1) / kSector;
        f.root_lba = f.fat_lba + nf * fsz;
        f.data_lba = f.root_lba + f.root_secs;
        const uint32_t clusters = (ts - (f.data_lba - f.part)) / f.spc;
        f.kind = clusters < 4085 ? 12 : clusters < 65525 ? 16 : 32;
        f.root_clus = f.kind == 32 ? rd32(&b[44]) : 0;

        // Root directory: look for the 8.3 entry "VSFILE" (no LFN needed).
        std::vector<uint8_t> dir;
        if (f.kind == 32) {
            for (uint32_t c : chain(f, f.root_clus, 64)) {
                std::vector<uint8_t> s;
                if (!sec(f.data_lba + (c - 2) * f.spc, f.spc * kSector, &s)) return fail(err, "root dir read failed");
                dir.insert(dir.end(), s.begin(), s.end());
            }
        } else if (!sec(f.root_lba, f.root_secs * kSector, &dir)) {
            return fail(err, "root dir read failed");
        }
        int64_t vs_clus = -1;
        uint32_t vs_size = 0;
        for (size_t i = 0; i + 32 <= dir.size() && dir[i]; i += 32) {
            const uint8_t *e = &dir[i];
            if (e[0] == 0xE5 || e[11] == 0x0F || (e[11] & 0x18)) continue;
            if (memcmp(e, "VSFILE     ", 11) == 0) {
                vs_clus = (f.kind == 32 ? uint32_t(rd16(e + 20)) << 16 : 0) | rd16(e + 26);
                vs_size = rd32(e + 28);
                break;
            }
        }
        if (vs_clus < 2) return fail(err, "no VSFILE in the FAT root directory");
        // VSFILE's clusters must form one run covering info .. video + kMaxFrame + kick.
        const uint32_t need_end = lay_.video + (kMaxFrame + kKickExtra) / kSector;
        const uint32_t r0 = f.data_lba + uint32_t(vs_clus - 2) * f.spc;
        uint32_t rn = 0;
        for (int64_t c = vs_clus; c >= 2 && f.data_lba + uint32_t(c - 2) * f.spc == r0 + rn && rn < 4096;
             c = next_clus(f, uint32_t(c)))
            rn += f.spc;
        char d[256];
        snprintf(d, sizeof(d), "layout %c (partition at LBA %u), FAT%d, VSFILE %u bytes, first run LBA 0x%x..0x%x",
                 f.part == 8 ? 'A' : 'B', f.part, f.kind, vs_size, r0, r0 + rn);
        desc_ = d;
        if (lay_.info < r0 || lay_.info + 1 + 16 + 1 > r0 + rn)
            return fail(err, std::string("VSFILE does not cover the info sector: ") + d);
        cmd_ok_ = hwc_covers(f, lay_.cmd_w);  // before the read window closes on VSFILE
        // As in the app and dvr_probe.py, only info + audio + the first video sector must lie in
        // VSFILE: on the tested unit VSFILE is 192 KiB, and frames run past its end (the firmware
        // serves those LBAs live). Reads stay within [VSFILE start, video + max frame + kick].
        vs_begin_ = r0;
        vs_end_ = std::max(r0 + rn, need_end);
        return true;
    }

    int fd_ = -1;
    uint8_t *buf_ = nullptr;
    std::string path_, desc_;
    Layout lay_ = kLayoutB;
    uint32_t vs_begin_ = 0, vs_end_ = 0;
    bool cmd_ok_ = false;
    int io_errno_ = 0;
};

// ------------------------------------------------------------ info sector ---

struct Info {
    uint16_t flags = 0;
    uint32_t vsize = 0, vseq = 0, alen = 0, check = 0;
    bool valid = false, playback = false;
};

bool is_an_num(uint16_t v) {  // isAnNum(): integrity of the u16 at 0x30
    const uint8_t lo = v & 0xFF, hi = v >> 8;
    const int cnt = __builtin_popcount(hi & 0x1F);
    return uint8_t(((hi >> 5) ^ 0x0B) | ((cnt ^ 0x0C) << 3)) == lo;
}

Info parse_info(const uint8_t *b) {
    Info h;
    h.flags = rd16(b);
    h.vsize = rd32(b + 0x0C);
    h.vseq = rd32(b + 0x10);
    h.alen = rd32(b + 0x14);
    h.check = rd32(b + 0x2C);
    h.playback = h.flags & 0x8000;
    h.valid = (h.alen == 0 || h.alen == kAudioLen) && h.vsize > 0 && h.vsize <= kMaxFrame && h.flags != 0 &&
              is_an_num(rd16(b + 0x30));
    return h;
}

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
    if (level < 0) ++reinterpret_cast<JpegErr *>(c->err)->warnings;  // corrupt-data warnings
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

    // Decodes a 1280x720 4:2:0 YCbCr JPEG to NV12 (y: kW x kH, uv: kW x kH/2).
    // Returns false with *err on failure. *warnings = libjpeg corrupt-data warnings.
    bool decode(const uint8_t *data, size_t size, uint8_t *y, uint8_t *uv, std::string *err, int *warnings) {
        err_.warnings = 0;
        if (setjmp(err_.jb)) {
            jpeg_abort_decompress(&cinfo_);
            *err = err_.msg;
            return false;
        }
        jpeg_mem_src(&cinfo_, const_cast<unsigned char *>(data), (unsigned long)size);
        jpeg_read_header(&cinfo_, TRUE);
        const jpeg_component_info *c = cinfo_.comp_info;
        if (cinfo_.image_width != unsigned(kW) || cinfo_.image_height != unsigned(kH) || cinfo_.num_components != 3 ||
            cinfo_.jpeg_color_space != JCS_YCbCr || c[0].h_samp_factor != 2 || c[0].v_samp_factor != 2 ||
            c[1].h_samp_factor != 1 || c[1].v_samp_factor != 1 || c[2].h_samp_factor != 1 || c[2].v_samp_factor != 1) {
            jpeg_abort_decompress(&cinfo_);
            char b[128];
            snprintf(b, sizeof(b), "unsupported JPEG %ux%u, %d comps (need 1280x720 YCbCr 4:2:0)", cinfo_.image_width,
                     cinfo_.image_height, cinfo_.num_components);
            *err = b;
            return false;
        }
        cinfo_.raw_data_out = TRUE;
        cinfo_.out_color_space = JCS_YCbCr;
        cinfo_.do_fancy_upsampling = FALSE;
        cinfo_.dct_method = JDCT_ISLOW;
        jpeg_start_decompress(&cinfo_);
        JSAMPROW yr[16], cbr[8], crr[8];
        JSAMPARRAY planes[3] = {yr, cbr, crr};
        while (cinfo_.output_scanline < unsigned(kH)) {
            const unsigned r0 = cinfo_.output_scanline;  // multiple of 16
            for (int i = 0; i < 16; ++i) yr[i] = y + size_t(r0 + i) * kW;
            for (int i = 0; i < 8; ++i) {
                cbr[i] = cb_ + size_t(i) * (kW / 2);
                crr[i] = cr_ + size_t(i) * (kW / 2);
            }
            if (jpeg_read_raw_data(&cinfo_, planes, 16) != 16) {
                jpeg_abort_decompress(&cinfo_);
                *err = "short raw read";
                return false;
            }
            for (int i = 0; i < 8; ++i) {  // CbCr rows r0/2 .. r0/2+7, interleaved
                uint8_t *d = uv + size_t(r0 / 2 + i) * kW;
                const uint8_t *b = cbr[i], *r = crr[i];
                for (int x = 0; x < kW / 2; ++x) {
                    d[2 * x] = b[x];
                    d[2 * x + 1] = r[x];
                }
            }
        }
        // The stream may carry trailing bytes after EOI; no need to parse them.
        jpeg_abort_decompress(&cinfo_);
        *warnings = err_.warnings;
        return true;
    }

private:
    jpeg_decompress_struct cinfo_{};
    JpegErr err_{};
    alignas(64) uint8_t cb_[8 * kW / 2], cr_[8 * kW / 2];
};

// Length of the JPEG up to and including the first EOI after SOS (entropy data
// never contains FF D9), or 0 if there is none.
size_t jpeg_length(const uint8_t *d, size_t n) {
    for (size_t i = 2; i + 3 < n;) {  // walk marker segments up to SOS
        if (d[i] != 0xFF) return 0;
        const uint8_t m = d[i + 1];
        const size_t len = size_t(d[i + 2]) << 8 | d[i + 3];
        if (m == 0xDA) {
            for (size_t j = i + 2 + len; j + 1 < n; ++j)
                if (d[j] == 0xFF && d[j + 1] == 0xD9) return j + 2;
            return 0;
        }
        i += 2 + len;
    }
    return 0;
}


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

bool write_preview(const std::string &path, const uint8_t *jpg, size_t n) {
    static const uint8_t eoi[2] = {0xFF, 0xD9};
    size_t len = jpeg_length(jpg, n);
    const std::string tmp = path + ".tmp";
    int fd = open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) return false;
    bool ok = write(fd, jpg, len ? len : n) == ssize_t(len ? len : n);
    if (ok && !len) ok = write(fd, eoi, 2) == 2;  // truncated stream: terminate it
    close(fd);
    if (ok) ok = rename(tmp.c_str(), path.c_str()) == 0;
    if (!ok) unlink(tmp.c_str());
    return ok;
}

// ----------------------------------------------------------------- daemon ---

struct Options {
    std::string dev;
    bool autodetect = false, verbose = false, no_preview = false, set_time = true;
    uint64_t frames = 0;
};

void usage() {
    fprintf(stderr,
            "usage: camerad-dvr (--auto | --dev /dev/sdX) [--verbose] [--frames N] [--no-preview] [--no-set-time]\n"
            "  --auto        first whole-disk block device whose USB parent is 1902:3241\n"
            "  --dev PATH    that block device (refused unless it is 1902:3241)\n"
            "  --no-set-time do not send the time sync (then the camera leaves the bus once ~28 s after power-up)\n"
            "Writes one sector per connection: the vendor app's time sync, to the command sector of the HWC\n"
            "file, and only when the FAT confirms that sector belongs to HWC. Everything else is read-only.\n"
            "Needs access to the raw disk (root or group disk).\n"
            "env: ROADSENSE_RUNDIR (default /run/roadsense), ROADSENSE_CONFIG_DIR (default /etc/roadsense,\n"
            "     file camera-dvr.json), ROADSENSE_RING (default /dev/shm/roadsense-frames)\n");
}

void sleep_us(int us) {
    timespec ts{us / 1000000, long(us % 1000000) * 1000};
    nanosleep(&ts, nullptr);
}

int run(const Options &opt, const Config &cfg) {
    Device dev;
    Decoder dec;
    RingWriter ring;
    JsonLineServer server;
    const std::string rd = rundir(), preview_path = rd + "/preview.jpg";
    PreviewGate preview_gate;
    bool ring_ready = false;

    uint64_t seq = 0, gaps = 0, rejected = 0, decode_fail = 0, warnings_total = 0, reconnects = 0, polls = 0;
    uint32_t last_vseq = 0;
    bool have_vseq = false, time_sent = false;
    int retry_same = 0;
    int64_t last_new_ns = 0, rt_off = 0, next_rt = 0, next_stat = 0;
    std::deque<int64_t> ts_hist;
    double fps = 0;
    // per-second stats
    int st_n = 0;
    double st_read = 0, st_dec = 0, st_dec_max = 0;
    uint64_t st_polls = 0;
    std::string last_err;

    while (g_run) {
        if (!dev.is_open()) {
            std::string path = opt.autodetect ? find_auto() : opt.dev, err;
            if (path.empty()) err = "no block device with USB parent 1902:3241";
            else if (dev.open(path, &err)) {
                fprintf(stderr, "camerad-dvr: %s: %s\n", path.c_str(), dev.describe().c_str());
                have_vseq = false;
                time_sent = false;
                last_err.clear();
                ++reconnects;
            }
            if (!dev.is_open()) {
                if (!opt.autodetect && err.find("refusing") != std::string::npos) {  // wrong device: fatal
                    fprintf(stderr, "camerad-dvr: %s\n", err.c_str());
                    return 1;
                }
                if (err != last_err) fprintf(stderr, "camerad-dvr: %s; retrying every 1 s\n", err.c_str());
                last_err = err;
                for (int i = 0; i < 100 && g_run; ++i) {
                    if (ring_ready) server.service();
                    sleep_us(10000);
                }
                continue;
            }
            if (!ring_ready) {  // ring and socket only once a camera is ours (O_EXCL)
                if (!mkdir_p(rd)) {
                    fprintf(stderr, "camerad-dvr: cannot create %s: %s\n", rd.c_str(), strerror(errno));
                    return 1;
                }
                if (!ring.create(ring_path(), kW, kH, cfg.focal_px, cfg.cx, cfg.cy, &err) ||
                    !server.listen(rd + "/camerad.sock", &err)) {
                    fprintf(stderr, "camerad-dvr: %s\n", err.c_str());
                    return 1;
                }
                ring_ready = true;
            }
        }

        const int64_t now = clock_ns(CLOCK_MONOTONIC);
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
                        "camerad-dvr: seq %" PRIu64 " fps %.1f read %.2f ms decode+nv12 %.2f ms avg %.2f max, "
                        "polls %" PRIu64 "/s, gaps %" PRIu64 " rejected %" PRIu64 " decode_fail %" PRIu64 "\n",
                        seq, fps, st_read / st_n, st_dec / st_n, st_dec_max, st_polls, gaps, rejected, decode_fail);
            st_n = 0, st_read = st_dec = st_dec_max = 0, st_polls = 0;
            next_stat = now + 1000000000;
        }
        server.service();

        // ---- poll the info sector
        const uint8_t *b = dev.read(dev.layout().info, kSector);
        ++polls, ++st_polls;
        if (!b) {
            fprintf(stderr, "camerad-dvr: info read failed (%s): device gone? reopening\n", strerror(dev.io_errno()));
            dev.close();
            continue;
        }
        const int64_t t_seen = clock_ns(CLOCK_MONOTONIC);
        const Info h = parse_info(b);
        if (!h.valid) {
            sleep_us(10000);
            continue;
        }
        if (opt.set_time && !time_sent) {  // once per connection, on the first valid info sector
            time_sent = true;
            std::string terr;
            const bool board_v1 = b[0x34] == 0xE9;
            if (dev.set_time(board_v1, &terr))
                fprintf(stderr, "camerad-dvr: time sync sent (board %s)\n", board_v1 ? "v1" : "v0");
            else
                fprintf(stderr, "camerad-dvr: time sync not sent: %s\n", terr.c_str());
            continue;  // b points into the device buffer; poll again
        }
        if (have_vseq && h.vseq == last_vseq) {
            if (!h.playback && t_seen - last_new_ns > 400000000) {  // stalled: the app's dummy "kick" read
                dev.read(dev.layout().video, (h.vsize & ~0x1FFu) + kKickExtra);
                last_new_ns = t_seen;
            }
            sleep_us(cfg.poll_us);
            continue;
        }
        // ---- new frame
        if (have_vseq && h.vseq > last_vseq + 1) gaps += h.vseq - last_vseq - 1;
        const int64_t t0 = clock_ns(CLOCK_MONOTONIC);
        const uint8_t *f = dev.read(dev.layout().video, (h.vsize & ~0x1FFu) + 0x200);
        if (!f) {
            fprintf(stderr, "camerad-dvr: frame read failed (%s): reopening\n", strerror(dev.io_errno()));
            dev.close();
            continue;
        }
        const double read_ms = ms_since(t0);
        if (!h.playback && rd32(f + ((h.vsize - 8) >> 2) * 4) != h.check) {
            // frame buffer still being rewritten: re-poll, give up on this seq after 3 tries
            if (++retry_same < 3) continue;
            ++rejected;
        } else {
            const int64_t t1 = clock_ns(CLOCK_MONOTONIC);
            uint8_t *dst = ring.begin(seq + 1);
            std::string err;
            int warn = 0;
            if (dec.decode(f, h.vsize, dst, dst + size_t(kW) * kH, &err, &warn)) {
                ++seq;
                SlotHeader *s = ring.slot_for(seq);
                s->ts_mono_ns = t_seen;
                s->ts_real_ns = t_seen + rt_off;
                s->exposure_us = 0;
                s->analogue_gain = s->colour_gain_r = s->colour_gain_b = 0;
                ring.commit(seq);
                const double dec_ms = ms_since(t1);
                warnings_total += uint64_t(warn);
                ts_hist.push_back(t_seen);
                while (ts_hist.size() > 2 && t_seen - ts_hist.front() > 1000000000) ts_hist.pop_front();
                if (ts_hist.size() >= 2)
                    fps = double(ts_hist.size() - 1) * 1e9 / double(ts_hist.back() - ts_hist.front());
                ++st_n, st_read += read_ms, st_dec += dec_ms, st_dec_max = std::max(st_dec_max, dec_ms);
                char line[640];
                snprintf(line, sizeof(line),
                         "{\"type\":\"camera\",\"seq\":%" PRIu64 ",\"ts_mono_ns\":%" PRId64 ",\"ts_real_ns\":%" PRId64
                         ",\"exposure_us\":0,\"analogue_gain\":0,\"colour_gains\":[0,0],\"fps\":%.2f,\"dropped\":0,"
                         "\"sensor_drops\":%" PRIu64 ",\"conv_ms\":%.2f,\"source\":\"dvr\",\"video_seq\":%u,"
                         "\"jpeg_bytes\":%u,\"read_ms\":%.2f,\"rejected\":%" PRIu64 ",\"decode_fail\":%" PRIu64
                         ",\"jpeg_warnings\":%" PRIu64 ",\"recording\":%s,\"reconnects\":%" PRIu64 "}",
                         seq, t_seen, t_seen + rt_off, fps, gaps, dec_ms, h.vseq, h.vsize, read_ms, rejected,
                         decode_fail, warnings_total, (h.flags & 0x4000) ? "true" : "false", reconnects - 1);
                server.publish(line);
                if (!opt.no_preview && preview_gate.pass(seq, cfg.preview_every, t_seen))
                    write_preview(preview_path, f, h.vsize);
                if (opt.frames && seq >= opt.frames) g_run = 0;
            } else {
                ++decode_fail;
                if (opt.verbose) fprintf(stderr, "camerad-dvr: video_seq %u: decode failed: %s\n", h.vseq, err.c_str());
            }
        }
        if (have_vseq && h.vseq < last_vseq)
            fprintf(stderr, "camerad-dvr: video_seq went back %u -> %u (camera restarted)\n", last_vseq, h.vseq);
        last_vseq = h.vseq;
        have_vseq = true;
        retry_same = 0;
        last_new_ns = t_seen;
    }
    if (ring_ready) {
        server.close();
        unlink((rd + "/camerad.sock").c_str());
    }
    fprintf(stderr, "camerad-dvr: stopped after %" PRIu64 " frames (%" PRIu64 " polls, gaps %" PRIu64
                    ", rejected %" PRIu64 ", decode_fail %" PRIu64 ")\n",
            seq, polls, gaps, rejected, decode_fail);
    return 0;
}

}  // namespace

int main(int argc, char **argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--auto") opt.autodetect = true;
        else if (a == "--dev" && i + 1 < argc) opt.dev = argv[++i];
        else if (a == "--verbose" || a == "-v") opt.verbose = true;
        else if (a == "--no-preview") opt.no_preview = true;
        else if (a == "--no-set-time") opt.set_time = false;
        else if (a == "--frames" && i + 1 < argc) opt.frames = strtoull(argv[++i], nullptr, 10);
        else {
            usage();
            return a == "--help" || a == "-h" ? 0 : 2;
        }
    }
    if (opt.autodetect == !opt.dev.empty()) {
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

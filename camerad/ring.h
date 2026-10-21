// pocopilot shared-memory frame ring (/dev/shm/pocopilot-frames).
//
// Header-only, shared by the writer (camerad) and every reader (modeld, signd,
// tools). The layout and protocol are specified in docs/ARCHITECTURE.md; this
// file is the reference implementation of that contract.
//
// Pixel format: NV12 (format = 1). Y plane (width x height, y_stride bytes per
// row) followed by an interleaved CbCr plane (width/2 pairs x height/2 rows,
// uv_stride bytes per row, Cb first). 4:2:0, chroma sited at the centre of
// each 2x2 luma block (box average).
//
// Colour encoding: BT.601 FULL RANGE (JFIF / JPEG convention):
//   Y  =  0.299 R + 0.587 G + 0.114 B                     (0..255)
//   Cb = -0.168736 R - 0.331264 G + 0.5 B      + 128      (0..255)
//   Cr =  0.5 R - 0.418688 G - 0.081312 B      + 128      (0..255)
// Inverse: R = Y + 1.402 (Cr-128); G = Y - 0.344136 (Cb-128) - 0.714136 (Cr-128);
//          B = Y + 1.772 (Cb-128).
// The values are sRGB-gamma encoded as delivered by the libcamera software ISP
// (not linear light). Full range was chosen because it is exactly what a
// baseline JPEG stores (preview needs no re-conversion) and because it keeps
// all 256 code values for the models.
//
// Protocol (seqlock-like, drop on overwrite, one writer, any number of readers):
//   writer: slot = seq % slot_count
//           slot.seq = seq (relaxed store); release fence
//           write the rest of the SlotHeader and the pixel data
//           header.write_seq = seq (release store)
//   reader: w = header.write_seq (acquire); if w == 0 -> nothing yet
//           slot = w % slot_count; if slot.seq (acquire) != w -> lapped, retry
//           copy/process the slot; acquire fence; if slot.seq != w -> the
//           writer started overwriting it mid-read: drop the frame.
// Because the writer stores the new slot seq *before* touching the pixels, any
// reader that observed even one overwritten byte also observes the new seq.
//
// File lifetime: camerad builds a fresh file under a temporary name and
// rename()s it over the ring path at start-up, so readers that still map an
// old ring never see it truncated under them (no SIGBUS). RingReader notices a
// replaced file (inode change) and remaps.
#pragma once

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace pocopilot {

constexpr const char *kRingPath = "/dev/shm/pocopilot-frames";
constexpr char kRingMagic[8] = {'P', 'P', 'F', 'R', 'A', 'M', 'E', '1'};
constexpr uint32_t kRingHeaderSize = 4096;
constexpr uint32_t kRingSlotCount = 6;
constexpr uint32_t kRingDataOffset = 4096;  // pixel data offset inside a slot
constexpr uint32_t kFormatNV12 = 1;

// Ring path: POCOPILOT_RING overrides the contract path (tests only).
inline std::string ring_path() {
    const char *p = getenv("POCOPILOT_RING");
    return (p && *p) ? std::string(p) : std::string(kRingPath);
}

struct RingHeader {             // offset 0, little-endian
    char magic[8];              //  0 "PPFRAME1"
    uint32_t header_size;       //  8 4096
    uint32_t slot_count;        // 12 6
    uint32_t slot_size;         // 16 bytes per slot incl. SlotHeader, 4096-aligned
    uint32_t width, height;     // 20, 24
    uint32_t y_stride;          // 28
    uint32_t uv_stride;         // 32
    uint32_t format;            // 36 1 = NV12
    float focal_px;             // 40
    float cx, cy;               // 44, 48
    uint32_t reserved0;         // 52 natural alignment padding, always 0
    uint64_t write_seq;         // 56 atomic (release store / acquire load)
    uint64_t camerad_pid;       // 64
};
static_assert(offsetof(RingHeader, focal_px) == 40, "layout");
static_assert(offsetof(RingHeader, write_seq) == 56, "layout");
static_assert(offsetof(RingHeader, camerad_pid) == 64, "layout");
static_assert(sizeof(RingHeader) == 72, "layout");

struct SlotHeader {             // at header_size + i*slot_size
    uint64_t seq;               //  0 atomic, frame number, starts at 1
    int64_t ts_mono_ns;         //  8
    int64_t ts_real_ns;         // 16
    uint32_t exposure_us;       // 24
    float analogue_gain;        // 28
    float colour_gain_r;        // 32
    float colour_gain_b;        // 36
    uint32_t data_offset;       // 40 from slot start (4096)
    uint32_t data_size;         // 44 y_stride*height + uv_stride*height/2
};
static_assert(sizeof(SlotHeader) == 48, "layout");

inline uint64_t ring_load_acquire(const uint64_t *p) { return __atomic_load_n(p, __ATOMIC_ACQUIRE); }
inline void ring_store_release(uint64_t *p, uint64_t v) { __atomic_store_n(p, v, __ATOMIC_RELEASE); }

inline uint32_t ring_align4k(uint64_t n) { return uint32_t((n + 4095) & ~uint64_t(4095)); }

inline uint32_t nv12_data_size(uint32_t y_stride, uint32_t uv_stride, uint32_t height) {
    return y_stride * height + uv_stride * (height / 2);
}

// ---------------------------------------------------------------- writer ---

class RingWriter {
public:
    RingWriter() = default;
    RingWriter(const RingWriter &) = delete;
    RingWriter &operator=(const RingWriter &) = delete;
    ~RingWriter() { close(); }

    // Creates (replaces) the ring file. Returns false and sets err on failure.
    bool create(const std::string &path, uint32_t width, uint32_t height, float focal_px, float cx,
                float cy, std::string *err = nullptr) {
        close();
        path_ = path;
        const uint32_t y_stride = width, uv_stride = width;
        const uint32_t data_size = nv12_data_size(y_stride, uv_stride, height);
        const uint32_t slot_size = ring_align4k(uint64_t(kRingDataOffset) + data_size);
        size_ = size_t(kRingHeaderSize) + size_t(kRingSlotCount) * slot_size;

        std::string tmp = path + ".tmp." + std::to_string(getpid());
        int fd = ::open(tmp.c_str(), O_RDWR | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
        if (fd < 0) return fail(err, "open " + tmp);
        if (ftruncate(fd, off_t(size_)) != 0) {
            ::close(fd);
            unlink(tmp.c_str());
            return fail(err, "ftruncate");
        }
        void *p = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        ::close(fd);
        if (p == MAP_FAILED) {
            unlink(tmp.c_str());
            return fail(err, "mmap");
        }
        base_ = static_cast<uint8_t *>(p);
        std::memset(base_, 0, kRingHeaderSize);
        RingHeader *h = header();
        std::memcpy(h->magic, kRingMagic, 8);
        h->header_size = kRingHeaderSize;
        h->slot_count = kRingSlotCount;
        h->slot_size = slot_size;
        h->width = width;
        h->height = height;
        h->y_stride = y_stride;
        h->uv_stride = uv_stride;
        h->format = kFormatNV12;
        h->focal_px = focal_px;
        h->cx = cx;
        h->cy = cy;
        h->camerad_pid = uint64_t(getpid());
        for (uint32_t i = 0; i < kRingSlotCount; ++i) {
            SlotHeader *s = slot(i);
            std::memset(s, 0, sizeof(*s));
            s->data_offset = kRingDataOffset;
            s->data_size = data_size;
        }
        ring_store_release(&h->write_seq, 0);
        if (rename(tmp.c_str(), path.c_str()) != 0) {
            unlink(tmp.c_str());
            close();
            return fail(err, "rename");
        }
        return true;
    }

    void close() {
        if (base_) munmap(base_, size_);
        base_ = nullptr;
    }

    RingHeader *header() const { return reinterpret_cast<RingHeader *>(base_); }
    SlotHeader *slot(uint32_t i) const {
        return reinterpret_cast<SlotHeader *>(base_ + kRingHeaderSize + size_t(i) * header()->slot_size);
    }

    // Step 1: claim the slot for `seq` (marks it as being overwritten) and
    // return its pixel buffer (Y plane; UV plane follows at y_stride*height).
    uint8_t *begin(uint64_t seq) {
        SlotHeader *s = slot(uint32_t(seq % kRingSlotCount));
        __atomic_store_n(&s->seq, seq, __ATOMIC_RELAXED);
        std::atomic_thread_fence(std::memory_order_release);
        return reinterpret_cast<uint8_t *>(s) + s->data_offset;
    }
    SlotHeader *slot_for(uint64_t seq) const { return slot(uint32_t(seq % kRingSlotCount)); }

    // Step 2: after pixels and slot metadata are written, publish.
    void commit(uint64_t seq) { ring_store_release(&header()->write_seq, seq); }

    const std::string &path() const { return path_; }

private:
    static bool fail(std::string *err, const std::string &what) {
        if (err) *err = what + ": " + strerror(errno);
        return false;
    }
    std::string path_;
    uint8_t *base_ = nullptr;
    size_t size_ = 0;
};

// ---------------------------------------------------------------- reader ---

// One frame as seen by a reader. y/uv point into the reader's private copy and
// stay valid until the next latest()/next() call on the same RingReader.
struct FrameView {
    uint64_t seq = 0;
    int64_t ts_mono_ns = 0, ts_real_ns = 0;
    uint32_t exposure_us = 0;
    float analogue_gain = 0, colour_gain_r = 0, colour_gain_b = 0;
    uint32_t width = 0, height = 0, y_stride = 0, uv_stride = 0, format = 0;
    float focal_px = 0, cx = 0, cy = 0;
    const uint8_t *y = nullptr;
    const uint8_t *uv = nullptr;
};

class RingReader {
public:
    RingReader() = default;
    explicit RingReader(std::string path) : path_(std::move(path)) {}
    RingReader(const RingReader &) = delete;
    RingReader &operator=(const RingReader &) = delete;
    ~RingReader() { unmap(); }

    // Maps the ring if not mapped yet (or if camerad replaced the file).
    // Cheap to call repeatedly; returns false if no valid ring is present.
    bool open() {
        if (path_.empty()) path_ = ring_path();
        struct stat st;
        if (stat(path_.c_str(), &st) != 0) return false;
        if (base_ && st.st_ino == ino_ && size_t(st.st_size) == size_) return true;
        unmap();
        int fd = ::open(path_.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd < 0) return false;
        if (fstat(fd, &st) != 0 || size_t(st.st_size) < kRingHeaderSize) {
            ::close(fd);
            return false;
        }
        void *p = mmap(nullptr, size_t(st.st_size), PROT_READ, MAP_SHARED, fd, 0);
        ::close(fd);
        if (p == MAP_FAILED) return false;
        base_ = static_cast<const uint8_t *>(p);
        size_ = size_t(st.st_size);
        ino_ = st.st_ino;
        const RingHeader *h = header();
        if (std::memcmp(h->magic, kRingMagic, 8) != 0 || h->header_size != kRingHeaderSize ||
            h->slot_count == 0 || h->format != kFormatNV12 ||
            size_t(h->header_size) + size_t(h->slot_count) * h->slot_size > size_) {
            unmap();
            return false;
        }
        last_seq_ = 0;
        return true;
    }

    const RingHeader *header() const { return reinterpret_cast<const RingHeader *>(base_); }
    bool is_open() const { return base_ != nullptr; }
    uint64_t write_seq() const { return base_ ? ring_load_acquire(&header()->write_seq) : 0; }

    // true if the writer process recorded in the header is alive.
    bool writer_alive() const {
        if (!base_) return false;
        pid_t pid = pid_t(header()->camerad_pid);
        return pid > 0 && (kill(pid, 0) == 0 || errno == EPERM);
    }

    // Copies the most recent complete frame into `out`. Returns false if there
    // is no frame yet, the newest frame was already returned (only_new), or the
    // slot was overwritten while it was being copied (frame dropped; just
    // call again on the next poll).
    bool latest(FrameView &out, bool only_new = true) {
        if (!base_ && !open()) return false;
        const RingHeader *h = header();
        const uint64_t w = ring_load_acquire(&h->write_seq);
        if (w == 0) return false;
        if (only_new && w == last_seq_) return false;
        const SlotHeader *s = slot(uint32_t(w % h->slot_count));
        if (ring_load_acquire(&s->seq) != w) return false;  // already lapped
        const SlotHeader meta = *s;
        if (meta.data_offset + uint64_t(meta.data_size) > h->slot_size) return false;
        if (buf_.size() < meta.data_size) buf_.resize(meta.data_size);
        std::memcpy(buf_.data(), reinterpret_cast<const uint8_t *>(s) + meta.data_offset, meta.data_size);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (__atomic_load_n(&s->seq, __ATOMIC_RELAXED) != w) {
            ++dropped_;
            return false;
        }
        last_seq_ = w;
        out.seq = w;
        out.ts_mono_ns = meta.ts_mono_ns;
        out.ts_real_ns = meta.ts_real_ns;
        out.exposure_us = meta.exposure_us;
        out.analogue_gain = meta.analogue_gain;
        out.colour_gain_r = meta.colour_gain_r;
        out.colour_gain_b = meta.colour_gain_b;
        out.width = h->width;
        out.height = h->height;
        out.y_stride = h->y_stride;
        out.uv_stride = h->uv_stride;
        out.format = h->format;
        out.focal_px = h->focal_px;
        out.cx = h->cx;
        out.cy = h->cy;
        out.y = buf_.data();
        out.uv = buf_.data() + size_t(h->y_stride) * h->height;
        return true;
    }

    // Polls (1 ms sleep) until a new frame arrives or timeout_ms elapses.
    // Re-opens the ring if camerad restarted.
    bool next(FrameView &out, int timeout_ms) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        int idle = 0;
        for (;;) {
            if (latest(out, true)) return true;
            if (std::chrono::steady_clock::now() >= deadline) return false;
            if (++idle % 500 == 0) open();  // every ~0.5 s: check for a replaced ring file
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    uint64_t dropped() const { return dropped_; }  // frames lost to overwrite mid-copy

private:
    const SlotHeader *slot(uint32_t i) const {
        return reinterpret_cast<const SlotHeader *>(base_ + header()->header_size + size_t(i) * header()->slot_size);
    }
    void unmap() {
        if (base_) munmap(const_cast<uint8_t *>(base_), size_);
        base_ = nullptr;
        size_ = 0;
        ino_ = 0;
    }
    std::string path_;
    const uint8_t *base_ = nullptr;
    size_t size_ = 0;
    ino_t ino_ = 0;
    uint64_t last_seq_ = 0;
    uint64_t dropped_ = 0;
    std::vector<uint8_t> buf_;
};

}  // namespace pocopilot

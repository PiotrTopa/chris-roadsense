// GPU turn-taking between modeld and a sign provider (docs/ARCHITECTURE.md, "GPU schedule").
//
// Both daemons submit Vulkan work to the same GPU ring (Adreno 630: one ring, no preemption), so a
// sign-detector submit that lands in the middle of a modeld run makes that run wait for it. This
// optional shared file lets the sign provider put its detector into the gap at the end of a modeld
// run instead, and lets modeld hand the GPU over for one detector run on request.
//
// File: <rundir>/gpu.sched (default /run/roadsense/gpu.sched), 4096 bytes, created by whichever side
// opens it first, mapped MAP_SHARED by both. Little-endian. Only modeld writes `run_seq`, `pid`,
// `run_end_ns`, `run_ms`; only the sign provider writes `want` and clears `turn`.
//
//   offset 0   char     magic[8]     "RSGPU001" (written by modeld after the other fields)
//   offset 8   uint32   run_seq      futex word; modeld adds 1 after every model run (FUTEX_WAKE all)
//   offset 12  uint32   turn         futex word; 1 = modeld granted the GPU to the sign provider and
//                                    waits (at most its own yield timeout) until it is 0 again
//   offset 16  uint32   want         1 = the sign provider wants the next turn (it sets 0 when done)
//   offset 20  uint32   reserved     0
//   offset 24  int64    pid          modeld's pid (liveness: kill(pid, 0))
//   offset 32  int64    run_end_ns   CLOCK_MONOTONIC at the end of the last model run
//   offset 40  float    run_ms       wall time of the last model run
//   offset 44  float    det_ms       the sign provider's detector wall time during a turn (moving average);
//                                    modeld may start its next run `lead` ms before this runs out, so its
//                                    own CPU preamble overlaps the end of the detector (0 = unknown)
//
// Rules that keep each daemon working alone: every wait has a timeout; a sign provider that finds no
// file, no magic, a dead pid or no run for 1 s does not wait at all; modeld never waits longer than its
// yield timeout for a turn to be returned (a provider that crashes holding the turn costs one timeout).
#pragma once

#include <fcntl.h>
#include <linux/futex.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>

namespace roadsense {

struct GpuSchedShm {
    char magic[8];
    uint32_t run_seq;
    uint32_t turn;
    uint32_t want;
    uint32_t reserved;
    int64_t pid;
    int64_t run_end_ns;
    float run_ms;
    float det_ms;
};
static_assert(sizeof(GpuSchedShm) <= 64, "gpu.sched header");

inline const char *kGpuSchedMagic = "RSGPU001";

inline int64_t gpusched_mono_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}

class GpuSched {
public:
    GpuSched() = default;
    GpuSched(const GpuSched &) = delete;
    GpuSched &operator=(const GpuSched &) = delete;
    ~GpuSched() { close(); }

    // create = true (modeld): create/size the file if needed. false (sign provider): open only.
    bool open(const std::string &path, bool create) {
        close();
        path_ = path;  // kept on failure: the provider retries (peer_alive)
        int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC | (create ? O_CREAT : 0), 0644);
        if (fd < 0) return false;
        struct stat st;
        if (fstat(fd, &st) != 0 || (st.st_size < 4096 && (!create || ftruncate(fd, 4096) != 0))) {
            ::close(fd);
            return false;
        }
        void *p = mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        ::close(fd);
        if (p == MAP_FAILED) return false;
        s_ = static_cast<GpuSchedShm *>(p);
        return true;
    }
    void close() {
        if (s_) munmap(s_, 4096);
        s_ = nullptr;
    }
    bool is_open() const { return s_ != nullptr; }

    // ---- modeld side
    void publish_pid(int64_t pid) {
        if (!s_) return;
        s_->pid = pid;
        a32(&s_->turn).store(0, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        std::memcpy(s_->magic, kGpuSchedMagic, 8);
    }
    // Called at the end of every model run: publishes the tick. If allow_grant and the provider asked
    // for a turn, grants it (returns true); modeld then calls wait_turn_back() before its next run.
    bool run_done(double run_ms, bool allow_grant) {
        if (!s_) return false;
        s_->run_ms = float(run_ms);
        s_->run_end_ns = gpusched_mono_ns();
        const bool grant = allow_grant && a32(&s_->want).load(std::memory_order_acquire) != 0;
        if (grant) a32(&s_->turn).store(1, std::memory_order_release);
        a32(&s_->run_seq).fetch_add(1, std::memory_order_release);
        wake(&s_->run_seq);
        return grant;
    }
    // Waits until the provider returns a granted turn, at most until yield_ms after `since_ns`
    // (then takes it back). With lead_ms > 0 and a known det_ms, stops waiting lead_ms before the
    // detector is expected to end (the turn stays with the provider until it returns it).
    // Returns the ms spent waiting here.
    double wait_turn_back(int64_t since_ns, int yield_ms, double lead_ms = 0) {
        if (!s_) return 0;
        const int64_t t0 = gpusched_mono_ns(), deadline = since_ns + int64_t(yield_ms) * 1000000;
        int64_t early = deadline;
        const float det = s_->det_ms;
        if (lead_ms > 0 && det > 0 && det < yield_ms) early = since_ns + int64_t((det - lead_ms) * 1e6);
        for (;;) {
            if (a32(&s_->turn).load(std::memory_order_acquire) == 0) break;
            const int64_t now = gpusched_mono_ns();
            if (now >= early && early < deadline) break;
            if (now >= deadline) {
                a32(&s_->turn).store(0, std::memory_order_release);  // provider gone or too slow: take it back
                break;
            }
            wait(&s_->turn, 1, std::min(early, deadline) - now);
        }
        return double(gpusched_mono_ns() - t0) * 1e-6;
    }

    // ---- sign provider side
    // True if a live modeld published a run within the last second.
    bool peer_alive() {
        if (!s_ && !path_.empty()) reopen();
        if (!s_ || std::memcmp(s_->magic, kGpuSchedMagic, 8) != 0) return false;
        const int64_t pid = s_->pid;
        if (pid <= 0 || (kill(pid_t(pid), 0) != 0 && errno == ESRCH)) return false;
        return gpusched_mono_ns() - s_->run_end_ns < 1000000000;
    }
    // Waits for the end of the next modeld run (at most timeout_ms). With want_turn, asks modeld for
    // the GPU first; returns true if a turn was granted (the caller must call end_turn() after its
    // GPU work). Returns false immediately if modeld is not there.
    bool wait_run_end(int timeout_ms, bool want_turn, bool *got_tick = nullptr) {
        if (got_tick) *got_tick = false;
        if (!peer_alive()) return false;
        if (want_turn) a32(&s_->want).store(1, std::memory_order_release);
        const uint32_t seq0 = a32(&s_->run_seq).load(std::memory_order_acquire);
        const int64_t deadline = gpusched_mono_ns() + int64_t(timeout_ms) * 1000000;
        for (;;) {
            if (a32(&s_->run_seq).load(std::memory_order_acquire) != seq0) {
                if (got_tick) *got_tick = true;
                break;
            }
            const int64_t now = gpusched_mono_ns();
            if (now >= deadline) break;
            wait(&s_->run_seq, seq0, deadline - now);
        }
        if (!want_turn) return false;
        a32(&s_->want).store(0, std::memory_order_release);
        return a32(&s_->turn).load(std::memory_order_acquire) == 1;
    }
    // det_ms > 0: the wall time of the detector run just done in this turn (feeds `det_ms`).
    void end_turn(double det_ms = 0) {
        if (!s_) return;
        if (det_ms > 0) s_->det_ms = s_->det_ms > 0 ? float(0.8 * s_->det_ms + 0.2 * det_ms) : float(det_ms);
        a32(&s_->turn).store(0, std::memory_order_release);
        wake(&s_->turn);
    }

private:
    static std::atomic<uint32_t> &a32(uint32_t *p) { return *reinterpret_cast<std::atomic<uint32_t> *>(p); }
    static void wake(uint32_t *p) { syscall(SYS_futex, p, FUTEX_WAKE, INT32_MAX, nullptr, nullptr, 0); }
    static void wait(uint32_t *p, uint32_t val, int64_t ns) {
        timespec ts{time_t(ns / 1000000000), long(ns % 1000000000)};
        syscall(SYS_futex, p, FUTEX_WAIT, val, &ts, nullptr, 0);
    }
    void reopen() {
        // the provider retries at most once per second while the file is missing
        const int64_t now = gpusched_mono_ns();
        if (now - last_try_ns_ < 1000000000) return;
        last_try_ns_ = now;
        std::string p = path_;
        open(p, false);
    }

    GpuSchedShm *s_ = nullptr;
    std::string path_;
    int64_t last_try_ns_ = 0;
};

}  // namespace roadsense

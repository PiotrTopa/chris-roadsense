// Reader of the load governor's caps file `<rundir>/governor` (docs/ARCHITECTURE.md, "Load governor").
// Header-only, no dependencies; used by every daemon that has a stage the governor may slow down.
//
// The file is one flat JSON object rewritten atomically by the governor (roadsense api) at least every 2 s:
//   {"v":1,"ts_mono_ns":...,"valid_s":10,"level":3,"limited":0,"headroom_k":4.2,
//    "preview_hz":1,"record_frames":0,"env_hz":0.2,"view_hz":-1,"signs_hz":-1,"lights_hz":-1,"policy_hz":-1,"road_hz":-1}
// A cap of -1 (or a missing key, a missing file, a file older than valid_s) leaves the daemon's own
// configured rate; 0 pauses the stage; any other value is an upper bound.
#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

namespace roadsense {

class GovernorReader {
public:
    // path empty: $ROADSENSE_RUNDIR/governor (default /run/roadsense/governor)
    explicit GovernorReader(std::string path = "") : path_(std::move(path)) {
        if (path_.empty()) {
            const char *rd = getenv("ROADSENSE_RUNDIR");
            path_ = std::string((rd && *rd) ? rd : "/run/roadsense") + "/governor";
        }
    }

    // The cap for `key` ("road_hz", "env_hz", ...): < 0 = none. Re-reads the file at most twice a second.
    double cap(const char *key) {
        refresh();
        if (!valid_) return -1;
        double v;
        return number(key, &v) ? v : -1;
    }
    // min(configured, cap); a configured rate <= 0 means "as fast as possible" and takes the cap as it is.
    // Returns 0 when the stage is paused.
    double limit(const char *key, double configured) {
        const double c = cap(key);
        if (c < 0) return configured;
        if (configured <= 0) return c;
        return c < configured ? c : configured;
    }
    // Shed level (0 = nothing shed), -1 without a valid file.
    int level() {
        refresh();
        double v;
        return valid_ && number("level", &v) ? int(v) : -1;
    }
    bool limited() {
        refresh();
        double v;
        return valid_ && number("limited", &v) && v != 0;
    }

private:
    static int64_t mono_ns() {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
    }
    bool number(const char *key, double *v) const {
        const std::string k = std::string("\"") + key + "\":";
        const size_t p = buf_.find(k);
        if (p == std::string::npos) return false;
        const char *s = buf_.c_str() + p + k.size();
        char *end;
        *v = strtod(s, &end);
        return end != s;
    }
    void refresh() {
        const int64_t now = mono_ns();
        if (read_ns_ && now - read_ns_ < 500000000LL) {
            if (valid_ && now > expire_ns_) valid_ = false;
            return;
        }
        read_ns_ = now;
        valid_ = false;
        FILE *f = fopen(path_.c_str(), "r");
        if (!f) return;
        char tmp[1024];
        const size_t n = fread(tmp, 1, sizeof tmp - 1, f);
        fclose(f);
        buf_.assign(tmp, n);
        double ts, valid_s = 10;
        if (!number("ts_mono_ns", &ts)) return;
        number("valid_s", &valid_s);
        expire_ns_ = int64_t(ts) + int64_t(valid_s * 1e9);
        valid_ = now <= expire_ns_;
    }

    std::string path_, buf_;
    int64_t read_ns_ = 0, expire_ns_ = 0;
    bool valid_ = false;
};

}  // namespace roadsense

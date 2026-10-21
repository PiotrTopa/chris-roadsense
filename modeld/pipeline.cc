#include "pipeline.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>

namespace roadsense {

std::string config_dir() {
    const char *d = getenv("ROADSENSE_CONFIG_DIR");
    return (d && *d) ? std::string(d) : std::string("/etc/roadsense");
}

bool read_file(const std::string &path, std::string *out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    *out = ss.str();
    return true;
}

static const char *find_value(const std::string &s, const char *key) {
    const std::string k = std::string("\"") + key + "\"";
    size_t p = s.find(k);
    while (p != std::string::npos) {
        size_t q = p + k.size();
        while (q < s.size() && isspace((unsigned char)s[q])) ++q;
        if (q < s.size() && s[q] == ':') {
            ++q;
            while (q < s.size() && isspace((unsigned char)s[q])) ++q;
            return s.c_str() + q;
        }
        p = s.find(k, p + 1);
    }
    return nullptr;
}

bool json_number(const std::string &s, const char *key, double *v) {
    const char *p = find_value(s, key);
    if (!p) return false;
    if (!strncmp(p, "true", 4)) return *v = 1, true;
    if (!strncmp(p, "false", 5)) return *v = 0, true;
    char *end = nullptr;
    double d = strtod(p, &end);
    if (end == p) return false;
    *v = d;
    return true;
}

bool json_string(const std::string &s, const char *key, std::string *v) {
    const char *p = find_value(s, key);
    if (!p || *p != '"') return false;
    const char *e = strchr(p + 1, '"');
    if (!e) return false;
    v->assign(p + 1, e);
    return true;
}

bool load_calib(const std::string &path, Calib *c, std::string *err) {
    std::string s;
    if (!read_file(path, &s)) {
        if (err) *err = "cannot read " + path + " (using defaults)";
        return false;
    }
    double v;
    if (json_number(s, "pitch", &v)) c->pitch = v;
    if (json_number(s, "yaw", &v)) c->yaw = v;
    if (json_number(s, "roll", &v)) c->roll = v;
    if (json_number(s, "height_m", &v) || json_number(s, "height", &v)) c->height_m = v;
    return true;
}

bool FrameSelector::accept(int64_t ts) {
    if (last_any_ >= 0 && ts > last_any_) {
        const double dt = double(ts - last_any_) * 1e-6;
        if (dt < 1000) dt_ema_ms_ = dt_ema_ms_ > 0 ? 0.9 * dt_ema_ms_ + 0.1 * dt : dt;
    }
    const bool restart = last_any_ >= 0 && ts <= last_any_;
    last_any_ = ts;
    if (last_sel_ < 0 || restart) {
        last_sel_ = ts;
        return true;
    }
    // stream at <= ~20 fps (mean interval >= 47.5 ms): consecutive frames
    if (dt_ema_ms_ >= 47.5 || ts - last_sel_ >= 45'000'000) {
        last_sel_ = ts;
        return true;
    }
    return false;
}

void Preprocessor::pack(const NV12Frame &f, const Calib &c, PackedFrame *out) {
    const auto t0 = std::chrono::steady_clock::now();
    out->med.resize(sc::kFramePacked);
    out->big.resize(sc::kFramePacked);
    med_.pack(f.y, f.y_stride, f.uv, f.uv_stride, f.k, c, out->med.data());
    big_.pack(f.y, f.y_stride, f.uv, f.uv_stride, f.k, c, out->big.data());
    out->seq = f.seq;
    out->ts_mono_ns = f.ts_mono_ns;
    out->ts_real_ns = f.ts_real_ns;
    out->calib = c;
    out->k = f.k;
    out->prep_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

std::string calib_json(const Calib &c, const Intrinsics &k, uint64_t seq, const char *source) {
    char b[512];
    std::snprintf(b, sizeof b,
                  "{\"type\":\"calib\",\"seq\":%llu,\"ts_mono_ns\":%lld,\"ts_real_ns\":%lld,\"pitch\":%.6g,"
                  "\"yaw\":%.6g,\"roll\":%.6g,\"height_m\":%.4g,\"focal_px\":%.6g,\"cx\":%.6g,\"cy\":%.6g,"
                  "\"width\":%d,\"height\":%d,\"source\":\"%s\"}",
                  (unsigned long long)seq, (long long)mono_ns(), (long long)real_ns(), c.pitch, c.yaw, c.roll,
                  c.height_m, k.fx, k.cx, k.cy, k.width, k.height, source);
    return b;
}

int64_t mono_ns() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return int64_t(t.tv_sec) * 1000000000 + t.tv_nsec;
}
int64_t real_ns() {
    timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return int64_t(t.tv_sec) * 1000000000 + t.tv_nsec;
}

}  // namespace roadsense

#include "calibrator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "pipeline.h"

namespace roadsense {

namespace {

double median(std::vector<double> v) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

// 1.4826 * median absolute deviation: the sigma of a normal sample, not moved by a few odd blocks
double robust_sigma(const std::vector<double> &v, double med) {
    std::vector<double> d;
    d.reserve(v.size());
    for (double x : v) d.push_back(std::fabs(x - med));
    return 1.4826 * median(d);
}

bool same_config(const Calib &a, const Calib &b) { return a == b; }

}  // namespace

void OnlineCalib::reset_locked() {
    blocks_.clear();
    cur_pitch_.clear();
    cur_yaw_.clear();
    have_ = valid_ = unstable_ = out_of_range_ = restored_ = false;
    est_pitch_ = est_yaw_ = sp_pitch_ = sp_yaw_ = 0;
    applied_ = config_;
}

void OnlineCalib::set_config(const Calib &config, const Intrinsics &k) {
    std::lock_guard<std::mutex> l(m_);
    if (have_config_ && same_config(config, config_) && k == k_) return;
    config_ = config;
    k_ = k;
    have_config_ = true;
    reset_locked();
    if (o_.enabled && load_locked()) {
        restored_ = true;
        update_locked();
    }
}

Calib OnlineCalib::calib() const {
    std::lock_guard<std::mutex> l(m_);
    return applied_;
}

bool OnlineCalib::observe(const Calib &used, const float pose[6], const float pose_std[6]) {
    std::lock_guard<std::mutex> l(m_);
    if (!o_.enabled || !have_config_) return false;
    ++seen_;
    // a frame warped for another configuration (calib.json changed a moment ago) says nothing about this one
    if (used.roll != config_.roll || used.height_m != config_.height_m) return false;
    const double vx = pose[0], vy = pose[1], vz = pose[2];
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz) || !std::isfinite(pose[5]) ||
        !std::isfinite(pose_std[1]))
        return false;
    if (vx < o_.min_speed || std::fabs(pose[5]) > o_.max_yaw_rate) return false;
    if (std::atan2(double(pose_std[1]), vx) > o_.max_angle_std) return false;
    ++accepted_;
    cur_pitch_.push_back(used.pitch - std::atan2(vz, vx));
    cur_yaw_.push_back(used.yaw + std::atan2(vy, vx));
    if (int(cur_pitch_.size()) < o_.block) return false;
    blocks_.push_back({median(cur_pitch_), median(cur_yaw_)});
    cur_pitch_.clear();
    cur_yaw_.clear();
    while (int(blocks_.size()) > o_.max_blocks) blocks_.pop_front();
    if (int(blocks_.size()) >= 2 * o_.min_blocks) {
        // the camera was moved: the newest blocks agree with each other and not with the rest
        std::vector<double> ap, ay, rp, ry;
        const size_t n0 = blocks_.size() - size_t(o_.min_blocks);
        for (size_t i = 0; i < blocks_.size(); ++i) {
            (i < n0 ? ap : rp).push_back(blocks_[i].pitch);
            (i < n0 ? ay : ry).push_back(blocks_[i].yaw);
        }
        const double mp = median(ap), my = median(ay);
        const double lp = std::max(o_.moved, 4 * robust_sigma(ap, mp)), ly = std::max(o_.moved, 4 * robust_sigma(ay, my));
        auto all_off = [](const std::vector<double> &v, double m, double lim) {
            bool pos = true, neg = true;
            for (double x : v) pos &= x - m > lim, neg &= m - x > lim;
            return pos || neg;
        };
        if (all_off(rp, mp, lp) || all_off(ry, my, ly)) blocks_.erase(blocks_.begin(), blocks_.begin() + long(n0));
    }
    update_locked();
    save_locked();
    return true;
}

void OnlineCalib::update_locked() {
    have_ = !blocks_.empty();
    const bool was_valid = valid_;
    valid_ = unstable_ = out_of_range_ = false;
    if (!have_) return;
    std::vector<double> p, y;
    for (const Block &b : blocks_) p.push_back(b.pitch), y.push_back(b.yaw);
    est_pitch_ = median(p);
    est_yaw_ = median(y);
    sp_pitch_ = robust_sigma(p, est_pitch_);
    sp_yaw_ = robust_sigma(y, est_yaw_);
    if (int(blocks_.size()) < o_.min_blocks) return;
    const double k = 1.2533 / std::sqrt(double(blocks_.size()));
    const double se_lim = was_valid ? o_.max_se_keep : o_.max_se;
    if (sp_pitch_ > o_.max_spread || sp_yaw_ > o_.max_spread || k * sp_pitch_ > se_lim || k * sp_yaw_ > se_lim) {
        unstable_ = true;
    } else if (std::fabs(est_pitch_ - config_.pitch) > o_.max_dev || std::fabs(est_yaw_ - config_.yaw) > o_.max_dev) {
        out_of_range_ = true;
    } else {
        valid_ = true;
    }
    if (!o_.apply) return;
    if (!valid_) {
        // an estimate that stopped being valid is not kept in the warp: unknown beats confident wrong
        applied_ = config_;
        return;
    }
    if (std::fabs(est_pitch_ - applied_.pitch) > o_.step || std::fabs(est_yaw_ - applied_.yaw) > o_.step) {
        applied_.pitch = est_pitch_;
        applied_.yaw = est_yaw_;
    }
}

OnlineCalibStatus OnlineCalib::status_locked() const {
    OnlineCalibStatus s;
    s.state = !o_.enabled ? "off" : valid_ ? "valid" : unstable_ ? "unstable" : out_of_range_ ? "out_of_range" : "collecting";
    s.applied = have_config_ && (applied_.pitch != config_.pitch || applied_.yaw != config_.yaw);
    s.have = have_;
    s.pitch = est_pitch_;
    s.yaw = est_yaw_;
    s.spread_pitch = sp_pitch_;
    s.spread_yaw = sp_yaw_;
    const double kk = blocks_.empty() ? 0.0 : 1.2533 / std::sqrt(double(blocks_.size()));
    s.se_pitch = kk * sp_pitch_;
    s.se_yaw = kk * sp_yaw_;
    s.blocks = int(blocks_.size());
    s.samples = int(cur_pitch_.size());
    s.accepted = accepted_;
    s.seen = seen_;
    s.config_pitch = config_.pitch;
    s.config_yaw = config_.yaw;
    s.restored = restored_;
    return s;
}

OnlineCalibStatus OnlineCalib::status() const {
    std::lock_guard<std::mutex> l(m_);
    return status_locked();
}

std::string OnlineCalib::status_json() const {
    std::lock_guard<std::mutex> l(m_);
    const OnlineCalibStatus s = status_locked();
    char b[640];
    if (s.have)
        std::snprintf(b, sizeof b,
                      "\"online\":{\"state\":\"%s\",\"applied\":%s,\"pitch\":%.6g,\"yaw\":%.6g,\"spread_pitch\":%.4g,"
                      "\"spread_yaw\":%.4g,\"se_pitch\":%.4g,\"se_yaw\":%.4g,\"blocks\":%d,\"min_blocks\":%d,\"samples\":%d,\"block\":%d,\"accepted\":%llu,"
                      "\"seen\":%llu,\"restored\":%s,\"config\":{\"pitch\":%.6g,\"yaw\":%.6g}}",
                      s.state, s.applied ? "true" : "false", s.pitch, s.yaw, s.spread_pitch, s.spread_yaw, s.se_pitch, s.se_yaw, s.blocks,
                      o_.min_blocks, s.samples, o_.block, (unsigned long long)s.accepted, (unsigned long long)s.seen,
                      s.restored ? "true" : "false", s.config_pitch, s.config_yaw);
    else
        std::snprintf(b, sizeof b,
                      "\"online\":{\"state\":\"%s\",\"applied\":false,\"pitch\":null,\"yaw\":null,\"spread_pitch\":null,"
                      "\"spread_yaw\":null,\"blocks\":0,\"min_blocks\":%d,\"samples\":%d,\"block\":%d,\"accepted\":%llu,"
                      "\"seen\":%llu,\"restored\":false,\"config\":{\"pitch\":%.6g,\"yaw\":%.6g}}",
                      s.state, o_.min_blocks, s.samples, o_.block, (unsigned long long)s.accepted,
                      (unsigned long long)s.seen, s.config_pitch, s.config_yaw);
    return b;
}

// State file: one flat JSON object. The estimate belongs to one configuration (calib.json + intrinsics); a file
// written for another one is ignored. Blocks are stored as two arrays.
void OnlineCalib::save_locked() const {
    if (o_.state_path.empty()) return;
    const std::string tmp = o_.state_path + ".tmp";
    FILE *f = std::fopen(tmp.c_str(), "w");
    if (!f) return;
    std::fprintf(f,
                 "{\"cfg_pitch\":%.9g,\"cfg_yaw\":%.9g,\"cfg_roll\":%.9g,\"cfg_height_m\":%.9g,\"focal_px\":%.9g,"
                 "\"cx\":%.9g,\"cy\":%.9g,\"width\":%d,\"height\":%d,\"n\":%zu",
                 config_.pitch, config_.yaw, config_.roll, config_.height_m, k_.fx, k_.cx, k_.cy, k_.width, k_.height,
                 blocks_.size());
    for (size_t i = 0; i < blocks_.size(); ++i)
        std::fprintf(f, ",\"p%zu\":%.9g,\"y%zu\":%.9g", i, blocks_[i].pitch, i, blocks_[i].yaw);
    std::fprintf(f, "}\n");
    const bool ok = std::fclose(f) == 0;
    if (ok) std::rename(tmp.c_str(), o_.state_path.c_str());
}

bool OnlineCalib::load_locked() {
    if (o_.state_path.empty()) return false;
    std::string s;
    if (!read_file(o_.state_path, &s)) return false;
    auto num = [&](const char *key, double *v) { return json_number(s, key, v); };
    auto near = [](double a, double b) { return std::fabs(a - b) < 1e-6; };
    double v[9], n;
    static const char *keys[9] = {"cfg_pitch", "cfg_yaw", "cfg_roll", "cfg_height_m", "focal_px", "cx", "cy", "width", "height"};
    for (int i = 0; i < 9; ++i)
        if (!num(keys[i], &v[i])) return false;
    if (!near(v[0], config_.pitch) || !near(v[1], config_.yaw) || !near(v[2], config_.roll) || !near(v[3], config_.height_m) ||
        !near(v[4], k_.fx) || !near(v[5], k_.cx) || !near(v[6], k_.cy) || int(v[7]) != k_.width || int(v[8]) != k_.height)
        return false;
    if (!num("n", &n) || n < 1 || n > 1000) return false;
    std::deque<Block> bl;
    for (int i = 0; i < int(n); ++i) {
        char kp[16], ky[16];
        std::snprintf(kp, sizeof kp, "p%d", i);
        std::snprintf(ky, sizeof ky, "y%d", i);
        Block b;
        if (!num(kp, &b.pitch) || !num(ky, &b.yaw) || !std::isfinite(b.pitch) || !std::isfinite(b.yaw)) return false;
        bl.push_back(b);
    }
    while (int(bl.size()) > o_.max_blocks) bl.pop_front();
    blocks_ = std::move(bl);
    return true;
}

std::string calib_json(const Calib &c, const Intrinsics &k, uint64_t seq, const OnlineCalib &oc) {
    const OnlineCalibStatus s = oc.status();
    // the calibration of these frames: "auto" when it is not the configured one
    const bool autoc = c.pitch != s.config_pitch || c.yaw != s.config_yaw;
    std::string j = calib_json(c, k, seq, autoc ? "auto" : "config");
    j.pop_back();  // '}'
    j += "," + oc.status_json() + "}";
    return j;
}

}  // namespace roadsense

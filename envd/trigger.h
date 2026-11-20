// Inference schedule with a cheap scene-change trigger (lane L48-E, docs/lanes/L48-envd.md).
//
// envd normally runs its image tower on one frame per period. With the trigger on, it also looks at the
// newest camera frame a few times a second through a 32x18 block-mean thumbnail of Y, U and V (one pass
// over the frame, ~1 ms on an A55) and compares it with the thumbnail of the frame of the last inference:
// a mean-luma step, a change of the luma histogram or a mean-chroma step above its threshold (a tunnel
// portal, nightfall, entering a garage) asks for an inference at once and for a short burst at the burst
// period afterwards, so the smoothing window fills as it would at 1 Hz around the transition.
// The thumbnail and the score are exactly tools/envd/rate_sim.py's thumb_change() (the study's replica).
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "prep.h"  // Nv12View

namespace roadsense {
namespace envd {

struct Thumb {
    static constexpr int W = 32, H = 18, N = W * H;
    uint8_t y[N], u[N], v[N];
    bool valid = false;
};

// Block means of the whole frame (width a multiple of 64, height of 36: 1280x720, 1920x1080; other sizes use
// the integer block bounds, the last block takes the remainder).
inline void make_thumb(const Nv12View &f, Thumb *t) {
    const int W = f.width & ~1, H = f.height & ~1;
    for (int by = 0; by < Thumb::H; ++by) {
        const int y0 = by * H / Thumb::H, y1 = (by + 1) * H / Thumb::H;
        for (int bx = 0; bx < Thumb::W; ++bx) {
            const int x0 = bx * W / Thumb::W, x1 = (bx + 1) * W / Thumb::W;
            uint64_t sy = 0;
            for (int y = y0; y < y1; ++y) {
                const uint8_t *r = f.y + size_t(y) * f.y_stride;
                for (int x = x0; x < x1; ++x) sy += r[x];
            }
            uint64_t su = 0, sv = 0;
            const int cy0 = y0 / 2, cy1 = y1 / 2, cx0 = x0 / 2, cx1 = x1 / 2;
            for (int y = cy0; y < cy1; ++y) {
                const uint8_t *r = f.uv + size_t(y) * f.uv_stride;
                for (int x = cx0; x < cx1; ++x) {
                    su += r[2 * x];
                    sv += r[2 * x + 1];
                }
            }
            const double ny = double(y1 - y0) * (x1 - x0), nc = double(cy1 - cy0) * (cx1 - cx0);
            const int i = by * Thumb::W + bx;
            t->y[i] = uint8_t(std::nearbyint(sy / ny));
            t->u[i] = uint8_t(std::nearbyint(nc > 0 ? su / nc : 128));
            t->v[i] = uint8_t(std::nearbyint(nc > 0 ? sv / nc : 128));
        }
    }
    t->valid = true;
}

struct ChangeScore {
    double dy = 0;  // |mean luma step|, 0..255
    double dh = 0;  // luma histogram distance (16 bins, half the L1 norm), 0..1
    double dc = 0;  // |step of the mean |chroma - 128||, 0..128
};

inline ChangeScore thumb_change(const Thumb &a, const Thumb &b) {
    ChangeScore s;
    double ya = 0, yb = 0, ca = 0, cb = 0;
    int ha[16] = {0}, hb[16] = {0};
    for (int i = 0; i < Thumb::N; ++i) {
        ya += a.y[i];
        yb += b.y[i];
        ++ha[a.y[i] >> 4];
        ++hb[b.y[i] >> 4];
        ca += std::abs(int(a.u[i]) - 128) + std::abs(int(a.v[i]) - 128);
        cb += std::abs(int(b.u[i]) - 128) + std::abs(int(b.v[i]) - 128);
    }
    s.dy = std::fabs(yb - ya) / Thumb::N;
    int l1 = 0;
    for (int k = 0; k < 16; ++k) l1 += std::abs(ha[k] - hb[k]);
    s.dh = 0.5 * l1 / Thumb::N;
    s.dc = std::fabs(cb - ca) / (2.0 * Thumb::N);
    return s;
}

struct TriggerConfig {
    double luma = 30;      // mean-luma step (0..255)
    double hist = 0.50;    // luma histogram distance (0..1)
    double chroma = 10;    // mean-chroma step
    double burst_s = 6;    // after a trigger: this long at burst_period
    double burst_period_s = 1;
    double min_gap_s = 1;  // no trigger within this of the last inference
};

// Decides when the next inference is due. Times in seconds (any monotonic clock).
class TriggerSchedule {
public:
    explicit TriggerSchedule(const TriggerConfig &c) : c_(c) {}
    // A thumbnail of the newest frame at time t: true if it asks for an inference now.
    bool look(const Thumb &cur, double t) {
        if (!ref_.valid || pending_ || t - last_ < c_.min_gap_s) return pending_;
        const ChangeScore s = thumb_change(ref_, cur);
        last_score_ = s;
        if (s.dy > c_.luma || s.dh > c_.hist || s.dc > c_.chroma) pending_ = true;
        return pending_;
    }
    // Is an inference due at t, with `period_s` the regular period (the governor may have lengthened it)?
    bool due(double t, double period_s) const {
        if (pending_) return true;
        const double per = t < burst_until_ ? std::fmin(c_.burst_period_s, period_s) : period_s;
        return t - last_ >= per - 1e-6;
    }
    // An inference ran on the frame with this thumbnail at time t.
    void ran(const Thumb &frame, double t) {
        if (pending_) {
            ++triggers_;
            burst_until_ = t + c_.burst_s;
        }
        pending_ = false;
        last_ = t;
        ref_ = frame;
    }
    unsigned triggers() const { return triggers_; }
    const ChangeScore &last_score() const { return last_score_; }

private:
    TriggerConfig c_;
    Thumb ref_;
    bool pending_ = false;
    double last_ = -1e18, burst_until_ = -1e18;
    unsigned triggers_ = 0;
    ChangeScore last_score_;
};

}  // namespace envd
}  // namespace roadsense

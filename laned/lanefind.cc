#include "lanefind.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace roadsense {
namespace laned {

namespace {

const std::vector<double> kOutX = {0, 2, 4, 6, 8, 10, 12.5, 15, 20, 25, 30, 35, 40, 50, 60, 70, 80};

inline double poly(const double *p, double x) { return p[0] + x * (p[1] + x * p[2]); }

// Weighted least squares of y = p0 + p1 x + p2 x^2 with a Gaussian prior on p2 (mean c0, weight wc).
bool fit_quad(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &w, double c0,
              double wc, double *p) {
    double s[5] = {0, 0, 0, 0, 0}, t[3] = {0, 0, 0};
    for (size_t k = 0; k < x.size(); ++k) {
        const double xx = x[k], ww = w[k];
        double xp = 1;
        for (int m = 0; m < 5; ++m) {
            s[m] += ww * xp;
            if (m < 3) t[m] += ww * xp * y[k];
            xp *= xx;
        }
    }
    double a[3][4] = {{s[0], s[1], s[2], t[0]}, {s[1], s[2], s[3], t[1]}, {s[2], s[3], s[4] + wc, t[2] + wc * c0}};
    for (int c = 0; c < 3; ++c) {
        int piv = c;
        for (int r = c + 1; r < 3; ++r)
            if (std::fabs(a[r][c]) > std::fabs(a[piv][c])) piv = r;
        if (std::fabs(a[piv][c]) < 1e-12) return false;
        if (piv != c)
            for (int k = 0; k < 4; ++k) std::swap(a[piv][k], a[c][k]);
        for (int r = 0; r < 3; ++r) {
            if (r == c) continue;
            const double f = a[r][c] / a[c][c];
            for (int k = c; k < 4; ++k) a[r][k] -= f * a[c][k];
        }
    }
    for (int c = 0; c < 3; ++c) p[c] = a[c][3] / a[c][c];
    return true;
}

}  // namespace

bool config_set(Config *c, const std::string &key, double v) {
    if (key == "x_min") return c->x_min = v, true;
    if (key == "x_max") return c->x_max = v, true;
    if (key == "nx") return c->nx = int(v), true;
    if (key == "y_half") return c->y_half = v, true;
    if (key == "dy") return c->dy = v, true;
    if (key == "ignore_bottom") return c->ignore_bottom = int(v), true;
    if (key == "mark_w") return c->mark_w = v, true;
    if (key == "r_lo") return c->r_lo = v, true;
    if (key == "r_hi") return c->r_hi = v, true;
    if (key == "rel_lo") return c->rel_lo = v, true;
    if (key == "seed_x_max") return c->seed_x_max = v, true;
    if (key == "seed_min") return c->seed_min = v, true;
    if (key == "follow_win") return c->follow_win = v, true;
    if (key == "min_span_m") return c->min_span_m = v, true;
    if (key == "min_cover") return c->min_cover = v, true;
    if (key == "max_rms") return c->max_rms = v, true;
    if (key == "max_slope") return c->max_slope = v, true;
    if (key == "curv_prior") return c->curv_prior = v, true;
    if (key == "x_ref") return c->x_ref = v, true;
    if (key == "assoc_m") return c->assoc_m = v, true;
    if (key == "gain") return c->gain = v, true;
    if (key == "conf_tau_s") return c->conf_tau_s = v, true;
    if (key == "on_conf") return c->on_conf = v, true;
    if (key == "off_conf") return c->off_conf = v, true;
    if (key == "coast_s") return c->coast_s = v, true;
    if (key == "q_cover_lo") return c->q_cover_lo = v, true;
    if (key == "q_cover_hi") return c->q_cover_hi = v, true;
    if (key == "lane_min_m") return c->lane_min_m = v, true;
    if (key == "lane_max_m") return c->lane_max_m = v, true;
    if (key == "under_car_m") return c->under_car_m = v, true;
    if (key == "reach_min_m") return c->reach_min_m = v, true;
    if (key == "trim_tau_s") return c->trim_tau_s = v, true;
    if (key == "trim_max") return c->trim_max = v, true;
    if (key == "trim") return c->trim = v != 0, true;
    if (key == "max_clutter") return c->max_clutter = v, true;
    if (key == "radial_m") return c->radial_m = v, true;
    if (key == "trim_sign") return c->trim_sign = v, true;
    if (key == "inner_max_m") return c->inner_max_m = v, true;
    if (key == "pair_min_m") return c->pair_min_m = v, true;
    if (key == "pair_ratio") return c->pair_ratio = v, true;
    if (key == "yaw_tau_s") return c->yaw_tau_s = v, true;
    if (key == "trim_step") return c->trim_step = v, true;
    return false;
}

const std::vector<double> &LaneFinder::out_x() { return kOutX; }

LaneFinder::LaneFinder(const Config &cfg, const Camera &cam) : cfg_(cfg), cam_(cam) {
    ny_ = int(std::lround(2 * cfg_.y_half / cfg_.dy)) + 1;
    xs_.resize(size_t(cfg_.nx));
    const double r = std::pow(cfg_.x_max / cfg_.x_min, 1.0 / (cfg_.nx - 1));
    for (int i = 0; i < cfg_.nx; ++i) xs_[size_t(i)] = cfg_.x_min * std::pow(r, i);
    e_.assign(size_t(cfg_.nx) * size_t(ny_), 0.f);
    reset();
}

void LaneFinder::reset() {
    tracks_.clear();
    last_ts_ = 0;
    fam_b_ = fam_c_ = 0;
    // the pitch trim is kept: it describes the camera, not the scene
}

void LaneFinder::set_camera(const Camera &cam) {
    cam_ = cam;
    grid_trim_ = 1e9;
}

void LaneFinder::build_grid() {
    grid_trim_ = trim_, grid_yaw_ = yaw_trim_;
    const double roll = cam_.roll, pitch = cam_.pitch + trim_, yaw = cam_.yaw + yaw_trim_;
    const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch), cy = std::cos(yaw),
                 sy = std::sin(yaw);
    // R = Rz(yaw) Ry(pitch) Rx(roll); p_dev = R p_calib; camera X = dev y, Y = dev z, Z = dev x
    const double R[3][3] = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                            {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                            {-sp, cp * sr, cp * cr}};
    nodes_.assign(size_t(cfg_.nx) * size_t(sub_) * size_t(ny_), Node{-1, -1, 0});
    const double ratio = std::pow(cfg_.x_max / cfg_.x_min, 1.0 / (cfg_.nx - 1));
    const int vmax = cam_.height - 1 - cfg_.ignore_bottom;
    for (int i = 0; i < cfg_.nx; ++i)
        for (int s = 0; s < sub_; ++s) {
            const double x = xs_[size_t(i)] * std::pow(ratio, (s - (sub_ - 1) * 0.5) / sub_);
            for (int j = 0; j < ny_; ++j) {
                const double y = -cfg_.y_half + j * cfg_.dy, z = cam_.height_m;
                const double dx = R[0][0] * x + R[0][1] * y + R[0][2] * z;
                const double dyv = R[1][0] * x + R[1][1] * y + R[1][2] * z;
                const double dz = R[2][0] * x + R[2][1] * y + R[2][2] * z;
                if (dx < 1.0) continue;
                const double u = cam_.f * dyv / dx + cam_.cx, v = cam_.f * dz / dx + cam_.cy;
                if (u < 4 || u > cam_.width - 5 || v < 0 || v > vmax) continue;
                Node &n = nodes_[(size_t(i) * size_t(sub_) + size_t(s)) * size_t(ny_) + size_t(j)];
                n.u = float(u);
                n.v = float(v);
                n.wpx = float(cam_.f * cfg_.mark_w / dx);
            }
        }
}

void LaneFinder::fill_evidence_luma(const uint8_t *yp, int stride) {
    const int W = cam_.width;
    psum_.resize(size_t(W) + 1);
    std::fill(e_.begin(), e_.end(), 0.f);
    const float inv = float(1.0 / (cfg_.r_hi - cfg_.r_lo));
    for (int i = 0; i < cfg_.nx; ++i)
        for (int s = 0; s < sub_; ++s) {
            const Node *row = &nodes_[(size_t(i) * size_t(sub_) + size_t(s)) * size_t(ny_)];
            int vrow = -1;
            for (int j = 0; j < ny_; ++j) {
                const Node &n = row[j];
                if (n.u < 0) continue;
                const int v = int(n.v + 0.5f);
                if (v != vrow) {         // prefix sums of this image row (a grid row is one image row up to roll)
                    vrow = v;
                    const uint8_t *p = yp + size_t(v) * size_t(stride);
                    uint32_t acc = 0;
                    psum_[0] = 0;
                    for (int k = 0; k < W; ++k) psum_[size_t(k) + 1] = (acc += p[k]);
                }
                const float hw = std::max(0.5f, 0.5f * n.wpx);              // half width of the centre box
                const float gap = std::max(2.0f, 1.5f * n.wpx + 1.0f);      // centre-to-side distance
                auto box = [&](float c) {
                    int a = int(std::floor(c - hw + 0.5f)), b = int(std::floor(c + hw + 0.5f));
                    if (b <= a) b = a + 1;
                    a = std::max(0, a);
                    b = std::min(W, b);
                    return b > a ? float(psum_[size_t(b)] - psum_[size_t(a)]) / float(b - a) : 0.f;
                };
                const float c = box(n.u), l = box(n.u - gap), r = box(n.u + gap);
                const float resp = std::min(c - l, c - r);
                const float lo = std::max(float(cfg_.r_lo), float(cfg_.rel_lo) * std::max(l, r));
                if (resp <= lo) continue;
                const float ev = std::min(1.0f, (resp - lo) * inv);
                float &dst = e_[size_t(i) * size_t(ny_) + size_t(j)];
                dst = std::max(dst, ev);
            }
        }
}

void LaneFinder::fill_evidence_map(const uint8_t *e, int ew, int eh) {
    std::fill(e_.begin(), e_.end(), 0.f);
    const float sx = float(ew) / float(cam_.width), sy = float(eh) / float(cam_.height);
    for (int i = 0; i < cfg_.nx; ++i)
        for (int s = 0; s < sub_; ++s) {
            const Node *row = &nodes_[(size_t(i) * size_t(sub_) + size_t(s)) * size_t(ny_)];
            for (int j = 0; j < ny_; ++j) {
                const Node &n = row[j];
                if (n.u < 0) continue;
                const int u = std::min(ew - 1, int(n.u * sx)), v = std::min(eh - 1, int(n.v * sy));
                const float ev = e[size_t(v) * size_t(ew) + size_t(u)] * (1.0f / 255.0f);
                float &dst = e_[size_t(i) * size_t(ny_) + size_t(j)];
                dst = std::max(dst, ev);
            }
        }
}

// The strongest evidence of row i within `win` m of y (and where it is).
double LaneFinder::eval_e(int i, double y, double win, double *ybest) const {
    const double jc = (y + cfg_.y_half) / cfg_.dy;
    const int w = std::max(1, int(win / cfg_.dy));
    const int j0 = std::max(0, int(std::lround(jc)) - w), j1 = std::min(ny_ - 1, int(std::lround(jc)) + w);
    const float *row = &e_[size_t(i) * size_t(ny_)];
    float best = 0;
    int bj = -1;
    for (int j = j0; j <= j1; ++j)
        if (row[j] > best) best = row[j], bj = j;
    if (bj < 0) return 0;
    // centre of the run of cells around the maximum (a mask from another detector is several cells wide)
    double sw = 0, sj = 0;
    for (int j = bj; j >= j0 && row[j] > 0.5f * best; --j) sw += row[j], sj += row[j] * j;
    for (int j = bj + 1; j <= j1 && row[j] > 0.5f * best; ++j) sw += row[j], sj += row[j] * j;
    *ybest = -cfg_.y_half + (sj / sw) * cfg_.dy;
    return best;
}

// Follow a line outwards from a seed and fit it. Returns false when it is not a line.
bool LaneFinder::follow(double p0, double p1, double p2, Cand *c) const {
    double p[3] = {p0, p1, p2};
    std::vector<double> x, y, w;
    const int nx = cfg_.nx;
    const double wc = 1.0 / (cfg_.curv_prior * cfg_.curv_prior) * 1e-2;   // against sum of w ~ rows
    for (int pass = 0; pass < 3; ++pass) {
        x.clear(), y.clear(), w.clear();
        // pass 0: the seed rows; later passes: all rows, each predicted by the fit so far
        const double xlim = pass == 0 ? cfg_.seed_x_max : 1e9;
        int gap_rows = 0;
        for (int i = 0; i < nx; ++i) {
            const double xi = xs_[size_t(i)];
            if (xi > xlim) break;
            double yb;
            const double ev = eval_e(i, poly(p, xi), pass == 0 ? cfg_.follow_win * 1.5 : cfg_.follow_win, &yb);
            if (ev > 0.15) {
                x.push_back(xi), y.push_back(yb), w.push_back(ev);
                gap_rows = 0;
            } else if (!x.empty() && ++gap_rows > nx / 3 && pass > 0) {
                break;                   // nothing for a third of the grid: the line has ended
            }
        }
        if (x.size() < 4) return false;
        if (!fit_quad(x, y, w, fam_c_, wc * double(x.size()), p)) return false;
    }
    double rms = 0, sw = 0;
    for (size_t k = 0; k < x.size(); ++k) {
        const double d = y[k] - poly(p, x[k]);
        rms += w[k] * d * d;
        sw += w[k];
    }
    rms = std::sqrt(rms / sw);
    const double span = x.back() - x.front();
    int rows_in = 0;
    for (int i = 0; i < nx; ++i)
        if (xs_[size_t(i)] >= x.front() && xs_[size_t(i)] <= x.back()) ++rows_in;
    c->p[0] = p[0], c->p[1] = p[1], c->p[2] = p[2];
    c->n = int(x.size());
    c->span = span;
    c->cover = sw / std::max(1, rows_in);
    c->rms = rms;
    c->far_x = x.back();
    const double slope = p[1] + 2 * p[2] * cfg_.x_ref;
    if (span < cfg_.min_span_m || c->cover < cfg_.min_cover || rms > cfg_.max_rms || std::fabs(slope) > cfg_.max_slope)
        return false;
    // Paint lies on a road that is plain on both sides. Rows of the span with evidence 0.45-1.3 m beside the line
    // (wheel tracks in snow, a wet road's reflections, cobbles, a hatched area) say this is texture, not a line.
    int side = 0;
    const int jn = int(0.45 / cfg_.dy), jf = int(1.3 / cfg_.dy);
    for (int i = 0; i < nx; ++i) {
        const double xi = xs_[size_t(i)];
        if (xi < x.front() || xi > x.back()) continue;
        const int jc = int(std::lround((poly(p, xi) + cfg_.y_half) / cfg_.dy));
        const float *row = &e_[size_t(i) * size_t(ny_)];
        bool hit = false;
        for (int j = jc - jf; j <= jc + jf && !hit; ++j) {
            if (j < 0 || j >= ny_ || std::abs(j - jc) < jn) continue;
            hit = row[j] > 0.3f;
        }
        side += hit;
    }
    c->clutter = double(side) / std::max(1, rows_in);
    if (c->clutter > cfg_.max_clutter) return false;
    c->q = std::min(1.0, std::max(0.0, (c->cover - cfg_.q_cover_lo) / (cfg_.q_cover_hi - cfg_.q_cover_lo)));
    return true;
}

void LaneFinder::process(const uint8_t *y, int stride, int64_t ts_ns, Result *out) {
    const auto t0 = std::chrono::steady_clock::now();
    if (grid_trim_ != trim_ || grid_yaw_ != yaw_trim_) build_grid();
    fill_evidence_luma(y, stride);
    analyse(ts_ns, out);
    out->ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void LaneFinder::process_evidence(const uint8_t *e, int ew, int eh, int64_t ts_ns, Result *out) {
    const auto t0 = std::chrono::steady_clock::now();
    if (grid_trim_ != trim_ || grid_yaw_ != yaw_trim_) build_grid();
    fill_evidence_map(e, ew, eh);
    analyse(ts_ns, out);
    out->ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void LaneFinder::analyse(int64_t ts_ns, Result *out) {
    const int nx = cfg_.nx;
    double dt = last_ts_ ? double(ts_ns - last_ts_) * 1e-9 : 0.0;
    if (dt < 0 || dt > 2.0) {
        reset();
        dt = 0;
    }
    last_ts_ = ts_ns;

    std::vector<Cand> cands;
    auto add = [&](const Cand &c) {
        const double yr = poly(c.p, cfg_.x_ref);
        for (Cand &o : cands) {
            if (std::fabs(poly(o.p, cfg_.x_ref) - yr) < 0.5) {
                if (c.cover * c.span > o.cover * o.span) o = c;
                return;
            }
        }
        cands.push_back(c);
    };
    // 1. the tracks of the last frame, each followed from where it was
    for (const Track &t : tracks_) {
        Cand c;
        if (follow(t.p[0], t.p[1], t.p[2], &c)) add(c);
    }
    // 2. new lines: the family of parallel lines y = a + b x + c x^2 that collects the most evidence on the
    //    near rows; its lateral profile's peaks are the seeds
    int n_seed = 0;
    while (n_seed < nx && xs_[size_t(n_seed)] <= cfg_.seed_x_max) ++n_seed;
    std::vector<float> prof(size_t(ny_), 0.f), best_prof(size_t(ny_), 0.f);
    double best = -1, bb = 0, bc = 0;
    for (int ib = -10; ib <= 10; ++ib)
        for (int ic = -6; ic <= 6; ++ic) {
            const double b = ib * 0.012, c = ic * 0.0012;
            std::fill(prof.begin(), prof.end(), 0.f);
            for (int i = 0; i < n_seed; ++i) {
                const double xi = xs_[size_t(i)];
                const int sh = int(std::lround((b * xi + c * xi * xi) / cfg_.dy));
                const float *row = &e_[size_t(i) * size_t(ny_)];
                const int j0 = std::max(0, -sh), j1 = std::min(ny_, ny_ - sh);
                for (int j = j0; j < j1; ++j) prof[size_t(j)] += row[j + sh];
            }
            double sc = 0;
            for (int j = 1; j + 1 < ny_; ++j) {      // 3-cell sum: tolerant of the coarse (b, c) steps
                const double v = prof[size_t(j - 1)] + prof[size_t(j)] + prof[size_t(j + 1)];
                sc += v * v;
            }
            sc *= 1.0 + 0.15 * std::exp(-0.5 * (std::pow((b - fam_b_) / 0.02, 2) + std::pow((c - fam_c_) / 0.002, 2)));
            if (sc > best) best = sc, bb = b, bc = c, best_prof = prof;
        }
    {
        std::vector<float> sm(size_t(ny_), 0.f);
        for (int j = 1; j + 1 < ny_; ++j)
            sm[size_t(j)] = best_prof[size_t(j - 1)] + best_prof[size_t(j)] + best_prof[size_t(j + 1)];
        const float thr = float(cfg_.seed_min * n_seed);
        const int nms = int(0.5 / cfg_.dy);
        for (int j = nms; j + nms < ny_; ++j) {
            if (sm[size_t(j)] < thr) continue;
            bool peak = true;
            for (int k = -nms; k <= nms && peak; ++k)
                if (sm[size_t(j + k)] > sm[size_t(j)] || (k < 0 && sm[size_t(j + k)] == sm[size_t(j)])) peak = false;
            if (!peak) continue;
            Cand c;
            if (follow(-cfg_.y_half + j * cfg_.dy, bb, bc, &c)) add(c);
        }
    }
    // Lane lines of one road run side by side at a lane's width. Lines 0.6-2 m apart are something else (wheel
    // tracks in snow, a tram rail next to a line, a hatched area, the joints of concrete slabs): the weaker of
    // such a pair loses its quality, two of similar strength both do. A double line (under 0.6 m) is one line.
    {
        std::vector<double> q0(cands.size());
        for (size_t a = 0; a < cands.size(); ++a) q0[a] = cands[a].cover * cands[a].span;
        for (size_t a = 0; a < cands.size(); ++a)
            for (size_t b = 0; b < cands.size(); ++b) {
                if (a == b) continue;
                const double d = std::fabs(poly(cands[a].p, cfg_.x_ref) - poly(cands[b].p, cfg_.x_ref));
                if (d >= 0.6 && d < cfg_.pair_min_m && q0[b] > cfg_.pair_ratio * q0[a]) cands[a].q = 0;
            }
    }
    // the frame's shared geometry (prior of the next frame's search and of weak lines' curvature)
    if (!cands.empty()) {
        double sw = 0, sb = 0, sc2 = 0;
        for (const Cand &c : cands) {
            const double w = c.cover * c.span;
            sw += w, sb += w * c.p[1], sc2 += w * c.p[2];
        }
        const double g = dt > 0 ? 1.0 - std::exp(-dt / 0.5) : 1.0;
        fam_b_ += g * (sb / sw - fam_b_);
        fam_c_ += g * (sc2 / sw - fam_c_);
    }

    // 3. tracks
    const double gconf = dt > 0 ? 1.0 - std::exp(-dt / cfg_.conf_tau_s) : 0.5;
    std::vector<char> used(cands.size(), 0);
    for (Track &t : tracks_) {
        const double yt = poly(t.p, cfg_.x_ref);
        int bi = -1;
        double bd = cfg_.assoc_m;
        for (size_t k = 0; k < cands.size(); ++k) {
            if (used[k]) continue;
            const double d = std::fabs(poly(cands[k].p, cfg_.x_ref) - yt);
            if (d < bd) bd = d, bi = int(k);
        }
        t.age_s += dt;
        if (bi >= 0) {
            used[size_t(bi)] = 1;
            const Cand &c = cands[size_t(bi)];
            for (int m = 0; m < 3; ++m) t.p[m] += cfg_.gain * (c.p[m] - t.p[m]);
            t.conf += gconf * (c.q - t.conf);
            t.miss_s = 0;
            // the reach falls slowly (the gaps of a dashed line come and go) and rises at once
            t.reach = std::max(c.far_x, t.reach - 15.0 * dt);
        } else {
            t.conf += gconf * (0.0 - t.conf);
            t.miss_s += dt;
        }
        if (!t.present && t.conf >= cfg_.on_conf) t.present = true;
        if (t.present && (t.conf < cfg_.off_conf || t.miss_s > cfg_.coast_s)) t.present = false;
    }
    for (size_t k = 0; k < cands.size(); ++k) {
        if (used[k]) continue;
        const Cand &c = cands[k];
        // not started: a line under the car, or one that points at the camera's foot (the streak of a lamp or of a
        // tail light on a wet road is a vertical image line: on the road plane it passes through x = 0, y = 0)
        if (std::fabs(poly(c.p, cfg_.x_ref)) < cfg_.under_car_m || std::fabs(c.p[0]) < cfg_.radial_m) continue;
        Track t;
        t.p[0] = c.p[0], t.p[1] = c.p[1], t.p[2] = c.p[2];
        t.conf = 0.35 * c.q;
        t.miss_s = 0, t.age_s = 0, t.reach = c.far_x, t.id = next_id_++, t.present = false;
        tracks_.push_back(t);
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track &t) { return !t.present && (t.conf < 0.03 || t.miss_s > cfg_.coast_s); }),
                  tracks_.end());
    // two tracks that have converged on one marking: the older one stays
    for (size_t a = 0; a < tracks_.size(); ++a)
        for (size_t b = a + 1; b < tracks_.size();) {
            if (std::fabs(poly(tracks_[a].p, cfg_.x_ref) - poly(tracks_[b].p, cfg_.x_ref)) < 0.45) {
                if (tracks_[b].present && !tracks_[a].present) std::swap(tracks_[a], tracks_[b]);
                tracks_.erase(tracks_.begin() + long(b));
            } else {
                ++b;
            }
        }

    // 4. pitch trim: lines of one road are parallel. With the pitch off by eps a line at lateral offset Y is
    //    seen as y(x) = Y - (eps / h) Y x: the slope of the present lines against their offset gives eps.
    if (cfg_.trim && dt > 0) {
        double n = 0, sa = 0, sb = 0, saa = 0, sab = 0;
        for (const Track &t : tracks_) {
            if (!t.present || t.conf < 0.6 || t.reach < 20) continue;
            n += 1, sa += t.p[0], sb += t.p[1], saa += t.p[0] * t.p[0], sab += t.p[0] * t.p[1];
        }
        const double var = n >= 2 ? saa - sa * sa / n : 0;
        if (var > 3.0) {                 // two lines at least 2.5 m apart
            const double kappa = (sab - sa * sb / n) / var;
            const double eps = kappa * cam_.height_m;
            const double g = 1.0 - std::exp(-dt / cfg_.trim_tau_s);
            trim_want_ = std::max(-cfg_.trim_max, std::min(cfg_.trim_max, trim_want_ + cfg_.trim_sign * g * std::max(-0.02, std::min(0.02, eps))));
            if (std::fabs(trim_want_ - trim_) >= cfg_.trim_step) trim_ = trim_want_;   // the grid is rebuilt only for a real change
        }
        // yaw trim: on a straight road the car drives along the lines, so their mean slope near the car is the
        // camera's yaw error. Slow (the car changes lanes and the road bends), and only while the lines are straight.
        double sn = 0, ss = 0;
        for (const Track &t : tracks_) {
            if (!t.present || t.conf < 0.6 || std::fabs(t.p[2]) > 0.0008) continue;
            sn += 1, ss += t.p[1] + 2 * t.p[2] * cfg_.x_ref;
        }
        if (sn >= 1) {
            const double g = 1.0 - std::exp(-dt / cfg_.yaw_tau_s);
            yaw_want_ = std::max(-cfg_.trim_max, std::min(cfg_.trim_max, yaw_want_ + g * std::max(-0.03, std::min(0.03, ss / sn))));
            if (std::fabs(yaw_want_ - yaw_trim_) >= 0.001) yaw_trim_ = yaw_want_;
        }
    }

    // 5. slots. The ego lane's lines are the nearest present line on each side within `inner_max_m` of the car's
    //    axis; an outer line is one a lane's width beyond its inner line (or, without an inner line, one beyond
    //    inner_max_m). A slot is never filled by promoting a farther line when its own line drops out: the
    //    picture loses a line, it does not jump by a lane.
    const Track *slot[4] = {nullptr, nullptr, nullptr, nullptr};
    {
        double yl = -1e9, yr = 1e9;
        for (const Track &t : tracks_) {
            if (!t.present) continue;
            const double y = poly(t.p, cfg_.x_ref);
            if (y < 0 && y > yl && y >= -cfg_.inner_max_m) yl = y, slot[1] = &t;
            if (y >= 0 && y < yr && y <= cfg_.inner_max_m) yr = y, slot[2] = &t;
        }
        if (slot[1] && slot[2] && yr - yl < 2.0) {       // an implausibly narrow "lane": the stronger line stays
            if (slot[1]->conf < slot[2]->conf) slot[1] = nullptr, yl = -1e9;
            else slot[2] = nullptr, yr = 1e9;
        }
        double bl = 1e9, br = 1e9;
        for (const Track &t : tracks_) {
            if (!t.present || &t == slot[1] || &t == slot[2]) continue;
            const double y = poly(t.p, cfg_.x_ref);
            if (y < 0) {
                const double lo = slot[1] ? yl - cfg_.lane_max_m : -cfg_.inner_max_m - cfg_.lane_max_m;
                const double hi = slot[1] ? yl - cfg_.lane_min_m : -cfg_.inner_max_m;
                if (y >= lo && y <= hi && hi - y < bl) bl = hi - y, slot[0] = &t;
            } else {
                const double lo = slot[2] ? yr + cfg_.lane_min_m : cfg_.inner_max_m;
                const double hi = slot[2] ? yr + cfg_.lane_max_m : cfg_.inner_max_m + cfg_.lane_max_m;
                if (y >= lo && y <= hi && y - lo < br) br = y - lo, slot[3] = &t;
            }
        }
    }
    out->ts_mono_ns = ts_ns;
    for (int k = 0; k < 4; ++k) {
        LineOut &lo = out->lines[k];
        lo = LineOut();
        if (!slot[k]) continue;
        lo.present = true;
        lo.conf = float(slot[k]->conf);
        lo.reach_m = float(std::max(cfg_.reach_min_m, slot[k]->reach));
        lo.id = slot[k]->id;
        for (int m = 0; m < 3; ++m) lo.p[m] = slot[k]->p[m];
    }
    out->pitch_trim = trim_;
    out->yaw_trim = yaw_trim_;
    out->family_b = bb;
    out->family_c = bc;
    out->candidates = int(cands.size());
    out->tracks = int(tracks_.size());
}

std::string LaneFinder::json(const Result &r, uint64_t seq, int64_t ts_real_ns, const char *source) {
    std::string s;
    char buf[160];
    snprintf(buf, sizeof buf, "{\"type\":\"display_lanes\",\"seq\":%llu,\"ts_mono_ns\":%lld,\"ts_real_ns\":%lld,",
             (unsigned long long)seq, (long long)r.ts_mono_ns, (long long)ts_real_ns);
    s += buf;
    s += "\"source\":\"";
    s += source;
    s += "\",\"x\":[";
    for (size_t k = 0; k < kOutX.size(); ++k) {
        snprintf(buf, sizeof buf, "%s%g", k ? "," : "", kOutX[k]);
        s += buf;
    }
    s += "],\"lines\":[";
    for (int k = 0; k < 4; ++k) {
        const LineOut &l = r.lines[k];
        if (k) s += ",";
        if (!l.present) {
            s += "{\"present\":false}";
            continue;
        }
        snprintf(buf, sizeof buf, "{\"present\":true,\"conf\":%.2f,\"reach_m\":%.1f,\"id\":%d,\"y\":[", l.conf, l.reach_m, l.id);
        s += buf;
        for (size_t i = 0; i < kOutX.size(); ++i) {
            snprintf(buf, sizeof buf, "%s%.3f", i ? "," : "", poly(l.p, kOutX[i]));
            s += buf;
        }
        s += "],\"y_std\":[";
        for (size_t i = 0; i < kOutX.size(); ++i) {
            // beyond the line's reach the value is an extrapolation: a sigma over the display's limit
            snprintf(buf, sizeof buf, "%s%.2f", i ? "," : "", kOutX[i] <= l.reach_m ? 0.08 + 0.004 * kOutX[i] : 9.0);
            s += buf;
        }
        s += "]}";
    }
    snprintf(buf, sizeof buf, "],\"pitch_trim\":%.4f,\"yaw_trim\":%.4f,\"analysis_ms\":%.2f}", r.pitch_trim, r.yaw_trim, r.ms);
    s += buf;
    return s;
}

}  // namespace laned
}  // namespace roadsense

// laned: lane lines for the DISPLAY from the luma plane of a camera frame, classical image analysis
// (docs/lanes/L29-display-lanes.md). No network, no GPU, no dependency: a few thousand samples per frame.
//
//   frame (Y plane) ── marking filter at the nodes of a road-plane grid ──> evidence E[x][y]
//   E ── seeds (tracks of the last frame first, then a search over parallel line families)
//     ── line following + weighted quadratic fit per line ──> candidate lines of the frame
//     ── tracks (association, hysteresis, a slow pitch trim that makes the lines parallel)
//     ── slots (far left, left, right, far right) ──> the `display_lanes` message
//
// The marking filter: a paint line is a stripe of about `mark_w` m that is brighter than the road on BOTH
// sides. At a road-plane node the stripe is `f * mark_w / distance` pixels wide; the response is
// min(centre - left, centre - right) of three box means on one image row (row prefix sums). An edge between
// a bright and a dark area (a kerb in the sun, a vehicle's side, a shadow) has one bright side and gives no
// response; a wide bright area (a zebra bar, an arrow's body, a white car) has two.
//
// Output frame: the calibrated device frame of the road model's lines (x forward from the camera, y right,
// flat road `height_m` below the camera), so a consumer draws these lines next to the model's objects.
// Display only: nothing here is a measurement and no measurement topic reads it.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace roadsense {
namespace laned {

struct Camera {
    double f = 975, cx = 640, cy = 360;
    int width = 1280, height = 720;
    double roll = 0, pitch = 0, yaw = 0, height_m = 1.22;   // calib.json (openpilot euler, rad)
};

struct Config {
    // road-plane grid
    double x_min = 4.0, x_max = 60.0;   // m ahead; rows are spaced geometrically
    int nx = 44;
    double y_half = 7.0, dy = 0.08;     // lateral extent and cell, m
    int ignore_bottom = 0;              // frame rows at the bottom that are not road (bonnet, burned-in text)
    // marking filter
    double mark_w = 0.15;               // nominal paint width, m
    double r_lo = 6.0, r_hi = 20.0;     // response (luma steps) mapped to evidence 0..1
    double rel_lo = 0.06;               // ... and at least this fraction of the local brightness
    // candidate lines of one frame
    double seed_x_max = 26.0;           // the family search uses rows up to here
    double seed_min = 0.10;             // a seed needs this mean evidence over the seed rows
    double follow_win = 0.32;           // m: a row's evidence counts within this of the predicted line
    double min_span_m = 9.0;            // a line's supported rows must span this much
    double min_cover = 0.10;            // ... and cover this fraction of the rows inside its span
    double max_rms = 0.14;              // m: residual of the supported rows about the fit
    double max_slope = 0.25;            // |dy/dx| at the reference distance
    double max_clutter = 1.0;           // share of a line's rows with other evidence 0.45-1.3 m beside it
    double curv_prior = 0.004;          // 1/m: sigma of a line's curvature about the frame's shared one
    // tracks
    double x_ref = 8.0;                 // m: lines are identified by their lateral offset here
    double assoc_m = 0.6;               // association gate at x_ref
    double gain = 0.4;                  // per-frame gain of a track's geometry towards the frame's line
    double conf_tau_s = 0.6, on_conf = 0.55, off_conf = 0.2;
    double coast_s = 1.0;               // a present line without a frame's line for this long is dropped
    double q_cover_lo = 0.10, q_cover_hi = 0.26;   // cover -> a frame's quality 0..1
    // slots
    double lane_min_m = 2.3, lane_max_m = 5.2;     // plausible lane width for an outer line
    double inner_max_m = 3.2;           // an ego-lane line is at most this far from the car's axis
    double pair_min_m = 2.0, pair_ratio = 0.5;     // lines nearer than this to each other are not lane lines
    double under_car_m = 0.9;           // a NEW line nearer than this to the car's axis is not started
    double radial_m = 0.6;              // ... nor one that passes this near the camera's foot point
    double reach_min_m = 15.0;
    // pitch trim (lines of one road are parallel) and yaw trim (the car drives along them)
    double trim_tau_s = 4.0, trim_max = 0.06, trim_sign = 1.0, yaw_tau_s = 15.0, trim_step = 0.002;
    bool trim = true;
};

// Sets the Config field named `key` (the names above); false for an unknown key.
bool config_set(Config *c, const std::string &key, double v);

struct LineOut {
    bool present = false;
    float conf = 0, reach_m = 0;
    int id = 0;
    double p[3] = {0, 0, 0};            // y(x) = p0 + p1 x + p2 x^2
};

struct Result {
    int64_t ts_mono_ns = 0;
    LineOut lines[4];                    // far left, left, right, far right
    double pitch_trim = 0, yaw_trim = 0, family_b = 0, family_c = 0;
    int candidates = 0, tracks = 0;
    double ms = 0;
};

class LaneFinder {
public:
    LaneFinder(const Config &cfg, const Camera &cam);
    void reset();
    void set_camera(const Camera &cam);
    // Luma plane of the frame. ts in ns (frame time).
    void process(const uint8_t *y, int stride, int64_t ts_ns, Result *out);
    // Instead of the marking filter: an evidence image (0..255, any size, covering the whole frame), from
    // another detector (the candidate comparison of lane L29; not used by the daemon).
    void process_evidence(const uint8_t *e, int ew, int eh, int64_t ts_ns, Result *out);
    // The evidence grid of the last frame (nx rows of ny values 0..1), row distances, for inspection.
    const std::vector<float> &evidence() const { return e_; }
    const std::vector<double> &rows_x() const { return xs_; }
    int ny() const { return ny_; }
    const Config &config() const { return cfg_; }

    static const std::vector<double> &out_x();   // distances of the message grid
    // The `display_lanes` line of a result (no trailing newline).
    static std::string json(const Result &r, uint64_t seq, int64_t ts_real_ns, const char *source);

private:
    struct Node { float u, v, wpx; };
    struct Cand { double p[3]; double cover, span, rms, q, far_x, clutter; int n; };
    struct Track { double p[3]; double conf, miss_s, age_s, reach; int id; bool present; };

    void build_grid();
    void fill_evidence_luma(const uint8_t *y, int stride);
    void fill_evidence_map(const uint8_t *e, int ew, int eh);
    void analyse(int64_t ts_ns, Result *out);
    bool follow(double p0, double p1, double p2, Cand *c) const;
    double eval_e(int i, double y, double win, double *ybest) const;

    Config cfg_;
    Camera cam_;
    int ny_ = 0, sub_ = 3;
    double trim_ = 0, trim_want_ = 0, grid_trim_ = 1e9;
    double yaw_trim_ = 0, yaw_want_ = 0, grid_yaw_ = 1e9;
    std::vector<double> xs_;             // row distances
    std::vector<Node> nodes_;            // nx * sub * ny
    std::vector<float> e_;               // nx * ny
    std::vector<uint32_t> psum_;
    std::vector<Track> tracks_;
    int64_t last_ts_ = 0;
    int next_id_ = 1;
    double fam_b_ = 0, fam_c_ = 0;
};

}  // namespace laned
}  // namespace roadsense

// Shared modeld pipeline pieces (daemon and tools/modeld_replay use the same code):
// config/calibration loading, frame selection for the 20 Hz model clock, and NV12 -> packed
// model frames for both inputs.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "supercombo.h"
#include "warp.h"

namespace pocopilot {

// Directory of calib.json / modeld.json: POCOPILOT_CONFIG_DIR or /etc/pocopilot.
std::string config_dir();
// Reads {"pitch":..,"yaw":..,"roll":..,"height_m":..} (radians, metres). Missing file or keys keep the
// defaults (0, 0, 0, 1.22). Returns true if the file was read.
bool load_calib(const std::string &path, Calib *c, std::string *err = nullptr);
// Minimal flat-JSON readers (number / string / bool value of a top-level key).
bool json_number(const std::string &s, const char *key, double *v);
bool json_string(const std::string &s, const char *key, std::string *v);
bool read_file(const std::string &path, std::string *out);

// Chooses which camera frames become model frames. openpilot pairs frames 50 ms apart (20 Hz).
// If the stream runs at <= ~20 fps every frame is used; faster streams use the newest frame that
// is at least 45 ms after the previously selected one.
class FrameSelector {
public:
    bool accept(int64_t ts_ns);
    double stream_dt_ms() const { return dt_ema_ms_; }
    void reset() { last_sel_ = last_any_ = -1, dt_ema_ms_ = 0; }

private:
    int64_t last_sel_ = -1, last_any_ = -1;
    double dt_ema_ms_ = 0;
};

// One NV12 frame as handed to the packer.
struct NV12Frame {
    uint64_t seq = 0;
    int64_t ts_mono_ns = 0, ts_real_ns = 0;
    const uint8_t *y = nullptr, *uv = nullptr;
    int y_stride = 0, uv_stride = 0;
    Intrinsics k;  // focal/cx/cy at the frame size
};

// One warped+packed frame for both inputs.
struct PackedFrame {
    uint64_t seq = 0;
    int64_t ts_mono_ns = 0, ts_real_ns = 0;
    double prep_ms = 0;
    Calib calib;
    Intrinsics k;
    std::vector<float> med, big;  // sc::kFramePacked each
};

class Preprocessor {
public:
    Preprocessor() : med_(false), big_(true) {}
    void pack(const NV12Frame &f, const Calib &c, PackedFrame *out);
    const FramePacker &med() const { return med_; }
    const FramePacker &big() const { return big_; }

private:
    FramePacker med_, big_;
};

// `calib` JSON line (docs/ARCHITECTURE.md).
std::string calib_json(const Calib &c, const Intrinsics &k, uint64_t seq, const char *source);

int64_t mono_ns();
int64_t real_ns();

}  // namespace pocopilot

// Online estimate of the camera's mounting pitch and yaw from the road model's own ego-motion (lane L32).
//
// The model's frames are warped with the calibration (calib.json: device_from_calib Euler angles). Its `pose`
// output is the camera's translation in that calibrated frame, so while the car drives straight the direction
// of the translation is the calibration's remaining error:
//     yaw_true   = yaw_used   + atan2(vy, vx)
//     pitch_true = pitch_used - atan2(vz, vx)
// (openpilot selfdrive/locationd/calibrationd.py, handle_cam_odom: the same observation and the same gates).
// Measured for both road models (docs/lanes/L32-lead-lateral.md): a calibration yaw changed by 1 deg moves the
// direction of `pose` by 0.9-1.0 deg the other way.
//
// Why it matters: the v0.11.1 vision model's lane lines and road edges turn with the calibration's yaw (they
// are geometry of the warped image), its lead and the policy's plan do not (the model refers them to its own
// motion). A yaw error therefore puts the lead beside its lane by that angle. Roll and the height are not
// observable this way and stay as configured.
//
// Method: accepted samples (fast, straight, the model sure of its lateral speed) are absolute observations
// (used + residual); `block` of them give one block median; the estimate is the median of the last `max_blocks`
// block medians (when the newest blocks leave it clearly, the camera was moved and the older ones are dropped).
// It is valid from `min_blocks` blocks on while the blocks agree (standard error of the median under `max_se`,
// robust sigma under `max_spread`) and it stays within `max_dev` of the configured value. A valid estimate replaces the configured
// pitch / yaw in the warp when `apply` is set, in steps (only when it left the applied value by `step`).
// Everything is reported (`calib` message, `online` object): nothing is corrected silently.
#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "warp.h"

namespace roadsense {

struct OnlineCalibOptions {
    bool enabled = true;        // estimate at all
    bool apply = true;          // use a valid estimate for the warp (false: report only)
    int block = 100;            // accepted samples per block (openpilot BLOCK_SIZE)
    int min_blocks = 5;         // blocks for a valid estimate (openpilot INPUTS_NEEDED)
    int max_blocks = 50;        // blocks kept (openpilot INPUTS_WANTED)
    double min_speed = 6.7;     // m/s (openpilot MIN_SPEED_FILTER, 15 mph)
    // rad/s: 1 deg/s. openpilot's MAX_YAW_RATE_FILTER is 2 deg/s; the direction of `pose` leans into a turn by
    // 0.1 (car, supercombo) to 0.65 deg (a1-night, vision) per deg/s, so gentle motorway curves are kept out
    double max_yaw_rate = 0.01745;
    // rad: atan2(vy_std, vx) <= 0.5 deg. openpilot's MAX_VEL_ANGLE_STD is 0.25 deg; on the car's recordings
    // (supercombo, camera 3.4 deg off) the model's own figure is 0.3-0.5 deg and 0.25 passes 0-15 % of the straight
    // runs, 0.5 passes about 90 %, with the same estimate (docs/lanes/L32-lead-lateral.md)
    double max_angle_std = 0.00873;
    // Validity, per angle, from the blocks' robust sigma s and their number n: the standard error of the median,
    // 1.2533 s / sqrt(n), must be <= max_se to become valid (0.3 deg) and <= max_se_keep to stay valid (0.45 deg:
    // no toggling at the limit), and s itself <= max_spread (1.5 deg: blocks that scatter more are not one camera
    // position). With 5 blocks that is s <= 0.53 deg; a town ride whose blocks scatter by 0.9 deg needs 14.
    double max_se = 0.00524, max_se_keep = 0.00785;
    double max_spread = 0.0262;
    double max_dev = 0.15;          // rad: |estimate - configured|, per angle; beyond it the estimate is refused
    // rad: the applied value follows the estimate when it is this far off (0.25 deg). Every change of the warp
    // costs one frame pair (0.2 s without a `road` message for the vision model): few, visible steps
    double step = 0.00436;
    double moved = 0.0262;          // rad: the last min_blocks blocks this far (and 4 sigma) from the estimate = the
                                    // camera was moved: older blocks are dropped (1.5 deg)
    std::string state_path;         // "" = the estimate is not kept between starts
};

struct OnlineCalibStatus {
    // off: not estimating; collecting: fewer than min_blocks blocks; valid; unstable: the blocks disagree;
    // out_of_range: the estimate is further than max_dev from the configured value
    const char *state = "off";
    bool applied = false;        // the warp uses the estimate (else the configured value)
    bool have = false;           // pitch / yaw below hold an estimate (>= 1 block)
    double pitch = 0, yaw = 0;   // the estimate, rad
    double spread_pitch = 0, spread_yaw = 0;  // robust sigma of the block medians, rad
    double se_pitch = 0, se_yaw = 0;          // standard error of the estimate (1.2533 sigma / sqrt(blocks)), rad
    int blocks = 0, samples = 0; // blocks kept; samples in the open block
    uint64_t accepted = 0, seen = 0;
    double config_pitch = 0, config_yaw = 0;
    bool restored = false;       // blocks were read from the state file
};

// Thread-safe: the frame-prep thread calls set_config() / calib(), the model thread observe() / status().
class OnlineCalib {
public:
    explicit OnlineCalib(const OnlineCalibOptions &o = OnlineCalibOptions()) : o_(o) {}
    // The configured calibration and the camera it belongs to. A change (another calib.json, another camera or
    // frame size) drops every block: the estimate belongs to one mounting. Reads the state file when it was
    // written for exactly this configuration.
    void set_config(const Calib &config, const Intrinsics &k);
    // The calibration the warp uses now.
    Calib calib() const;
    // One model run on a frame pair warped with `used` (both frames; the caller skips mixed pairs).
    // pose: vx, vy, vz (m/s), roll/pitch/yaw rate (rad/s); pose_std alike. Returns true when the status changed
    // in a way worth publishing (a block ended, the applied value moved).
    bool observe(const Calib &used, const float pose[6], const float pose_std[6]);
    OnlineCalibStatus status() const;
    // `"online":{...}` for the `calib` message.
    std::string status_json() const;

private:
    struct Block { double pitch, yaw; };
    void reset_locked();
    void update_locked();
    void save_locked() const;
    bool load_locked();
    OnlineCalibStatus status_locked() const;

    OnlineCalibOptions o_;
    mutable std::mutex m_;
    bool have_config_ = false;
    Calib config_, applied_;
    Intrinsics k_;
    std::deque<Block> blocks_;
    std::vector<double> cur_pitch_, cur_yaw_;
    bool have_ = false, valid_ = false, unstable_ = false, out_of_range_ = false, restored_ = false;
    double est_pitch_ = 0, est_yaw_ = 0, sp_pitch_ = 0, sp_yaw_ = 0;
    uint64_t accepted_ = 0, seen_ = 0;
};

// `calib` JSON line with the online estimate's state: source "auto" when the warp uses the estimate.
std::string calib_json(const Calib &c, const Intrinsics &k, uint64_t seq, const OnlineCalib &oc);

}  // namespace roadsense

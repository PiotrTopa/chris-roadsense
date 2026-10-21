// openpilot v0.9.7 supercombo interface constants.
//
// Every number in this file is taken from docs/SUPERCOMBO_SPEC.md (which was
// read from openpilot v0.9.7 selfdrive/modeld/constants.py,
// parse_model_outputs.py, common/transformations/model.py and from the ONNX
// graph itself). Section references are to that spec.
#pragma once

#include <cstddef>

namespace pocopilot::sc {

// ---- inputs (spec section 1) -----------------------------------------------
constexpr int kImgC = 12, kImgH = 128, kImgW = 256;               // input_imgs / big_input_imgs [1,12,128,256]
constexpr size_t kFramePacked = size_t(6) * kImgH * kImgW;       // 196,608 floats per frame (6 channels), 2.3
constexpr size_t kImgInput = 2 * kFramePacked;                   // 393,216: [prev frame | current frame], 2.3
constexpr int kModelW = 512, kModelH = 256;                      // warped Y plane size (MEDMODEL_INPUT_SIZE), 2.2
constexpr int kModelUVW = 256, kModelUVH = 128;                  // warped U, V size, 2.2
constexpr int kDesireLen = 100, kDesireW = 8;                    // desire [1,100,8], newest row 99, 1.1
constexpr int kTrafficConventionLen = 2;                         // [1,0] = right-hand traffic (LHD car), [0,1] = left-hand traffic, 1 table
constexpr int kLatParamsLen = 2;                                 // [v_ego m/s, steer_delay s], 1.1
constexpr int kPrevCurvLen = 100;                                // prev_desired_curv [1,100,1], newest row 99, 1.1
constexpr int kFeatLen = 99, kFeatW = 512;                       // features_buffer [1,99,512], newest row 98, 1.1
constexpr const char *kInputNames[7] = {"input_imgs", "big_input_imgs", "desire", "traffic_convention",
                                        "lateral_control_params", "prev_desired_curv", "features_buffer"};
constexpr const char *kOutputName = "outputs";
constexpr int kOutputLen = 6504;                                 // outputs [1,6504], 3

// Steer delay: lateral_control_params[1] = CP.steerActuatorDelay + 0.2 (1.1). No car here:
// 0.2 s nominal actuator delay + 0.2 s = 0.4 s (spec suggests 0.3..0.5 s).
constexpr float kDefaultSteerDelay = 0.4f;

// ---- model intrinsics / warp (spec 2.4) --------------------------------------
// calib_from_medmodel = inv(medmodel_intrinsics @ view_from_device)[:3,:3], row-major.
constexpr double kCalibFromMedmodel[9] = {0.0, 0.0, 1.0,
                                          1.0 / 910.0, 0.0, -256.0 / 910.0,
                                          0.0, 1.0 / 910.0, -47.6 / 910.0};
// calib_from_sbigmodel (fl 455, cx 256, cy 151.8).
constexpr double kCalibFromSbigmodel[9] = {0.0, 0.0, 1.0,
                                           1.0 / 455.0, 0.0, -256.0 / 455.0,
                                           0.0, 1.0 / 455.0, -151.8 / 455.0};
// view_from_device (device x fwd, y right, z down -> camera x right, y down, z fwd).
constexpr double kViewFromDevice[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};
constexpr double kDefaultCameraHeight = 1.22;                    // HEIGHT_INIT, 2.4 (not used by the warp)

// ---- output slices, absolute indices (spec 3.1 / 3.2) ------------------------
constexpr int kIdxN = 33;                                        // IDX_N

// plan: 5 hypotheses x 991 = mean 495 (33x15) | log-std 495 | 1 weight logit.
constexpr int kPlanStart = 0, kPlanMhpN = 5, kPlanHypLen = 991, kPlanWidth = 15;
constexpr int kPlanStdOff = 495, kPlanWeightOff = 990;
// plan columns (3.3)
constexpr int kPlanPos = 0, kPlanVel = 3, kPlanAcc = 6, kPlanEuler = 9, kPlanRate = 12;

// lane lines: 4 x 33 x (y, z), mean 4955..5218, log-std 5219..5482. Order far-left, left, right, far-right.
constexpr int kLaneLinesMean = 4955, kLaneLinesStd = 5219, kNumLaneLines = 4, kLaneLineStride = 66;
// lane line presence: prob of line i = sigmoid(raw[5483 + 2i + 1]) (even entries unused), 3.3
constexpr int kLaneLinesProb = 5483;
// road edges: 2 x 33 x (y, z), mean 5491..5622, log-std 5623..5754. Order left, right.
constexpr int kRoadEdgesMean = 5491, kRoadEdgesStd = 5623, kNumRoadEdges = 2;

// lead: 2 hypotheses x 51 = mean 24 (6x4: x, y, v, a) | log-std 24 | 3 weight logits (one per output lead).
constexpr int kLeadStart = 5755, kLeadMhpN = 2, kLeadHypLen = 51, kLeadStdOff = 24, kLeadWeightOff = 48;
constexpr int kLeadSelection = 3, kLeadTrajLen = 6, kLeadWidth = 4;
constexpr int kLeadProb = 5857;                                  // 3 logits, sigmoid
constexpr int kDesireState = 5860;                               // 8, softmax
constexpr int kMeta = 5868, kMetaLen = 48;                       // 48, sigmoid each
constexpr int kDesirePred = 5916, kDesirePredLen = 4;            // 4 x 8, softmax per row
constexpr int kPoseMean = 5948, kPoseStd = 5954;                 // 6: vx vy vz (m/s), roll/pitch/yaw rate (rad/s)
constexpr int kWideFromDeviceEulerMean = 5960, kWideFromDeviceEulerStd = 5963;
constexpr int kSimPoseMean = 5966, kSimPoseStd = 5972;
constexpr int kRoadTransformMean = 5978, kRoadTransformStd = 5984;  // [2] = camera height above road (m)
constexpr int kDesiredCurvMean = 5990, kDesiredCurvStd = 5991;   // 1/m, already scaled in-graph
constexpr int kHiddenState = 5992, kHiddenLen = 512;             // raw, feeds features_buffer

// meta (3.3): 0 engagedProb; for k = 0..4 (t = 2,4,6,8,10 s) base = 1 + 7k:
// +0 gas disengage, +1 brake disengage, +2 steer override, +3 brake 3 m/s^2, +4 brake 4 m/s^2,
// +5 brake 5 m/s^2, +6 gas press. Blinkers for t = 0..10 s step 2: left = 36 + 2j, right = 37 + 2j.
constexpr int kMetaEngaged = 0, kMetaDisengageBase = 1, kMetaDisengageStride = 7, kMetaDisengageN = 5;
constexpr int kMetaBlinkerBase = 36, kMetaBlinkerN = 6;

// ---- grids (3.3): index_function(idx, max_val) = max_val * (idx/32)^2 --------
constexpr float T_IDXS[kIdxN] = {
    0.0f, 0.009765625f, 0.0390625f, 0.087890625f, 0.15625f, 0.244140625f, 0.3515625f, 0.478515625f,
    0.625f, 0.791015625f, 0.9765625f, 1.181640625f, 1.40625f, 1.650390625f, 1.9140625f, 2.197265625f,
    2.5f, 2.822265625f, 3.1640625f, 3.525390625f, 3.90625f, 4.306640625f, 4.7265625f, 5.166015625f,
    5.625f, 6.103515625f, 6.6015625f, 7.119140625f, 7.65625f, 8.212890625f, 8.7890625f, 9.384765625f, 10.0f};
constexpr float X_IDXS[kIdxN] = {
    0.0f, 0.1875f, 0.75f, 1.6875f, 3.0f, 4.6875f, 6.75f, 9.1875f, 12.0f, 15.1875f, 18.75f, 22.6875f, 27.0f,
    31.6875f, 36.75f, 42.1875f, 48.0f, 54.1875f, 60.75f, 67.6875f, 75.0f, 82.6875f, 90.75f, 99.1875f, 108.0f,
    117.1875f, 126.75f, 136.6875f, 147.0f, 157.6875f, 168.75f, 180.1875f, 192.0f};
constexpr float LEAD_T_IDXS[kLeadTrajLen] = {0.f, 2.f, 4.f, 6.f, 8.f, 10.f};
constexpr float LEAD_T_OFFSETS[kLeadSelection] = {0.f, 2.f, 4.f};
constexpr float META_T_IDXS[kMetaDisengageN] = {2.f, 4.f, 6.f, 8.f, 10.f};

// Model clock: 20 Hz, image pair 50 ms apart (1.1 "Timing assumption").
constexpr double kModelPeriodS = 0.05;

}  // namespace pocopilot::sc

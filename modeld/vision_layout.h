// openpilot v0.11.1 driving_vision / driving_policy interface constants.
//
// Read from the ONNX files of the v0.11.1 release (metadata `output_slices`, input shapes) and from
// v0.11.1 selfdrive/modeld/{constants.py, parse_model_outputs.py, compile_modeld.py, modeld.py};
// table in docs/lanes/L21-budget-and-models.md section 5 and docs/lanes/L24-M-vision-model.md.
// Grids (T_IDXS, X_IDXS, lead times) are those of v0.9.7 (supercombo_layout.h).
#pragma once

#include <cstddef>

namespace roadsense::vis {

// ---- vision inputs: img, big_img uint8 [1,12,128,256] = [frame 200 ms earlier | current frame], the
// six-channel YUV packing of v0.9.7 (pack_yuv6), nearest-neighbour warp (WarpKind::kNearestV011).
constexpr const char *kInputNames[2] = {"img", "big_img"};
constexpr const char *kOutputName = "outputs";
constexpr int kOutputLen = 1576;
constexpr double kPairDtS = 0.2;  // MODEL_RUN_FREQ 20 / MODEL_CONTEXT_FREQ 5 -> frames 4 model ticks apart

// ---- vision output slices (absolute indices into the 1576 values) ------------------------------
constexpr int kMeta = 0, kMetaLen = 55;  // sigmoid each, layout below
constexpr int kDesirePred = 55;          // 4 x 8, softmax per row
constexpr int kPoseMean = 87, kPoseStd = 93;
constexpr int kWideFromDeviceEulerMean = 99, kWideFromDeviceEulerStd = 102;
constexpr int kRoadTransformMean = 105, kRoadTransformStd = 111;
constexpr int kLaneLinesMean = 117, kLaneLinesStd = 381;  // 4 x 33 x (y, z)
constexpr int kLaneLinesProb = 645;                       // line i = sigmoid(raw[645 + 2 i + 1])
constexpr int kRoadEdgesMean = 653, kRoadEdgesStd = 785;  // 2 x 33 x (y, z)
constexpr int kLeadMean = 917, kLeadStd = 989;            // 3 leads (t offset 0, 2, 4 s) x 6 times x (x, y, v, a)
constexpr int kLeadProb = 1061;                           // 3, sigmoid
constexpr int kHiddenState = 1064, kHiddenLen = 512;      // the policy's features

// meta (constants.py class Meta): 0 engaged; for k = 0..4 (t = 2..10 s): 1 + 6k + {0 gas disengage,
// 1 brake disengage, 2 steer override, 3 hard brake 3, 4 hard brake 4, 5 hard brake 5};
// for j = 0..5 (t = 0..10 s): 31 + 4j + {0 gas press, 1 brake press, 2 left blinker, 3 right blinker}.
constexpr int kMetaDisengageBase = 1, kMetaDisengageStride = 6;
constexpr int kMetaPressBase = 31, kMetaPressStride = 4;

// ---- policy (driving_policy-fp32.onnx) ------------------------------------------------------------
constexpr const char *kPolicyInputNames[3] = {"desire_pulse", "traffic_convention", "features_buffer"};
constexpr int kPolicyHist = 25;            // features_buffer / desire_pulse rows, 5 Hz, newest last
constexpr double kPolicyHistDtS = 0.2;     // spacing of the rows (frame_skip 4 at 20 Hz)
constexpr int kPolicyOutputLen = 1000;
constexpr int kPlanMean = 0, kPlanStd = 495, kPlanWidth = 15;  // 33 x 15, single hypothesis (no MHP)
constexpr int kDesireState = 990;                              // 8, softmax

}  // namespace roadsense::vis

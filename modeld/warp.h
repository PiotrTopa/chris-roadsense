// NV12 -> supercombo 6-channel input: perspective warp + YUV packing.
//
// Implements docs/SUPERCOMBO_SPEC.md section 2 exactly:
//   2.2 warpPerspective (openpilot transforms/transform.cl): 1/32 px fixed point,
//       15-bit bilinear weights, replicate border; U/V with M_uv = S_in*M*S_out.
//   2.3 packing: ch0 Y[0::2,0::2], ch1 Y[1::2,0::2], ch2 Y[0::2,1::2], ch3 Y[1::2,1::2], ch4 U, ch5 V,
//       plain uint8 -> float (0..255), no normalisation.
//   2.4/2.5 warp matrix M = K_phone @ view_from_device @ R(rpy) @ calib_from_{med,sbig}model.
#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace pocopilot {

using Mat3 = std::array<double, 9>;  // row-major

Mat3 mat3_mul(const Mat3 &a, const Mat3 &b);

struct Intrinsics {
    double fx = 1470, fy = 1470, cx = 960, cy = 540;
    int width = 1920, height = 1080;  // image size the intrinsics refer to (= the NV12 frame fed to the warp)
    bool operator==(const Intrinsics &o) const {
        return fx == o.fx && fy == o.fy && cx == o.cx && cy == o.cy && width == o.width && height == o.height;
    }
};

// Camera calibration: openpilot liveCalibration.rpyCalib (device_from_calib Euler, rad).
// Positive pitch = camera tilted down w.r.t. the road, positive yaw = camera yawed left.
struct Calib {
    double roll = 0, pitch = 0, yaw = 0;
    double height_m = 1.22;  // published only; the warp is a pure rotation (spec 2.4)
    bool operator==(const Calib &o) const {
        return roll == o.roll && pitch == o.pitch && yaw == o.yaw && height_m == o.height_m;
    }
};

// rot_from_euler([roll, pitch, yaw]) = Rz(yaw) Ry(pitch) Rx(roll) (openpilot orientation.cc).
Mat3 rot_from_euler(double roll, double pitch, double yaw);

// Model pixel -> camera pixel. big = false: medmodel (input_imgs), true: sbigmodel (big_input_imgs).
Mat3 model_warp_matrix(const Intrinsics &k, const Calib &c, bool big);

// transform_scale_buffer(M, 0.5): matrix for the half-resolution chroma planes.
Mat3 uv_warp_matrix(const Mat3 &m);

// Precomputed bilinear sampling table for one plane (the warp is constant between calibration changes).
class WarpTable {
public:
    // src_px_stride: 1 for Y, 2 for interleaved NV12 UV (U at +0, V at +1).
    void build(const Mat3 &m, int src_w, int src_h, int src_row_stride, int src_px_stride, int dst_w, int dst_h);
    // Sample: dst[i] for i in [0, dst_w*dst_h). src points to the plane (for V: uv plane + 1).
    void apply(const uint8_t *src, uint8_t *dst) const;
    int dst_w() const { return dst_w_; }
    int dst_h() const { return dst_h_; }

private:
    struct Entry {
        uint32_t o00, o01, o10, o11;  // byte offsets of the 4 (clamped) neighbours
        int16_t w00, w01, w10, w11;   // 15-bit weights (convert_short_sat_rte)
    };
    std::vector<Entry> tab_;
    int dst_w_ = 0, dst_h_ = 0;
};

// Warps one NV12 frame into one model frame (Y 512x256, U/V 256x128) and packs it to
// 6 x 128 x 256 floats. Tables are rebuilt only when intrinsics/calibration/strides change.
class FramePacker {
public:
    explicit FramePacker(bool big) : big_(big) {}
    // y/uv: NV12 planes with the given strides; k describes the frame (width/height must match).
    void pack(const uint8_t *y, int y_stride, const uint8_t *uv, int uv_stride, const Intrinsics &k, const Calib &c,
              float *out6);
    const Mat3 &matrix() const { return m_; }
    // Last warped planes (for --dump-input): Y 512x256, U 256x128, V 256x128.
    const std::vector<uint8_t> &y() const { return y_; }
    const std::vector<uint8_t> &u() const { return u_; }
    const std::vector<uint8_t> &v() const { return v_; }

private:
    bool big_;
    bool valid_ = false;
    Intrinsics k_;
    Calib c_;
    int y_stride_ = 0, uv_stride_ = 0;
    Mat3 m_{};
    WarpTable ty_, tu_, tv_;
    std::vector<uint8_t> y_, u_, v_;
};

// 6-channel packing of a warped frame (spec 2.3). y: 512x256, u/v: 256x128, out: 6*128*256.
void pack_yuv6(const uint8_t *y, const uint8_t *u, const uint8_t *v, float *out);
// Inverse of pack_yuv6 (for dumps/tests).
void unpack_yuv6(const float *in, uint8_t *y, uint8_t *u, uint8_t *v);

}  // namespace pocopilot

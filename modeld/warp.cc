#include "warp.h"

#include <algorithm>
#include <cmath>

#include "supercombo_layout.h"

namespace pocopilot {

Mat3 mat3_mul(const Mat3 &a, const Mat3 &b) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += a[3 * i + k] * b[3 * k + j];
            r[3 * i + j] = s;
        }
    return r;
}

Mat3 rot_from_euler(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch),
                 cy = std::cos(yaw), sy = std::sin(yaw);
    const Mat3 rx = {1, 0, 0, 0, cr, -sr, 0, sr, cr};
    const Mat3 ry = {cp, 0, sp, 0, 1, 0, -sp, 0, cp};
    const Mat3 rz = {cy, -sy, 0, sy, cy, 0, 0, 0, 1};
    return mat3_mul(rz, mat3_mul(ry, rx));
}

Mat3 model_warp_matrix(const Intrinsics &k, const Calib &c, bool big) {
    const Mat3 kk = {k.fx, 0, k.cx, 0, k.fy, k.cy, 0, 0, 1};
    Mat3 v, cfm;
    std::copy(std::begin(sc::kViewFromDevice), std::end(sc::kViewFromDevice), v.begin());
    const double *src = big ? sc::kCalibFromSbigmodel : sc::kCalibFromMedmodel;
    std::copy(src, src + 9, cfm.begin());
    return mat3_mul(mat3_mul(mat3_mul(kk, v), rot_from_euler(c.roll, c.pitch, c.yaw)), cfm);
}

Mat3 uv_warp_matrix(const Mat3 &m) {
    // common/mat.h transform_scale_buffer(in, 0.5): in_pt = (transform(out_pt/s + 0.5) - 0.5) * s
    const double s = 0.5;
    const Mat3 s_out = {1 / s, 0, 0.5, 0, 1 / s, 0.5, 0, 0, 1};
    const Mat3 s_in = {s, 0, -0.5 * s, 0, s, -0.5 * s, 0, 0, 1};
    return mat3_mul(mat3_mul(s_in, m), s_out);
}

static inline int16_t short_sat_rte(float v) {
    float r = std::nearbyint(v);  // default rounding mode = round half to even
    return int16_t(std::clamp(r, -32768.0f, 32767.0f));
}

void WarpTable::build(const Mat3 &md, int src_w, int src_h, int src_row_stride, int src_px_stride, int dst_w,
                      int dst_h) {
    dst_w_ = dst_w;
    dst_h_ = dst_h;
    tab_.resize(size_t(dst_w) * dst_h);
    float m[9];
    for (int i = 0; i < 9; ++i) m[i] = float(md[i]);  // the kernel uses __constant float * M
    for (int dy = 0; dy < dst_h; ++dy)
        for (int dx = 0; dx < dst_w; ++dx) {
            const float X0 = m[0] * dx + m[1] * dy + m[2];
            const float Y0 = m[3] * dx + m[4] * dy + m[5];
            float W = m[6] * dx + m[7] * dy + m[8];
            W = W != 0.0f ? 32.0f / W : 0.0f;
            const int X = int(std::nearbyint(X0 * W)), Y = int(std::nearbyint(Y0 * W));
            const int sx = X >> 5, sy = Y >> 5;
            const int sx0 = std::clamp(sx, 0, src_w - 1), sx1 = std::clamp(sx + 1, 0, src_w - 1);
            const int sy0 = std::clamp(sy, 0, src_h - 1), sy1 = std::clamp(sy + 1, 0, src_h - 1);
            const float taby = float(Y & 31) / 32.0f, tabx = float(X & 31) / 32.0f;
            Entry &e = tab_[size_t(dy) * dst_w + dx];
            e.o00 = uint32_t(sy0 * src_row_stride + sx0 * src_px_stride);
            e.o01 = uint32_t(sy0 * src_row_stride + sx1 * src_px_stride);
            e.o10 = uint32_t(sy1 * src_row_stride + sx0 * src_px_stride);
            e.o11 = uint32_t(sy1 * src_row_stride + sx1 * src_px_stride);
            e.w00 = short_sat_rte((1.0f - taby) * (1.0f - tabx) * 32768.0f);
            e.w01 = short_sat_rte((1.0f - taby) * tabx * 32768.0f);
            e.w10 = short_sat_rte(taby * (1.0f - tabx) * 32768.0f);
            e.w11 = short_sat_rte(taby * tabx * 32768.0f);
        }
}

void WarpTable::apply(const uint8_t *src, uint8_t *dst) const {
    const size_t n = tab_.size();
    const Entry *t = tab_.data();
    for (size_t i = 0; i < n; ++i) {
        const Entry &e = t[i];
        const int val = src[e.o00] * e.w00 + src[e.o01] * e.w01 + src[e.o10] * e.w10 + src[e.o11] * e.w11;
        const int p = (val + (1 << 14)) >> 15;
        dst[i] = uint8_t(std::clamp(p, 0, 255));
    }
}

void pack_yuv6(const uint8_t *y, const uint8_t *u, const uint8_t *v, float *out) {
    constexpr int W = sc::kModelW, H = sc::kModelH, CW = W / 2, CH = H / 2;
    constexpr size_t P = size_t(CW) * CH;  // UV_SIZE = 128*256
    float *c0 = out, *c1 = out + P, *c2 = out + 2 * P, *c3 = out + 3 * P;
    for (int r = 0; r < CH; ++r) {
        const uint8_t *ye = y + size_t(2 * r) * W;  // even row
        const uint8_t *yo = ye + W;                 // odd row
        float *o0 = c0 + size_t(r) * CW, *o1 = c1 + size_t(r) * CW, *o2 = c2 + size_t(r) * CW,
              *o3 = c3 + size_t(r) * CW;
        for (int c = 0; c < CW; ++c) {
            o0[c] = ye[2 * c];      // Y[0::2, 0::2]
            o1[c] = yo[2 * c];      // Y[1::2, 0::2]
            o2[c] = ye[2 * c + 1];  // Y[0::2, 1::2]
            o3[c] = yo[2 * c + 1];  // Y[1::2, 1::2]
        }
    }
    float *c4 = out + 4 * P, *c5 = out + 5 * P;
    for (size_t i = 0; i < P; ++i) {
        c4[i] = u[i];
        c5[i] = v[i];
    }
}

void unpack_yuv6(const float *in, uint8_t *y, uint8_t *u, uint8_t *v) {
    constexpr int W = sc::kModelW, CW = W / 2, CH = sc::kModelH / 2;
    constexpr size_t P = size_t(CW) * CH;
    auto q = [](float f) { return uint8_t(std::clamp(f, 0.0f, 255.0f)); };
    for (int r = 0; r < CH; ++r)
        for (int c = 0; c < CW; ++c) {
            const size_t i = size_t(r) * CW + c;
            y[size_t(2 * r) * W + 2 * c] = q(in[i]);
            y[size_t(2 * r + 1) * W + 2 * c] = q(in[P + i]);
            y[size_t(2 * r) * W + 2 * c + 1] = q(in[2 * P + i]);
            y[size_t(2 * r + 1) * W + 2 * c + 1] = q(in[3 * P + i]);
        }
    for (size_t i = 0; i < P; ++i) {
        u[i] = q(in[4 * P + i]);
        v[i] = q(in[5 * P + i]);
    }
}

void FramePacker::pack(const uint8_t *y, int y_stride, const uint8_t *uv, int uv_stride, const Intrinsics &k,
                       const Calib &c, float *out6) {
    if (!valid_ || !(k == k_) || !(c == c_) || y_stride != y_stride_ || uv_stride != uv_stride_) {
        k_ = k;
        c_ = c;
        y_stride_ = y_stride;
        uv_stride_ = uv_stride;
        m_ = model_warp_matrix(k, c, big_);
        const Mat3 muv = uv_warp_matrix(m_);
        ty_.build(m_, k.width, k.height, y_stride, 1, sc::kModelW, sc::kModelH);
        tu_.build(muv, k.width / 2, k.height / 2, uv_stride, 2, sc::kModelUVW, sc::kModelUVH);
        tv_ = tu_;  // same geometry, V is at +1 byte
        y_.resize(size_t(sc::kModelW) * sc::kModelH);
        u_.resize(size_t(sc::kModelUVW) * sc::kModelUVH);
        v_.resize(u_.size());
        valid_ = true;
    }
    ty_.apply(y, y_.data());
    tu_.apply(uv, u_.data());
    tv_.apply(uv + 1, v_.data());
    pack_yuv6(y_.data(), u_.data(), v_.data(), out6);
}

}  // namespace pocopilot

// XRGB8888 -> NV12 conversion and NV12 2x downscale (camerad internals).
//
// BT.601 full range, see the colour comment in ring.h. Integer math on 2x2
// blocks. On aarch64 explicit NEON intrinsics are used (the auto-vectorised
// reference loop is kept as the portable fallback and as the spec).
#pragma once

#include <cstddef>
#include <cstdint>

#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#define POCOPILOT_NEON 1
#endif

namespace pocopilot {

// libcamera XRGB8888 (DRM XR24) is little-endian 0xXXRRGGBB: bytes B, G, R, X.
// Portable reference version (auto-vectorised). tmp must hold 3*width
// uint16_t. Processes rows [row0, row1), both even.
inline void xrgb_to_nv12_rows_ref(const uint8_t *__restrict src, size_t src_stride, int width, uint8_t *__restrict y,
                              size_t y_stride, uint8_t *__restrict uv, size_t uv_stride, int row0, int row1,
                              uint16_t *__restrict tmp) {
    uint16_t *__restrict sr = tmp;
    uint16_t *__restrict sg = tmp + width;
    uint16_t *__restrict sb = tmp + 2 * width;
    for (int r = row0; r < row1; r += 2) {
        const uint8_t *__restrict a = src + size_t(r) * src_stride;
        const uint8_t *__restrict b = a + src_stride;
        uint8_t *__restrict ya = y + size_t(r) * y_stride;
        uint8_t *__restrict yb = ya + y_stride;
        for (int x = 0; x < width; ++x) {
            const uint16_t b0 = a[4 * x], g0 = a[4 * x + 1], r0 = a[4 * x + 2];
            const uint16_t b1 = b[4 * x], g1 = b[4 * x + 1], r1 = b[4 * x + 2];
            // 77+150+29 = 256; max 65280+128 fits in uint16.
            ya[x] = uint8_t(uint16_t(77 * r0 + 150 * g0 + 29 * b0 + 128) >> 8);
            yb[x] = uint8_t(uint16_t(77 * r1 + 150 * g1 + 29 * b1 + 128) >> 8);
            sr[x] = uint16_t(r0 + r1);
            sg[x] = uint16_t(g0 + g1);
            sb[x] = uint16_t(b0 + b1);
        }
        uint8_t *__restrict o = uv + size_t(r / 2) * uv_stride;
        const int cw = width / 2;
        for (int i = 0; i < cw; ++i) {
            const int32_t R = sr[2 * i] + sr[2 * i + 1];  // sum of 4 pixels, 0..1020
            const int32_t G = sg[2 * i] + sg[2 * i + 1];
            const int32_t B = sb[2 * i] + sb[2 * i + 1];
            // Cb = (-43 R - 85 G + 128 B) / 256 + 128, Cr = (128 R - 107 G - 21 B) / 256 + 128,
            // on 4-pixel sums -> divide by 1024; +128*1024 offset keeps it non-negative.
            int32_t u = (-43 * R - 85 * G + 128 * B + 131072 + 512) >> 10;
            int32_t v = (128 * R - 107 * G - 21 * B + 131072 + 512) >> 10;
            o[2 * i] = uint8_t(u > 255 ? 255 : u);
            o[2 * i + 1] = uint8_t(v > 255 ? 255 : v);
        }
    }
}

#ifdef POCOPILOT_NEON
// NEON version: 2 rows x 16 pixels per iteration. GCC's auto-vectorised
// reference loop runs at ~40 ms/frame on a Cortex-A55 (structure-load
// shuffles); this one is several times faster on both core types.
// Chroma: the 2x2 RGB sums are rounded to averages (0..255) first, then
//   Cb = (128 B - 43 R - 85 G + 32895) >> 8,  Cr = (128 R - 107 G - 21 B + 32895) >> 8
// in modular uint16 arithmetic (the true value is always in [255, 65535]).
// Differs from the reference by at most 1 code value in chroma.
template <int PF = 512>
inline void xrgb_to_nv12_rows_neon(const uint8_t *__restrict src, size_t src_stride, int width,
                                   uint8_t *__restrict y, size_t y_stride, uint8_t *__restrict uv,
                                   size_t uv_stride, int row0, int row1) {
    const uint8x8_t k77 = vdup_n_u8(77), k150 = vdup_n_u8(150), k29 = vdup_n_u8(29);
    const int wv = width & ~15;
    for (int r = row0; r < row1; r += 2) {
        const uint8_t *a = src + size_t(r) * src_stride;
        const uint8_t *b = a + src_stride;
        uint8_t *ya = y + size_t(r) * y_stride;
        uint8_t *yb = ya + y_stride;
        uint8_t *o = uv + size_t(r / 2) * uv_stride;
        for (int x = 0; x < wv; x += 16) {
            // The in-order A55 stalls on every miss; software prefetch PF bytes
            // ahead on both source rows keeps it streaming (A55: 40 ms -> 6.6 ms per
            // 1080p frame; 512 B measured best on the A55, ~neutral on the A75).
            __builtin_prefetch(a + 4 * x + PF);
            __builtin_prefetch(b + 4 * x + PF);
            const uint8x16x4_t pa = vld4q_u8(a + 4 * x);  // val[0]=B, [1]=G, [2]=R
            const uint8x16x4_t pb = vld4q_u8(b + 4 * x);
            uint16x8_t lo = vmull_u8(vget_low_u8(pa.val[2]), k77);
            lo = vmlal_u8(lo, vget_low_u8(pa.val[1]), k150);
            lo = vmlal_u8(lo, vget_low_u8(pa.val[0]), k29);
            uint16x8_t hi = vmull_u8(vget_high_u8(pa.val[2]), k77);
            hi = vmlal_u8(hi, vget_high_u8(pa.val[1]), k150);
            hi = vmlal_u8(hi, vget_high_u8(pa.val[0]), k29);
            vst1q_u8(ya + x, vcombine_u8(vrshrn_n_u16(lo, 8), vrshrn_n_u16(hi, 8)));
            lo = vmull_u8(vget_low_u8(pb.val[2]), k77);
            lo = vmlal_u8(lo, vget_low_u8(pb.val[1]), k150);
            lo = vmlal_u8(lo, vget_low_u8(pb.val[0]), k29);
            hi = vmull_u8(vget_high_u8(pb.val[2]), k77);
            hi = vmlal_u8(hi, vget_high_u8(pb.val[1]), k150);
            hi = vmlal_u8(hi, vget_high_u8(pb.val[0]), k29);
            vst1q_u8(yb + x, vcombine_u8(vrshrn_n_u16(lo, 8), vrshrn_n_u16(hi, 8)));
            // 2x2 sums -> rounded averages (8 chroma samples)
            const uint16x8_t R = vrshrq_n_u16(vpadalq_u8(vpaddlq_u8(pa.val[2]), pb.val[2]), 2);
            const uint16x8_t G = vrshrq_n_u16(vpadalq_u8(vpaddlq_u8(pa.val[1]), pb.val[1]), 2);
            const uint16x8_t B = vrshrq_n_u16(vpadalq_u8(vpaddlq_u8(pa.val[0]), pb.val[0]), 2);
            const uint16x8_t off = vdupq_n_u16(32895);
            uint16x8_t u = vmlaq_n_u16(off, B, 128);
            u = vmlsq_n_u16(u, R, 43);
            u = vmlsq_n_u16(u, G, 85);
            uint16x8_t v = vmlaq_n_u16(off, R, 128);
            v = vmlsq_n_u16(v, G, 107);
            v = vmlsq_n_u16(v, B, 21);
            uint8x8x2_t cuv;
            cuv.val[0] = vshrn_n_u16(u, 8);
            cuv.val[1] = vshrn_n_u16(v, 8);
            vst2_u8(o + x, cuv);
        }
        for (int x = wv; x < width; x += 2) {  // tail (width not a multiple of 16)
            int R = 0, G = 0, B = 0;
            for (int dy = 0; dy < 2; ++dy)
                for (int dx = 0; dx < 2; ++dx) {
                    const uint8_t *p = (dy ? b : a) + 4 * (x + dx);
                    (dy ? yb : ya)[x + dx] = uint8_t((77 * p[2] + 150 * p[1] + 29 * p[0] + 128) >> 8);
                    R += p[2];
                    G += p[1];
                    B += p[0];
                }
            R = (R + 2) >> 2;
            G = (G + 2) >> 2;
            B = (B + 2) >> 2;
            o[x] = uint8_t((128 * B - 43 * R - 85 * G + 32895) >> 8);
            o[x + 1] = uint8_t((128 * R - 107 * G - 21 * B + 32895) >> 8);
        }
    }
}
#endif

inline void xrgb_to_nv12_rows(const uint8_t *src, size_t src_stride, int width, uint8_t *y, size_t y_stride,
                              uint8_t *uv, size_t uv_stride, int row0, int row1, uint16_t *tmp) {
#ifdef POCOPILOT_NEON
    (void)tmp;
    xrgb_to_nv12_rows_neon(src, src_stride, width, y, y_stride, uv, uv_stride, row0, row1);
#else
    xrgb_to_nv12_rows_ref(src, src_stride, width, y, y_stride, uv, uv_stride, row0, row1, tmp);
#endif
}

// 2x box downscale of an NV12 image (width, height multiples of 4).
// Output strides are the output widths (tightly packed).
inline void nv12_half(const uint8_t *__restrict y, size_t y_stride, const uint8_t *__restrict uv, size_t uv_stride,
                      int width, int height, uint8_t *__restrict oy, uint8_t *__restrict ouv) {
    const int ow = width / 2, oh = height / 2;
    for (int r = 0; r < oh; ++r) {
        const uint8_t *__restrict a = y + size_t(2 * r) * y_stride;
        const uint8_t *__restrict b = a + y_stride;
        uint8_t *__restrict o = oy + size_t(r) * ow;
        int x = 0;
#ifdef POCOPILOT_NEON
        for (; x + 8 <= ow; x += 8)
            vst1_u8(o + x, vrshrn_n_u16(vpadalq_u8(vpaddlq_u8(vld1q_u8(a + 2 * x)), vld1q_u8(b + 2 * x)), 2));
#endif
        for (; x < ow; ++x)
            o[x] = uint8_t((uint16_t(a[2 * x]) + a[2 * x + 1] + b[2 * x] + b[2 * x + 1] + 2) >> 2);
    }
    const int cw = ow / 2, ch = oh / 2;  // output chroma pairs per row, rows
    for (int r = 0; r < ch; ++r) {
        const uint8_t *__restrict a = uv + size_t(2 * r) * uv_stride;
        const uint8_t *__restrict b = a + uv_stride;
        uint8_t *__restrict o = ouv + size_t(r) * ow;
        int i = 0;
#ifdef POCOPILOT_NEON
        for (; i + 8 <= cw; i += 8) {  // 8 output CbCr pairs from 16 input pairs
            const uint8x8x4_t pa = vld4_u8(a + 4 * i);  // U even, V even, U odd, V odd
            const uint8x8x4_t pb = vld4_u8(b + 4 * i);
            uint16x8_t su = vaddq_u16(vaddl_u8(pa.val[0], pa.val[2]), vaddl_u8(pb.val[0], pb.val[2]));
            uint16x8_t sv = vaddq_u16(vaddl_u8(pa.val[1], pa.val[3]), vaddl_u8(pb.val[1], pb.val[3]));
            uint8x8x2_t out;
            out.val[0] = vrshrn_n_u16(su, 2);
            out.val[1] = vrshrn_n_u16(sv, 2);
            vst2_u8(o + 2 * i, out);
        }
#endif
        for (; i < cw; ++i) {
            o[2 * i] = uint8_t((uint16_t(a[4 * i]) + a[4 * i + 2] + b[4 * i] + b[4 * i + 2] + 2) >> 2);
            o[2 * i + 1] = uint8_t((uint16_t(a[4 * i + 1]) + a[4 * i + 3] + b[4 * i + 1] + b[4 * i + 3] + 2) >> 2);
        }
    }
}

}  // namespace pocopilot

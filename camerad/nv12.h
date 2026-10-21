// XRGB8888 -> NV12 conversion and NV12 2x downscale (camerad internals).
//
// BT.601 full range, see the colour comment in ring.h. Integer math, written
// so GCC/Clang vectorise the inner loops (structure loads ld4/ld2 on aarch64):
// pass 1 per row pair computes luma for both rows and per-column vertical RGB
// sums, pass 2 adds horizontal neighbours and computes one CbCr pair per 2x2.
#pragma once

#include <cstddef>
#include <cstdint>

namespace pocopilot {

// libcamera XRGB8888 (DRM XR24) is little-endian 0xXXRRGGBB: bytes B, G, R, X.
// tmp must hold 3*width uint16_t. Processes rows [row0, row1), both even.
inline void xrgb_to_nv12_rows(const uint8_t *__restrict src, size_t src_stride, int width, uint8_t *__restrict y,
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

// 2x box downscale of an NV12 image (width, height multiples of 4).
// Output strides are the output widths (tightly packed).
inline void nv12_half(const uint8_t *__restrict y, size_t y_stride, const uint8_t *__restrict uv, size_t uv_stride,
                      int width, int height, uint8_t *__restrict oy, uint8_t *__restrict ouv) {
    const int ow = width / 2, oh = height / 2;
    for (int r = 0; r < oh; ++r) {
        const uint8_t *__restrict a = y + size_t(2 * r) * y_stride;
        const uint8_t *__restrict b = a + y_stride;
        uint8_t *__restrict o = oy + size_t(r) * ow;
        for (int x = 0; x < ow; ++x)
            o[x] = uint8_t((uint16_t(a[2 * x]) + a[2 * x + 1] + b[2 * x] + b[2 * x + 1] + 2) >> 2);
    }
    const int cw = ow / 2, ch = oh / 2;  // output chroma pairs per row, rows
    for (int r = 0; r < ch; ++r) {
        const uint8_t *__restrict a = uv + size_t(2 * r) * uv_stride;
        const uint8_t *__restrict b = a + uv_stride;
        uint8_t *__restrict o = ouv + size_t(r) * ow;
        for (int i = 0; i < cw; ++i) {
            o[2 * i] = uint8_t((uint16_t(a[4 * i]) + a[4 * i + 2] + b[4 * i] + b[4 * i + 2] + 2) >> 2);
            o[2 * i + 1] = uint8_t((uint16_t(a[4 * i + 1]) + a[4 * i + 3] + b[4 * i + 1] + b[4 * i + 3] + 2) >> 2);
        }
    }
}

}  // namespace pocopilot

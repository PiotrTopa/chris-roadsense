#include "prep.h"

#include <algorithm>
#include <cstring>

namespace roadsense {
namespace envd {

Preprocessor::Preprocessor(int size, const float mean[3], const float std[3], int ignore_bottom)
    : size_(size), ignore_bottom_(ignore_bottom) {
    for (int i = 0; i < 3; ++i) {
        mean_[i] = mean[i];
        std_[i] = std[i];
    }
    const size_t n = size_t(size) * size;
    sum_y_.assign(n, 0);
    sum_u_.assign(n, 0);
    sum_v_.assign(n, 0);
    cnt_y_.assign(n, 0);
    cnt_uv_.assign(n, 0);
}

void Preprocessor::run(const Nv12View &v, float *out, FrameStats *stats) {
    const int W = v.width, H = std::max(2, v.height - ignore_bottom_) & ~1;
    const int S = size_;
    const size_t n = size_t(S) * S;
    std::fill(sum_y_.begin(), sum_y_.end(), 0);
    std::fill(sum_u_.begin(), sum_u_.end(), 0);
    std::fill(sum_v_.begin(), sum_v_.end(), 0);
    std::fill(cnt_y_.begin(), cnt_y_.end(), 0);
    std::fill(cnt_uv_.begin(), cnt_uv_.end(), 0);

    // Column -> output column, precomputed once per frame (W is small, a few hundred to 2k).
    std::vector<uint16_t> colmap(static_cast<size_t>(W));
    for (int x = 0; x < W; ++x) colmap[size_t(x)] = uint16_t(std::min(S - 1, int(int64_t(x) * S / W)));

    uint64_t sy = 0, sy_top = 0, sy_bot = 0, dark = 0, bright = 0, schroma = 0;
    const int top_end = H / 3, bot_start = H - H / 3;
    for (int yy = 0; yy < H; ++yy) {
        const int oy = std::min(S - 1, int(int64_t(yy) * S / H));
        uint32_t *sy_row = sum_y_.data() + size_t(oy) * S;
        uint32_t *cy_row = cnt_y_.data() + size_t(oy) * S;
        const uint8_t *row = v.y + size_t(yy) * v.y_stride;
        uint64_t row_sum = 0;
        for (int x = 0; x < W; ++x) {
            const uint8_t p = row[x];
            const int ox = colmap[size_t(x)];
            sy_row[ox] += p;
            ++cy_row[ox];
            row_sum += p;
            dark += p < 32;
            bright += p > 224;
        }
        sy += row_sum;
        if (yy < top_end) sy_top += row_sum;
        if (yy >= bot_start) sy_bot += row_sum;
        if ((yy & 1) == 0) {  // one chroma row per two luma rows
            uint32_t *su_row = sum_u_.data() + size_t(oy) * S;
            uint32_t *sv_row = sum_v_.data() + size_t(oy) * S;
            uint32_t *cuv_row = cnt_uv_.data() + size_t(oy) * S;
            const uint8_t *uvrow = v.uv + size_t(yy / 2) * v.uv_stride;
            for (int x = 0; x < W; x += 2) {
                const uint8_t u = uvrow[x], w = uvrow[x + 1];
                const int ox = colmap[size_t(x)];
                su_row[ox] += u;
                sv_row[ox] += w;
                ++cuv_row[ox];
                schroma += unsigned(std::abs(int(u) - 128) + std::abs(int(w) - 128));
            }
        }
    }
    float *R = out, *G = out + n, *B = out + 2 * n;
    for (size_t i = 0; i < n; ++i) {
        float Y, U, V;
        if (cnt_y_[i] && cnt_uv_[i]) {
            Y = float(sum_y_[i]) / float(cnt_y_[i]);
            U = float(sum_u_[i]) / float(cnt_uv_[i]) - 128.f;
            V = float(sum_v_[i]) / float(cnt_uv_[i]) - 128.f;
        } else {  // frame smaller than the input (upscaling): nearest source pixel
            const int ox = int(i % size_t(S)), oy = int(i / size_t(S));
            const int sx = std::min(W - 1, int(int64_t(ox) * W / S)), sy = std::min(H - 1, int(int64_t(oy) * H / S));
            Y = v.y[size_t(sy) * v.y_stride + sx];
            const uint8_t *uv = v.uv + size_t(sy / 2) * v.uv_stride + (sx & ~1);
            U = float(uv[0]) - 128.f;
            V = float(uv[1]) - 128.f;
        }
        // BT.601 full range inverse (ring.h): R = Y + 1.402 V, G = Y - 0.344136 U - 0.714136 V, B = Y + 1.772 U
        const float r = std::min(255.f, std::max(0.f, Y + 1.402f * V));
        const float g = std::min(255.f, std::max(0.f, Y - 0.344136f * U - 0.714136f * V));
        const float b = std::min(255.f, std::max(0.f, Y + 1.772f * U));
        R[i] = (r / 255.f - mean_[0]) / std_[0];
        G[i] = (g / 255.f - mean_[1]) / std_[1];
        B[i] = (b / 255.f - mean_[2]) / std_[2];
    }
    if (stats) {
        const double npx = double(W) * H;
        stats->luma = float(sy / npx);
        stats->luma_top = float(sy_top / (double(W) * std::max(1, top_end)));
        stats->luma_bottom = float(sy_bot / (double(W) * std::max(1, H - bot_start)));
        stats->dark_frac = float(dark / npx);
        stats->bright_frac = float(bright / npx);
        stats->chroma = float(schroma / (npx / 4.0) / 2.0);
    }
}

}  // namespace envd
}  // namespace roadsense

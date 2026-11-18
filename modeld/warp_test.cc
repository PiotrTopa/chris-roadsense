// warp_test: the compact sampling table of warp.cc against a direct implementation of the definition
// (openpilot's warpPerspective kernel: 5-bit fractions, 15-bit weights, round half up of the sum), on random
// frames, several cameras and calibrations, both model inputs. Exit code 0 = every packed frame identical.
// Prints the time per packed frame (both planes sets), which is what the prep thread pays per camera frame.
//   warp_test [--bench-only]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "supercombo_layout.h"
#include "warp.h"

using namespace roadsense;

namespace {
int16_t short_sat_rte(float v) { return int16_t(std::clamp(std::nearbyint(v), -32768.0f, 32767.0f)); }

void ref_plane(const Mat3 &md, const uint8_t *src, int src_w, int src_h, int row_stride, int px_stride, int dst_w, int dst_h,
               uint8_t *dst) {
    float m[9];
    for (int i = 0; i < 9; ++i) m[i] = float(md[i]);
    for (int dy = 0; dy < dst_h; ++dy)
        for (int dx = 0; dx < dst_w; ++dx) {
            const float X0 = m[0] * dx + m[1] * dy + m[2], Y0 = m[3] * dx + m[4] * dy + m[5];
            float W = m[6] * dx + m[7] * dy + m[8];
            W = W != 0.0f ? 32.0f / W : 0.0f;
            const int X = int(std::nearbyint(X0 * W)), Y = int(std::nearbyint(Y0 * W));
            const int sx = X >> 5, sy = Y >> 5;
            const int sx0 = std::clamp(sx, 0, src_w - 1), sx1 = std::clamp(sx + 1, 0, src_w - 1);
            const int sy0 = std::clamp(sy, 0, src_h - 1), sy1 = std::clamp(sy + 1, 0, src_h - 1);
            const float ty = float(Y & 31) / 32.0f, tx = float(X & 31) / 32.0f;
            const int val = src[sy0 * row_stride + sx0 * px_stride] * short_sat_rte((1.0f - ty) * (1.0f - tx) * 32768.0f) +
                            src[sy0 * row_stride + sx1 * px_stride] * short_sat_rte((1.0f - ty) * tx * 32768.0f) +
                            src[sy1 * row_stride + sx0 * px_stride] * short_sat_rte(ty * (1.0f - tx) * 32768.0f) +
                            src[sy1 * row_stride + sx1 * px_stride] * short_sat_rte(ty * tx * 32768.0f);
            dst[size_t(dy) * dst_w + dx] = uint8_t(std::clamp((val + (1 << 14)) >> 15, 0, 255));
        }
}
}  // namespace

int main(int argc, char **argv) {
    const bool bench_only = argc > 1 && !strcmp(argv[1], "--bench-only");
    std::mt19937 rng(1);
    struct C {
        int w, h, ys, uvs;
        double f, cx, cy, pitch, yaw, roll;
    } cases[] = {{1280, 720, 1280, 1280, 690, 640, 360, -0.085, 0.015, 0},  {1280, 720, 1280, 1280, 975, 640, 360, 0, 0, 0},
                 {1920, 1080, 1920, 1920, 1500, 960, 540, 0.05, -0.2, 0.03}, {640, 480, 704, 704, 300, 320, 240, 0.3, 0.4, -0.1},
                 {1280, 720, 1344, 1344, 2500, 100, 700, -0.4, 0.5, 0.2}};
    int bad = 0;
    for (const C &c : cases) {
        std::vector<uint8_t> y(size_t(c.ys) * c.h), uv(size_t(c.uvs) * c.h / 2);
        for (auto &v : y) v = uint8_t(rng());
        for (auto &v : uv) v = uint8_t(rng());
        Intrinsics k;
        k.fx = k.fy = c.f, k.cx = c.cx, k.cy = c.cy, k.width = c.w, k.height = c.h;
        Calib cal;
        cal.pitch = c.pitch, cal.yaw = c.yaw, cal.roll = c.roll;
        for (int big = 0; big < 2; ++big) {
            FramePacker p(big);
            std::vector<float> got(sc::kFramePacked), want(sc::kFramePacked);
            p.pack(y.data(), c.ys, uv.data(), c.uvs, k, cal, got.data());
            bool same = true;
            if (!bench_only) {
                const Mat3 m = model_warp_matrix(k, cal, big), muv = uv_warp_matrix(m);
                std::vector<uint8_t> ry(size_t(sc::kModelW) * sc::kModelH), ru(size_t(sc::kModelUVW) * sc::kModelUVH), rv(ru.size());
                ref_plane(m, y.data(), c.w, c.h, c.ys, 1, sc::kModelW, sc::kModelH, ry.data());
                ref_plane(muv, uv.data(), c.w / 2, c.h / 2, c.uvs, 2, sc::kModelUVW, sc::kModelUVH, ru.data());
                ref_plane(muv, uv.data() + 1, c.w / 2, c.h / 2, c.uvs, 2, sc::kModelUVW, sc::kModelUVH, rv.data());
                pack_yuv6(ry.data(), ru.data(), rv.data(), want.data());
                same = !memcmp(got.data(), want.data(), got.size() * sizeof(float));
                bad += !same;
            }
            const auto t0 = std::chrono::steady_clock::now();
            for (int r = 0; r < 100; ++r) p.pack(y.data(), c.ys, uv.data(), c.uvs, k, cal, got.data());
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / 100;
            printf("%dx%d f %.0f %s: %s, %.2f ms per frame\n", c.w, c.h, c.f, big ? "big" : "med", same ? "identical" : "DIFFERENT", ms);
        }
    }
    return bad ? 1 : 0;
}

// NV12 frame -> the encoder's normalised NCHW float input, plus cheap image statistics.
//
// The whole frame (minus an optional bottom band, e.g. a dashcam's burned-in overlay) is box-
// downscaled to size x size: every output pixel is the mean of its source block (integer bounds
// ox*W/size .. (ox+1)*W/size), computed on the Y plane and the half-resolution UV plane in one
// pass over the source. No crop: the frame is squashed, the 16:9 aspect is not kept (the
// evaluation in docs/lanes/L2-envd.md was scored with the same squash). Colour: BT.601 full range
// (the ring's format) -> RGB in 0..1 -> (x - mean) / std per channel.
#pragma once

#include <cstdint>
#include <vector>

namespace roadsense {
namespace envd {

struct Nv12View {
    const uint8_t *y = nullptr;
    const uint8_t *uv = nullptr;
    int width = 0, height = 0, y_stride = 0, uv_stride = 0;
};

// Statistics of the (cropped) frame, from the same pass: luma in 0..255.
struct FrameStats {
    float luma = 0;           // mean Y
    float luma_top = 0;       // mean Y of the top third (sky / ceiling)
    float luma_bottom = 0;    // mean Y of the bottom third (road)
    float dark_frac = 0;      // fraction of pixels with Y < 32
    float bright_frac = 0;    // fraction of pixels with Y > 224
    float chroma = 0;         // mean (|Cb-128| + |Cr-128|) / 2: colourfulness
};

class Preprocessor {
public:
    Preprocessor(int size, const float mean[3], const float std[3], int ignore_bottom = 0);
    int size() const { return size_; }
    // out: 3 * size * size floats (planes R, G, B). stats may be null.
    void run(const Nv12View &v, float *out, FrameStats *stats);

private:
    int size_;
    float mean_[3], std_[3];
    int ignore_bottom_;
    std::vector<uint32_t> sum_y_, sum_u_, sum_v_, cnt_y_, cnt_uv_;
};

}  // namespace envd
}  // namespace roadsense

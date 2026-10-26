// Image encoder runners: one network, input [1,3,S,S] float NCHW (normalised), output the image
// embedding [1,D] (projected, not necessarily normalised; the pipeline normalises).
//   ort   ONNX Runtime CPU (tools/envd/export_onnx.py output). Always built; the reference.
//   ncnn  ncnn with Vulkan (fp16-packed), <stem>.param/.bin from tools/envd/make_ncnn.sh; built
//         with -DROADSENSE_NCNN=ON. Same turnip workaround as modeld (TU_DEBUG=flushall).
#pragma once

#include <memory>
#include <string>

namespace roadsense {
namespace envd {

struct EncoderOptions {
    std::string backend = "ort";
    std::string model;            // ort: .onnx; ncnn: .param (weights in the .bin next to it)
    int threads = 2;              // ort: intra-op threads; ncnn: CPU threads for CPU layers
    bool allow_spinning = false;  // ort only
    bool ncnn_fp16 = true;
    bool ncnn_flushall = true;
    int size = 224;               // input side
    int embed_dim = 512;          // output size
};

class ImageEncoder {
public:
    virtual ~ImageEncoder() = default;
    virtual const char *name() const = 0;
    // in: 3*size*size floats; out: embed_dim floats. Throws std::runtime_error.
    virtual void encode(const float *in, float *out) = 0;
};

std::unique_ptr<ImageEncoder> make_encoder(const EncoderOptions &opt);
std::unique_ptr<ImageEncoder> make_ort_encoder(const EncoderOptions &opt);
#ifdef ROADSENSE_HAVE_NCNN
std::unique_ptr<ImageEncoder> make_ncnn_encoder(const EncoderOptions &opt);
#endif
bool encoder_available(const std::string &backend);

}  // namespace envd
}  // namespace roadsense

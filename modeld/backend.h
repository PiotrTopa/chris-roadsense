// Inference backends for supercombo v0.9.7. A backend owns the network and runs it once on the
// seven input tensors exactly as ONNX Runtime sees them (spec section 1: sc::kInputNames order,
// full shapes, features_buffer [1,99,512]); everything around the network (FIFOs, parsing,
// JSON) stays in Supercombo and is shared by all backends.
//
//   ort   ONNX Runtime, CPU EP, fp32 graph (supercombo-fp32.onnx). Always built.
//   ncnn  ncnn with Vulkan (fp16-packed storage, fp32 compute), converted graph
//         supercombo-ncnn.param/.bin (tools/ncnn/make_ncnn.sh). Built with -DROADSENSE_NCNN=ON.
#pragma once

#include <array>
#include <memory>
#include <string>

namespace roadsense {

struct SupercomboOptions;

class InferenceBackend {
public:
    virtual ~InferenceBackend() = default;
    virtual const char *name() const = 0;
    // in[i]: input i in sc::kInputNames order, fp32, ORT layout. out: sc::kOutputLen floats.
    // Returns false and sets err on failure.
    virtual bool run(const std::array<const float *, 7> &in, float *out, std::string *err) = 0;
};

// Throws std::runtime_error if the model cannot be loaded or the backend is not built in.
std::unique_ptr<InferenceBackend> make_backend(const SupercomboOptions &opt);
std::unique_ptr<InferenceBackend> make_ort_backend(const SupercomboOptions &opt);
#ifdef ROADSENSE_HAVE_NCNN
std::unique_ptr<InferenceBackend> make_ncnn_backend(const SupercomboOptions &opt);
#endif
bool backend_available(const std::string &kind);

}  // namespace roadsense

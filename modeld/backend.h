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

// One run split in two stages (ncnn only, see can_split()).
class Stage1Job {
public:
    virtual ~Stage1Job() = default;
};

class InferenceBackend {
public:
    virtual ~InferenceBackend() = default;
    virtual const char *name() const = 0;
    // in[i]: input i in sc::kInputNames order, fp32, ORT layout. out: sc::kOutputLen floats.
    // Returns false and sets err on failure.
    virtual bool run(const std::array<const float *, 7> &in, float *out, std::string *err) = 0;

    // Optional two-stage run. Stage 1 = the GPU layers that depend only on the two image inputs (the
    // vision backbone); it can be recorded on another thread while the previous run's stage 2 is still
    // going, and submitted the moment that run ends, so the GPU does not idle while modeld prepares
    // the next run (docs/lanes/L4-gpu-sched.md). Stage 2 = everything else, with the recurrent inputs.
    // record + submit + run_stage2 compute exactly what run() computes. record_stage1 and submit_stage1
    // may run on another thread than run_stage2 of a different job; img0/img1 must outlive the job.
    virtual bool can_split() const { return false; }
    virtual std::unique_ptr<Stage1Job> record_stage1(const float * /*img0*/, const float * /*img1*/, std::string *err) {
        if (err) *err = "split run not supported";
        return nullptr;
    }
    virtual bool submit_stage1(Stage1Job *, std::string *err) {
        if (err) *err = "split run not supported";
        return false;
    }
    // in[0], in[1] are ignored (stage 1 has them).
    virtual bool run_stage2(Stage1Job *, const std::array<const float *, 7> & /*in*/, float * /*out*/, std::string *err) {
        if (err) *err = "split run not supported";
        return false;
    }
};

// Throws std::runtime_error if the model cannot be loaded or the backend is not built in.
std::unique_ptr<InferenceBackend> make_backend(const SupercomboOptions &opt);
std::unique_ptr<InferenceBackend> make_ort_backend(const SupercomboOptions &opt);
#ifdef ROADSENSE_HAVE_NCNN
std::unique_ptr<InferenceBackend> make_ncnn_backend(const SupercomboOptions &opt);
#endif
bool backend_available(const std::string &kind);

}  // namespace roadsense

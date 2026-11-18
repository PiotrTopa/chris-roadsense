#include "ncnn_backend.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <gpu.h>
#include <command.h>
#include <net.h>

#include "supercombo.h"
#include "vision_layout.h"

namespace roadsense {

namespace {

// features_buffer rows the graph reads: Gather(axis 1, indices -5, -10, ..., -40) of [1,99,512].
constexpr int kFeatRows = 8;
inline int feat_row(int k) { return sc::kFeatLen - 5 * (k + 1); }  // 94, 89, ..., 59

// The extractor's VkMat path uses whatever allocators its options hold (none by default), so a job owns
// a blob and a staging allocator from the device pool; `alloc` is declared first so it is destroyed
// last, after the extractor and the GPU blobs that live in it.
struct JobAllocators {
    explicit JobAllocators(const ncnn::VulkanDevice *d)
        : dev(d), blob(d->acquire_blob_allocator()), staging(d->acquire_staging_allocator()) {}
    ~JobAllocators() {
        dev->reclaim_blob_allocator(blob);
        dev->reclaim_staging_allocator(staging);
    }
    const ncnn::VulkanDevice *dev;
    ncnn::VkAllocator *blob, *staging;
};

struct NcnnJob : public Stage1Job {
    NcnnJob(const ncnn::Net &net) : alloc(net.vulkan_device()), ex(net.create_extractor()), cmd(net.vulkan_device()) {
        ex.set_blob_vkallocator(alloc.blob);
        ex.set_workspace_vkallocator(alloc.blob);
        ex.set_staging_vkallocator(alloc.staging);
    }
    JobAllocators alloc;
    ncnn::Extractor ex;
    ncnn::VkCompute cmd;
    ncnn::Mat img0, img1;
    std::vector<ncnn::VkMat> frontier;
};

class NcnnBackend : public InferenceBackend {
public:
    explicit NcnnBackend(const SupercomboOptions &opt) {
        if (opt.ncnn_flushall) {  // must be set before ncnn creates the Vulkan instance (ncnn_backend.h)
            const char *t = getenv("TU_DEBUG");
            if (!t || !*t) setenv("TU_DEBUG", "flushall", 1);
            else if (!strstr(t, "flushall")) setenv("TU_DEBUG", (std::string(t) + ",flushall").c_str(), 1);
        }
        if (ncnn::get_gpu_count() <= 0) throw std::runtime_error("ncnn: no Vulkan device");
        const bool fp16 = opt.ncnn_fp16;
        net_.opt.use_vulkan_compute = true;
        net_.opt.use_fp16_packed = fp16;
        net_.opt.use_fp16_storage = fp16;      // not supported by turnip on the Adreno 630: ncnn drops it
        net_.opt.use_fp16_arithmetic = fp16;   // ditto
        net_.opt.use_bf16_storage = false;
        net_.opt.use_packing_layout = true;
        net_.opt.num_threads = opt.threads > 0 ? opt.threads : 1;

        std::string param = opt.model_path, bin;
        const std::string ext = ".param";
        if (param.size() > ext.size() && param.compare(param.size() - ext.size(), ext.size(), ext) == 0)
            bin = param.substr(0, param.size() - ext.size()) + ".bin";
        else
            throw std::runtime_error("ncnn: --model must be the .param file (weights: same stem .bin)");
        if (net_.load_param(param.c_str()) != 0) throw std::runtime_error("ncnn: cannot load " + param);
        if (net_.load_model(bin.c_str()) != 0) throw std::runtime_error("ncnn: cannot load " + bin);
        const auto &in = net_.input_names(), &out = net_.output_names();
        vision_ = is_vision(opt);
        out_len_ = vision_ ? vis::kOutputLen : sc::kOutputLen;
        if (vision_ && (in.size() != 2 || out.size() != 1))
            throw std::runtime_error("ncnn: expected 2 inputs and 1 output (driving_vision-ncnn.param)");
        if (!vision_ && (in.size() != 7 || out.size() != 1))
            throw std::runtime_error("ncnn: expected 7 inputs and 1 output (supercombo-ncnn.param)");

        in0_.create(sc::kImgW, sc::kImgH, sc::kImgC);
        in1_.create(sc::kImgW, sc::kImgH, sc::kImgC);
        in2_.create(sc::kDesireW, sc::kDesireLen);
        in3_.create(sc::kTrafficConventionLen);
        in4_.create(sc::kLatParamsLen);
        in5_.create(1, sc::kPrevCurvLen);
        in6_.create(sc::kFeatW, kFeatRows);
        for (ncnn::Mat *m : {&in0_, &in1_, &in2_, &in3_, &in4_, &in5_, &in6_}) m->fill(0.0f);

        find_stage1();

        // Warm-up: the first run builds the Vulkan pipelines and uploads the weights (~0.1-1.5 s).
        const auto t0 = std::chrono::steady_clock::now();
        std::string err;
        if (!extract(in0_, in1_, &err)) throw std::runtime_error("ncnn: warm-up run failed: " + err);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        const ncnn::GpuInfo &gi = ncnn::get_gpu_info(ncnn::get_default_gpu_index());
        const char *td = getenv("TU_DEBUG");
        fprintf(stderr, "modeld: ncnn on %s (fp16 packed %d, storage %d, arithmetic %d), TU_DEBUG=%s, warm-up %.0f ms\n",
                gi.device_name(), fp16 && gi.support_fp16_packed(), fp16 && gi.support_fp16_storage(),
                fp16 && gi.support_fp16_arithmetic(), td ? td : "", ms);
    }

    ~NcnnBackend() override {
        net_.clear();
        // ncnn's global Vulkan instance outlives the Net; release it so the driver shuts down cleanly.
        ncnn::destroy_gpu_instance();
    }

    const char *name() const override { return "ncnn"; }

    bool can_split() const override { return !frontier_.empty(); }

    std::unique_ptr<Stage1Job> record_stage1(const float *img0, const float *img1, std::string *err) override {
        auto job = std::make_unique<NcnnJob>(net_);
        job->img0 = ncnn::Mat(sc::kImgW, sc::kImgH, sc::kImgC, const_cast<float *>(img0), 4u);
        job->img1 = ncnn::Mat(sc::kImgW, sc::kImgH, sc::kImgC, const_cast<float *>(img1), 4u);
        if (job->ex.input("in0", job->img0) != 0 || job->ex.input("in1", job->img1) != 0) {
            if (err) *err = "ncnn: bad image input";
            return nullptr;
        }
        job->frontier.resize(frontier_.size());
        for (size_t i = 0; i < frontier_.size(); ++i) {
            const int r = job->ex.extract(frontier_[i], job->frontier[i], job->cmd);
            if (r != 0) {
                if (err) *err = "ncnn: stage 1 record failed (" + std::to_string(r) + ")";
                return nullptr;
            }
        }
        return job;
    }

    bool submit_stage1(Stage1Job *j, std::string *err) override {
        auto *job = static_cast<NcnnJob *>(j);
        const int r = job->cmd.submit_and_wait();
        if (r != 0) {
            if (err) *err = "ncnn: stage 1 submit failed (" + std::to_string(r) + ")";
            return false;
        }
        return true;
    }

    bool run_stage2(Stage1Job *j, const std::array<const float *, 7> &in, float *out, std::string *err) override {
        auto *job = static_cast<NcnnJob *>(j);
        copy_aux(in);
        if (!extract_rest(job->ex, err)) return false;
        std::memcpy(out, out_.data, sizeof(float) * size_t(out_len_));
        return true;
    }

    bool run(const std::array<const float *, 7> &in, float *out, std::string *err) override {
        // The two image inputs (2 x 1.5 MB) are wrapped, not copied: a 256x128 fp32 channel is a multiple of
        // 16 bytes, so ncnn's channel stride equals the plane and the ORT layout is already ncnn's. The copy
        // sat in the GPU's idle gap between runs (docs/lanes/L4-gpu-sched.md).
        const ncnn::Mat img0(sc::kImgW, sc::kImgH, sc::kImgC, const_cast<float *>(in[0]), 4u);
        const ncnn::Mat img1(sc::kImgW, sc::kImgH, sc::kImgC, const_cast<float *>(in[1]), 4u);
        static_assert((sc::kImgW * sc::kImgH * 4) % 16 == 0, "image plane must need no channel padding");
        copy_aux(in);
        if (!extract(img0, img1, err)) return false;
        std::memcpy(out, out_.data, sizeof(float) * size_t(out_len_));
        return true;
    }

private:
    void copy_aux(const std::array<const float *, 7> &in) {
        if (vision_) return;
        std::memcpy(in2_.data, in[2], sizeof(float) * sc::kDesireLen * sc::kDesireW);
        std::memcpy(in3_.data, in[3], sizeof(float) * sc::kTrafficConventionLen);
        std::memcpy(in4_.data, in[4], sizeof(float) * sc::kLatParamsLen);
        std::memcpy(in5_.data, in[5], sizeof(float) * sc::kPrevCurvLen);
        for (int k = 0; k < kFeatRows; ++k)
            std::memcpy(in6_.row(k), in[6] + size_t(feat_row(k)) * sc::kFeatW, sizeof(float) * sc::kFeatW);
    }

    // Stage 1 = layers on the GPU whose inputs all come from in0/in1 (directly or through other stage-1
    // layers) or from constants (MemoryData); the frontier = stage-1 blobs consumed outside stage 1.
    // ncnn's layer list is in topological order.
    void find_stage1() {
        const std::vector<ncnn::Layer *> &layers = net_.layers();
        const std::vector<ncnn::Blob> &blobs = net_.blobs();
        int i0 = -1, i1 = -1;
        for (size_t b = 0; b < blobs.size(); ++b) {
            if (blobs[b].name == "in0") i0 = int(b);
            if (blobs[b].name == "in1") i1 = int(b);
        }
        enum { kNo = 0, kConst = 1, kImg = 2 };
        std::vector<int> bk(blobs.size(), kNo);
        std::vector<char> s1(layers.size(), 0);
        for (size_t li = 0; li < layers.size(); ++li) {
            const ncnn::Layer *L = layers[li];
            int kind = kNo;
            if (L->type == "Input") {
                kind = (!L->tops.empty() && (L->tops[0] == i0 || L->tops[0] == i1)) ? kImg : kNo;
            } else if (L->support_vulkan) {
                // no inputs (MemoryData) = constant; any non-stage-1 input = not stage 1;
                // otherwise stage 1 if at least one input derives from the images
                kind = kConst;
                for (int b : L->bottoms) {
                    if (bk[size_t(b)] == kNo) {
                        kind = kNo;
                        break;
                    }
                    if (bk[size_t(b)] == kImg) kind = kImg;
                }
            }
            if (kind == kImg) s1[li] = 1;
            for (int t : L->tops) bk[t] = kind;
        }
        int n_layers = 0;
        for (size_t li = 0; li < layers.size(); ++li) n_layers += s1[li];
        for (size_t b = 0; b < blobs.size(); ++b) {
            if (bk[b] != kImg || int(b) == i0 || int(b) == i1) continue;
            const int c = blobs[b].consumer;
            if (c < 0 || !s1[size_t(c)]) frontier_.push_back(int(b));
        }
        if (n_layers < 10) frontier_.clear();  // nothing worth splitting
        std::string names;
        for (int b : frontier_) names += (names.empty() ? "" : ",") + blobs[size_t(b)].name;
        fprintf(stderr, "modeld: ncnn stage 1 = %d of %zu layers, frontier blob(s) %s\n", n_layers, layers.size(),
                names.empty() ? "(none: no split)" : names.c_str());
    }

    // vision: no inputs besides the images. In a split run the whole graph is stage 1 (its frontier is
    // out0 itself), so stage 2 only downloads the result the prefetch thread's submit computed.
    bool extract_rest(ncnn::Extractor &ex, std::string *err) {
        if (vision_) return extract_out(ex, err);
        const ncnn::Mat *ins[5] = {&in2_, &in3_, &in4_, &in5_, &in6_};
        static const char *names[5] = {"in2", "in3", "in4", "in5", "in6"};
        for (int i = 0; i < 5; ++i)
            if (ex.input(names[i], *ins[i]) != 0) {
                if (err) *err = std::string("ncnn: bad input ") + names[i];
                return false;
            }
        return extract_out(ex, err);
    }

    bool extract(const ncnn::Mat &img0, const ncnn::Mat &img1, std::string *err) {
        ncnn::Extractor ex = net_.create_extractor();
        if (ex.input("in0", img0) != 0 || ex.input("in1", img1) != 0) {
            if (err) *err = "ncnn: bad image input";
            return false;
        }
        return extract_rest(ex, err);
    }

    bool extract_out(ncnn::Extractor &ex, std::string *err) {
        int r = ex.extract("out0", out_);
        if (r != 0 || out_.empty() || out_.total() != size_t(out_len_) || out_.elemsize != 4) {
            if (err) *err = "ncnn: extract out0 failed (" + std::to_string(r) + ", " + std::to_string(out_.total()) + " values)";
            return false;
        }
        return true;
    }

    bool vision_ = false;
    int out_len_ = sc::kOutputLen;
    std::vector<int> frontier_;
    ncnn::Net net_;
    ncnn::Mat in0_, in1_, in2_, in3_, in4_, in5_, in6_, out_;
};

}  // namespace

std::unique_ptr<InferenceBackend> make_ncnn_backend(const SupercomboOptions &opt) {
    return std::make_unique<NcnnBackend>(opt);
}

}  // namespace roadsense

// modeld_replay: runs the exact modeld pipeline (frame selection, warp, packing, supercombo,
// parser, JSON) on recorded frames and prints `calib` + `road` JSON lines to stdout.
//
// Input (positional):
//   DIR   frames sorted by file name: *.nv12 (raw NV12, needs --width/--height), *.pgm (P5 grey,
//         chroma set to 128) or *.ppm (P6 RGB, converted to NV12 exactly like camerad: BT.601 full range)
//   FILE  a raw NV12 stream (frames back to back, needs --width/--height)
// Frame timestamps are seq/fps. Every pair the selector produces is run (no skipping).
#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "camerad/nv12.h"
#include "pipeline.h"

using namespace roadsense;

namespace {

void usage() {
    fprintf(stderr,
            "usage: modeld_replay (DIR | FILE.nv12) [--width W --height H] [--fps F]\n"
            "         [--focal PX] [--cx PX] [--cy PX] [--calib FILE | --pitch R --yaw R --roll R --height-m M]\n"
            "         [--backend ort|ncnn] [--model PATH] [--threads N] [--ncnn-fp32] [--ncnn-no-flushall] [--rhd] [--steer-delay S]\n"
            "         [--start N] [--max-frames N] [--split]\n"
            "         [--road-model supercombo|vision] [--policy PATH|none] [--policy-threads N] [--policy-hz F] [--rates R1,R2,.. --out-prefix P]\n"
            "         [--dump-input DIR] [--dump-runs N] [--dump-tensors]\n"
            "  defaults: fps 20, focal 1470*W/1920 (camerad default), cx W/2, cy H/2, calib from\n"
            "  $ROADSENSE_CONFIG_DIR/calib.json if present, backend ort, model /var/lib/roadsense/models/supercombo-fp32.onnx\n"
            "  (ncnn: supercombo-ncnn.param there, 1 CPU thread)\n"
            "  --dump-input DIR  write the packed model inputs of the first --dump-runs runs (default 3) as PGM:\n"
            "                    <run>_{input_imgs,big_input_imgs}_yuv.pgm (current frame, Y 512x256 over U|V)\n"
            "                    and <run>_{...}_ch.pgm (all 12 channels, 3x4 tiles of 256x128)\n"
            "  --dump-tensors    also write <run>_in<i>.f32 (all 7 inputs) and <run>_out.f32 (raw 6504 outputs)\n"
            "  --road-model M    supercombo (default) or vision (v0.11.1 driving_vision; default model\n"
            "                    driving_vision-fp32.onnx / -ncnn.param, policy driving_policy-fp32.onnx; --policy none = no plan)\n"
            "  --rates R,..      simulate modeld's run rate: at R runs per second of stream time the run takes the newest\n"
            "                    pair at each scheduled time (pairs skipped in between, as on the phone); one output file\n"
            "                    P<R>.jsonl per rate, each with its own model state (vision: one network run per frame shared)\n"
            "  --split           ncnn: two-stage run as modeld pipelines it (vision stage recorded + submitted, then the\n"
            "                    rest); must give the same outputs as the default one-stage run\n");
}

struct Image {
    int w = 0, h = 0;
    std::vector<uint8_t> nv12;  // Y then UV, strides = w
};

bool read_pnm(const std::string &path, Image *img, std::string *err) {
    std::string s;
    if (!read_file(path, &s)) return *err = "cannot read " + path, false;
    size_t p = 0;
    auto token = [&]() -> std::string {
        for (;;) {
            while (p < s.size() && isspace((unsigned char)s[p])) ++p;
            if (p < s.size() && s[p] == '#') {
                while (p < s.size() && s[p] != '\n') ++p;
                continue;
            }
            break;
        }
        size_t b = p;
        while (p < s.size() && !isspace((unsigned char)s[p])) ++p;
        return s.substr(b, p - b);
    };
    const std::string magic = token();
    const int w = atoi(token().c_str()), h = atoi(token().c_str()), mx = atoi(token().c_str());
    ++p;  // single whitespace after maxval
    if ((magic != "P5" && magic != "P6") || w <= 0 || h <= 0 || mx != 255 || (w & 1) || (h & 1))
        return *err = path + ": need P5/P6, maxval 255, even size", false;
    const int ch = magic == "P6" ? 3 : 1;
    if (s.size() < p + size_t(w) * h * ch) return *err = path + ": truncated", false;
    const uint8_t *px = reinterpret_cast<const uint8_t *>(s.data() + p);
    img->w = w;
    img->h = h;
    img->nv12.assign(size_t(w) * h * 3 / 2, 128);
    if (ch == 1) {
        std::memcpy(img->nv12.data(), px, size_t(w) * h);
        return true;
    }
    std::vector<uint8_t> xrgb(size_t(w) * h * 4);
    for (size_t i = 0; i < size_t(w) * h; ++i) {
        xrgb[4 * i] = px[3 * i + 2];  // B
        xrgb[4 * i + 1] = px[3 * i + 1];
        xrgb[4 * i + 2] = px[3 * i];  // R
        xrgb[4 * i + 3] = 255;
    }
    std::vector<uint16_t> tmp(size_t(3) * w);
    xrgb_to_nv12_rows(xrgb.data(), size_t(w) * 4, w, img->nv12.data(), size_t(w), img->nv12.data() + size_t(w) * h,
                      size_t(w), 0, h, tmp.data());
    return true;
}

bool write_pgm(const std::string &path, const uint8_t *px, int w, int h) {
    FILE *f = fopen(path.c_str(), "wb");
    if (!f) return false;
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    fwrite(px, 1, size_t(w) * h, f);
    fclose(f);
    return true;
}

bool write_f32(const std::string &path, const float *d, size_t n) {
    FILE *f = fopen(path.c_str(), "wb");
    if (!f) return false;
    fwrite(d, sizeof(float), n, f);
    fclose(f);
    return true;
}

// Current-frame dump of one packed input tensor (12 x 128 x 256 floats).
void dump_input(const std::string &base, const float *t) {
    constexpr int W = sc::kModelW, H = sc::kModelH, CW = 256, CH = 128;
    // YUV view of the current frame (channels 6..11): Y 512x256 on top, U | V (256x128 each) below
    std::vector<uint8_t> y(size_t(W) * H), u(size_t(CW) * CH), v(size_t(CW) * CH), yuv(size_t(W) * (H + CH));
    unpack_yuv6(t + sc::kFramePacked, y.data(), u.data(), v.data());
    std::memcpy(yuv.data(), y.data(), y.size());
    for (int r = 0; r < CH; ++r) {
        std::memcpy(&yuv[size_t(H + r) * W], &u[size_t(r) * CW], CW);
        std::memcpy(&yuv[size_t(H + r) * W + CW], &v[size_t(r) * CW], CW);
    }
    write_pgm(base + "_yuv.pgm", yuv.data(), W, H + CH);
    // raw channel mosaic: row = frame (prev, cur) x channel groups; tiles 3 wide x 4 high
    // tile (r, c): channel 6*(r/2) + 3*(r%2) + c
    std::vector<uint8_t> mos(size_t(3 * CW) * 4 * CH);
    for (int ch = 0; ch < 12; ++ch) {
        const int tr = ch / 3, tc = ch % 3;
        const float *src = t + size_t(ch) * CW * CH;
        for (int r = 0; r < CH; ++r)
            for (int c = 0; c < CW; ++c)
                mos[size_t(tr * CH + r) * 3 * CW + tc * CW + c] = uint8_t(std::clamp(src[r * CW + c], 0.f, 255.f));
    }
    write_pgm(base + "_ch.pgm", mos.data(), 3 * CW, 4 * CH);
}

bool ends_with(const std::string &s, const char *suf) {
    size_t n = strlen(suf);
    return s.size() >= n && s.compare(s.size() - n, n, suf) == 0;
}

}  // namespace

int main(int argc, char **argv) {
    std::string input, dump_dir, calib_file;
    int width = 0, height = 0;
    double fps = 20, focal = 0, cx = -1, cy = -1;
    Calib calib;
    bool calib_cli = false, dump_tensors = false;
    bool split = false;
    std::string rates_s, out_prefix;
    bool policy_set = false;  // --split: modeld's two-stage run (ncnn), for checking it against the one-stage run
    int dump_runs = 3;
    long start = 0, max_frames = -1;
    SupercomboOptions so;
    bool model_set = false, threads_set = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto val = [&]() -> const char * {
            if (i + 1 >= argc) {
                usage();
                exit(2);
            }
            return argv[++i];
        };
        if (a == "--width") width = atoi(val());
        else if (a == "--height") height = atoi(val());
        else if (a == "--fps") fps = atof(val());
        else if (a == "--focal") focal = atof(val());
        else if (a == "--cx") cx = atof(val());
        else if (a == "--cy") cy = atof(val());
        else if (a == "--calib") calib_file = val();
        else if (a == "--pitch") calib.pitch = atof(val()), calib_cli = true;
        else if (a == "--yaw") calib.yaw = atof(val()), calib_cli = true;
        else if (a == "--roll") calib.roll = atof(val()), calib_cli = true;
        else if (a == "--height-m") calib.height_m = atof(val()), calib_cli = true;
        else if (a == "--backend") so.backend = val();
        else if (a == "--road-model") so.road_model = val();
        else if (a == "--policy") so.policy_path = val(), policy_set = true;
        else if (a == "--policy-threads") so.policy_threads = atoi(val());
        else if (a == "--policy-hz") so.policy_hz = atof(val());
        else if (a == "--rates") rates_s = val();
        else if (a == "--out-prefix") out_prefix = val();
        else if (a == "--model") so.model_path = val(), model_set = true;
        else if (a == "--threads") so.threads = atoi(val()), threads_set = true;
        else if (a == "--ncnn-fp32") so.ncnn_fp16 = false;
        else if (a == "--ncnn-no-flushall") so.ncnn_flushall = false;
        else if (a == "--rhd") so.rhd = true;
        else if (a == "--steer-delay") so.steer_delay = float(atof(val()));
        else if (a == "--start") start = atol(val());
        else if (a == "--max-frames") max_frames = atol(val());
        else if (a == "--dump-input") dump_dir = val();
        else if (a == "--dump-runs") dump_runs = atoi(val());
        else if (a == "--dump-tensors") dump_tensors = true;
        else if (a == "--split") split = true;
        else if (a == "-h" || a == "--help") return usage(), 0;
        else if (a[0] != '-' && input.empty()) input = a;
        else return usage(), 2;
    }
    if (input.empty()) return usage(), 2;
    if (is_vision(so)) {
        if (!model_set)
            so.model_path = so.backend == "ncnn" ? "/var/lib/roadsense/models/driving_vision-ncnn.param"
                                                 : "/var/lib/roadsense/models/driving_vision-fp32.onnx";
        if (!policy_set) so.policy_path = "/var/lib/roadsense/models/driving_policy-fp32.onnx";
        if (so.policy_path == "none") so.policy_path.clear();
        model_set = true;
    }
    if (so.backend == "ncnn") {
        if (!model_set) so.model_path = "/var/lib/roadsense/models/supercombo-ncnn.param";
        if (!threads_set) so.threads = 1;
    }
    if (!calib_cli) {
        std::string err;
        const std::string cf = calib_file.empty() ? config_dir() + "/calib.json" : calib_file;
        if (!load_calib(cf, &calib, &err) && !calib_file.empty()) {
            fprintf(stderr, "modeld_replay: %s\n", err.c_str());
            return 1;
        }
    }

    // frame source
    std::vector<std::string> files;
    FILE *stream = nullptr;
    struct stat st;
    if (stat(input.c_str(), &st) != 0) return fprintf(stderr, "modeld_replay: %s not found\n", input.c_str()), 1;
    if (S_ISDIR(st.st_mode)) {
        DIR *d = opendir(input.c_str());
        while (dirent *e = readdir(d)) {
            std::string n = e->d_name;
            if (ends_with(n, ".nv12") || ends_with(n, ".pgm") || ends_with(n, ".ppm")) files.push_back(input + "/" + n);
        }
        closedir(d);
        std::sort(files.begin(), files.end());
        if (files.empty()) return fprintf(stderr, "modeld_replay: no frames in %s\n", input.c_str()), 1;
    } else {
        if (width <= 0 || height <= 0) return fprintf(stderr, "modeld_replay: raw NV12 needs --width/--height\n"), 2;
        stream = fopen(input.c_str(), "rb");
        if (!stream) return fprintf(stderr, "modeld_replay: cannot open %s\n", input.c_str()), 1;
    }

    std::vector<double> rates;  // --rates: simulated run rates (runs per second of stream time)
    for (size_t p = 0; p < rates_s.size();) {
        size_t q = rates_s.find(',', p);
        if (q == std::string::npos) q = rates_s.size();
        rates.push_back(atof(rates_s.substr(p, q - p).c_str()));
        p = q + 1;
    }
    if (!rates.empty() && out_prefix.empty()) return fprintf(stderr, "modeld_replay: --rates needs --out-prefix\n"), 2;
    if (rates.empty()) rates.push_back(0);  // 0 = every pair (the default)

    // One model instance per rate (each keeps its own state); with several rates the vision network runs
    // once per frame on its own instance and every due rate takes that output (run_raw).
    const bool vision = is_vision(so);
    const bool shared_net = vision && rates.size() > 1;
    std::vector<std::unique_ptr<Supercombo>> models;
    std::unique_ptr<Supercombo> net;
    try {
        if (!shared_net)
            for (size_t r = 0; r < rates.size(); ++r) models.push_back(std::make_unique<Supercombo>(so));
        if (shared_net) {
            SupercomboOptions o = so;
            o.policy_path.clear();
            net = std::make_unique<Supercombo>(o);
            for (size_t r = 0; r < rates.size(); ++r) {
                SupercomboOptions oo = so;
                oo.backend = "null";  // takes the shared run's outputs (run_raw)
                models.push_back(std::make_unique<Supercombo>(oo));
            }
        }
    } catch (const std::exception &e) {
        fprintf(stderr, "modeld_replay: cannot load %s: %s\n", so.model_path.c_str(), e.what());
        return 1;
    }
    Supercombo *model = models[0].get();
    std::vector<FILE *> outs;
    for (double r : rates) {
        if (out_prefix.empty()) {
            outs.push_back(stdout);
            continue;
        }
        char fn[64];
        snprintf(fn, sizeof fn, "%g.jsonl", r);
        FILE *f = fopen((out_prefix + fn).c_str(), "w");
        if (!f) return fprintf(stderr, "modeld_replay: cannot write %s%s\n", out_prefix.c_str(), fn), 1;
        outs.push_back(f);
    }
    if (!dump_dir.empty()) mkdir(dump_dir.c_str(), 0755);

    FrameSelector sel;
    Preprocessor pp(vision ? WarpKind::kNearestV011 : WarpKind::kBilinearV097);
    PairPicker picker(vision ? 0.2 : 0.05);
    std::vector<double> next_t(rates.size(), -1);  // stream time of each rate's next run
    bool calib_sent = false;
    Image img;
    std::string err;
    long idx = 0, used = 0;
    uint64_t runs = 0, runs0 = 0;
    double model_sum = 0, prep_sum = 0, policy_sum = 0;
    const double frame_dt = 1.0 / fps;
    for (;; ++idx) {
        if (max_frames >= 0 && used >= max_frames) break;
        // load frame idx
        if (stream) {
            img.w = width;
            img.h = height;
            img.nv12.resize(size_t(width) * height * 3 / 2);
            if (fread(img.nv12.data(), 1, img.nv12.size(), stream) != img.nv12.size()) break;
        } else {
            if (size_t(idx) >= files.size()) break;
            if (idx < start) continue;
            const std::string &fn = files[idx];
            if (ends_with(fn, ".nv12")) {
                if (width <= 0 || height <= 0)
                    return fprintf(stderr, "modeld_replay: .nv12 frames need --width/--height\n"), 2;
                std::string s;
                if (!read_file(fn, &s) || s.size() < size_t(width) * height * 3 / 2)
                    return fprintf(stderr, "modeld_replay: bad frame %s\n", fn.c_str()), 1;
                img.w = width;
                img.h = height;
                img.nv12.assign(s.begin(), s.begin() + long(size_t(width) * height * 3 / 2));
            } else if (!read_pnm(fn, &img, &err)) {
                return fprintf(stderr, "modeld_replay: %s\n", err.c_str()), 1;
            }
        }
        if (idx < start) continue;
        ++used;
        NV12Frame f;
        f.seq = uint64_t(idx + 1);
        f.ts_mono_ns = int64_t(double(idx) * 1e9 / fps);
        f.ts_real_ns = f.ts_mono_ns;
        f.y = img.nv12.data();
        f.uv = img.nv12.data() + size_t(img.w) * img.h;
        f.y_stride = f.uv_stride = img.w;
        f.k.width = img.w;
        f.k.height = img.h;
        f.k.fx = f.k.fy = focal > 0 ? focal : 1470.0 * img.w / 1920.0;
        f.k.cx = cx >= 0 ? cx : img.w / 2.0;
        f.k.cy = cy >= 0 ? cy : img.h / 2.0;
        if (!sel.accept(f.ts_mono_ns)) continue;
        auto curp = std::make_shared<PackedFrame>();
        pp.pack(f, calib, curp.get());
        const PackedFrame &cur = *curp;
        if (!calib_sent) {
            for (FILE *o : outs) fprintf(o, "%s\n", calib_json(calib, f.k, f.seq, "config").c_str());
            const auto &m = pp.med().matrix(), &b = pp.big().matrix();
            fprintf(stderr, "modeld_replay: %s, %dx%d f %.1f cx %.1f cy %.1f\n  M_med = [%g %g %g; %g %g %g; %g %g %g]\n"
                    "  M_big = [%g %g %g; %g %g %g; %g %g %g]\n", so.road_model.c_str(), f.k.width, f.k.height, f.k.fx,
                    f.k.cx, f.k.cy, m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], b[0], b[1], b[2], b[3], b[4], b[5],
                    b[6], b[7], b[8]);
            calib_sent = true;
        }
        auto prevp = picker.push(curp);
        if (!prevp) continue;
        const PackedFrame &prev = *prevp;
        // which rates run on this frame: the frame is the newest one at the rate's scheduled time
        const double t = double(cur.ts_mono_ns) * 1e-9;
        std::vector<size_t> due;
        for (size_t r = 0; r < rates.size(); ++r) {
            if (rates[r] <= 0 || next_t[r] < 0 || next_t[r] < t + frame_dt - 1e-6) {
                due.push_back(r);
                if (rates[r] > 0) next_t[r] = (next_t[r] < 0 || t > next_t[r] ? t : next_t[r]) + 1.0 / rates[r];
            }
        }
        if (due.empty()) continue;
        double net_ms = 0;
        if (shared_net) {
            const auto t0 = std::chrono::steady_clock::now();
            if (!net->run(prev.med.data(), cur.med.data(), prev.big.data(), cur.big.data(), nullptr, &err, cur.ts_mono_ns))
                return fprintf(stderr, "modeld_replay: run failed: %s\n", err.c_str()), 1;
            net_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        }
        for (size_t r : due) {
            Supercombo *mdl = models[r].get();
            const auto t0 = std::chrono::steady_clock::now();
            bool run_ok;
            if (shared_net) {
                run_ok = mdl->run_raw(net->raw().data(), cur.ts_mono_ns, &err);
            } else if (split) {  // modeld's pipelined path, sequentially: record stage 1, submit it, run stage 2
                auto pf = mdl->prefetch(prev.med.data(), cur.med.data(), prev.big.data(), cur.big.data(), &err);
                run_ok = pf && mdl->submit(*pf, &err) && mdl->run(*pf, nullptr, &err, cur.ts_mono_ns);
            } else {
                run_ok = mdl->run(prev.med.data(), cur.med.data(), prev.big.data(), cur.big.data(), nullptr, &err,
                                  cur.ts_mono_ns);
            }
            if (!run_ok) return fprintf(stderr, "modeld_replay: run failed: %s\n", err.c_str()), 1;
            const double ms = net_ms + std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            RoadMeta m;
            m.seq = cur.seq;
            m.seq_prev = prev.seq;
            m.ts_mono_ns = cur.ts_mono_ns;
            m.ts_real_ns = cur.ts_real_ns;
            m.model_ms = ms;
            m.prep_ms = cur.prep_ms;
            m.pair_dt_ms = double(cur.ts_mono_ns - prev.ts_mono_ns) * 1e-6;
            m.v_ego = mdl->v_ego();
            m.steer_delay = mdl->steer_delay();
            m.effective_hz = rates[r] > 0 ? rates[r] : fps;
            fprintf(outs[r], "%s\n", road_json(mdl->outputs(), m).c_str());
            if (r == 0) {
                ++runs0;
                model_sum += ms;
                policy_sum += mdl->policy_ms();
            }
        }
        fflush(outs[0]);
        if (!dump_dir.empty() && int(runs) < dump_runs && rates.size() == 1) {
            char base[64];
            snprintf(base, sizeof base, "/run%03llu", (unsigned long long)runs);
            dump_input(dump_dir + base + "_input_imgs", model->input(0).data());
            dump_input(dump_dir + base + "_big_input_imgs", model->input(1).data());
            if (dump_tensors) {
                // inputs as they were fed to this run (vision: the two image inputs only); raw outputs;
                // supercombo: feedback inputs of the next run (= state after this run)
                for (int i = 0; i < (vision ? 2 : 5); ++i)
                    write_f32(dump_dir + base + "_in" + std::to_string(i) + ".f32", model->input(i).data(),
                              model->input(i).size());
                write_f32(dump_dir + base + "_out.f32", model->raw().data(), model->raw().size());
                if (!vision) {
                    write_f32(dump_dir + base + "_next_in5.f32", model->input(5).data(), model->input(5).size());
                    write_f32(dump_dir + base + "_next_in6.f32", model->input(6).data(), model->input(6).size());
                }
            }
        }
        ++runs;
        prep_sum += cur.prep_ms;
    }
    if (stream) fclose(stream);
    for (FILE *o : outs)
        if (o != stdout) fclose(o);
    fprintf(stderr, "modeld_replay: %s: %ld frames read, %llu frames run, model %.1f ms avg (policy %.2f), prep %.2f ms avg (both inputs)\n",
            shared_net ? net->backend_name() : model->backend_name(), used, (unsigned long long)runs,
            runs0 ? model_sum / runs0 : 0.0, runs0 ? policy_sum / runs0 : 0.0, runs ? prep_sum / runs : 0.0);
    return 0;
}

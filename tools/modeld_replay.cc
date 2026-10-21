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
            "         [--model PATH] [--threads N] [--rhd] [--steer-delay S] [--start N] [--max-frames N]\n"
            "         [--dump-input DIR] [--dump-runs N] [--dump-tensors]\n"
            "  defaults: fps 20, focal 1470*W/1920 (camerad default), cx W/2, cy H/2, calib from\n"
            "  $ROADSENSE_CONFIG_DIR/calib.json if present, model /var/lib/roadsense/models/supercombo-fp32.onnx\n"
            "  --dump-input DIR  write the packed model inputs of the first --dump-runs runs (default 3) as PGM:\n"
            "                    <run>_{input_imgs,big_input_imgs}_yuv.pgm (current frame, Y 512x256 over U|V)\n"
            "                    and <run>_{...}_ch.pgm (all 12 channels, 3x4 tiles of 256x128)\n"
            "  --dump-tensors    also write <run>_in<i>.f32 (all 7 inputs) and <run>_out.f32 (raw 6504 outputs)\n");
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
    int dump_runs = 3;
    long start = 0, max_frames = -1;
    SupercomboOptions so;
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
        else if (a == "--model") so.model_path = val();
        else if (a == "--threads") so.threads = atoi(val());
        else if (a == "--rhd") so.rhd = true;
        else if (a == "--steer-delay") so.steer_delay = float(atof(val()));
        else if (a == "--start") start = atol(val());
        else if (a == "--max-frames") max_frames = atol(val());
        else if (a == "--dump-input") dump_dir = val();
        else if (a == "--dump-runs") dump_runs = atoi(val());
        else if (a == "--dump-tensors") dump_tensors = true;
        else if (a == "-h" || a == "--help") return usage(), 0;
        else if (a[0] != '-' && input.empty()) input = a;
        else return usage(), 2;
    }
    if (input.empty()) return usage(), 2;
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

    std::unique_ptr<Supercombo> model;
    try {
        model = std::make_unique<Supercombo>(so);
    } catch (const std::exception &e) {
        fprintf(stderr, "modeld_replay: cannot load %s: %s\n", so.model_path.c_str(), e.what());
        return 1;
    }
    if (!dump_dir.empty()) mkdir(dump_dir.c_str(), 0755);

    FrameSelector sel;
    Preprocessor pp;
    PackedFrame prev, cur;
    bool have_prev = false, calib_sent = false;
    Image img;
    std::string err;
    long idx = 0, used = 0;
    uint64_t runs = 0;
    double model_sum = 0, prep_sum = 0;
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
        std::swap(prev, cur);
        pp.pack(f, calib, &cur);
        if (!calib_sent) {
            printf("%s\n", calib_json(calib, f.k, f.seq, "config").c_str());
            const auto &m = pp.med().matrix(), &b = pp.big().matrix();
            fprintf(stderr, "modeld_replay: %dx%d f %.1f cx %.1f cy %.1f\n  M_med = [%g %g %g; %g %g %g; %g %g %g]\n"
                    "  M_big = [%g %g %g; %g %g %g; %g %g %g]\n", f.k.width, f.k.height, f.k.fx, f.k.cx, f.k.cy,
                    m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], b[0], b[1], b[2], b[3], b[4], b[5], b[6],
                    b[7], b[8]);
            calib_sent = true;
        }
        if (!have_prev) {
            have_prev = true;
            continue;
        }
        const auto t0 = std::chrono::steady_clock::now();
        if (!model->run(prev.med.data(), cur.med.data(), prev.big.data(), cur.big.data(), nullptr, &err))
            return fprintf(stderr, "modeld_replay: run failed: %s\n", err.c_str()), 1;
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        RoadMeta m;
        m.seq = cur.seq;
        m.seq_prev = prev.seq;
        m.ts_mono_ns = cur.ts_mono_ns;
        m.ts_real_ns = cur.ts_real_ns;
        m.model_ms = ms;
        m.prep_ms = cur.prep_ms;
        m.pair_dt_ms = double(cur.ts_mono_ns - prev.ts_mono_ns) * 1e-6;
        m.v_ego = model->v_ego();
        m.steer_delay = model->steer_delay();
        printf("%s\n", road_json(model->outputs(), m).c_str());
        fflush(stdout);
        if (!dump_dir.empty() && int(runs) < dump_runs) {
            char base[64];
            snprintf(base, sizeof base, "/run%03llu", (unsigned long long)runs);
            dump_input(dump_dir + base + "_input_imgs", model->input(0).data());
            dump_input(dump_dir + base + "_big_input_imgs", model->input(1).data());
            if (dump_tensors) {
                // inputs as they were fed to this run. Feedback FIFOs (5, 6) were already advanced
                // after the run, so they are written pre-shift by undoing nothing: dump them before
                // the next run instead (see below). Here: images, desire, traffic, latparams, raw out.
                for (int i = 0; i < 5; ++i)
                    write_f32(dump_dir + base + "_in" + std::to_string(i) + ".f32", model->input(i).data(),
                              model->input(i).size());
                write_f32(dump_dir + base + "_out.f32", model->raw().data(), model->raw().size());
                // state after this run = feedback inputs of the next run
                write_f32(dump_dir + base + "_next_in5.f32", model->input(5).data(), model->input(5).size());
                write_f32(dump_dir + base + "_next_in6.f32", model->input(6).data(), model->input(6).size());
            }
        }
        ++runs;
        model_sum += ms;
        prep_sum += cur.prep_ms;
    }
    if (stream) fclose(stream);
    fprintf(stderr, "modeld_replay: %ld frames read, %llu runs, model %.1f ms avg, prep %.2f ms avg (both inputs)\n",
            used, (unsigned long long)runs, runs ? model_sum / runs : 0.0, runs ? prep_sum / runs : 0.0);
    return 0;
}

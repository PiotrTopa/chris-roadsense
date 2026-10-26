// envd_replay: run the envd pipeline on image files (JPEG, needs libjpeg) or on a raw NV12
// sequence, one `env` JSON line per frame on stdout, a timing summary on stderr.
//
//   envd_replay [--model PATH] [--classes PATH] [--backend ort|ncnn] [--threads N] [--dt S]
//               [--window S] [--hysteresis X] [--min-hold S] [--ignore-bottom N] [--reset-each]
//               [--no-timing] [--emb] FILES...
//   envd_replay ... --nv12 WxH [--every N] [--max-frames N] seq.nv12
//
// --dt S         seconds between consecutive inputs for the smoother (default 1, i.e. 1 fps)
// --reset-each   forget the smoothing history before every input (per-image scoring, e.g. for the
//                accuracy check against the Python evaluation)
// --emb          add the L2-normalised embedding to each line ("emb": [...]) for the parity check
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifdef ENVD_HAVE_LIBJPEG
#include <jpeglib.h>
#endif

#include "output.h"
#include "pipeline.h"

using namespace roadsense::envd;

namespace {

struct Image {
    int w = 0, h = 0;
    std::vector<uint8_t> y, uv;
};

#ifdef ENVD_HAVE_LIBJPEG
bool load_jpeg(const std::string &path, Image *img) {
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return false;
    jpeg_decompress_struct c;
    jpeg_error_mgr jerr;
    c.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&c);
    jpeg_stdio_src(&c, f);
    if (jpeg_read_header(&c, TRUE) != JPEG_HEADER_OK) {
        jpeg_destroy_decompress(&c);
        fclose(f);
        return false;
    }
    c.out_color_space = JCS_RGB;
    jpeg_start_decompress(&c);
    const int w = int(c.output_width) & ~1, h = int(c.output_height) & ~1;
    std::vector<uint8_t> rgb(size_t(c.output_width) * c.output_height * 3);
    while (c.output_scanline < c.output_height) {
        uint8_t *row = rgb.data() + size_t(c.output_scanline) * c.output_width * 3;
        jpeg_read_scanlines(&c, &row, 1);
    }
    jpeg_finish_decompress(&c);
    const int stride = int(c.output_width) * 3;
    jpeg_destroy_decompress(&c);
    fclose(f);
    img->w = w;
    img->h = h;
    img->y.resize(size_t(w) * h);
    img->uv.resize(size_t(w) * h / 2);
    // BT.601 full range (ring.h), chroma = mean of each 2x2 block.
    for (int yy = 0; yy < h; ++yy) {
        const uint8_t *r0 = rgb.data() + size_t(yy) * stride;
        for (int x = 0; x < w; ++x) {
            const float R = r0[3 * x], G = r0[3 * x + 1], B = r0[3 * x + 2];
            img->y[size_t(yy) * w + x] = uint8_t(std::min(255.f, std::max(0.f, 0.299f * R + 0.587f * G + 0.114f * B + 0.5f)));
        }
    }
    for (int yy = 0; yy < h; yy += 2) {
        for (int x = 0; x < w; x += 2) {
            float cb = 0, cr = 0;
            for (int dy = 0; dy < 2; ++dy)
                for (int dx = 0; dx < 2; ++dx) {
                    const uint8_t *p = rgb.data() + size_t(yy + dy) * stride + size_t(x + dx) * 3;
                    const float R = p[0], G = p[1], B = p[2];
                    cb += -0.168736f * R - 0.331264f * G + 0.5f * B + 128.f;
                    cr += 0.5f * R - 0.418688f * G - 0.081312f * B + 128.f;
                }
            uint8_t *uv = img->uv.data() + size_t(yy / 2) * w + x;
            uv[0] = uint8_t(std::min(255.f, std::max(0.f, cb / 4 + 0.5f)));
            uv[1] = uint8_t(std::min(255.f, std::max(0.f, cr / 4 + 0.5f)));
        }
    }
    return true;
}
#endif

void usage() {
    fprintf(stderr, "usage: envd_replay [--model P] [--classes P] [--backend ort|ncnn] [--threads N] [--dt S] [--window S]\n"
                    "                   [--hysteresis X] [--min-hold S] [--ignore-bottom N] [--reset-each] [--no-timing] [--emb]\n"
                    "                   FILES... | --nv12 WxH [--every N] [--max-frames N] seq.nv12\n");
}

}  // namespace

int main(int argc, char **argv) {
    PipelineConfig pc;
    pc.enc.model = "models/envd-image.onnx";
    pc.classes_path = "models/envd-classes.json";
    double dt = 1.0;
    bool reset_each = false, timing = true, with_emb = false;
    int nv_w = 0, nv_h = 0, every = 1, max_frames = 0;
    std::vector<std::string> files;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto val = [&]() -> std::string {
            if (i + 1 >= argc) {
                usage();
                exit(2);
            }
            return argv[++i];
        };
        if (a == "--model") pc.enc.model = val();
        else if (a == "--classes") pc.classes_path = val();
        else if (a == "--backend") pc.enc.backend = val();
        else if (a == "--threads") pc.enc.threads = atoi(val().c_str());
        else if (a == "--dt") dt = atof(val().c_str());
        else if (a == "--window") pc.window_s = atof(val().c_str());
        else if (a == "--hysteresis") pc.hysteresis = atof(val().c_str());
        else if (a == "--min-hold") pc.min_hold_s = atof(val().c_str());
        else if (a == "--ignore-bottom") pc.ignore_bottom = atoi(val().c_str());
        else if (a == "--ncnn-fp32") pc.enc.ncnn_fp16 = false;
        else if (a == "--ncnn-no-flushall") pc.enc.ncnn_flushall = false;
        else if (a == "--reset-each") reset_each = true;
        else if (a == "--no-timing") timing = false;
        else if (a == "--emb") with_emb = true;
        else if (a == "--nv12") {
            if (sscanf(val().c_str(), "%dx%d", &nv_w, &nv_h) != 2) return 2;
        } else if (a == "--every") every = atoi(val().c_str());
        else if (a == "--max-frames") max_frames = atoi(val().c_str());
        else if (a == "--help" || a == "-h") {
            usage();
            return 0;
        } else files.push_back(a);
    }
    if (files.empty()) {
        usage();
        return 2;
    }
    std::unique_ptr<EnvPipeline> pipe;
    try {
        pipe = std::make_unique<EnvPipeline>(pc);
    } catch (const std::exception &e) {
        fprintf(stderr, "envd_replay: %s\n", e.what());
        return 1;
    }
    const std::string model_name = pipe->classes().model + "/" + pipe->classes().pretrained + " " + pipe->backend();
    EnvResult res;
    std::vector<double> ms_total, ms_enc;
    double t = 0;
    uint64_t seq = 0;

    auto run_one = [&](const Image &img, const std::string &name) {
        if (reset_each) pipe->reset();
        Nv12View v;
        v.y = img.y.data();
        v.uv = img.uv.data();
        v.width = img.w;
        v.height = img.h;
        v.y_stride = img.w;
        v.uv_stride = img.w;
        pipe->process(v, t, &res);
        FrameMeta m;
        m.seq = ++seq;
        m.ts_mono_ns = int64_t(t * 1e9);
        std::string line = env_json(m, res, model_name);
        line.pop_back();  // '}'
        line += ",\"file\":\"" + name + "\"";
        if (with_emb) {
            line += ",\"emb\":[";
            const auto &e = pipe->embedding();
            char b[32];
            for (size_t i = 0; i < e.size(); ++i) {
                snprintf(b, sizeof b, "%s%.6f", i ? "," : "", e[i]);
                line += b;
            }
            line += ']';
        }
        line += '}';
        puts(line.c_str());
        ms_total.push_back(res.total_ms);
        ms_enc.push_back(res.encode_ms);
        t += dt;
    };

    if (nv_w > 0) {
        FILE *f = fopen(files[0].c_str(), "rb");
        if (!f) {
            fprintf(stderr, "envd_replay: cannot open %s\n", files[0].c_str());
            return 1;
        }
        Image img;
        img.w = nv_w;
        img.h = nv_h;
        img.y.resize(size_t(nv_w) * nv_h);
        img.uv.resize(size_t(nv_w) * nv_h / 2);
        int n = 0, used = 0;
        while (fread(img.y.data(), 1, img.y.size(), f) == img.y.size() && fread(img.uv.data(), 1, img.uv.size(), f) == img.uv.size()) {
            if (n++ % every) continue;
            run_one(img, files[0] + "#" + std::to_string(n - 1));
            if (max_frames && ++used >= max_frames) break;
        }
        fclose(f);
    } else {
        for (const std::string &p : files) {
            Image img;
#ifdef ENVD_HAVE_LIBJPEG
            if (!load_jpeg(p, &img)) {
                fprintf(stderr, "envd_replay: cannot decode %s\n", p.c_str());
                continue;
            }
#else
            fprintf(stderr, "envd_replay: built without libjpeg; use --nv12\n");
            return 1;
#endif
            run_one(img, p);
        }
    }
    if (timing && !ms_total.empty()) {
        auto med = [](std::vector<double> v) {
            std::sort(v.begin(), v.end());
            return v[v.size() / 2];
        };
        auto p90 = [](std::vector<double> v) {
            std::sort(v.begin(), v.end());
            return v[std::min(v.size() - 1, size_t(v.size() * 0.9))];
        };
        fprintf(stderr, "envd_replay: %zu frames, %s: total median %.1f p90 %.1f ms, encode median %.1f p90 %.1f ms\n",
                ms_total.size(), model_name.c_str(), med(ms_total), p90(ms_total), med(ms_enc), p90(ms_enc));
    }
    return 0;
}

// laned_replay: the display lane analysis (laned/lanefind.h) on a file of NV12 frames, one `display_lanes`
// JSON line per frame on stdout. For tuning and the candidate comparison of lane L29.
//
//   laned_replay FRAMES.nv12 --width W --height H --focal F [--cx X --cy Y] --calib calib.json
//                [--frame0 N --step K]     frame k of the file is ride frame N + k K; ts = that * 50 ms, seq = that + 1
//                [--every M]               process every M-th frame of the file (a lower rate)
//                [--max N] [--set KEY=VALUE ...] [--source NAME]
//                [--evidence FILE EW EH]   instead of the marking filter: one EW x EH byte map per frame
//                [--dump DIR --dump-every N]   the evidence grid of every N-th processed frame as PGM
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "laned/lanefind.h"

using namespace roadsense::laned;

static bool json_num(const std::string &s, const char *key, double *out) {
    const std::string k = std::string("\"") + key + "\"";
    size_t p = s.find(k);
    if (p == std::string::npos) return false;
    p = s.find(':', p);
    if (p == std::string::npos) return false;
    char *end;
    const double v = strtod(s.c_str() + p + 1, &end);
    if (end == s.c_str() + p + 1) return false;
    *out = v;
    return true;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: see the head of tools/laned_replay.cc\n");
        return 2;
    }
    Camera cam;
    Config cfg;
    std::string frames = argv[1], calib, evid, dump, source = "laned";
    int ew = 0, eh = 0;
    long frame0 = 0, step = 1, every = 1, maxn = -1, dump_every = 50;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        auto val = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
        if (a == "--width") cam.width = atoi(val().c_str());
        else if (a == "--height") cam.height = atoi(val().c_str());
        else if (a == "--focal") cam.f = atof(val().c_str());
        else if (a == "--cx") cam.cx = atof(val().c_str());
        else if (a == "--cy") cam.cy = atof(val().c_str());
        else if (a == "--calib") calib = val();
        else if (a == "--frame0") frame0 = atol(val().c_str());
        else if (a == "--step") step = atol(val().c_str());
        else if (a == "--every") every = atol(val().c_str());
        else if (a == "--max") maxn = atol(val().c_str());
        else if (a == "--source") source = val();
        else if (a == "--dump") dump = val();
        else if (a == "--dump-every") dump_every = atol(val().c_str());
        else if (a == "--evidence") {
            evid = val();
            ew = atoi(val().c_str());
            eh = atoi(val().c_str());
        } else if (a == "--set") {
            const std::string kv = val();
            const size_t eq = kv.find('=');
            if (eq == std::string::npos || !config_set(&cfg, kv.substr(0, eq), atof(kv.c_str() + eq + 1))) {
                fprintf(stderr, "laned_replay: unknown option %s\n", kv.c_str());
                return 2;
            }
        } else {
            fprintf(stderr, "laned_replay: unknown argument %s\n", a.c_str());
            return 2;
        }
    }
    if (!calib.empty()) {
        std::ifstream f(calib);
        std::stringstream ss;
        ss << f.rdbuf();
        const std::string s = ss.str();
        json_num(s, "pitch", &cam.pitch), json_num(s, "yaw", &cam.yaw), json_num(s, "roll", &cam.roll);
        json_num(s, "height_m", &cam.height_m);
    }
    FILE *f = fopen(frames.c_str(), "rb");
    FILE *fe = evid.empty() ? nullptr : fopen(evid.c_str(), "rb");
    if (!f || (!evid.empty() && !fe)) {
        perror("laned_replay: open");
        return 1;
    }
    const size_t ysz = size_t(cam.width) * size_t(cam.height), fsz = ysz * 3 / 2;
    std::vector<uint8_t> buf(ysz), eb(size_t(ew) * size_t(eh));
    LaneFinder lf(cfg, cam);
    Result r;
    double ms = 0;
    long n = 0;
    for (long k = 0; maxn < 0 || n < maxn; k += every) {
        if (fe) {
            if (fseeko(fe, off_t(k) * off_t(eb.size()), SEEK_SET) != 0 || fread(eb.data(), 1, eb.size(), fe) != eb.size()) break;
        } else {
            if (fseeko(f, off_t(k) * off_t(fsz), SEEK_SET) != 0 || fread(buf.data(), 1, ysz, f) != ysz) break;
        }
        const long fr = frame0 + k * step;
        const int64_t ts = int64_t(fr) * 50000000LL;
        if (fe) lf.process_evidence(eb.data(), ew, eh, ts, &r);
        else lf.process(buf.data(), cam.width, ts, &r);
        ms += r.ms;
        puts(LaneFinder::json(r, uint64_t(fr + 1), ts, source.c_str()).c_str());
        if (!dump.empty() && n % dump_every == 0) {
            char path[512];
            snprintf(path, sizeof path, "%s/e_%06ld.pgm", dump.c_str(), fr);
            FILE *o = fopen(path, "wb");
            if (o) {
                const int nx = int(lf.rows_x().size()), ny = lf.ny();
                fprintf(o, "P5\n%d %d\n255\n", ny, nx);
                for (int i = nx - 1; i >= 0; --i)
                    for (int j = 0; j < ny; ++j) fputc(int(lf.evidence()[size_t(i) * size_t(ny) + size_t(j)] * 255), o);
                fclose(o);
            }
        }
        ++n;
    }
    fprintf(stderr, "laned_replay: %ld frames, %.3f ms per frame\n", n, n ? ms / n : 0.0);
    return 0;
}

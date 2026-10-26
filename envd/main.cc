// envd: camera frame ring -> zero-shot CLIP-type image encoder (ONNX Runtime CPU, or ncnn/Vulkan
// with --backend ncnn) -> place / weather / light scores, temporally smoothed -> `env` JSON lines
// on <rundir>/envd.sock. Contract: docs/ARCHITECTURE.md; taxonomy, prompts, accuracy:
// docs/lanes/L2-envd.md and tools/envd/.
//
// Slow by design: at most --max-hz frames per second (default 1), the NEWEST ring frame each time;
// one thread plus ORT's intra-op pool, pinned to the little cores by default. The last message is
// sent to every client when it connects (greeting), so the api has a value right away.
#include <sched.h>
#include <signal.h>
#include <sys/resource.h>
#include <sys/stat.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "camerad/ring.h"
#include "modeld/json_server.h"
#include "output.h"
#include "pipeline.h"

using namespace roadsense;
using namespace roadsense::envd;
using Clock = std::chrono::steady_clock;

namespace {

std::atomic<bool> g_run{true};
void on_signal(int) { g_run = false; }

struct Options {
    PipelineConfig pc;
    std::string cpus = "0-3";
    int nice = 5;
    double max_hz = 1;
    int pause_mc = 92000;  // hottest thermal zone above which frames are skipped (0 = never)
    uint64_t max_frames = 0;
    double stats_s = 30;
    bool frame_scores = true;
    bool verbose = false;
};

std::string config_dir() {
    const char *d = getenv("ROADSENSE_CONFIG_DIR");
    return d && *d ? d : "/etc/roadsense";
}

bool read_file(const std::string &path, std::string *out) {
    std::ifstream f(path);
    if (!f) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    *out = ss.str();
    return true;
}

// Minimal flat-JSON lookups for the config file (scalars and strings), as in the other daemons.
size_t json_value_pos(const std::string &s, const char *key) {
    const std::string k = std::string("\"") + key + "\"";
    size_t p = s.find(k);
    if (p == std::string::npos) return p;
    p = s.find(':', p + k.size());
    if (p == std::string::npos) return p;
    ++p;
    while (p < s.size() && isspace(uint8_t(s[p]))) ++p;
    return p;
}
bool json_number(const std::string &s, const char *key, double *out) {
    size_t p = json_value_pos(s, key);
    if (p == std::string::npos) return false;
    if (s.compare(p, 4, "true") == 0) return *out = 1, true;
    if (s.compare(p, 5, "false") == 0) return *out = 0, true;
    char *end = nullptr;
    double v = strtod(s.c_str() + p, &end);
    if (end == s.c_str() + p) return false;
    *out = v;
    return true;
}
bool json_string(const std::string &s, const char *key, std::string *out) {
    size_t p = json_value_pos(s, key);
    if (p == std::string::npos || p >= s.size() || s[p] != '"') return false;
    size_t e = s.find('"', p + 1);
    if (e == std::string::npos) return false;
    *out = s.substr(p + 1, e - p - 1);
    return true;
}

bool parse_cpus(const std::string &s, cpu_set_t *set) {
    CPU_ZERO(set);
    size_t i = 0;
    int n = 0;
    while (i < s.size()) {
        char *end;
        long a = strtol(s.c_str() + i, &end, 10);
        if (end == s.c_str() + i) return false;
        i = size_t(end - s.c_str());
        long b = a;
        if (i < s.size() && s[i] == '-') {
            b = strtol(s.c_str() + i + 1, &end, 10);
            i = size_t(end - s.c_str());
        }
        for (long c = a; c <= b && c < CPU_SETSIZE; ++c) CPU_SET(int(c), set), ++n;
        if (i < s.size() && s[i] == ',') ++i;
    }
    return n > 0;
}

int hottest_zone_mc() {
    int best = -1;
    for (int i = 0; i < 64; ++i) {
        char path[64];
        snprintf(path, sizeof path, "/sys/class/thermal/thermal_zone%d/temp", i);
        FILE *f = fopen(path, "r");
        if (!f) {
            if (i > 0 && errno == ENOENT) break;
            continue;
        }
        int v;
        if (fscanf(f, "%d", &v) == 1 && v > best && v < 200000) best = v;
        fclose(f);
    }
    return best;
}

void usage() {
    fprintf(stderr,
            "usage: envd [options]\n"
            "  --backend B       ort: ONNX Runtime on the CPU (default); ncnn: ncnn on the GPU through Vulkan (if built in)\n"
            "  --model PATH      ort: the image encoder .onnx (default /var/lib/roadsense/models/envd-image.onnx);\n"
            "                    ncnn: the .param (weights .bin next to it; default .../envd-image-ncnn.param)\n"
            "  --classes PATH    class text embeddings (default /var/lib/roadsense/models/envd-classes.json)\n"
            "  --threads N       ORT intra-op threads (default 2)\n"
            "  --cpus LIST       CPU affinity (default 0-3 = the little cores; none = no pinning)\n"
            "  --nice N          nice value (default 5)\n"
            "  --max-hz X        process at most X frames/s, newest frame each time (default 1)\n"
            "  --window S        smoothing window in s (default 10)\n"
            "  --hysteresis X    smoothed-score margin needed to change a label (default 0.10)\n"
            "  --min-hold S      no label change within S s of the previous one (default 3)\n"
            "  --ignore-bottom N ignore the bottom N frame rows (burned-in overlays; default 0)\n"
            "  --pause-temp MC   skip frames while the hottest zone is above MC millidegrees (default 92000, 0 = never)\n"
            "  --no-frame-scores omit per-frame scores from the message (smoothed scores stay)\n"
            "  --ncnn-fp32 / --ncnn-no-flushall   ncnn precision / turnip workaround (as in modeld)\n"
            "  --frames N        exit after N processed frames (testing)\n"
            "  --stats S         stderr stats interval in s (default 30, 0 = off)\n"
            "  --verbose\n"
            "config: $ROADSENSE_CONFIG_DIR (default /etc/roadsense)/envd.json, keys backend, model, classes, threads, cpus,\n"
            "        nice, max_hz, window, hysteresis, min_hold, ignore_bottom, pause_temp, frame_scores (CLI wins)\n"
            "env: ROADSENSE_RUNDIR (default /run/roadsense), ROADSENSE_RING (default /dev/shm/roadsense-frames)\n");
}

void load_config(Options &o) {
    std::string s;
    if (!read_file(config_dir() + "/envd.json", &s)) return;
    double v;
    std::string str;
    if (json_string(s, "backend", &str)) o.pc.enc.backend = str;
    if (json_string(s, "model", &str)) o.pc.enc.model = str;
    if (json_string(s, "classes", &str)) o.pc.classes_path = str;
    if (json_number(s, "threads", &v)) o.pc.enc.threads = int(v);
    if (json_string(s, "cpus", &str)) o.cpus = str;
    if (json_number(s, "nice", &v)) o.nice = int(v);
    if (json_number(s, "max_hz", &v)) o.max_hz = v;
    if (json_number(s, "window", &v)) o.pc.window_s = v;
    if (json_number(s, "hysteresis", &v)) o.pc.hysteresis = v;
    if (json_number(s, "min_hold", &v)) o.pc.min_hold_s = v;
    if (json_number(s, "ignore_bottom", &v)) o.pc.ignore_bottom = int(v);
    if (json_number(s, "pause_temp", &v)) o.pause_mc = int(v);
    if (json_number(s, "frame_scores", &v)) o.frame_scores = v != 0;
}

}  // namespace

int main(int argc, char **argv) {
    Options opt;
    load_config(opt);
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto val = [&]() -> std::string {
            if (i + 1 >= argc) {
                usage();
                exit(2);
            }
            return argv[++i];
        };
        if (a == "--backend") opt.pc.enc.backend = val();
        else if (a == "--model") opt.pc.enc.model = val();
        else if (a == "--classes") opt.pc.classes_path = val();
        else if (a == "--threads") opt.pc.enc.threads = atoi(val().c_str());
        else if (a == "--cpus") opt.cpus = val();
        else if (a == "--nice") opt.nice = atoi(val().c_str());
        else if (a == "--max-hz") opt.max_hz = atof(val().c_str());
        else if (a == "--window") opt.pc.window_s = atof(val().c_str());
        else if (a == "--hysteresis") opt.pc.hysteresis = atof(val().c_str());
        else if (a == "--min-hold") opt.pc.min_hold_s = atof(val().c_str());
        else if (a == "--ignore-bottom") opt.pc.ignore_bottom = atoi(val().c_str());
        else if (a == "--pause-temp") opt.pause_mc = atoi(val().c_str());
        else if (a == "--no-frame-scores") opt.frame_scores = false;
        else if (a == "--ncnn-fp32") opt.pc.enc.ncnn_fp16 = false;
        else if (a == "--ncnn-no-flushall") opt.pc.enc.ncnn_flushall = false;
        else if (a == "--frames") opt.max_frames = strtoull(val().c_str(), nullptr, 10);
        else if (a == "--stats") opt.stats_s = atof(val().c_str());
        else if (a == "--verbose" || a == "-v") opt.verbose = true;
        else {
            usage();
            return a == "--help" || a == "-h" ? 0 : 2;
        }
    }
    if (opt.pc.enc.model.empty())
        opt.pc.enc.model = opt.pc.enc.backend == "ncnn" ? "/var/lib/roadsense/models/envd-image-ncnn.param"
                                                        : "/var/lib/roadsense/models/envd-image.onnx";
    if (!encoder_available(opt.pc.enc.backend)) {
        fprintf(stderr, "envd: backend '%s' not available in this build\n", opt.pc.enc.backend.c_str());
        return 2;
    }
    const char *rd = getenv("ROADSENSE_RUNDIR");
    const std::string rundir = (rd && *rd) ? rd : "/run/roadsense";

    if (opt.cpus != "none" && !opt.cpus.empty()) {
        cpu_set_t set;
        if (!parse_cpus(opt.cpus, &set)) {
            fprintf(stderr, "envd: bad --cpus '%s'\n", opt.cpus.c_str());
            return 2;
        }
        if (sched_setaffinity(0, sizeof(set), &set) != 0)
            fprintf(stderr, "envd: sched_setaffinity(%s): %s\n", opt.cpus.c_str(), strerror(errno));
    }
    if (opt.nice != 0 && setpriority(PRIO_PROCESS, 0, opt.nice) != 0)
        fprintf(stderr, "envd: setpriority(%d): %s\n", opt.nice, strerror(errno));
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    std::unique_ptr<EnvPipeline> pipe;
    try {
        pipe = std::make_unique<EnvPipeline>(opt.pc);
    } catch (const std::exception &e) {
        fprintf(stderr, "envd: cannot start: %s\n", e.what());
        return 1;
    }
    const ClassFile &cf = pipe->classes();
    const std::string model_name = cf.model + "/" + cf.pretrained + " " + pipe->backend();
    size_t ncls = 0;
    for (const ClassGroup &g : cf.groups) ncls += g.classes.size();
    fprintf(stderr,
            "envd: %s (%s), input %dx%d, %zu classes in %zu groups, %d threads on cpus %s, max %.2f Hz, window %.0f s, "
            "hysteresis %.2f, hold %.0f s\n",
            model_name.c_str(), opt.pc.enc.model.c_str(), cf.image_size, cf.image_size, ncls, cf.groups.size(),
            opt.pc.enc.threads, opt.cpus.c_str(), opt.max_hz, opt.pc.window_s, opt.pc.hysteresis, opt.pc.min_hold_s);

    mkdir(rundir.c_str(), 0755);
    GreetingJsonServer srv;
    std::string err;
    if (!srv.listen(rundir + "/envd.sock", &err)) {
        fprintf(stderr, "envd: %s\n", err.c_str());
        return 1;
    }

    RingReader rr(ring_path());
    FrameView fv;
    EnvResult res;
    uint64_t processed = 0, last_seq = 0, paused = 0;
    int64_t last_ts = 0;
    bool waiting_logged = false;
    auto stats_t0 = Clock::now();
    uint64_t stats_n = 0;
    double stats_ms = 0, stats_enc = 0;
    const double period_s = opt.max_hz > 0 ? 1.0 / opt.max_hz : 1.0;
    auto next_allowed = Clock::now();
    auto next_temp = Clock::now();
    int temp_mc = -1;
    bool pausing = false;

    while (g_run) {
        srv.service();
        const auto now = Clock::now();
        if (now < next_allowed) {
            std::this_thread::sleep_for(std::min(next_allowed - now, Clock::duration(std::chrono::milliseconds(50))));
            continue;
        }
        if (!rr.is_open() && !rr.open()) {
            if (!waiting_logged) fprintf(stderr, "envd: waiting for ring %s\n", ring_path().c_str());
            waiting_logged = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        if (opt.pause_mc > 0 && now >= next_temp) {
            next_temp = now + std::chrono::seconds(2);
            temp_mc = hottest_zone_mc();
        }
        if (opt.pause_mc > 0 && temp_mc > opt.pause_mc) {
            if (!pausing) fprintf(stderr, "envd: hottest zone %.1f C > %.1f C, pausing\n", temp_mc / 1000.0, opt.pause_mc / 1000.0);
            pausing = true;
            ++paused;
            next_allowed = now + std::chrono::milliseconds(int64_t(period_s * 1000));
            continue;
        }
        if (pausing) fprintf(stderr, "envd: hottest zone %.1f C, resuming\n", temp_mc / 1000.0);
        pausing = false;
        if (!rr.next(fv, 200)) continue;  // newest frame
        next_allowed = Clock::now() + std::chrono::microseconds(int64_t(period_s * 1e6));
        if (fv.format != kFormatNV12 || (fv.width & 1) || (fv.height & 1)) {
            fprintf(stderr, "envd: unsupported frame %ux%u format %u\n", fv.width, fv.height, fv.format);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        if (fv.seq < last_seq || (last_ts && fv.ts_mono_ns - last_ts > 60'000'000'000LL)) pipe->reset();
        last_seq = fv.seq;
        last_ts = fv.ts_mono_ns;
        Nv12View v;
        v.y = fv.y;
        v.uv = fv.uv;
        v.width = int(fv.width);
        v.height = int(fv.height);
        v.y_stride = int(fv.y_stride);
        v.uv_stride = int(fv.uv_stride);
        try {
            pipe->process(v, double(fv.ts_mono_ns) * 1e-9, &res);
        } catch (const std::exception &e) {
            fprintf(stderr, "envd: inference failed: %s\n", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        FrameMeta m;
        m.seq = fv.seq;
        m.ts_mono_ns = fv.ts_mono_ns;
        m.ts_real_ns = fv.ts_real_ns;
        const std::string line = env_json(m, res, model_name, opt.frame_scores);
        srv.set_greeting(line);
        srv.publish(line);
        ++processed;
        ++stats_n;
        stats_ms += res.total_ms;
        stats_enc += res.encode_ms;
        if (opt.verbose) {
            std::string s;
            for (const GroupResult &g : res.groups)
                s += g.group->name + "=" + g.group->classes[size_t(g.label)] + "(" + std::to_string(int(g.confidence * 100)) +
                     "%, frame " + g.group->classes[size_t(g.frame_label)] + ") ";
            fprintf(stderr, "envd: seq %llu: %s%.0f ms (prep %.1f enc %.1f) luma %.0f\n", (unsigned long long)fv.seq,
                    s.c_str(), res.total_ms, res.prep_ms, res.encode_ms, res.stats.luma);
        }
        const double el = std::chrono::duration<double>(Clock::now() - stats_t0).count();
        if (opt.stats_s > 0 && el >= opt.stats_s) {
            std::string s;
            for (const GroupResult &g : res.groups) s += " " + g.group->name + "=" + g.group->classes[size_t(g.label)];
            fprintf(stderr, "envd: %.2f fps, env_ms %.1f (encode %.1f), paused %llu, clients %zu, temp %.1f C,%s\n",
                    stats_n / el, stats_ms / stats_n, stats_enc / stats_n, (unsigned long long)paused, srv.client_count(),
                    temp_mc / 1000.0, s.c_str());
            stats_t0 = Clock::now();
            stats_n = 0;
            stats_ms = stats_enc = 0;
        }
        if (opt.max_frames && processed >= opt.max_frames) break;
    }
    srv.close();
    fprintf(stderr, "envd: exit after %llu frames\n", (unsigned long long)processed);
    return 0;
}

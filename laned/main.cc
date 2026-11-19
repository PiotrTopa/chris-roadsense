// laned: lane lines for the DISPLAY. Camera frame ring -> classical lane-line analysis of the luma plane
// (lanefind.h) -> `display_lanes` JSON lines on <rundir>/laned.sock, which the api's display channel (`view`)
// may draw instead of the road model's lane lines (docs/ARCHITECTURE.md "Display lane source",
// docs/lanes/L29-display-lanes.md). No model, no GPU: about a millisecond or two of a little core per frame.
//
// Display only. Nothing in the measurement topics (`road`, `lanes`, `fused.lanes`, the ego lane) reads this
// daemon's output, and roadsense runs the same without it: the unit is off unless it is enabled, and the load
// governor pauses it early (`laned_hz` in <rundir>/governor), which returns the display to the model's lines.
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
#include <vector>

#include "camerad/governor.h"
#include "camerad/jsonsock.h"
#include "camerad/ring.h"
#include "lanefind.h"

using namespace roadsense;
using namespace roadsense::laned;
using Clock = std::chrono::steady_clock;

namespace {

std::atomic<bool> g_run{true};
void on_signal(int) { g_run = false; }

struct Options {
    Config cfg;
    std::string cpus = "0-3", calib_path, source = "laned";
    int nice = 5;
    double hz = 5;
    uint64_t max_frames = 0;
    double stats_s = 60;
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

time_t file_mtime(const std::string &path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0 ? st.st_mtime : 0;
}

// Flat JSON: every `"key": number|true|false` pair, in order.
std::vector<std::pair<std::string, double>> json_numbers(const std::string &s) {
    std::vector<std::pair<std::string, double>> out;
    size_t p = 0;
    while ((p = s.find('"', p)) != std::string::npos) {
        const size_t e = s.find('"', p + 1);
        if (e == std::string::npos) break;
        const std::string key = s.substr(p + 1, e - p - 1);
        size_t q = e + 1;
        while (q < s.size() && isspace(uint8_t(s[q]))) ++q;
        if (q >= s.size() || s[q] != ':') {
            p = e + 1;
            continue;
        }
        ++q;
        while (q < s.size() && isspace(uint8_t(s[q]))) ++q;
        if (s.compare(q, 4, "true") == 0) out.emplace_back(key, 1.0);
        else if (s.compare(q, 5, "false") == 0) out.emplace_back(key, 0.0);
        else if (q < s.size() && s[q] == '"') {      // a string value: skipped (escapes honoured)
            size_t e2 = q + 1;
            while (e2 < s.size() && s[e2] != '"') e2 += s[e2] == '\\' ? 2 : 1;
            p = e2 >= s.size() ? s.size() : e2 + 1;
            continue;
        } else {
            char *end = nullptr;
            const double v = strtod(s.c_str() + q, &end);
            if (end != s.c_str() + q) out.emplace_back(key, v);
        }
        p = q;
    }
    return out;
}

bool json_string(const std::string &s, const char *key, std::string *out) {
    const std::string k = std::string("\"") + key + "\"";
    size_t p = s.find(k);
    if (p == std::string::npos) return false;
    p = s.find(':', p + k.size());
    if (p == std::string::npos) return false;
    p = s.find('"', p);
    if (p == std::string::npos) return false;
    const size_t e = s.find('"', p + 1);
    if (e == std::string::npos) return false;
    *out = s.substr(p + 1, e - p - 1);
    return true;
}

void load_calib(const std::string &path, Camera *cam) {
    std::string s;
    cam->roll = cam->pitch = cam->yaw = 0;
    cam->height_m = 1.22;
    if (!read_file(path, &s)) return;
    for (const auto &kv : json_numbers(s)) {
        if (kv.first == "roll") cam->roll = kv.second;
        else if (kv.first == "pitch") cam->pitch = kv.second;
        else if (kv.first == "yaw") cam->yaw = kv.second;
        else if (kv.first == "height_m" && kv.second > 0.3) cam->height_m = kv.second;
    }
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

void usage() {
    fprintf(stderr,
            "usage: laned [options]\n"
            "  --hz X            analyse at most X frames a second, the newest frame each time (default 5)\n"
            "  --cpus LIST       CPU affinity (default 0-3 = the little cores; none = no pinning)\n"
            "  --nice N          nice value (default 5)\n"
            "  --calib PATH      calibration (default $ROADSENSE_CONFIG_DIR/calib.json; re-read when it changes)\n"
            "  --source NAME     the `source` of the messages = view.lines_source (default laned)\n"
            "  --set KEY=VALUE   an analysis constant (laned/lanefind.h, struct Config), repeatable\n"
            "  --frames N        exit after N analysed frames (testing)\n"
            "  --stats S         stderr stats interval in s (default 60, 0 = off)\n"
            "  --verbose\n"
            "config: $ROADSENSE_CONFIG_DIR (default /etc/roadsense)/laned.json: hz, nice, cpus, source and the analysis\n"
            "        constants by name (CLI wins)\n"
            "env: ROADSENSE_RUNDIR (default /run/roadsense), ROADSENSE_RING (default /dev/shm/roadsense-frames)\n"
            "the load governor's `laned_hz` cap in <rundir>/governor pauses laned under heat (0 = paused)\n");
}

void load_config(Options &o) {
    std::string s;
    if (!read_file(config_dir() + "/laned.json", &s)) return;
    json_string(s, "cpus", &o.cpus);
    json_string(s, "source", &o.source);
    for (const auto &kv : json_numbers(s)) {
        if (kv.first == "hz") o.hz = kv.second;
        else if (kv.first == "nice") o.nice = int(kv.second);
        else if (!config_set(&o.cfg, kv.first, kv.second) && kv.first[0] != '_')
            fprintf(stderr, "laned: laned.json: unknown key %s\n", kv.first.c_str());
    }
}

}  // namespace

int main(int argc, char **argv) {
    Options opt;
    opt.calib_path = config_dir() + "/calib.json";
    load_config(opt);
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto val = [&]() -> std::string {
            if (i + 1 >= argc) {
                usage();
                exit(2);
            }
            return argv[++i];
        };
        if (a == "--hz") opt.hz = atof(val().c_str());
        else if (a == "--cpus") opt.cpus = val();
        else if (a == "--nice") opt.nice = atoi(val().c_str());
        else if (a == "--calib") opt.calib_path = val();
        else if (a == "--source") opt.source = val();
        else if (a == "--frames") opt.max_frames = strtoull(val().c_str(), nullptr, 10);
        else if (a == "--stats") opt.stats_s = atof(val().c_str());
        else if (a == "--verbose" || a == "-v") opt.verbose = true;
        else if (a == "--set") {
            const std::string kv = val();
            const size_t eq = kv.find('=');
            if (eq == std::string::npos || !config_set(&opt.cfg, kv.substr(0, eq), atof(kv.c_str() + eq + 1))) {
                fprintf(stderr, "laned: unknown analysis constant '%s'\n", kv.c_str());
                return 2;
            }
        } else {
            usage();
            return a == "--help" || a == "-h" ? 0 : 2;
        }
    }
    if (opt.hz <= 0) opt.hz = 5;
    const char *rd = getenv("ROADSENSE_RUNDIR");
    const std::string rundir = (rd && *rd) ? rd : "/run/roadsense";

    if (opt.cpus != "none" && !opt.cpus.empty()) {
        cpu_set_t set;
        if (!parse_cpus(opt.cpus, &set)) {
            fprintf(stderr, "laned: bad --cpus '%s'\n", opt.cpus.c_str());
            return 2;
        }
        if (sched_setaffinity(0, sizeof(set), &set) != 0)
            fprintf(stderr, "laned: sched_setaffinity(%s): %s\n", opt.cpus.c_str(), strerror(errno));
    }
    if (opt.nice != 0 && setpriority(PRIO_PROCESS, 0, opt.nice) != 0)
        fprintf(stderr, "laned: setpriority(%d): %s\n", opt.nice, strerror(errno));
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    mkdir(rundir.c_str(), 0755);
    JsonLineServer srv;
    std::string err;
    if (!srv.listen(rundir + "/laned.sock", &err)) {
        fprintf(stderr, "laned: %s\n", err.c_str());
        return 1;
    }
    fprintf(stderr, "laned: classical lane-line analysis for the display, max %.1f Hz on cpus %s, calib %s, source \"%s\"\n",
            opt.hz, opt.cpus.c_str(), opt.calib_path.c_str(), opt.source.c_str());

    Camera cam;
    load_calib(opt.calib_path, &cam);
    time_t calib_mtime = file_mtime(opt.calib_path);
    auto next_calib_check = Clock::now();
    std::unique_ptr<LaneFinder> lf;
    RingReader rr(ring_path());
    FrameView fv;
    Result res;
    GovernorReader gov(rundir + "/governor");
    uint64_t processed = 0, last_seq = 0, paused = 0, stats_n = 0;
    double stats_ms = 0;
    bool waiting_logged = false, pausing = false;
    auto stats_t0 = Clock::now(), next_allowed = Clock::now();

    while (g_run) {
        srv.service();
        const auto now = Clock::now();
        if (now < next_allowed) {
            std::this_thread::sleep_for(std::min(next_allowed - now, Clock::duration(std::chrono::milliseconds(50))));
            continue;
        }
        if (!rr.is_open() && !rr.open()) {
            if (!waiting_logged) fprintf(stderr, "laned: waiting for ring %s\n", ring_path().c_str());
            waiting_logged = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        // Display only: shed early under heat. Paused, the display channel falls back to the road model's lines.
        const double gov_hz = gov.limit("laned_hz", opt.hz);
        if (gov_hz <= 0) {
            if (!pausing) fprintf(stderr, "laned: paused by the load governor\n");
            pausing = true;
            ++paused;
            next_allowed = now + std::chrono::milliseconds(500);
            continue;
        }
        if (pausing) fprintf(stderr, "laned: resumed by the load governor\n");
        pausing = false;
        if (!rr.next(fv, 200)) continue;  // the newest frame
        next_allowed = Clock::now() + std::chrono::microseconds(int64_t(1e6 / gov_hz));
        if (fv.format != kFormatNV12 || fv.width < 64 || fv.height < 64) {
            fprintf(stderr, "laned: unsupported frame %ux%u format %u\n", fv.width, fv.height, fv.format);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        if (Clock::now() >= next_calib_check) {
            next_calib_check = Clock::now() + std::chrono::seconds(2);
            const time_t mt = file_mtime(opt.calib_path);
            if (mt != calib_mtime) {
                calib_mtime = mt;
                load_calib(opt.calib_path, &cam);
                fprintf(stderr, "laned: calibration changed\n");
                lf.reset();
            }
        }
        if (!lf || cam.width != int(fv.width) || cam.height != int(fv.height) || cam.f != double(fv.focal_px) ||
            cam.cx != double(fv.cx) || cam.cy != double(fv.cy)) {
            cam.width = int(fv.width), cam.height = int(fv.height);
            cam.f = fv.focal_px, cam.cx = fv.cx, cam.cy = fv.cy;
            lf = std::make_unique<LaneFinder>(opt.cfg, cam);
            fprintf(stderr, "laned: frame %dx%d f %.0f, calib pitch %.4f yaw %.4f roll %.4f height %.2f\n", cam.width,
                    cam.height, cam.f, cam.pitch, cam.yaw, cam.roll, cam.height_m);
        }
        if (fv.seq < last_seq) lf->reset();
        last_seq = fv.seq;
        lf->process(fv.y, int(fv.y_stride), fv.ts_mono_ns, &res);
        srv.publish(LaneFinder::json(res, fv.seq, fv.ts_real_ns, opt.source.c_str()));
        ++processed;
        ++stats_n;
        stats_ms += res.ms;
        if (opt.verbose)
            fprintf(stderr, "laned: seq %llu lines %d%d%d%d candidates %d tracks %d trim %.4f %.4f, %.2f ms\n",
                    (unsigned long long)fv.seq, res.lines[0].present, res.lines[1].present, res.lines[2].present,
                    res.lines[3].present, res.candidates, res.tracks, res.pitch_trim, res.yaw_trim, res.ms);
        const double el = std::chrono::duration<double>(Clock::now() - stats_t0).count();
        if (opt.stats_s > 0 && el >= opt.stats_s) {
            fprintf(stderr, "laned: %.2f fps, %.2f ms per frame, lines %d%d%d%d, pitch trim %.4f yaw trim %.4f, paused %llu, clients %zu, governor %d\n",
                    stats_n / el, stats_n ? stats_ms / stats_n : 0.0, res.lines[0].present, res.lines[1].present,
                    res.lines[2].present, res.lines[3].present, res.pitch_trim, res.yaw_trim, (unsigned long long)paused,
                    srv.client_count(), gov.level());
            stats_t0 = Clock::now();
            stats_n = 0;
            stats_ms = 0;
        }
        if (opt.max_frames && processed >= opt.max_frames) break;
    }
    srv.close();
    fprintf(stderr, "laned: exit after %llu frames\n", (unsigned long long)processed);
    return 0;
}

// modeld: camera frame ring -> openpilot v0.9.7 supercombo (ONNX Runtime CPU) -> `road` JSON lines.
//
// Threads: a prep thread reads every new ring frame, selects model frames (20 Hz clock, see
// FrameSelector) and warps+packs each selected frame for both model inputs, so the image pair
// is always (previous selected frame, current selected frame) even when the network itself
// cannot keep up. The main thread runs the network on the newest pair and publishes on
// <rundir>/modeld.sock. Pairs the network had no time for are skipped (newest wins).
#include <sched.h>
#include <signal.h>
#include <sys/stat.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include "camerad/ring.h"
#include "json_server.h"
#include "pipeline.h"

using namespace roadsense;

namespace {

std::atomic<bool> g_run{true};
void on_signal(int) { g_run = false; }

struct Options {
    SupercomboOptions sc;
    std::string cpus = "4-7";
    std::string prep_cpus;  // empty: inherit the process mask
    std::string rundir;
    std::string ring;
    std::string calib_path;
    Options() { sc.allow_spinning = false; }  // spinning keeps 4 big cores busy between rate-capped runs (+10 C)
    uint64_t max_runs = 0;
    double max_hz = 20;              // cap on model runs per second (pairs stay 50 ms apart; extra pairs skipped)
    int throttle_mc = 80000;         // hottest thermal zone (millidegrees C) above which the rate is halved
    double stats_s = 10;
    bool verbose = false;
};

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
            "usage: modeld [--model PATH] [--threads N] [--cpus LIST|none] [--prep-cpus LIST] [--rhd] [--steer-delay S]\n"
            "              [--max-hz N] [--throttle-temp MC] [--spin] [--runs N] [--stats S] [--verbose]\n"
            "  --model PATH   fp32 supercombo v0.9.7 (default /var/lib/roadsense/models/supercombo-fp32.onnx)\n"
            "  --threads N    ORT intra-op threads (default 4)\n"
            "  --cpus LIST    CPU affinity for the whole process, e.g. 4-7 or 4,5 (default 4-7; none = no pinning)\n"
            "  --prep-cpus L  CPU affinity of the frame-prep thread only (default: same as --cpus), e.g. 0-3\n"
            "  --rhd          left-hand traffic (right-hand-drive car); default right-hand traffic\n"
            "  --steer-delay  lateral_control_params[1] in s (default 0.4 = 0.2 actuator + 0.2)\n"
            "  --max-hz N     max model runs per second (default 20); pairs stay 50 ms apart, extra pairs are skipped\n"
            "  --throttle-temp MC  hottest thermal zone (millidegrees C, 5 s average) above which the rate is halved,\n"
            "                 restored below MC - 5000 (default 80000)\n"
            "  --spin         let ORT intra-op threads spin between runs (default off: much cooler, ~+15 ms/run)\n"
            "  --runs N       exit after N model runs (testing)\n"
            "config: $ROADSENSE_CONFIG_DIR (default /etc/roadsense)/modeld.json keys model, threads, cpus, prep_cpus, rhd,\n"
            "        steer_delay, max_hz, throttle_temp (CLI wins); calibration from calib.json there (pitch, yaw, roll rad; height_m)\n"
            "env: ROADSENSE_RUNDIR (default /run/roadsense), ROADSENSE_RING (default /dev/shm/roadsense-frames)\n");
}

struct Pair {
    std::shared_ptr<const PackedFrame> prev, cur;
    double stream_dt_ms = 0;
};

class Shared {
public:
    void post(Pair p) {
        std::lock_guard<std::mutex> l(m_);
        if (has_) ++skipped_;
        pair_ = std::move(p);
        has_ = true;
        cv_.notify_one();
    }
    bool take(Pair *p, int timeout_ms) {
        std::unique_lock<std::mutex> l(m_);
        if (!cv_.wait_for(l, std::chrono::milliseconds(timeout_ms), [&] { return has_; })) return false;
        *p = std::move(pair_);
        has_ = false;
        return true;
    }
    uint64_t skipped() {
        std::lock_guard<std::mutex> l(m_);
        return skipped_;
    }

private:
    std::mutex m_;
    std::condition_variable cv_;
    Pair pair_;
    bool has_ = false;
    uint64_t skipped_ = 0;
};

// Hottest /sys/class/thermal/thermal_zone*/temp in millidegrees C, -1 if none readable.
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

time_t file_mtime(const std::string &p) {
    struct stat st;
    return stat(p.c_str(), &st) == 0 ? st.st_mtime : 0;
}

void prep_thread(const Options *opt, Shared *sh) {
    if (!opt->prep_cpus.empty()) {
        cpu_set_t set;
        if (!parse_cpus(opt->prep_cpus, &set) || sched_setaffinity(0, sizeof(set), &set) != 0)
            fprintf(stderr, "modeld: cannot pin the prep thread to '%s'\n", opt->prep_cpus.c_str());
    }
    RingReader rr(opt->ring);
    FrameSelector sel;
    Preprocessor pp;
    Calib calib;
    std::string err;
    time_t calib_mtime = file_mtime(opt->calib_path);
    if (!load_calib(opt->calib_path, &calib, &err)) fprintf(stderr, "modeld: %s\n", err.c_str());
    fprintf(stderr, "modeld: calib pitch %.4f yaw %.4f roll %.4f height %.2f\n", calib.pitch, calib.yaw, calib.roll,
            calib.height_m);
    auto next_calib_check = std::chrono::steady_clock::now();
    std::shared_ptr<const PackedFrame> prev;
    uint64_t last_seq = 0;
    bool waiting_logged = false;
    FrameView fv;
    while (g_run) {
        if (!rr.is_open() && !rr.open()) {
            if (!waiting_logged) fprintf(stderr, "modeld: waiting for ring %s\n", opt->ring.c_str());
            waiting_logged = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        if (!rr.next(fv, 200)) continue;
        if (std::chrono::steady_clock::now() >= next_calib_check) {
            next_calib_check += std::chrono::seconds(2);
            time_t mt = file_mtime(opt->calib_path);
            if (mt != calib_mtime) {
                calib_mtime = mt;
                Calib c;
                load_calib(opt->calib_path, &c);
                if (!(c == calib)) fprintf(stderr, "modeld: calibration changed\n");
                calib = c;
            }
        }
        if (fv.seq <= last_seq) {  // camerad restarted
            sel.reset();
            prev.reset();
        }
        last_seq = fv.seq;
        if (!sel.accept(fv.ts_mono_ns)) continue;
        NV12Frame f;
        f.seq = fv.seq;
        f.ts_mono_ns = fv.ts_mono_ns;
        f.ts_real_ns = fv.ts_real_ns;
        f.y = fv.y;
        f.uv = fv.uv;
        f.y_stride = int(fv.y_stride);
        f.uv_stride = int(fv.uv_stride);
        f.k.fx = f.k.fy = fv.focal_px;
        f.k.cx = fv.cx;
        f.k.cy = fv.cy;
        f.k.width = int(fv.width);
        f.k.height = int(fv.height);
        auto pf = std::make_shared<PackedFrame>();
        pp.pack(f, calib, pf.get());
        if (prev) sh->post(Pair{prev, pf, sel.stream_dt_ms()});
        prev = std::move(pf);
    }
}

}  // namespace

int main(int argc, char **argv) {
    Options opt;
    // config file first, CLI overrides
    {
        std::string s;
        if (read_file(config_dir() + "/modeld.json", &s)) {
            double v;
            std::string str;
            if (json_string(s, "model", &str)) opt.sc.model_path = str;
            if (json_number(s, "threads", &v)) opt.sc.threads = int(v);
            if (json_string(s, "cpus", &str)) opt.cpus = str;
            if (json_string(s, "prep_cpus", &str)) opt.prep_cpus = str;
            if (json_number(s, "rhd", &v)) opt.sc.rhd = v != 0;
            if (json_number(s, "steer_delay", &v)) opt.sc.steer_delay = float(v);
            if (json_number(s, "max_hz", &v)) opt.max_hz = v;
            if (json_number(s, "throttle_temp", &v)) opt.throttle_mc = int(v);
        }
    }
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto val = [&]() -> std::string {
            if (i + 1 >= argc) {
                usage();
                exit(2);
            }
            return argv[++i];
        };
        if (a == "--model") opt.sc.model_path = val();
        else if (a == "--threads") opt.sc.threads = atoi(val().c_str());
        else if (a == "--cpus") opt.cpus = val();
        else if (a == "--prep-cpus") opt.prep_cpus = val();
        else if (a == "--rhd") opt.sc.rhd = true;
        else if (a == "--steer-delay") opt.sc.steer_delay = float(atof(val().c_str()));
        else if (a == "--spin") opt.sc.allow_spinning = true;
        else if (a == "--no-spin") opt.sc.allow_spinning = false;
        else if (a == "--max-hz") opt.max_hz = atof(val().c_str());
        else if (a == "--throttle-temp") opt.throttle_mc = atoi(val().c_str());
        else if (a == "--runs") opt.max_runs = strtoull(val().c_str(), nullptr, 10);
        else if (a == "--stats") opt.stats_s = atof(val().c_str());
        else if (a == "--verbose" || a == "-v") opt.verbose = true;
        else {
            usage();
            return a == "--help" || a == "-h" ? 0 : 2;
        }
    }
    const char *rd = getenv("ROADSENSE_RUNDIR");
    opt.rundir = (rd && *rd) ? rd : "/run/roadsense";
    opt.ring = ring_path();
    opt.calib_path = config_dir() + "/calib.json";

    if (opt.cpus != "none" && !opt.cpus.empty()) {
        cpu_set_t set;
        if (!parse_cpus(opt.cpus, &set)) {
            fprintf(stderr, "modeld: bad --cpus '%s'\n", opt.cpus.c_str());
            return 2;
        }
        // Before any thread exists: ORT's pool and the prep thread inherit the mask.
        if (sched_setaffinity(0, sizeof(set), &set) != 0)
            fprintf(stderr, "modeld: sched_setaffinity(%s): %s\n", opt.cpus.c_str(), strerror(errno));
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    std::unique_ptr<Supercombo> model;
    try {
        model = std::make_unique<Supercombo>(opt.sc);
    } catch (const std::exception &e) {
        fprintf(stderr, "modeld: cannot load %s: %s\n", opt.sc.model_path.c_str(), e.what());
        return 1;
    }
    fprintf(stderr, "modeld: model %s, %d threads, cpus %s, %s traffic, steer delay %.2f s\n",
            opt.sc.model_path.c_str(), opt.sc.threads, opt.cpus.c_str(), opt.sc.rhd ? "left-hand" : "right-hand",
            opt.sc.steer_delay);

    mkdir(opt.rundir.c_str(), 0755);
    GreetingJsonServer srv;
    std::string err;
    const std::string sock = opt.rundir + "/modeld.sock";
    if (!srv.listen(sock, &err)) {
        fprintf(stderr, "modeld: %s\n", err.c_str());
        return 1;
    }
    fprintf(stderr, "modeld: publishing on %s, ring %s\n", sock.c_str(), opt.ring.c_str());

    Shared sh;
    std::thread prep(prep_thread, &opt, &sh);

    Calib pub_calib;
    Intrinsics pub_k;
    bool calib_published = false;
    uint64_t runs = 0, win_runs = 0;
    double win_model = 0, win_prep = 0, win_model_max = 0;
    auto win_t0 = std::chrono::steady_clock::now();
    uint64_t win_skipped0 = 0;
    // Rate cap + thermal guard: the next run may start at next_run; pairs arriving earlier are
    // overwritten by newer ones in Shared (counted as skipped), so pair spacing never stretches.
    const double max_hz = opt.max_hz > 0 ? opt.max_hz : 20;
    // The hottest zone reads ~20 C higher during an inference burst than between runs, so the guard
    // works on a smoothed value: sampled every 250 ms, exponential average with a 5 s time constant.
    bool throttled = false;
    int raw_mc = hottest_zone_mc();
    double temp_mc = raw_mc;
    auto next_temp = std::chrono::steady_clock::now();
    auto next_run = next_temp;
    Pair p;
    while (g_run) {
        srv.service();
        auto now = std::chrono::steady_clock::now();
        if (now >= next_temp) {
            next_temp = now + std::chrono::milliseconds(250);
            raw_mc = hottest_zone_mc();
            if (raw_mc >= 0) temp_mc = temp_mc < 0 ? raw_mc : temp_mc + 0.05 * (raw_mc - temp_mc);
            if (!throttled && temp_mc >= opt.throttle_mc) {
                throttled = true;
                fprintf(stderr, "modeld: thermal %.1f C >= %.1f C, halving the model rate\n", temp_mc / 1000.0,
                        opt.throttle_mc / 1000.0);
            } else if (throttled && temp_mc >= 0 && temp_mc < opt.throttle_mc - 5000) {
                throttled = false;
                fprintf(stderr, "modeld: thermal %.1f C, rate restored\n", temp_mc / 1000.0);
            }
        }
        const double eff_hz = throttled ? max_hz / 2 : max_hz;
        if (now < next_run) {
            std::this_thread::sleep_for(std::min(std::chrono::duration<double>(next_run - now),
                                                 std::chrono::duration<double>(0.02)));
            continue;
        }
        if (!sh.take(&p, 20)) continue;
        next_run = std::chrono::steady_clock::now() +
                   std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / eff_hz));
        if (!calib_published || !(p.cur->calib == pub_calib) || !(p.cur->k == pub_k)) {
            pub_calib = p.cur->calib;
            pub_k = p.cur->k;
            const std::string cj = calib_json(pub_calib, pub_k, p.cur->seq, "config");
            srv.set_greeting(cj);
            srv.publish(cj);
            calib_published = true;
        }
        const auto t0 = std::chrono::steady_clock::now();
        if (!model->run(p.prev->med.data(), p.cur->med.data(), p.prev->big.data(), p.cur->big.data(), nullptr, &err)) {
            fprintf(stderr, "modeld: run failed: %s\n", err.c_str());
            continue;
        }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        RoadMeta m;
        m.seq = p.cur->seq;
        m.seq_prev = p.prev->seq;
        m.ts_mono_ns = p.cur->ts_mono_ns;
        m.ts_real_ns = p.cur->ts_real_ns;
        m.model_ms = ms;
        m.prep_ms = p.cur->prep_ms;
        m.pair_dt_ms = double(p.cur->ts_mono_ns - p.prev->ts_mono_ns) * 1e-6;
        m.v_ego = model->v_ego();
        m.steer_delay = model->steer_delay();
        m.frames_skipped = sh.skipped();
        m.thermal_c = temp_mc >= 0 ? temp_mc / 1000.0 : -1;
        m.effective_hz = eff_hz;
        m.rate_limited = throttled;
        srv.publish(road_json(model->outputs(), m));
        ++runs;
        ++win_runs;
        win_model += ms;
        win_prep += m.prep_ms;
        win_model_max = std::max(win_model_max, ms);
        const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - win_t0).count();
        if (el >= opt.stats_s) {
            fprintf(stderr,
                    "modeld: %.1f runs/s, model %.1f ms avg %.1f max, prep %.1f ms, pair dt %.0f ms, stream %.1f ms, "
                    "skipped %llu, clients %zu, thermal_c %.1f, effective_hz %.1f\n",
                    win_runs / el, win_model / win_runs, win_model_max, win_prep / win_runs, m.pair_dt_ms,
                    p.stream_dt_ms, (unsigned long long)(m.frames_skipped - win_skipped0), srv.client_count(), m.thermal_c, eff_hz);
            win_t0 = std::chrono::steady_clock::now();
            win_runs = 0;
            win_model = win_prep = win_model_max = 0;
            win_skipped0 = m.frames_skipped;
        }
        if (opt.verbose)
            fprintf(stderr, "modeld: seq %llu/%llu model %.1f ms\n", (unsigned long long)m.seq,
                    (unsigned long long)m.seq_prev, ms);
        if (opt.max_runs && runs >= opt.max_runs) break;
    }
    g_run = false;
    prep.join();
    return 0;
}

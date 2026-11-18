// modeld: camera frame ring -> road model (openpilot v0.9.7 supercombo, or v0.11.1 driving_vision [+ policy] with
// --road-model vision; ONNX Runtime CPU or ncnn Vulkan) -> `road` JSON lines.
//
// Threads: a prep thread reads every new ring frame, selects model frames (20 Hz clock, see
// FrameSelector) and warps+packs each selected frame for both model inputs, so the image pair
// is always (previous selected frame, current selected frame) even when the network itself
// cannot keep up. The main thread runs the network on the newest pair and publishes on
// <rundir>/modeld.sock. Pairs the network had no time for are skipped (newest wins).
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

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
#include <vector>

#include "camerad/gpusched.h"
#include "camerad/ring.h"
#include "json_server.h"
#include "pipeline.h"

using namespace roadsense;

namespace {

std::atomic<bool> g_run{true};
void on_signal(int) { g_run = false; }

struct Options {
    SupercomboOptions sc;
    std::string cpus = "4-7";  // ncnn backend: 0-3 unless set (little cores do prep + GPU dispatch)
    bool cpus_set = false, threads_set = false, model_set = false, policy_set = false;
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
    std::string gpu_sched = "auto";  // <rundir>/gpu.sched; "none" = off (docs/ARCHITECTURE.md, GPU schedule)
    int gpu_yield_ms = 0;            // >0: hand the GPU to a sign provider that asks, for at most this long
    double gpu_lead_ms = 0;          // start the next run this long before the provider's detector should end
    bool pipeline = true;            // ncnn: record the next run's vision stage during the current run
    int cpu_latency_us = -1;         // >= 0: hold /dev/cpu_dma_latency at this value while running
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
            "usage: modeld [--road-model supercombo|vision] [--policy PATH|none] [--backend ort|ncnn] [--model PATH] [--threads N] [--cpus LIST|none] [--prep-cpus LIST] [--rhd]\n"
            "              [--steer-delay S] [--max-hz N] [--throttle-temp MC] [--spin] [--ncnn-fp32] [--ncnn-no-flushall] [--runs N] [--stats S]\n"
            "              [--gpu-sched PATH|none] [--gpu-yield MS] [--verbose]\n"
            "  --road-model M supercombo: openpilot v0.9.7 supercombo (default); vision: v0.11.1 driving_vision (pair\n"
            "                 200 ms apart, nearest warp, no state; default model driving_vision-fp32.onnx / -ncnn.param)\n"
            "  --policy PATH  vision: driving_policy-fp32.onnx run on the CPU after every vision run for the plan\n"
            "                 (default /var/lib/roadsense/models/driving_policy-fp32.onnx if present; none = no plan)\n"
            "  --policy-threads N  vision: ORT threads of the policy (default 1)\n"
            "  --policy-hz F  vision: run the policy at most F times a second, the plan is held in between (default\n"
            "                 0 = after every vision run)\n"
            "  --backend B    ort: ONNX Runtime on the CPU (default); ncnn: ncnn on the GPU through Vulkan (if built in)\n"
            "  --model PATH   ort: fp32 supercombo v0.9.7 (default /var/lib/roadsense/models/supercombo-fp32.onnx)\n"
            "                 ncnn: the .param file, weights in the .bin next to it\n"
            "                 (default /var/lib/roadsense/models/supercombo-ncnn.param)\n"
            "  --threads N    ort: intra-op threads (default 4); ncnn: CPU threads for its CPU-only layers (default 1)\n"
            "  --cpus LIST    CPU affinity for the whole process, e.g. 4-7 or 4,5 (default 4-7 with ort, 0-3 with ncnn;\n"
            "                 none = no pinning)\n"
            "  --prep-cpus L  CPU affinity of the frame-prep thread only (default: same as --cpus), e.g. 0-3\n"
            "  --rhd          left-hand traffic (right-hand-drive car); default right-hand traffic\n"
            "  --steer-delay  lateral_control_params[1] in s (default 0.4 = 0.2 actuator + 0.2)\n"
            "  --max-hz N     max model runs per second (default 20); pairs stay 50 ms apart, extra pairs are skipped\n"
            "  --throttle-temp MC  hottest thermal zone (millidegrees C, 5 s average) above which the rate is halved,\n"
            "                 restored below MC - 5000 (default 80000)\n"
            "  --spin         let ORT intra-op threads spin between runs (default off: much cooler, ~+15 ms/run)\n"
            "  --ncnn-fp32    ncnn: disable fp16 packing (fp32 storage on the GPU; ~5x slower on the Adreno 630)\n"
            "  --ncnn-no-flushall  ncnn: do not add TU_DEBUG=flushall (turnip needs it for correct results)\n"
            "  --no-pipeline  ncnn: do not prepare the next run's vision stage during the current run (default: do,\n"
            "                 the GPU then starts the next run as soon as the current one ends)\n"
            "  --cpu-latency-us N  hold a CPU wake-up latency request (/dev/cpu_dma_latency, needs root) of N us\n"
            "                 while modeld runs: N < ~800 keeps the cores out of their deep idle state (~0.9 ms exit\n"
            "                 on the SD845), so fence and thread wake-ups between GPU submits are faster (default off)\n"
            "  --runs N       exit after N model runs (testing)\n"
            "  --gpu-sched P  GPU schedule file shared with a sign provider (default <rundir>/gpu.sched; none = off):\n"
            "                 modeld publishes a tick at the end of every run (docs/ARCHITECTURE.md, GPU schedule)\n"
            "  --gpu-yield MS when the sign provider asks for a turn, wait up to MS ms after a run for it to\n"
            "                 finish its detector before starting the next run (default 0 = never wait)\n"
            "  --gpu-lead MS  with --gpu-yield: start the next run MS ms before the detector is expected to end\n"
            "                 (its published det_ms), so modeld's CPU preamble overlaps it (default 0)\n"
            "config: $ROADSENSE_CONFIG_DIR (default /etc/roadsense)/modeld.json keys backend, model, threads, cpus, prep_cpus, rhd,\n"
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
    pthread_setname_np(pthread_self(), "md-prep");  // thread names: per-stage CPU accounting (tools/budget)
    if (!opt->prep_cpus.empty()) {
        cpu_set_t set;
        if (!parse_cpus(opt->prep_cpus, &set) || sched_setaffinity(0, sizeof(set), &set) != 0)
            fprintf(stderr, "modeld: cannot pin the prep thread to '%s'\n", opt->prep_cpus.c_str());
    }
    RingReader rr(opt->ring);
    FrameSelector sel;
    Preprocessor pp(is_vision(opt->sc) ? WarpKind::kNearestV011 : WarpKind::kBilinearV097);
    PairPicker picker(is_vision(opt->sc) ? 0.2 : 0.05);  // vision: (frame 200 ms earlier, current)
    Calib calib;
    std::string err;
    time_t calib_mtime = file_mtime(opt->calib_path);
    if (!load_calib(opt->calib_path, &calib, &err)) fprintf(stderr, "modeld: %s\n", err.c_str());
    fprintf(stderr, "modeld: calib pitch %.4f yaw %.4f roll %.4f height %.2f\n", calib.pitch, calib.yaw, calib.roll,
            calib.height_m);
    auto next_calib_check = std::chrono::steady_clock::now();
    std::shared_ptr<const PackedFrame> prev;
    uint64_t last_seq = 0;
    int64_t last_ts = 0;
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
        // camerad restarted, or the stream stalled (e.g. a USB camera reconnecting): never pair
        // across the gap
        if (fv.seq <= last_seq || (last_ts && fv.ts_mono_ns - last_ts > 500'000'000)) {
            sel.reset();
            prev.reset();
            picker.reset();
        }
        last_seq = fv.seq;
        last_ts = fv.ts_mono_ns;
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
        prev = picker.push(pf);
        if (prev) sh->post(Pair{prev, pf, sel.stream_dt_ms()});
    }
}

// Two-stage runs (ncnn): while the main thread runs stage 2 (policy) of run N, this thread takes the
// newest pair and records stage 1 (vision backbone) of run N+1; it submits that stage when the main
// thread says go() (right after run N, or after a granted GPU turn), so the GPU goes from run N to run
// N+1 without waiting for modeld's CPU work in between (docs/lanes/L4-gpu-sched.md).
class Prefetcher {
public:
    struct Ready {
        Pair pair;
        std::unique_ptr<Supercombo::Prefetch> pf;
        bool ok = false;
        std::string err;
        std::chrono::steady_clock::time_point t_go;  // when the submit started
        double record_ms = 0;
        int rerecords = 0;  // newer pairs recorded while waiting for go
    };
    Prefetcher(Supercombo *model, Shared *sh) : model_(model), sh_(sh), th_([this] { loop(); }) {}
    ~Prefetcher() {
        {
            std::lock_guard<std::mutex> l(m_);
            stop_ = true;
        }
        cv_.notify_all();
        th_.join();
    }
    void go() {
        {
            std::lock_guard<std::mutex> l(m_);
            go_ = true;
        }
        cv_.notify_all();
    }
    bool take(Ready *r, int timeout_ms) {
        std::unique_lock<std::mutex> l(m_);
        if (!cv_.wait_for(l, std::chrono::milliseconds(timeout_ms), [&] { return has_; })) return false;
        *r = std::move(ready_);
        has_ = false;
        cv_.notify_all();
        return true;
    }

private:
    void record(Ready *r) {
        const auto t0 = std::chrono::steady_clock::now();
        const Pair &p = r->pair;
        r->pf.reset();
        r->err.clear();
        r->pf = model_->prefetch(p.prev->med.data(), p.cur->med.data(), p.prev->big.data(), p.cur->big.data(), &r->err);
        r->record_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }

    void loop() {
        pthread_setname_np(pthread_self(), "md-prefetch");
        while (true) {
            {
                std::unique_lock<std::mutex> l(m_);
                cv_.wait(l, [&] { return !has_ || stop_; });  // the previous one was taken
                if (stop_ || !g_run) return;
            }
            Ready r;
            if (!sh_->take(&r.pair, 20)) {
                if (!g_run) return;
                continue;
            }
            record(&r);
            // Wait for go; if a newer pair arrives meanwhile (the current run is long, or the GPU was
            // handed to the sign provider), record that one instead, so the submitted stage 1 always has
            // the newest frame. Re-recording is CPU work done while the GPU is busy.
            for (;;) {
                {
                    std::unique_lock<std::mutex> l(m_);
                    cv_.wait_for(l, std::chrono::milliseconds(2), [&] { return go_ || stop_; });
                    if (stop_) return;
                    if (go_) {
                        go_ = false;
                        break;
                    }
                }
                Pair newer;
                if (sh_->take(&newer, 0)) {
                    r.pair = std::move(newer);
                    record(&r);
                    ++r.rerecords;
                }
            }
            r.t_go = std::chrono::steady_clock::now();
            r.ok = r.pf && model_->submit(*r.pf, &r.err);
            {
                std::lock_guard<std::mutex> l(m_);
                ready_ = std::move(r);
                has_ = true;
            }
            cv_.notify_all();
        }
    }

    Supercombo *model_;
    Shared *sh_;
    std::mutex m_;
    std::condition_variable cv_;
    bool go_ = false, has_ = false, stop_ = false;
    Ready ready_;
    std::thread th_;  // last: starts after the members above exist
};

}  // namespace

int main(int argc, char **argv) {
    Options opt;
    // config file first, CLI overrides
    {
        std::string s;
        if (read_file(config_dir() + "/modeld.json", &s)) {
            double v;
            std::string str;
            if (json_string(s, "backend", &str)) opt.sc.backend = str;
            if (json_string(s, "road_model", &str)) opt.sc.road_model = str;
            if (json_string(s, "policy", &str)) opt.sc.policy_path = str, opt.policy_set = true;
            if (json_number(s, "policy_hz", &v)) opt.sc.policy_hz = v;
            if (json_string(s, "model", &str)) opt.sc.model_path = str, opt.model_set = true;
            if (json_number(s, "threads", &v)) opt.sc.threads = int(v), opt.threads_set = true;
            if (json_string(s, "cpus", &str)) opt.cpus = str, opt.cpus_set = true;
            if (json_string(s, "prep_cpus", &str)) opt.prep_cpus = str;
            if (json_number(s, "rhd", &v)) opt.sc.rhd = v != 0;
            if (json_number(s, "steer_delay", &v)) opt.sc.steer_delay = float(v);
            if (json_number(s, "max_hz", &v)) opt.max_hz = v;
            if (json_number(s, "throttle_temp", &v)) opt.throttle_mc = int(v);
            if (json_string(s, "gpu_sched", &str)) opt.gpu_sched = str;
            if (json_number(s, "gpu_yield_ms", &v)) opt.gpu_yield_ms = int(v);
            if (json_number(s, "gpu_lead_ms", &v)) opt.gpu_lead_ms = v;
            if (json_number(s, "pipeline", &v)) opt.pipeline = v != 0;
            if (json_number(s, "cpu_latency_us", &v)) opt.cpu_latency_us = int(v);
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
        if (a == "--backend") opt.sc.backend = val();
        else if (a == "--road-model") opt.sc.road_model = val();
        else if (a == "--policy") opt.sc.policy_path = val(), opt.policy_set = true;
        else if (a == "--policy-threads") opt.sc.policy_threads = atoi(val().c_str());
        else if (a == "--policy-hz") opt.sc.policy_hz = atof(val().c_str());
        else if (a == "--model") opt.sc.model_path = val(), opt.model_set = true;
        else if (a == "--threads") opt.sc.threads = atoi(val().c_str()), opt.threads_set = true;
        else if (a == "--cpus") opt.cpus = val(), opt.cpus_set = true;
        else if (a == "--ncnn-fp32") opt.sc.ncnn_fp16 = false;
        else if (a == "--ncnn-no-flushall") opt.sc.ncnn_flushall = false;
        else if (a == "--prep-cpus") opt.prep_cpus = val();
        else if (a == "--rhd") opt.sc.rhd = true;
        else if (a == "--steer-delay") opt.sc.steer_delay = float(atof(val().c_str()));
        else if (a == "--spin") opt.sc.allow_spinning = true;
        else if (a == "--no-spin") opt.sc.allow_spinning = false;
        else if (a == "--max-hz") opt.max_hz = atof(val().c_str());
        else if (a == "--throttle-temp") opt.throttle_mc = atoi(val().c_str());
        else if (a == "--runs") opt.max_runs = strtoull(val().c_str(), nullptr, 10);
        else if (a == "--stats") opt.stats_s = atof(val().c_str());
        else if (a == "--gpu-sched") opt.gpu_sched = val();
        else if (a == "--gpu-yield") opt.gpu_yield_ms = atoi(val().c_str());
        else if (a == "--gpu-lead") opt.gpu_lead_ms = atof(val().c_str());
        else if (a == "--no-pipeline") opt.pipeline = false;
        else if (a == "--cpu-latency-us") opt.cpu_latency_us = atoi(val().c_str());
        else if (a == "--pipeline") opt.pipeline = true;
        else if (a == "--verbose" || a == "-v") opt.verbose = true;
        else {
            usage();
            return a == "--help" || a == "-h" ? 0 : 2;
        }
    }
    if (!backend_available(opt.sc.backend)) {
        fprintf(stderr, "modeld: backend '%s' not available in this build (ort%s)\n", opt.sc.backend.c_str(),
                backend_available("ncnn") ? ", ncnn" : "");
        return 2;
    }
    if (is_vision(opt.sc)) {  // road-model defaults, unless set by config or CLI
        if (!opt.model_set)
            opt.sc.model_path = opt.sc.backend == "ncnn" ? "/var/lib/roadsense/models/driving_vision-ncnn.param"
                                                         : "/var/lib/roadsense/models/driving_vision-fp32.onnx";
        if (!opt.policy_set) {
            opt.sc.policy_path = "/var/lib/roadsense/models/driving_policy-fp32.onnx";
            if (access(opt.sc.policy_path.c_str(), R_OK) != 0) opt.sc.policy_path.clear();
        }
        if (opt.sc.policy_path == "none") opt.sc.policy_path.clear();
        fprintf(stderr, "modeld: road model vision, policy %s\n",
                opt.sc.policy_path.empty() ? "none (no plan in `road`)" : opt.sc.policy_path.c_str());
        opt.model_set = true;
    }
    if (opt.sc.backend == "ncnn") {  // backend-specific defaults, unless set by config or CLI
        if (!opt.model_set) opt.sc.model_path = "/var/lib/roadsense/models/supercombo-ncnn.param";
        if (!opt.threads_set) opt.sc.threads = 1;
        if (!opt.cpus_set) opt.cpus = "0-3";
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
    pthread_setname_np(pthread_self(), "md-main");  // the backend's own threads inherit it

    // PM QoS request: valid while the fd is open, dropped by the kernel when modeld exits.
    int qos_fd = -1;
    if (opt.cpu_latency_us >= 0) {
        qos_fd = open("/dev/cpu_dma_latency", O_WRONLY | O_CLOEXEC);
        const int32_t v = opt.cpu_latency_us;
        if (qos_fd < 0 || write(qos_fd, &v, sizeof v) != ssize_t(sizeof v))
            fprintf(stderr, "modeld: cannot set /dev/cpu_dma_latency: %s (continuing)\n", strerror(errno));
        else
            fprintf(stderr, "modeld: CPU wake-up latency request %d us\n", opt.cpu_latency_us);
    }

    std::unique_ptr<Supercombo> model;
    try {
        model = std::make_unique<Supercombo>(opt.sc);
    } catch (const std::exception &e) {
        fprintf(stderr, "modeld: cannot load %s: %s\n", opt.sc.model_path.c_str(), e.what());
        return 1;
    }
    fprintf(stderr, "modeld: backend %s, model %s, %d threads, cpus %s, %s traffic, steer delay %.2f s\n",
            model->backend_name(), opt.sc.model_path.c_str(), opt.sc.threads, opt.cpus.c_str(),
            opt.sc.rhd ? "left-hand" : "right-hand",
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

    GpuSched gs;
    if (opt.gpu_sched != "none") {
        const std::string gp = opt.gpu_sched == "auto" || opt.gpu_sched.empty() ? opt.rundir + "/gpu.sched" : opt.gpu_sched;
        if (gs.open(gp, true)) {
            gs.publish_pid(getpid());
            fprintf(stderr, "modeld: GPU schedule %s, yield %d ms, lead %.1f ms\n", gp.c_str(), opt.gpu_yield_ms, opt.gpu_lead_ms);
        } else {
            fprintf(stderr, "modeld: cannot open GPU schedule %s: %s (continuing without)\n", gp.c_str(), strerror(errno));
        }
    }

    Shared sh;
    std::thread prep(prep_thread, &opt, &sh);
    const bool pipelined = opt.pipeline && model->can_prefetch();
    std::unique_ptr<Prefetcher> pre;
    if (pipelined) pre = std::make_unique<Prefetcher>(model.get(), &sh);
    fprintf(stderr, "modeld: pipelined runs %s\n", pipelined ? "on" : "off");
    bool go_sent = false;
    Prefetcher::Ready job;
    double win_rec = 0;
    uint64_t win_rerec = 0;

    Calib pub_calib;
    Intrinsics pub_k;
    bool calib_published = false;
    uint64_t runs = 0, win_runs = 0;
    double win_model = 0, win_prep = 0, win_model_max = 0, win_post = 0, win_gap = 0, win_yield = 0;
    uint64_t win_granted = 0;
    std::vector<double> win_ms;
    auto last_run_end = std::chrono::steady_clock::time_point{};
    bool turn_out = false;  // a granted GPU turn not yet returned
    int64_t turn_since_ns = 0;
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
    int run_failures = 0;
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
        if (turn_out) {  // the sign provider has the GPU: let it finish (bounded) before the next run
            win_yield += gs.wait_turn_back(turn_since_ns, opt.gpu_yield_ms, opt.gpu_lead_ms);
            turn_out = false;
            continue;  // re-check stop/rate, then take the newest pair
        }
        auto t0 = std::chrono::steady_clock::now();
        if (pipelined) {
            if (!go_sent) pre->go(), go_sent = true;
            if (!pre->take(&job, 20)) continue;
            go_sent = false;
            p = job.pair;
            t0 = job.t_go;
        } else if (!sh.take(&p, 20)) {
            continue;
        } else {
            t0 = std::chrono::steady_clock::now();
        }
        next_run = t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / eff_hz));
        if (!calib_published || !(p.cur->calib == pub_calib) || !(p.cur->k == pub_k)) {
            pub_calib = p.cur->calib;
            pub_k = p.cur->k;
            const std::string cj = calib_json(pub_calib, pub_k, p.cur->seq, "config");
            srv.set_greeting(cj);
            srv.publish(cj);
            calib_published = true;
        }
        bool ok;
        if (pipelined) {
            ok = job.ok && model->run(*job.pf, nullptr, &err, p.cur->ts_mono_ns);
            if (!job.ok) err = job.err;
            win_rec += job.record_ms;
            win_rerec += job.rerecords;
        } else {
            ok = model->run(p.prev->med.data(), p.cur->med.data(), p.prev->big.data(), p.cur->big.data(), nullptr, &err,
                            p.cur->ts_mono_ns);
        }
        if (!ok) {
            fprintf(stderr, "modeld: run failed: %s\n", err.c_str());
            // A lost Vulkan device (e.g. after a GPU hang recovery) never comes back inside this
            // process: exit so systemd restarts modeld with a fresh device instead of failing forever.
            if (++run_failures >= 20) {
                fprintf(stderr, "modeld: %d consecutive run failures, exiting\n", run_failures);
                fflush(stderr);
                std::_Exit(1);  // no teardown: destroying objects on a lost device can block
            }
            continue;
        }
        run_failures = 0;
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (gs.is_open() && gs.run_done(ms, opt.gpu_yield_ms > 0)) {
            turn_out = true;
            turn_since_ns = gpusched_mono_ns();
            ++win_granted;
        }
        // Next run's stage 1 goes to the GPU now (before the JSON work) unless the GPU was handed over
        // or the rate cap says wait; the loop top sends it otherwise.
        if (pipelined && !turn_out && std::chrono::steady_clock::now() >= next_run) pre->go(), go_sent = true;
        if (last_run_end.time_since_epoch().count()) win_gap += std::chrono::duration<double, std::milli>(t0 - last_run_end).count();
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
        last_run_end = std::chrono::steady_clock::now();
        win_post += std::chrono::duration<double, std::milli>(last_run_end - t1).count();
        win_ms.push_back(ms);
        ++runs;
        ++win_runs;
        win_model += ms;
        win_prep += m.prep_ms;
        win_model_max = std::max(win_model_max, ms);
        const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - win_t0).count();
        if (el >= opt.stats_s) {
            std::sort(win_ms.begin(), win_ms.end());
            const double p50 = win_ms[win_ms.size() / 2], p90 = win_ms[std::min(win_ms.size() - 1, win_ms.size() * 9 / 10)];
            fprintf(stderr,
                    "modeld: %.1f runs/s, model %.1f ms avg %.1f max, prep %.1f ms, pair dt %.0f ms, stream %.1f ms, "
                    "skipped %llu, clients %zu, thermal_c %.1f, effective_hz %.1f, p50 %.1f p90 %.1f, post %.1f ms, "
                    "gap %.1f ms, gpu turns %llu yield %.1f ms, rec %.1f ms rerec %llu\n",
                    win_runs / el, win_model / win_runs, win_model_max, win_prep / win_runs, m.pair_dt_ms,
                    p.stream_dt_ms, (unsigned long long)(m.frames_skipped - win_skipped0), srv.client_count(), m.thermal_c, eff_hz,
                    p50, p90, win_post / win_runs, win_gap / win_runs,
                    (unsigned long long)win_granted, win_granted ? win_yield / win_granted : 0.0, win_rec / win_runs, (unsigned long long)win_rerec);
            win_t0 = std::chrono::steady_clock::now();
            win_runs = 0;
            win_model = win_prep = win_model_max = win_post = win_gap = win_yield = win_rec = 0;
            win_granted = 0;
            win_rerec = 0;
            win_ms.clear();
            win_skipped0 = m.frames_skipped;
        }
        if (opt.verbose)
            fprintf(stderr, "modeld: seq %llu/%llu model %.1f ms\n", (unsigned long long)m.seq,
                    (unsigned long long)m.seq_prev, ms);
        if (opt.max_runs && runs >= opt.max_runs) break;
    }
    g_run = false;
    pre.reset();  // stops the prefetch thread (it may hold a pair from the prep thread's queue)
    prep.join();
    if (qos_fd >= 0) close(qos_fd);
    return 0;
}

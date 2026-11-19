// calibrator_test: the online pitch / yaw estimate on synthetic ego-motion.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>

#include <unistd.h>

#include "calibrator.h"

using namespace roadsense;

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { ++fails; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

// The pose a model would report for a camera really mounted at (pitch, yaw) while the warp uses `used`.
static void pose_for(const Calib &used, double true_pitch, double true_yaw, double v, double yaw_rate, std::mt19937 &g,
                     double noise, float pose[6], float st[6]) {
    std::normal_distribution<double> n(0, noise);
    const double ry = true_yaw - used.yaw + n(g), rp = -(true_pitch - used.pitch) + n(g);
    pose[0] = float(v);
    pose[1] = float(v * std::tan(ry));
    pose[2] = float(v * std::tan(rp));
    pose[3] = pose[4] = 0;
    pose[5] = float(yaw_rate);
    for (int i = 0; i < 6; ++i) st[i] = 0.02f;
}

int main() {
    const double D = M_PI / 180;
    Intrinsics k;
    std::mt19937 g(1);
    float pose[6], st[6];

    {   // converges to the truth in a closed loop, in min_blocks blocks, and reports it
        OnlineCalib oc;
        Calib cfg;  // 0, 0, 0
        oc.set_config(cfg, k);
        const double tp = -1.3 * D, ty = -3.4 * D;
        int first_valid = -1;
        for (int i = 0; i < 3000; ++i) {
            const Calib used = oc.calib();
            pose_for(used, tp, ty, 20, 0.001, g, 0.3 * D, pose, st);
            oc.observe(used, pose, st);
            if (first_valid < 0 && oc.status().applied) first_valid = i;
        }
        const OnlineCalibStatus s = oc.status();
        CHECK(first_valid == 499, "first applied after %d samples", first_valid);
        CHECK(std::string(s.state) == "valid" && s.applied, "state %s", s.state);
        CHECK(std::fabs(s.pitch - tp) < 0.05 * D && std::fabs(s.yaw - ty) < 0.05 * D, "estimate %.3f %.3f deg", s.pitch / D, s.yaw / D);
        CHECK(std::fabs(oc.calib().yaw - ty) < 0.3 * D && std::fabs(oc.calib().pitch - tp) < 0.3 * D, "applied");
        CHECK(oc.calib().roll == cfg.roll && oc.calib().height_m == cfg.height_m, "roll / height untouched");
        printf("closed loop: estimate pitch %+.3f yaw %+.3f deg (truth %+.1f %+.1f), spread %.3f %.3f deg, %d blocks\n",
               s.pitch / D, s.yaw / D, tp / D, ty / D, s.spread_pitch / D, s.spread_yaw / D, s.blocks);
        CHECK(oc.status_json().find("\"state\":\"valid\"") != std::string::npos, "json");
    }
    {   // gates: slow, turning, or unsure samples are not used
        OnlineCalib oc;
        oc.set_config(Calib(), k);
        for (int i = 0; i < 2000; ++i) {
            pose_for(Calib(), 0, 2 * D, i % 2 ? 5.0 : 20.0, i % 2 ? 0.0 : 0.03, g, 0, pose, st);
            oc.observe(Calib(), pose, st);
        }
        CHECK(oc.status().accepted == 0, "accepted %llu", (unsigned long long)oc.status().accepted);
        for (int i = 0; i < 2000; ++i) {
            pose_for(Calib(), 0, 2 * D, 20, 0, g, 0, pose, st);
            st[1] = 0.5f;  // 1.4 deg of doubt about the lateral direction
            oc.observe(Calib(), pose, st);
        }
        CHECK(oc.status().accepted == 0 && std::string(oc.status().state) == "collecting", "std gate");
    }
    {   // observe only: estimated, not applied
        OnlineCalibOptions o;
        o.apply = false;
        OnlineCalib oc(o);
        oc.set_config(Calib(), k);
        for (int i = 0; i < 1000; ++i) {
            pose_for(Calib(), 1 * D, 2 * D, 20, 0, g, 0.2 * D, pose, st);
            oc.observe(Calib(), pose, st);
        }
        CHECK(std::string(oc.status().state) == "valid" && !oc.status().applied && oc.calib() == Calib(), "observe mode");
        CHECK(std::fabs(oc.status().yaw - 2 * D) < 0.05 * D, "observe estimate");
    }
    {   // blocks that disagree: not valid, the configured value stays in the warp
        OnlineCalib oc;
        oc.set_config(Calib(), k);
        for (int i = 0; i < 1000; ++i) {
            pose_for(Calib(), 0, ((i / 100) % 2 ? 3.0 : -1.0) * D, 20, 0, g, 0, pose, st);
            oc.observe(Calib(), pose, st);
        }
        CHECK(std::string(oc.status().state) == "unstable" && !oc.status().applied, "unstable: %s", oc.status().state);
    }
    {   // the camera is moved (pitch by 2.5 deg): the estimate follows after min_blocks blocks, not after max_blocks / 2
        OnlineCalib oc;
        oc.set_config(Calib(), k);
        int moved_at = -1;
        for (int i = 0; i < 6000; ++i) {
            const Calib used = oc.calib();
            const double tp = (i < 4000 ? -1.1 : 1.35) * D;
            pose_for(used, tp, -3.4 * D, 20, 0, g, 0.3 * D, pose, st);
            oc.observe(used, pose, st);
            if (i >= 4000 && moved_at < 0 && std::fabs(oc.calib().pitch - tp) < 0.3 * D) moved_at = i - 4000;
        }
        CHECK(moved_at == 499, "followed the moved camera after %d samples", moved_at);
        CHECK(std::string(oc.status().state) == "valid" && std::fabs(oc.status().yaw + 3.4 * D) < 0.1 * D, "after the move");
    }
    {   // a town ride: blocks scatter by 0.9 deg; valid once the median is known to 0.3 deg (14 blocks), not before
        OnlineCalib oc;
        oc.set_config(Calib(), k);
        std::normal_distribution<double> bn(0, 0.9 * D);
        int valid_at = -1;
        for (int b = 0; b < 40; ++b) {
            const double off = bn(g);
            for (int i = 0; i < 100; ++i) {
                const Calib used = oc.calib();
                pose_for(used, 0, 2 * D + off, 12, 0, g, 0.1 * D, pose, st);
                oc.observe(used, pose, st);
            }
            if (valid_at < 0 && std::string(oc.status().state) == "valid") valid_at = b + 1;
        }
        CHECK(valid_at >= 8 && valid_at <= 30, "valid after %d blocks", valid_at);
        CHECK(std::fabs(oc.status().yaw - 2 * D) < 0.5 * D, "town estimate %.2f deg", oc.status().yaw / D);
        printf("town ride: valid after %d blocks, estimate %+.2f deg (truth +2), sigma %.2f, se %.2f deg\n", valid_at,
               oc.status().yaw / D, oc.status().spread_yaw / D, oc.status().se_yaw / D);
    }
    {   // too far from the configured value: refused
        OnlineCalib oc;
        oc.set_config(Calib(), k);
        for (int i = 0; i < 1000; ++i) {
            pose_for(Calib(), 0, 12 * D, 20, 0, g, 0, pose, st);
            oc.observe(Calib(), pose, st);
        }
        CHECK(std::string(oc.status().state) == "out_of_range" && !oc.status().applied, "range: %s", oc.status().state);
    }
    {   // state file: kept for the same configuration, ignored for another; a config change drops the blocks
        char path[] = "/tmp/calibrator_test_XXXXXX";
        const int fd = mkstemp(path);
        if (fd >= 0) close(fd);
        OnlineCalibOptions o;
        o.state_path = path;
        Calib cfg;
        cfg.pitch = 0.01;
        {
            OnlineCalib oc(o);
            oc.set_config(cfg, k);
            for (int i = 0; i < 700; ++i) {
                const Calib used = oc.calib();
                pose_for(used, -2 * D, 3 * D, 20, 0, g, 0.2 * D, pose, st);
                oc.observe(used, pose, st);
            }
            CHECK(oc.status().applied, "applied before restart");
        }
        {
            OnlineCalib oc(o);
            oc.set_config(cfg, k);
            const OnlineCalibStatus s = oc.status();
            CHECK(s.restored && s.applied && s.blocks == 7 && std::fabs(oc.calib().yaw - 3 * D) < 0.1 * D, "restored: blocks %d yaw %.3f", s.blocks, oc.calib().yaw / D);
            Calib other = cfg;
            other.yaw = 0.02;
            oc.set_config(other, k);
            CHECK(oc.status().blocks == 0 && oc.calib() == other, "config change resets");
        }
        {
            OnlineCalib oc(o);
            Intrinsics k2 = k;
            k2.fx = k2.fy = 900;
            oc.set_config(cfg, k2);
            CHECK(!oc.status().restored && oc.status().blocks == 0, "state of another camera ignored");
        }
        remove(path);
    }
    printf(fails ? "calibrator_test: %d FAILED\n" : "calibrator_test: all passed\n", fails);
    return fails ? 1 : 0;
}

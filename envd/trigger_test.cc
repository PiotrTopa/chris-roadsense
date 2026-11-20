// trigger_test: the scene-change trigger of envd (trigger.h) on synthetic frames, and (with --nv12) the
// thumbnails of a raw NV12 sequence as hex lines, for the parity check against tools/envd/rate_sim.py.
//
//   trigger_test                      unit checks, exit status 0 = pass
//   trigger_test --nv12 WxH file      one line per frame: 576 Y, 576 U, 576 V bytes as hex
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "trigger.h"

using namespace roadsense::envd;

namespace {

int fails = 0;
#define CHECK(c)                                                         \
    do {                                                                 \
        if (!(c)) {                                                      \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            ++fails;                                                     \
        }                                                                \
    } while (0)

struct Frame {
    int w, h;
    std::vector<uint8_t> y, uv;
    Frame(int w_, int h_, uint8_t Y, uint8_t U, uint8_t V) : w(w_), h(h_), y(size_t(w_) * h_, Y), uv(size_t(w_) * h_ / 2) {
        for (size_t i = 0; i < uv.size(); i += 2) uv[i] = U, uv[i + 1] = V;
    }
    Nv12View view() const {
        Nv12View v;
        v.y = y.data();
        v.uv = uv.data();
        v.width = w;
        v.height = h;
        v.y_stride = w;
        v.uv_stride = w;
        return v;
    }
};

Thumb thumb_of(const Frame &f) {
    Thumb t;
    make_thumb(f.view(), &t);
    return t;
}

void unit() {
    const Frame grey(1280, 720, 100, 128, 128), bright(1280, 720, 160, 128, 128), red(1280, 720, 100, 110, 170);
    const Thumb g = thumb_of(grey), b = thumb_of(bright), r = thumb_of(red);
    CHECK(g.y[0] == 100 && g.y[Thumb::N - 1] == 100 && g.u[5] == 128);
    ChangeScore s = thumb_change(g, g);
    CHECK(s.dy == 0 && s.dh == 0 && s.dc == 0);
    s = thumb_change(g, b);
    CHECK(s.dy == 60 && s.dh == 1.0 && s.dc == 0);
    s = thumb_change(g, r);
    CHECK(s.dy == 0 && s.dh == 0 && s.dc == 30);  // (|110-128| + |170-128|) / 2
    // half the frame brighter: the histogram moves by half, the mean by half the step
    Frame half(1280, 720, 100, 128, 128);
    for (int yy = 0; yy < 360; ++yy) memset(half.y.data() + size_t(yy) * 1280, 200, 1280);
    s = thumb_change(g, thumb_of(half));
    CHECK(s.dy == 50 && s.dh == 0.5);
    // odd size (block bounds by integer division) and a 1920x1080 frame
    CHECK(thumb_of(Frame(1918, 1080, 77, 128, 128)).y[100] == 77);
    CHECK(thumb_of(Frame(642, 360, 33, 128, 128)).y[Thumb::N - 1] == 33);

    // schedule: period 5 s, a luma step triggers at once and a burst at 1 s for 6 s, then the period again
    TriggerConfig c;  // luma 30, hist 0.5, chroma 10, burst 6 s at 1 s, min gap 1 s
    TriggerSchedule ts(c);
    CHECK(ts.due(0, 5));  // nothing ran yet
    ts.ran(g, 0);
    CHECK(!ts.due(4.9, 5) && ts.due(5.0, 5));
    CHECK(!ts.look(g, 0.5));    // within the minimum gap
    CHECK(!ts.look(b, 0.75));   // still within it
    CHECK(ts.look(b, 1.25));    // luma step 60 > 30
    CHECK(ts.due(1.25, 5));
    ts.ran(b, 1.25);
    CHECK(ts.triggers() == 1);
    CHECK(!ts.due(2.0, 5) && ts.due(2.25, 5));  // burst: one per second
    ts.ran(b, 2.25);
    CHECK(ts.due(7.0, 5));                      // 7.0 < 1.25 + 6: still the burst period
    ts.ran(b, 7.0);
    CHECK(!ts.due(8.0, 5) && !ts.due(11.9, 5) && ts.due(12.0, 5));  // burst over: the period again
    CHECK(!ts.look(b, 8.0));                    // same scene as the last run: no trigger
    // the period may change (the governor); in a burst the shorter of burst period and period applies
    TriggerSchedule t2(c);
    t2.ran(g, 0);
    CHECK(t2.look(b, 2));
    t2.ran(b, 2);
    CHECK(!t2.due(2.5, 5) && t2.due(3.0, 5));
    CHECK(t2.due(3.0, 10));           // in the burst the 1 s burst period applies even with a 10 s period
    CHECK(!t2.due(2.4, 0.5) && t2.due(2.5, 0.5));  // a period shorter than the burst period wins
}

int dump(int w, int h, const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return 1;
    Frame fr(w, h, 0, 0, 0);
    while (fread(fr.y.data(), 1, fr.y.size(), f) == fr.y.size() && fread(fr.uv.data(), 1, fr.uv.size(), f) == fr.uv.size()) {
        const Thumb t = thumb_of(fr);
        std::string s;
        char b[3];
        for (const uint8_t *p : {t.y, t.u, t.v})
            for (int i = 0; i < Thumb::N; ++i) snprintf(b, sizeof b, "%02x", p[i]), s += b;
        puts(s.c_str());
    }
    fclose(f);
    return 0;
}

}  // namespace

int main(int argc, char **argv) {
    if (argc == 4 && !strcmp(argv[1], "--nv12")) {
        int w, h;
        if (sscanf(argv[2], "%dx%d", &w, &h) != 2) return 2;
        return dump(w, h, argv[3]);
    }
    unit();
    if (fails) {
        fprintf(stderr, "trigger_test: %d failures\n", fails);
        return 1;
    }
    fprintf(stderr, "trigger_test: all checks passed\n");
    return 0;
}

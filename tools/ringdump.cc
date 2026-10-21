// ringdump: read the latest frame from the pocopilot frame ring via RingReader
// and write it as <prefix>.pgm (Y plane) and <prefix>.ppm (RGB, full-range
// BT.601 inverse of camerad's NV12). Prints the ring header.
//
//   ringdump [prefix] [--timeout-ms N]      (default prefix: frame)
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ring.h"

using namespace pocopilot;

static uint8_t clamp8(int v) { return uint8_t(v < 0 ? 0 : (v > 255 ? 255 : v)); }

int main(int argc, char **argv) {
    std::string prefix = "frame";
    int timeout_ms = 2000;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--timeout-ms") && i + 1 < argc) timeout_ms = atoi(argv[++i]);
        else prefix = argv[i];
    }
    RingReader rr;
    if (!rr.open()) {
        fprintf(stderr, "ringdump: no valid ring at %s\n", ring_path().c_str());
        return 1;
    }
    const RingHeader *h = rr.header();
    printf("magic=%.8s header_size=%u slot_count=%u slot_size=%u %ux%u y_stride=%u uv_stride=%u format=%u\n",
           h->magic, h->header_size, h->slot_count, h->slot_size, h->width, h->height, h->y_stride, h->uv_stride,
           h->format);
    printf("focal_px=%.1f cx=%.1f cy=%.1f write_seq=%" PRIu64 " camerad_pid=%" PRIu64 " alive=%d\n", h->focal_px,
           h->cx, h->cy, rr.write_seq(), h->camerad_pid, int(rr.writer_alive()));

    FrameView f;
    if (!rr.next(f, timeout_ms)) {
        fprintf(stderr, "ringdump: no frame within %d ms (dropped %" PRIu64 ")\n", timeout_ms, rr.dropped());
        return 1;
    }
    printf("frame seq=%" PRIu64 " ts_mono_ns=%" PRId64 " ts_real_ns=%" PRId64
           " exposure_us=%u analogue_gain=%.2f colour_gains=[%.3f,%.3f]\n",
           f.seq, f.ts_mono_ns, f.ts_real_ns, f.exposure_us, f.analogue_gain, f.colour_gain_r, f.colour_gain_b);

    // follow-up reads to show seq advancing
    FrameView g;
    uint64_t prev = f.seq;
    for (int i = 0; i < 5 && rr.next(g, 1000); ++i) {
        printf("next seq=%" PRIu64 " dt=%.1f ms\n", g.seq, (g.ts_mono_ns - f.ts_mono_ns) / 1e6 / double(g.seq - prev));
        prev = g.seq;
        f.ts_mono_ns = g.ts_mono_ns;
    }
    if (!rr.next(f, timeout_ms)) return 1;

    const int w = int(f.width), hgt = int(f.height);
    std::string pgm = prefix + ".pgm", ppm = prefix + ".ppm";
    FILE *fp = fopen(pgm.c_str(), "wb");
    if (!fp) return 1;
    fprintf(fp, "P5\n%d %d\n255\n", w, hgt);
    for (int r = 0; r < hgt; ++r) fwrite(f.y + size_t(r) * f.y_stride, 1, size_t(w), fp);
    fclose(fp);

    std::vector<uint8_t> rgb(size_t(w) * hgt * 3);
    for (int r = 0; r < hgt; ++r) {
        const uint8_t *yr = f.y + size_t(r) * f.y_stride;
        const uint8_t *uvr = f.uv + size_t(r / 2) * f.uv_stride;
        for (int x = 0; x < w; ++x) {
            const int Y = yr[x], U = uvr[(x & ~1)] - 128, V = uvr[(x & ~1) + 1] - 128;
            uint8_t *o = &rgb[(size_t(r) * w + x) * 3];
            o[0] = clamp8(Y + ((91881 * V + 32768) >> 16));               // 1.402
            o[1] = clamp8(Y - ((22554 * U + 46802 * V + 32768) >> 16));   // 0.344136, 0.714136
            o[2] = clamp8(Y + ((116130 * U + 32768) >> 16));              // 1.772
        }
    }
    fp = fopen(ppm.c_str(), "wb");
    if (!fp) return 1;
    fprintf(fp, "P6\n%d %d\n255\n", w, hgt);
    fwrite(rgb.data(), 1, rgb.size(), fp);
    fclose(fp);
    printf("wrote %s %s (seq %" PRIu64 ", dropped %" PRIu64 ")\n", pgm.c_str(), ppm.c_str(), f.seq, rr.dropped());
    return 0;
}

#!/usr/bin/env python3
"""Grab still pictures for the lens calibration from the running camera, without touching the stack.

    python3 capture.py --out ~/calib-shots [--count 40] [--every 1.5] [--source ring|preview] [--all]

Runs on the camera host (the phone). Python standard library only: no OpenCV, no numpy, no new packages.
It only reads: the frame ring (`/dev/shm/roadsense-frames`, mapped read-only, world-readable; the reader protocol
of docs/ARCHITECTURE.md "Frame ring") or camerad's preview picture (`<rundir>/preview.jpg`). Nothing is stopped,
restarted or written outside --out. The pictures are solved on the laptop (tools/calib/lens.py).

  --source ring     (default) the luma plane of the full-size frame as a PGM file: lossless, any camera
  --source preview  camerad's preview JPEG, copied as it is (camerad-dvr: the camera's own 1280x720 JPEG;
                    the libcamera camerad's preview is half size: use the ring there). Stops while the load
                    governor has paused the preview.

A picture is kept when the scene holds still (two frames 0.2 s apart look the same, so the board was not moving
and is not smeared) and differs from the last kept picture (the board was moved). `--all` keeps one picture every
--every seconds regardless. It prints one line per kept picture and ends after --count pictures or Ctrl-C.
"""
import argparse
import mmap
import os
import shutil
import struct
import sys
import time

HDR_FMT = "<8s8I3f4xQQ"           # RingHeader (docs/ARCHITECTURE.md), 72 bytes
SLOT_FMT = "<QqqIfffII"           # SlotHeader, 48 bytes
WRITE_SEQ_OFF = 56
STEP, ROW_GAP = 16, 4             # the stillness test compares means over cells of 16 pixels x 4 rows (every 4th
                                  # row), one cell every 32 pixels: about 900 cells of a 1280x720 frame


class Ring:
    """Read-only reader of the frame ring. Remaps when camerad replaced the file (new inode)."""

    def __init__(self, path):
        self.path, self.m, self.ino = path, None, None

    def _map(self):
        st = os.stat(self.path)
        if self.m is not None and st.st_ino == self.ino:
            return
        if self.m is not None:
            self.m.close()
        with open(self.path, "rb") as f:
            self.m = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        self.ino = st.st_ino
        (magic, self.header_size, self.slots, self.slot_size, self.width, self.height, self.y_stride, self.uv_stride,
         fmt, self.focal, self.cx, self.cy, _ws, _pid) = struct.unpack_from(HDR_FMT, self.m, 0)
        if magic != b"RSFRAME1" or fmt != 1:
            raise RuntimeError("%s is not a roadsense frame ring" % self.path)

    def latest(self):
        """(seq, ts_real_ns, luma bytes with stride y_stride) of the newest complete frame, or None."""
        self._map()
        for _ in range(4):
            seq = struct.unpack_from("<Q", self.m, WRITE_SEQ_OFF)[0]
            if seq == 0:
                return None
            slot = self.header_size + (seq % self.slots) * self.slot_size
            sseq, _mono, real, _e, _g, _r, _b, off, size = struct.unpack_from(SLOT_FMT, self.m, slot)
            if sseq != seq:
                continue                                    # lapped between the two reads
            y = self.m[slot + off:slot + off + self.y_stride * self.height]
            if struct.unpack_from("<Q", self.m, slot)[0] == seq:
                return seq, real, y                         # not overwritten while it was copied
        return None

    def info(self):
        self._map()
        return {"width": self.width, "height": self.height, "focal_px": round(self.focal, 3), "cx": round(self.cx, 3),
                "cy": round(self.cy, 3)}


def thumb(y, stride, width, height):
    """A coarse grid of the luma plane: sums over cells of STEP pixels by 4 rows (averaging takes the sensor noise
    out, so that "holds still" also works in a dim garage)."""
    out = []
    for r in range(0, height - 4 * ROW_GAP + 1, 2 * STEP):
        rows = [y[(r + k * ROW_GAP) * stride:(r + k * ROW_GAP) * stride + width] for k in range(4)]
        for c in range(0, width - STEP + 1, 2 * STEP):
            out.append(sum(rows[0][c:c + STEP]) + sum(rows[1][c:c + STEP]) + sum(rows[2][c:c + STEP])
                       + sum(rows[3][c:c + STEP]))
    return out


def diff(a, b):
    """Mean absolute difference of two thumbs, in grey levels."""
    if a is None or b is None or len(a) != len(b) or not a:
        return 255.0
    return sum(abs(p - q) for p, q in zip(a, b)) / float(len(a) * 4 * STEP)


def write_pgm(path, y, stride, width, height):
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (width, height))
        if stride == width:
            f.write(y)
        else:
            for r in range(height):
                f.write(y[r * stride:r * stride + width])
    os.replace(tmp, path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, help="directory for the pictures (created)")
    ap.add_argument("--count", type=int, default=40, help="pictures to keep (default 40)")
    ap.add_argument("--every", type=float, default=1.5, help="shortest time between kept pictures, s (default 1.5)")
    ap.add_argument("--source", choices=("ring", "preview"), default="ring")
    ap.add_argument("--ring", default=os.environ.get("ROADSENSE_RING", "/dev/shm/roadsense-frames"))
    ap.add_argument("--preview", default=os.path.join(os.environ.get("ROADSENSE_RUNDIR", "/run/roadsense"), "preview.jpg"))
    ap.add_argument("--all", action="store_true", help="keep a picture every --every seconds, still or not")
    ap.add_argument("--still", type=float, default=2.0, help="largest change between two frames 0.2 s apart for "
                    "\"holds still\", grey levels (default 2.0)")
    ap.add_argument("--moved", type=float, default=4.0, help="smallest change from the last kept picture, grey "
                    "levels (default 4.0)")
    ap.add_argument("--timeout", type=float, default=0.0, help="give up after this many seconds (0 = never)")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    kept, last_kept, last_t, t_end = 0, None, 0.0, (time.monotonic() + a.timeout if a.timeout > 0 else None)

    if a.source == "preview":
        last_mtime = None
        print("capture: copying %s into %s; move the board between pictures" % (a.preview, a.out), flush=True)
        while kept < a.count and (t_end is None or time.monotonic() < t_end):
            time.sleep(0.1)
            try:
                mt = os.stat(a.preview).st_mtime_ns
            except OSError:
                continue
            if mt == last_mtime or time.monotonic() - last_t < a.every:
                continue
            last_mtime, last_t, kept = mt, time.monotonic(), kept + 1
            dst = os.path.join(a.out, "shot-%04d.jpg" % kept)
            shutil.copyfile(a.preview, dst + ".tmp")
            os.replace(dst + ".tmp", dst)
            print("capture: %d/%d %s" % (kept, a.count, dst), flush=True)
        return 0 if kept else 1

    ring = Ring(a.ring)
    try:
        info = ring.info()
    except (OSError, RuntimeError, struct.error) as e:
        print("capture: no frame ring at %s (%s): is the camera daemon running?" % (a.ring, e), file=sys.stderr)
        return 1
    print("capture: ring %dx%d, pictures into %s; hold the board still, then move it; Ctrl-C ends"
          % (info["width"], info["height"], a.out), flush=True)
    with open(os.path.join(a.out, "capture.txt"), "w") as f:
        f.write("source ring %(width)dx%(height)d, configured focal_px %(focal_px)s cx %(cx)s cy %(cy)s\n" % info)
    prev, last_seq, waited = None, 0, 0.0
    try:
        while kept < a.count and (t_end is None or time.monotonic() < t_end):
            time.sleep(0.2)
            try:
                fr = ring.latest()
            except (OSError, RuntimeError, struct.error):
                fr = None
            if fr is None or fr[0] == last_seq:
                waited += 0.2
                if waited >= 5.0:
                    print("capture: no new frame for 5 s (camera stopped?)", flush=True)
                    waited = 0.0
                continue
            waited = 0.0
            seq, real, y = fr
            last_seq = seq
            th = thumb(y, ring.y_stride, ring.width, ring.height)
            still = a.all or diff(th, prev) <= a.still
            prev = th
            if not still or time.monotonic() - last_t < a.every:
                continue
            if not a.all and diff(th, last_kept) < a.moved:
                continue
            kept, last_kept, last_t = kept + 1, th, time.monotonic()
            dst = os.path.join(a.out, "shot-%04d-seq%d.pgm" % (kept, seq))
            write_pgm(dst, y, ring.y_stride, ring.width, ring.height)
            print("capture: %d/%d %s" % (kept, a.count, dst), flush=True)
    except KeyboardInterrupt:
        pass
    print("capture: %d pictures in %s" % (kept, a.out), flush=True)
    return 0 if kept else 1


if __name__ == "__main__":
    sys.exit(main())

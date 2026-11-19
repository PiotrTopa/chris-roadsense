#!/usr/bin/env python3
"""mjpeg_nv12.py RIDE.mjpeg START_FRAME N_FRAMES OUT [OUT2 ...]   (lane L28; for hosts without ffmpeg, e.g. the phone)

Decodes frames [START_FRAME, START_FRAME + N_FRAMES) of a concatenated MJPEG stream (one JPEG per frame, as the
rides are stored) to raw NV12, full range (BT.601 YCbCr as the JPEG holds it; chroma 2x2-averaged), and writes
every frame to each OUT in turn (files or FIFOs; '-' = stdout). Feeds modeld_replay /dev/stdin (or a FIFO) on a
host where ffmpeg is missing; several OUTs let two arithmetic paths read the same decoded frames at once.
Needs Pillow and numpy.
"""
import io
import sys

import numpy as np
from PIL import Image


def frames(path):
    """Yields each JPEG of the stream (SOI .. EOI)."""
    buf = b""
    with open(path, "rb") as f:
        while True:
            chunk = f.read(1 << 20)
            if not chunk:
                break
            buf += chunk
            while True:
                a = buf.find(b"\xff\xd8")
                if a < 0:
                    buf = buf[-1:]
                    break
                b = buf.find(b"\xff\xd9", a + 2)
                if b < 0:
                    buf = buf[a:]
                    break
                yield buf[a:b + 2]
                buf = buf[b + 2:]


def nv12(jpeg):
    im = Image.open(io.BytesIO(jpeg)).convert("YCbCr")
    a = np.asarray(im)
    h, w = a.shape[0] & ~1, a.shape[1] & ~1
    a = a[:h, :w]
    y = a[:, :, 0]
    c = a[:, :, 1:].astype(np.uint16).reshape(h // 2, 2, w // 2, 2, 2).sum(axis=(1, 3))
    uv = ((c + 2) // 4).astype(np.uint8)
    return y.tobytes() + uv.tobytes()


def main():
    path, s0, n = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    outs = [sys.stdout.buffer if o == "-" else open(o, "wb") for o in sys.argv[4:]]
    k = 0
    for i, jp in enumerate(frames(path)):
        if i < s0:
            continue
        if k >= n:
            break
        data = nv12(jp)
        for o in outs:
            o.write(data)
        k += 1
    for o in outs:
        o.close()
    print("mjpeg_nv12: %d frames from %d" % (k, s0), file=sys.stderr)


if __name__ == "__main__":
    main()

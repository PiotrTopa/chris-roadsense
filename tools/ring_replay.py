#!/usr/bin/env python3
"""Write a raw NV12 sequence into a roadsense frame ring at a given rate (a fake camerad for
laptop tests of the ring consumers: envd, signd, modeld).

    tools/ring_replay.py --nv12 1280x720 seq.nv12 [--fps 20] [--loop] [--ring /tmp/roadsense-frames]

Implements the writer protocol of docs/ARCHITECTURE.md (header 4096 bytes, 6 slots, slot seq stored
before the pixels, write_seq after). Does not serve camerad.sock or preview.jpg.
"""
import argparse
import mmap
import os
import struct
import time

HDR_FMT = "<8s8I3f4xQQ"          # RingHeader, 72 bytes
SLOT_FMT = "<QqqIfffII"          # SlotHeader, 48 bytes
HEADER_SIZE, SLOTS, DATA_OFFSET = 4096, 6, 4096


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--nv12", required=True, help="WxH")
    ap.add_argument("file")
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--ring", default=os.environ.get("ROADSENSE_RING", "/dev/shm/roadsense-frames"))
    ap.add_argument("--focal", type=float, default=975.0)
    ap.add_argument("--frames", type=int, default=0)
    a = ap.parse_args()
    w, h = (int(v) for v in a.nv12.split("x"))
    frame_bytes = w * h * 3 // 2
    data_size = frame_bytes
    slot_size = (DATA_OFFSET + data_size + 4095) // 4096 * 4096
    size = HEADER_SIZE + SLOTS * slot_size
    tmp = a.ring + ".tmp." + str(os.getpid())
    with open(tmp, "wb") as f:
        f.truncate(size)
    fd = os.open(tmp, os.O_RDWR)
    m = mmap.mmap(fd, size)
    m[:HEADER_SIZE] = b"\0" * HEADER_SIZE
    m[0:struct.calcsize(HDR_FMT)] = struct.pack(HDR_FMT, b"RSFRAME1", HEADER_SIZE, SLOTS, slot_size, w, h, w, w, 1,
                                                a.focal, w / 2, h / 2, 0, os.getpid())
    os.replace(tmp, a.ring)
    seq = 0
    period = 1.0 / a.fps
    nxt = time.monotonic()
    try:
        while True:
            with open(a.file, "rb") as f:
                while True:
                    px = f.read(frame_bytes)
                    if len(px) < frame_bytes:
                        break
                    seq += 1
                    slot = HEADER_SIZE + (seq % SLOTS) * slot_size
                    mono, real = time.monotonic_ns(), time.time_ns()
                    m[slot:slot + 8] = struct.pack("<Q", seq)                 # slot seq first
                    m[slot:slot + struct.calcsize(SLOT_FMT)] = struct.pack(SLOT_FMT, seq, mono, real, 0, 0.0, 0.0, 0.0,
                                                                            DATA_OFFSET, data_size)
                    m[slot + DATA_OFFSET:slot + DATA_OFFSET + frame_bytes] = px
                    m[56:64] = struct.pack("<Q", seq)                         # write_seq last
                    if a.frames and seq >= a.frames:
                        return
                    nxt += period
                    time.sleep(max(0.0, nxt - time.monotonic()))
            if not a.loop:
                return
    except KeyboardInterrupt:
        pass
    finally:
        m.close()
        os.close(fd)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Camera glass-to-host latency with the RP2040 LED reference (ledlat firmware on /dev/ttyACM0).
Random-phase LED-on events; detect the LED in the frames as a cluster of pixels that got much
brighter than the dark reference; latency = frame receive time - serial write time."""
import sys, os, time, io, random, statistics, serial
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))  # tools/dvr_probe.py
import dvr_probe as pp
from PIL import Image, ImageChops
N = int(sys.argv[2]) if len(sys.argv) > 2 else 20
be = pp.BlockBackend(sys.argv[1] if len(sys.argv) > 1 else "/dev/sda"); cam = pp.Camera(be)
ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5); time.sleep(0.3); ser.read(100)
def led(on):
    t0 = time.monotonic(); ser.write(b"1" if on else b"0"); r = ser.readline(); t1 = time.monotonic()
    return t0, t1
def next_frame(last_seq, timeout=1.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        info = cam.info()
        if info["valid"] and info["video_seq"] != last_seq:
            t = time.monotonic(); jpg, err = cam.read_frame(info)
            if jpg is None: last_seq = info["video_seq"]; continue
            return info["video_seq"], t, jpg
        time.sleep(0.001)
    return last_seq, None, None
def small(jpg): return Image.open(io.BytesIO(jpg)).convert("L").resize((320, 180))
def bright_pixels(a, b, thr=60):
    d = ImageChops.subtract(b, a)  # pixels that got brighter
    return sum(1 for v in d.getdata() if v > thr)
led(False); seq, _, jpg = next_frame(-1)
for _ in range(10): seq, _, jpg = next_frame(seq)
ref = small(jpg)
# sanity: can we see the LED at all?
led(True); got = 0
for _ in range(6):
    seq, _, jpg = next_frame(seq); got = max(got, bright_pixels(ref, small(jpg)))
led(False)
print(f"LED visibility: {got} brightened pixels (need > 3)")
if got <= 3: sys.exit("LED not visible to the camera; move the Pico into view")
for _ in range(6): seq, _, jpg = next_frame(seq)
lat = []; frames_idx = []
for i in range(N):
    for _ in range(2): seq, _, jpg = next_frame(seq)
    ref = small(jpg)
    time.sleep(random.uniform(0.0, 0.1))  # random phase relative to the frame clock
    t0, t1 = led(True)
    for k in range(8):
        seq, t, jpg = next_frame(seq)
        if t is None: break
        if bright_pixels(ref, small(jpg)) > 3:
            lat.append((t - t0) * 1000); frames_idx.append(k + 1); break
    led(False)
    for _ in range(3): seq, _, jpg = next_frame(seq)
lat.sort()
print(f"n={len(lat)}  min {lat[0]:.0f}  p25 {lat[len(lat)//4]:.0f}  median {lat[len(lat)//2]:.0f}  p75 {lat[3*len(lat)//4]:.0f}  max {lat[-1]:.0f} ms;  detected on frame #: {sorted(set(frames_idx))}")
print("all:", " ".join(f"{x:.0f}" for x in lat))

#!/usr/bin/env python3
"""evidence.py CAND RIDE : lane evidence maps (640x360 uint8 per frame) of a learned candidate on the extracted
5 fps frames -> $SCRATCH/l29/evid/RIDE.CAND.u8 ; prints ms per frame (1 thread).
CAND: twinlite | yolop640 | yolop320 | ufld"""
import importlib.util
import os
import sys
import time
import numpy as np
from PIL import Image, ImageDraw

DL = os.environ.get("L29_DL", os.path.expanduser("~/datasets/l29/dl"))        # the candidates' files (docs/lanes/L29-display-lanes.md)
FR = os.environ.get("L29_FRAMES", os.path.join(os.environ.get("SCRATCH", os.path.expanduser("~/scratch")), "l29", "frames"))
EV = os.environ.get("L29_EVID", os.path.join(os.environ.get("SCRATCH", os.path.expanduser("~/scratch")), "l29", "evid"))
W, H, EW, EH = 1280, 720, 640, 360
MEAN = np.array([0.485, 0.456, 0.406], np.float32).reshape(3, 1, 1)
STD = np.array([0.229, 0.224, 0.225], np.float32).reshape(3, 1, 1)
UFLD_ROWS = [121, 131, 141, 150, 160, 170, 180, 189, 199, 209, 219, 228, 238, 248, 258, 267, 277, 287]


def rgb(buf):
    y = buf[:W * H].reshape(H, W).astype(np.float32)
    uv = buf[W * H:].reshape(H // 2, W // 2, 2).astype(np.float32) - 128
    cb = np.repeat(np.repeat(uv[..., 0], 2, 0), 2, 1); cr = np.repeat(np.repeat(uv[..., 1], 2, 0), 2, 1)
    return np.clip(np.stack([y + 1.402 * cr, y - 0.344136 * cb - 0.714136 * cr, y + 1.772 * cb], -1), 0, 255).astype(np.uint8)


def main():
    cand, ride = sys.argv[1], sys.argv[2]
    n = int(open(f"{FR}/{ride}.txt").read().split()[3])
    if len(sys.argv) > 3:
        n = min(n, int(sys.argv[3]))
    if cand == "twinlite":
        import torch
        torch.set_num_threads(1)
        torch.set_flush_denormal(True)
        spec = importlib.util.spec_from_file_location("TwinLite", f"{DL}/twinlite/model/TwinLite.py")
        mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
        net = mod.TwinLiteNet()
        sd = torch.load(f"{DL}/twinlite/pretrained/best.pth", map_location="cpu", weights_only=True)
        net.load_state_dict({k.replace("module.", "", 1): v for k, v in sd.items()}); net.eval()

        def run(img):
            x = torch.from_numpy(np.asarray(Image.fromarray(img).resize((640, 360), Image.BILINEAR), np.float32).transpose(2, 0, 1)[None] / 255.0)
            with torch.no_grad():
                _, ll = net(x)
            p = torch.softmax(ll, 1)[0, 1].numpy()
            return (p * 255).astype(np.uint8)
    elif cand in ("yolop640", "yolop320"):
        import onnxruntime as ort
        S = 640 if cand == "yolop640" else 320
        so = ort.SessionOptions(); so.intra_op_num_threads = 1
        sess = ort.InferenceSession(f"{DL}/yolop/yolop-{S}-{S}.onnx", so, providers=["CPUExecutionProvider"])
        hh = S * 9 // 16; pad = (S - hh) // 2

        def run(img):
            im = np.asarray(Image.fromarray(img).resize((S, hh), Image.BILINEAR), np.float32).transpose(2, 0, 1) / 255.0
            x = np.full((1, 3, S, S), 114 / 255.0, np.float32)
            x[0, :, pad:pad + hh] = im
            x = (x - MEAN) / STD
            ll = sess.run(["lane_line_seg"], {"images": x})[0][0, 1, pad:pad + hh]
            return np.asarray(Image.fromarray((np.clip(ll, 0, 1) * 255).astype(np.uint8)).resize((EW, EH), Image.BILINEAR))
    elif cand == "ufld":
        import onnxruntime as ort
        so = ort.SessionOptions(); so.intra_op_num_threads = 1
        sess = ort.InferenceSession(f"{DL}/ufld1/ufld.onnx", so, providers=["CPUExecutionProvider"])
        idx = np.arange(200, dtype=np.float32).reshape(-1, 1, 1)

        def run(img):
            im = np.asarray(Image.fromarray(img).resize((800, 288), Image.BILINEAR), np.float32).transpose(2, 0, 1) / 255.0
            out = sess.run(None, {"serving_default_args_0": ((im - MEAN) / STD)[None]})[0][0]      # 201, 18, 4
            e = np.exp(out[:200] - out[:200].max(0, keepdims=True)); prob = e / e.sum(0, keepdims=True)
            loc = (prob * idx).sum(0)                                                              # 18, 4
            none = out.argmax(0) == 200
            ev = Image.new("L", (EW, EH), 0); d = ImageDraw.Draw(ev)
            for lane in range(4):
                pts = [(float(loc[r, lane]) * (800.0 / 199.0) * EW / 800.0, UFLD_ROWS[r] * EH / 288.0)
                       for r in range(18) if not none[r, lane]]
                if len(pts) >= 2:
                    d.line(pts, fill=255, width=3)
            return np.asarray(ev)
    else:
        sys.exit("unknown candidate")
    ms = 0.0
    with open(f"{FR}/{ride}.nv12", "rb") as f, open(f"{EV}/{ride}.{cand}.u8.tmp", "wb") as o:
        for k in range(n):
            buf = np.frombuffer(f.read(W * H * 3 // 2), np.uint8)
            img = rgb(buf)
            t0 = time.perf_counter()
            ev = run(img)
            ms += (time.perf_counter() - t0) * 1e3
            assert ev.shape == (EH, EW), ev.shape
            o.write(ev.tobytes())
    import os
    if len(sys.argv) <= 3:
        os.rename(f"{EV}/{ride}.{cand}.u8.tmp", f"{EV}/{ride}.{cand}.u8")
    print(f"{cand} {ride} frames {n} ms_per_frame_1thread {ms / n:.1f} (incl. resize and decode)", flush=True)


if __name__ == "__main__":
    main()

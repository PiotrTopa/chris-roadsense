# openpilot v0.9.7 "supercombo" driving model: standalone interface spec (C++ / ONNX Runtime CPU)

Status: research spec, 2026-10-21. Everything below was read from openpilot at tag **v0.9.7** (and the
ONNX file itself) unless it says otherwise. The graph facts (names, shapes, dtypes, slices, Gather
indices, constants inside the graph) were dumped from the actual `.onnx` with the `onnx` Python package.
Where the source disagrees with itself, the authoritative code is quoted.

Model file: `selfdrive/modeld/models/supercombo.onnx` @ v0.9.7, 51,452,435 bytes,
sha256 `b31b504bc0b440d3bc72967507a00eb4f112285626fbfb3135011500325ee6d6`.
ONNX metadata: `vision_model = 6e252e9c-…/500`, `policy_model = 86d23b15-…/700`, producer pytorch 2.2.2,
IR 7, **opset 14** (default domain only).

Local copies in the scratchpad (`/tmp/claude-1000/-home-piotr/4097b1e0-437a-49af-9dad-9e35d3c588f6/scratchpad/`):

| file | what |
|---|---|
| `models/supercombo-v0.9.7.onnx` | original (fp16 weights + fp16 I/O) |
| `models/supercombo-v0.9.7-fp32.onnx` | same graph with fp16 initializers/I/O converted to fp32 exactly as openpilot's `onnxmodel.py::convert_fp16_to_fp32` does (102,734,802 B, sha256 `d2d83a46…c687`) |
| `models/v0.11.1/driving_vision.onnx`, `driving_policy.onnx` | split models of the last tagged release (for section 6) |
| `models/master/driving_supercombo.onnx` | current master single graph (for section 6) |
| `src097/`, `srcmaster/`, `srcflow/` | the source files this spec was written from |
| `scripts/*.py` | inspection / benchmark scripts |

Coordinate frames used by openpilot (from `common/transformations/camera.py`):

* **device frame** (and the **calibrated frame**, which is the device frame rotated to align with the road/car): x forward, y right, z down.
* **view frame** (camera optical frame): x right, y down, z forward.
* `view_frame_from_device_frame = [[0,1,0],[0,0,1],[1,0,0]]`.

---

## 1. ONNX inputs (v0.9.7)

All inputs are **float16** in the original file. openpilot never feeds fp16 on PC: its ONNX runner converts
the model to fp32 at load time (`convert_fp16_to_fp32`) and feeds float32. In C++ either load
`supercombo-v0.9.7-fp32.onnx` and feed `float`, or load the original and feed `Ort::Float16_t`.
There are **no** nav inputs (`nav_features`, `nav_instructions`) in v0.9.7.

| # | name | shape | elements | dtype | per-frame update |
|---|---|---|---|---|---|
| 0 | `input_imgs` | [1, 12, 128, 256] | 393,216 | fp16 | road camera, frames t-1 and t, warped to the **medmodel** frame, YUV packed (section 2) |
| 1 | `big_input_imgs` | [1, 12, 128, 256] | 393,216 | fp16 | wide camera, frames t-1 and t, warped to the **sbigmodel** frame, same packing |
| 2 | `desire` | [1, 100, 8] | 800 | fp16 | FIFO of one-hot desire *pulses*, 100 x 20 Hz = 5 s, newest at row 99 |
| 3 | `traffic_convention` | [1, 2] | 2 | fp16 | one-hot: `[1,0]` = right-hand traffic (LHD car), `[0,1]` = left-hand traffic (RHD) |
| 4 | `lateral_control_params` | [1, 2] | 2 | fp16 | `[v_ego (m/s), steer_delay (s)]` |
| 5 | `prev_desired_curv` | [1, 100, 1] | 100 | fp16 | FIFO of the model's own previous `desired_curvature` mean, newest at row 99 |
| 6 | `features_buffer` | [1, 99, 512] | 50,688 | fp16 | FIFO of the model's own previous `hidden_state` outputs, newest at row 98 |

Single output: `outputs`, shape [1, 6504], fp16 (section 3).

### 1.1 How openpilot fills them (`selfdrive/modeld/modeld.py`, v0.9.7)

Initial values: **all zeros** (`np.zeros(...)` for every buffer; the image double-buffer
`input_frames = std::make_unique<float[]>(buf_size)` is value-initialised to zeros too, so the very first
run sees a black "previous" frame).

Per frame, in this exact order:

```python
# desire: rising-edge pulse, then FIFO shift by one row (8 floats)
inputs['desire'][0] = 0                          # index 0 = Desire.none is never set
self.inputs['desire'][:-8] = self.inputs['desire'][8:]
self.inputs['desire'][-8:] = np.where(inputs['desire'] - self.prev_desire > .99, inputs['desire'], 0)
self.prev_desire[:] = inputs['desire']

self.inputs['traffic_convention'][:] = inputs['traffic_convention']
self.inputs['lateral_control_params'][:] = inputs['lateral_control_params']
# images: frame.prepare() shifts slot1 -> slot0 and writes the new frame into slot1
...
self.model.execute()
outputs = self.parser.parse_outputs(self.slice_outputs(self.output))

# AFTER the run: feed-back FIFOs
self.inputs['features_buffer'][:-512] = self.inputs['features_buffer'][512:]
self.inputs['features_buffer'][-512:] = outputs['hidden_state'][0, :]
self.inputs['prev_desired_curv'][:-1] = self.inputs['prev_desired_curv'][1:]
self.inputs['prev_desired_curv'][-1:] = outputs['desired_curvature'][0, :]
```

Semantics, stated for a C++ port:

* `desire` (100x8, row-major, row = time, newest last). The one-hot desire for this frame is
  `vec[d] = 1` for `d = DesireHelper.desire` (0 <= d < 8), then `vec[0] = 0`. Only the **rising edge**
  is written (a value that was already 1 last frame becomes 0), so a lane change is a single 1 in the
  newest row, which then scrolls back through the 5 s window. Desire enum (`cereal/log.capnp`):
  0 none, 1 turnLeft, 2 turnRight, 3 laneChangeLeft, 4 laneChangeRight, 5 keepLeft, 6 keepRight, 7 unused.
  Inside the graph the whole 800-vector is used: `Reshape(desire,[1,-1])` -> `Concat([desire_800, traffic_convention_2])` -> `Gemm(512x802)`.
* `traffic_convention`: `traffic_convention[int(is_rhd)] = 1`, the rest 0. Not a FIFO.
* `lateral_control_params = [carState.vEgo, CP.steerActuatorDelay + 0.2]` (source: "use .2s extra for now
  to estimate other delays"). Not a FIFO. Without a car, use your own speed estimate (GPS / the model's
  own `pose` vx) and a nominal delay (e.g. 0.3 to 0.5 s).
* `prev_desired_curv` (100x1, newest last). Pushed **after** each run with the model's own
  `desired_curvature` mean (raw output index 5990). **Only the newest row is read by the graph**:
  `Gather(prev_desired_curv, index=99, axis=1)`. Rows 0..98 are dead inputs in this model, but keep the FIFO
  for fidelity.
* `features_buffer` (99x512, newest last). Pushed **after** each run with `hidden_state`
  (raw output 5992..6503). **The graph reads only 8 rows**: `Gather(features_buffer, indices=[-5,-10,-15,-20,-25,-30,-35,-40], axis=1)`,
  i.e. rows 94, 89, 84, 79, 74, 69, 64, 59. Since row 98 holds the hidden state of the previous run, row
  `99-k` holds the state from k frames ago; the model therefore sees its own features from **5, 10, ..., 40
  frames (0.25 s ... 2.0 s) ago at 20 Hz**. These 8 tokens plus the current frame's feature and one token
  built from `desire+traffic_convention` (10 tokens) go through a small attention block (`/_encode`,
  `/_attention`, `/_mlp`).
* The current `hidden_state` output is the L2-normalised summariser vector (`/summarizer/Div` =
  `mu / clip(sqrt(sum(mu^2)))`); its norm is 1.000 (checked in the smoke test).

**Timing assumption baked into the model:** 20 Hz. The image pair is (previous frame, current frame), 50 ms
apart; the feature taps are 5-frame multiples. openpilot skips the model execution (but still updates the
image double-buffer, `prepare_only`) when a camera frame was dropped; the FIFOs are then not advanced. If
your device cannot sustain 20 Hz, the input semantics break (see section 5.4).

---

## 2. Image preprocessing (exact)

### 2.1 Camera input format

openpilot's camerad delivers **NV12**: a Y plane (`width x height`, row stride `stride`) followed, at byte
offset `uv_offset`, by one interleaved UV plane (`width/2 x height/2` samples, same row stride,
U at even bytes, V at odd bytes). From `transforms/transform.cc`:

```c
const int in_uv_width = in_width/2;  const int in_uv_height = in_height/2;
const int in_uv_px_stride = 2;
const int in_u_offset = in_uv_offset;
const int in_v_offset = in_uv_offset + 1;
```

For I420/YV12/YUV_420_888 sources only the plane pointer, row stride and pixel stride change.

### 2.2 Warp (perspective, model pixel -> camera pixel)

Output per camera per frame: Y 512x256, U 256x128, V 256x128, uint8. The 3x3 matrix `M` maps a
**destination (model) pixel to a source (camera) pixel** (this is the `warp_matrix` from section 2.4,
passed flattened row-major). Kernel `transforms/transform.cl::warpPerspective` (an OpenCV clone):

```
for each dst pixel (dx, dy) (integer, 0-based):
  X0 = M0*dx + M1*dy + M2;  Y0 = M3*dx + M4*dy + M5;  W = M6*dx + M7*dy + M8
  W  = (W != 0) ? 32/W : 0                 # INTER_BITS = 5, INTER_TAB_SIZE = 32
  X  = rint(X0*W); Y = rint(Y0*W)          # fixed point, 1/32 px
  sx = X >> 5; sy = Y >> 5; ax = X & 31; ay = Y & 31
  v0..v3 = src[clamp(sy  ,0,rows-1)][clamp(sx  ,0,cols-1)], [sy][sx+1], [sy+1][sx], [sy+1][sx+1]  (all clamped)
  bilinear with weights round((1-ay/32)(1-ax/32)*32768) etc., result = (sum + 16384) >> 15, saturate to uint8
```

Border handling = **replicate** (coordinates are clamped, not zero-filled). No prefilter/antialiasing.

U and V use the same kernel with `src_px_stride = 2`, source size `w/2 x h/2`, destination 256x128 and
the matrix `M_uv = transform_scale_buffer(M, 0.5)` (`common/mat.h`):

```
M_uv = S_in * M * S_out,
S_out = [[1/s, 0, 0.5], [0, 1/s, 0.5], [0, 0, 1]] = [[2,0,0.5],[0,2,0.5],[0,0,1]]
S_in  = [[s, 0, -0.5*s], [0, s, -0.5*s], [0, 0, 1]] = [[0.5,0,-0.25],[0,0.5,-0.25],[0,0,1]]
// "in_pt = ( transform(out_pt/s + 0.5) - 0.5) * s"
```

(The Y warp uses `M` unchanged, i.e. integer pixel indices, no half-pixel shift.)

### 2.3 Packing to 6 channels, frame pairing, scaling

Per frame, `transforms/loadyuv.cl::loadys/loaduv` write `6*128*256 = 196,608` floats (`MODEL_FRAME_SIZE =
512*256*3/2`), **plain `convert_float` of the uint8 value, no normalisation**: values are 0..255.
The model's first layer (`stem.0/reparam_conv`, Conv 24->64, 3x3, stride 2, on the concat of
`input_imgs` and `big_input_imgs`) consumes them directly; there is no Sub/Div in front of it in the graph.

Channel layout per frame (Y is the 256x512 warped luma, numpy notation `Y[row, col]`):

| channel | content | source line |
|---|---|---|
| 0 | `Y[0::2, 0::2]` (even rows, even cols) | `outy0 = out + 0` for even `oy`, `ysf.s0246` |
| 1 | `Y[1::2, 0::2]` (odd rows, even cols) | `outy0 = out + UV_SIZE` for odd `oy`, `ysf.s0246` |
| 2 | `Y[0::2, 1::2]` (even rows, odd cols) | `outy1 = out + 2*UV_SIZE` for even `oy`, `ysf.s1357` |
| 3 | `Y[1::2, 1::2]` (odd rows, odd cols) | `outy1 = out + 3*UV_SIZE` for odd `oy`, `ysf.s1357` |
| 4 | U, 128x256 | `loaduv` at offset `W*H` |
| 5 | V, 128x256 | `loaduv` at offset `W*H + W*H/4` |

**Watch out:** `selfdrive/modeld/models/README.md` in v0.9.7 says channels 1 and 2 are
`Y[::2, 1::2]` and `Y[1::2, ::2]`, i.e. **swapped** relative to the kernel above. The kernel is what
openpilot actually runs, and flowpilot's CPU path (`Preprocess.YUV420toTensor`) agrees with the kernel:
`ch1 = rows 1::2, cols 0::2`, `ch2 = rows 0::2, cols 1::2`. Follow the table.

Kernel excerpt (`loadyuv.cl`):

```c
// 02
// 13
if ((oy & 1) == 0) { outy0 = out + out_offset;            //y0
                     outy1 = out + out_offset + UV_SIZE*2; } //y2
else               { outy0 = out + out_offset + UV_SIZE;   //y1
                     outy1 = out + out_offset + UV_SIZE*3; } //y3
vstore4(ysf.s0246, 0, outy0 + (oy/2) * (TRANSFORMED_WIDTH/2) + ox/2);
vstore4(ysf.s1357, 0, outy1 + (oy/2) * (TRANSFORMED_WIDTH/2) + ox/2);
```

Frame pairing (`models/commonmodel.cc::ModelFrame::prepare`): a 2-slot buffer,

```c
std::memmove(&input_frames[0], &input_frames[MODEL_FRAME_SIZE], sizeof(float) * MODEL_FRAME_SIZE);
clEnqueueReadBuffer(..., &input_frames[MODEL_FRAME_SIZE], ...);   // new frame -> slot 1
```

so **channels 0..5 = previous frame (t-1), channels 6..11 = current frame (t)**. The tensor is NCHW,
`[1, 12, 128, 256]`, contiguous; the 12x128x256 floats are exactly `[prev 196,608 | cur 196,608]`.
`big_input_imgs` is built identically from the wide stream with its own `ModelFrame`.

Reference implementation (numpy, for test vectors):

```python
def pack(y512x256, u256x128, v256x128):          # uint8 arrays, shapes (256,512),(128,256),(128,256)
    Y = y512x256.astype(np.float32)
    return np.stack([Y[0::2,0::2], Y[1::2,0::2], Y[0::2,1::2], Y[1::2,1::2],
                     u256x128.astype(np.float32), v256x128.astype(np.float32)])   # (6,128,256)
input_imgs = np.concatenate([pack(*prev), pack(*cur)])[None]                       # (1,12,128,256)
```

### 2.4 Model intrinsics and the warp matrix

From `common/transformations/model.py` (v0.9.7):

```python
MEDMODEL_INPUT_SIZE = (512, 256);  MEDMODEL_YUV_SIZE = (512, 384);  MEDMODEL_CY = 47.6
medmodel_fl = 910.0
medmodel_intrinsics = [[910.0, 0.0, 256.0],
                       [0.0, 910.0, 47.6 ],
                       [0.0, 0.0,   1.0  ]]
SBIGMODEL_INPUT_SIZE = (512, 256); SBIGMODEL_YUV_SIZE = (512, 384)
sbigmodel_fl = 455.0
sbigmodel_intrinsics = [[455.0, 0.0, 256.0],
                        [0.0, 455.0, 0.5*(256+47.6) = 151.8],
                        [0.0, 0.0,   1.0]]
# (BIGMODEL 1024x512, fl 910, cy 256+47.6 exists but is not used by modeld in v0.9.7)

medmodel_frame_from_calib_frame  = medmodel_intrinsics  @ get_view_frame_from_calib_frame(0,0,0,0)
sbigmodel_frame_from_calib_frame = sbigmodel_intrinsics @ get_view_frame_from_calib_frame(0,0,0,0)
calib_from_medmodel  = inv(medmodel_frame_from_calib_frame[:, :3])
calib_from_sbigmodel = inv(sbigmodel_frame_from_calib_frame[:, :3])

def get_warp_matrix(device_from_calib_euler, intrinsics, bigmodel_frame=False):
  calib_from_model  = calib_from_sbigmodel if bigmodel_frame else calib_from_medmodel
  device_from_calib = rot_from_euler(device_from_calib_euler)
  camera_from_calib = intrinsics @ view_frame_from_device_frame @ device_from_calib
  return camera_from_calib @ calib_from_model          # maps model px -> camera px
```

`get_view_frame_from_calib_frame(r,p,y,height)` returns `[view_from_device @ R | (0, height, 0)]`; only
its rotational 3x3 is used, so **camera height plays no role in the warp** (the warp is a pure-rotation
homography; height is implicit in what the model learned, about 1.22 m, `HEIGHT_INIT` in calibrationd).

Numeric values:

```
medmodel_frame_from_calib[:, :3] = [[256.0, 910.0,   0.0],
                                    [ 47.6,   0.0, 910.0],
                                    [  1.0,   0.0,   0.0]]
calib_from_medmodel  = [[0,          0,          1         ],
                        [0.00109890, 0,         -0.28131868],
                        [0,          0.00109890,-0.05230769]]
calib_from_sbigmodel = [[0,          0,          1         ],
                        [0.00219780, 0,         -0.56263736],
                        [0,          0.00219780,-0.33362637]]
```

`rot_from_euler([roll, pitch, yaw])` = `euler2rot` (`common/transformations/orientation.cc`):
`R = Rz(yaw) * Ry(pitch) * Rx(roll)` (Eigen `AngleAxis(z)*AngleAxis(y)*AngleAxis(x)`), with the
standard right-handed `Rx, Ry, Rz`. The angles are `liveCalibration.rpyCalib` = **device_from_calib**
Euler angles in radians. Sign check (device x fwd, y right, z down): the road-forward direction in device
coordinates is `R @ [1,0,0] = [cos p cos y, cos p sin y, -sin p]`, so **positive pitch = device tilted down
w.r.t. the road** (horizon appears higher in the image), **positive yaw = device yawed left** (road ahead
appears to the right).

Device cameras (`common/transformations/camera.py`); `CameraConfig.intrinsics = [[f,0,w/2],[0,f,h/2],[0,0,1]]`
(principal point always assumed at the image centre, no distortion model anywhere in the warp):

| device | fcam (road, "narrow") | ecam (wide) |
|---|---|---|
| comma 3 / 3X, AR0231 or OX03C10 | 1928x1208, f = 2648.0 | 1928x1208, f = 567.0 ("focal length probably wrong? magnification is not consistent across frame": it is a fisheye) |
| comma 3X, OS04C10 | 2688x1520, f = 1765.33 (= 2648*2/3) | 2688x1520, f = 850.5 (= 567/2*3) |
| EON/neo (legacy) | 1164x874, f = 910.0 | none |

openpilot's per-frame warps (`modeld.py`):

```python
model_transform_main  = get_warp_matrix(rpyCalib, dc.ecam.intrinsics if main_wide_camera else dc.fcam.intrinsics, False)
model_transform_extra = get_warp_matrix(rpyCalib, dc.ecam.intrinsics, True)
```

i.e. `input_imgs` = road (narrow) camera -> medmodel frame; `big_input_imgs` = wide camera -> sbigmodel
frame, **both with the same `rpyCalib`** (v0.9.7 ignores `wideFromDeviceEuler` for the warp).
Quirk: when only one stream exists, `buf_extra = buf_main` but the extra warp still uses
`dc.ecam.intrinsics` (fixed on master, which uses the narrow intrinsics when there is no wide camera).

Example: comma 3 at `rpy = 0`:

```
medmodel  warp (fcam, f 2648):  [[2.90989, 0, 219.068], [0, 2.90989, 465.489], [0, 0, 1]]
sbigmodel warp (ecam, f 567):   [[1.24615, 0, 644.985], [0, 1.24615, 414.834], [0, 0, 1]]
```

So the road model input is the fcam image downsampled ~2.9x (no prefilter) and the big input is the wide
image downsampled ~1.25x.

Field of view that the model frames cover (pinhole):

| frame | HFOV | above horizon | below horizon |
|---|---|---|---|
| medmodel (fl 910, 512x256, cy 47.6) | 31.4 deg | 3.0 deg | 12.9 deg |
| sbigmodel (fl 455, 512x256, cy 151.8) | 58.7 deg | 18.5 deg | 12.9 deg |

### 2.5 Adapting to an arbitrary phone camera

Everything is captured by replacing `intrinsics` with the phone's K **at the capture resolution you feed
the warp**:

```
K_phone = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]           # pixels at capture resolution W x H
M_med  = K_phone @ V @ R(rpy) @ calib_from_medmodel       # for input_imgs
M_big  = K_phone @ V @ R(rpy) @ calib_from_sbigmodel      # for big_input_imgs (same camera, same frame)
V = view_from_device = [[0,1,0],[0,0,1],[1,0,0]]
```

* If you only know the physical focal length: `fx = f_mm / sensor_width_mm * W` (use the active-array width
  for the chosen crop/binning mode; Android: `LENS_INFO_AVAILABLE_FOCAL_LENGTHS` and
  `SENSOR_INFO_PHYSICAL_SIZE`, or better `LENS_INTRINSIC_CALIBRATION`). If you scale the capture, scale
  `fx, fy, cx, cy` by the same factor. A checkerboard calibration is better (flowpilot does this, see below).
* Using the real `cx, cy` is fine (openpilot uses `w/2, h/2` only because its sensors are centred).
* A single phone camera feeds **both** inputs: same frame, two warps (master does exactly this:
  `extra_intrinsics = dc.wide_road.intrinsics if has_wide_camera else dc.narrow_road.intrinsics`).
* Coverage: the camera must cover >= 58.7 deg HFOV and about 18.5 deg above / 12.9 deg below the
  road horizon (after pitch), otherwise the replicate-border smear shows up in `big_input_imgs`. Example:
  1280x720 with f = 1000 px (HFOV 65.2 deg, VFOV 39.6 deg) at pitch 0.02 rad gives
  `M_med = [[1.0989, 0.01407, 357.884], [0, 1.10659, 287.256], [0, 2.198e-5, 0.99875]]`,
  `M_big = [[2.1978, 0.02813, 72.965], [0, 2.21319, 3.968], [0, 4.395e-5, 0.99313]]`:
  the big frame spans x 73..1198 and starts at y about 4, so it just fits.
* Sampling density: the comma road input is 2.9 camera px per model px and the wide input 1.25 px.
  With a phone at f ~ 1000 px the medmodel ratio is ~1.1 (sharper than training) and the sbig ratio ~2.2
  with no prefilter (aliasing). Picking the capture resolution so that f is about 1300 to 2600 px for
  the narrow path, and box-downsampling before the sbig warp, keeps the image statistics closer to training.
* Domain gap you cannot remove with a homography: comma's wide camera is a fisheye with uncorrected
  distortion (the warp assumes pinhole), so the training `big_input_imgs` has barrel distortion in its
  periphery. A rectilinear phone image differs there. Lens distortion of the phone should be removed
  (or ignored if small); there is no distortion term in openpilot's pipeline.
* Extrinsics `rpy`: start from the mount (pitch from the accelerometer gravity vector relative to the car,
  yaw ~0, roll ~0), then refine online like `selfdrive/locationd/calibrationd.py` does. Its per-sample
  observation (only when `v_ego > 15 mph`, `trans[0] > 15 mph`, `|yaw rate| < 2 deg/s` and the std
  checks pass):

  ```python
  observed_rpy = np.array([0, -np.arctan2(trans[2], trans[0]), np.arctan2(trans[1], trans[0])])
  new_rpy = euler_from_rot(rot_from_euler(smooth_rpy).dot(rot_from_euler(observed_rpy)))
  # trans = model 'pose' mean[:3] (calibrated-frame velocity), averaged in blocks of 100, 5..50 blocks
  # PITCH_LIMITS = [-0.0907, 0.17] rad, YAW_LIMITS = [-0.0691, 0.0691] rad; height = road_transform mean[2]
  ```

  openpilot's limits are what keeps the medmodel frame inside the comma narrow camera; with a phone the
  usable range depends on its FOV.

Flowpilot (github.com/flowdriveai/flowpilot, **license GPL-3.0, not MIT**: read for ideas, do not copy
code into an MIT/proprietary program). What it does, for a phone (it targets an older supercombo with
`input_imgs` + `initial_state`, no big input):

* Captures 1280x720 (`Camera.frameSize`), default `fcam_intrinsics = [[910,0,640],[0,910,360],[0,0,1]]`,
  overridden by a checkerboard-calibrated matrix stored in the param `CameraMatrix` (`CameraCalibratorIntrinsic.java`).
  Only `fx, cx, fy, cy` (indices 0, 2, 4, 5) are used.
* `Preprocess.getWrapMatrix(rpy, K, K, wide=false, big=false)` = `K @ view_from_device @ R(rpy) @ inv(medmodel_K @ view_from_device)`,
  the same formula as openpilot with the phone K. Its own `eulerAnglesToRotationMatrix` is
  `Rz(yaw) Ry(pitch) Rx(roll)` too.
* CPU path (`ImagePrepareCPU`): YUV -> RGB, `cv::warpPerspective(rgb, out, inv(M), 512x256)`, RGB -> I420
  (`COLOR_RGB2YUV_I420`), then the 6-channel split above; shifts channels 6..11 to 0..5 each frame.
  (A direct NV12 warp as in 2.2 is cheaper and closer to openpilot.)

---

## 3. Output tensor `outputs` [1, 6504]

### 3.1 Slices (from the ONNX `metadata_props['output_slices']`, base64 pickle; identical to what
`get_model_metadata.py` writes into `supercombo_metadata.pkl`)

| name | raw range | length | parse (parse_model_outputs.py) | parsed shape |
|---|---|---|---|---|
| `plan` | 0 .. 4954 | 4955 | `parse_mdn(in_N=5, out_N=1, out_shape=(33,15))` | (33,15) + stds + 5 hypotheses |
| `lane_lines` | 4955 .. 5482 | 528 | `parse_mdn(in_N=0,out_N=0, (4,33,2))` | (4,33,2) |
| `lane_lines_prob` | 5483 .. 5490 | 8 | sigmoid | (8,) |
| `road_edges` | 5491 .. 5754 | 264 | `parse_mdn(0,0,(2,33,2))` | (2,33,2) |
| `lead` | 5755 .. 5856 | 102 | `parse_mdn(in_N=2, out_N=3, (6,4))` | (3,6,4) |
| `lead_prob` | 5857 .. 5859 | 3 | sigmoid | (3,) |
| `desire_state` | 5860 .. 5867 | 8 | softmax | (8,) |
| `meta` | 5868 .. 5915 | 48 | sigmoid | (48,) |
| `desire_pred` | 5916 .. 5947 | 32 | softmax per row of (4,8) | (4,8) |
| `pose` | 5948 .. 5959 | 12 | `parse_mdn(0,0,(6,))` | (6,) |
| `wide_from_device_euler` | 5960 .. 5965 | 6 | `parse_mdn(0,0,(3,))` | (3,) |
| `sim_pose` | 5966 .. 5977 | 12 | `parse_mdn(0,0,(6,))` | (6,) |
| `road_transform` | 5978 .. 5989 | 12 | `parse_mdn(0,0,(6,))` | (6,) |
| `desired_curvature` | 5990 .. 5991 | 2 | `parse_mdn(0,0,(1,))` | (1,) |
| `hidden_state` | 5992 .. 6503 | 512 | none (raw), feeds `features_buffer` | (512,) |

(The pickled slice for `hidden_state` is `slice(5992, None)`. `lat_planner_solution` is not present in
this model; the parser skips it.)

### 3.2 Mean / std layout (`parse_mdn`)

```python
raw = raw.reshape((1, max(in_N, 1), -1))
n_values = (raw.shape[2] - out_N)//2
pred_mu  = raw[:, :, :n_values]
pred_std = np.exp(raw[:, :, n_values: 2*n_values])     # stored as log-std
# if in_N > 1: the last out_N columns of each hypothesis are weight logits
weights[:, :, i - out_N] = softmax(raw[:, :, i - out_N], axis=-1)   # softmax ACROSS the in_N hypotheses
# out_N == 1: hypotheses sorted by weight; final = hypothesis with the largest weight
# out_N > 1 : final[hidx] = hypothesis with the largest weight in column hidx
```

So for every single-hypothesis output: **first half = means, second half = log-stds (apply exp)**, each
half reshaped row-major to the parsed shape.

Absolute indices:

* `plan`: 5 hypotheses x 991 floats. Hypothesis h starts at `991*h`:
  mean `[991h, 991h+495)`, log-std `[991h+495, 991h+990)`, weight logit `991h+990`.
  (h0: 0..494 / 495..989 / 990; h1: 991.. / 1486.. / 1981; h2: 1982 / 2477 / 2972; h3: 2973 / 3468 / 3963;
  h4: 3964 / 4459 / 4954.) Inside a mean block: `t*15 + col`, t = 0..32, col per 3.3.
  Selected plan = hypothesis with the max weight logit (softmax is monotonic).
* `lane_lines`: mean 4955..5218, log-std 5219..5482. Mean index = `4955 + line*66 + xidx*2 + c`, c = 0 (y), 1 (z).
* `road_edges`: mean 5491..5622, log-std 5623..5754. Mean index = `5491 + edge*66 + xidx*2 + c`.
* `lead`: 2 hypotheses x 51. Hyp h at `5755 + 51h`: mean 24 floats (`t*4 + c`, t = 0..5, c = x,y,v,a),
  log-std next 24, then 3 weight logits (selection 0, 1, 2). Output lead i (i = 0,1,2) = the hypothesis
  whose logit `[5755+51h+48+i]` is larger. h0 = 5755..5778 / 5779..5802 / 5803..5805,
  h1 = 5806..5829 / 5830..5853 / 5854..5856.
* `pose` mean 5948..5953, log-std 5954..5959; `wide_from_device_euler` mean 5960..5962, log-std 5963..5965;
  `sim_pose` mean 5966..5971, log-std 5972..5977; `road_transform` mean 5978..5983, log-std 5984..5989;
  `desired_curvature` mean 5990, log-std 5991.

Note on `desired_curvature`: the graph already scales it, `outputs[5990:5992] = [0.01, 1.0] * Gemm(...)`
(`/action_block/Mul` with constant `[[0.01, 1.0]]`), so raw[5990] is the curvature in 1/m directly.

### 3.3 Physical meaning

Time / distance grids (`constants.py`: `index_function(idx, max_val, max_idx=32) = max_val*(idx/32)**2`, IDX_N = 33):

```
T_IDXS = [0.0, 0.009765625, 0.0390625, 0.087890625, 0.15625, 0.244140625, 0.3515625, 0.478515625,
          0.625, 0.791015625, 0.9765625, 1.181640625, 1.40625, 1.650390625, 1.9140625, 2.197265625,
          2.5, 2.822265625, 3.1640625, 3.525390625, 3.90625, 4.306640625, 4.7265625, 5.166015625,
          5.625, 6.103515625, 6.6015625, 7.119140625, 7.65625, 8.212890625, 8.7890625, 9.384765625, 10.0]   # s
X_IDXS = [0.0, 0.1875, 0.75, 1.6875, 3.0, 4.6875, 6.75, 9.1875, 12.0, 15.1875, 18.75, 22.6875, 27.0,
          31.6875, 36.75, 42.1875, 48.0, 54.1875, 60.75, 67.6875, 75.0, 82.6875, 90.75, 99.1875, 108.0,
          117.1875, 126.75, 136.6875, 147.0, 157.6875, 168.75, 180.1875, 192.0]                             # m
LEAD_T_IDXS    = [0., 2., 4., 6., 8., 10.]   # s
LEAD_T_OFFSETS = [0., 2., 4.]                # s, one per lead output / lead_prob
META_T_IDXS    = [2., 4., 6., 8., 10.]       # s
```

All spatial outputs are in the **calibrated frame** (x forward, y right, z down, origin at the camera),
because the images were warped into it.

* **plan** (33 x 15, at T_IDXS):
  cols 0..2 position x,y,z (m); 3..5 velocity (m/s); 6..8 acceleration (m/s^2);
  9..11 `T_FROM_CURRENT_EULER` orientation roll, pitch, yaw relative to now (rad);
  12..14 orientation rate (rad/s). Stds only published for position. The plan is the camera's own
  future trajectory, so plan z is ~0; openpilot's UI draws the path at `z + 1.22` to put it on the road.
* **lane_lines** (4 lines x 33 x (y, z), at X_IDXS): order = far-left, left (ego), right (ego), far-right.
  y (m, right positive), z (m, down positive: road surface ~ +camera height, UI draws them with z offset 0).
  `laneLineStds` = `lane_lines_stds[0,:,0,0]` (y std at x = 0). `fill_model_msg` also maps X_IDXS to times
  by interpolating along the plan x (`PLAN_T_IDXS`); that is bookkeeping, not a model output.
* **lane_lines_prob** (8 logits): presence probability of line i = `sigmoid(raw[5483 + 2i + 1])`
  (`laneLineProbs = lane_lines_prob[0, 1::2]`); the even entries are unused.
* **road_edges** (2 x 33 x (y, z)): left, right; same units/frame as lane lines. `roadEdgeStds` = y std at x = 0.
* **lead** (3 x 6 x 4): lead i is the prediction made for horizon `LEAD_T_OFFSETS[i]` (0, 2, 4 s);
  each has a trajectory at LEAD_T_IDXS of x (m, longitudinal distance), y (m, lateral), v (m/s, lead speed),
  a (m/s^2). **lead_prob[i]** = sigmoid(raw[5857+i]) = probability that a lead exists at that offset.
* **meta** (48, sigmoid each):
  index 0 `engagedProb`; for k = 0..4 (t = 2,4,6,8,10 s), base = 1 + 7k:
  +0 gas disengage, +1 brake disengage, +2 steer override, +3 hard brake 3 m/s^2, +4 hard brake 4 m/s^2,
  +5 hard brake 5 m/s^2, +6 gas press (`Meta.GAS_DISENGAGE = slice(1,36,7)` ... `GAS_PRESS = slice(7,36,7)`);
  blinkers for t = 0,2,4,6,8,10 s: left = 36 + 2j, right = 37 + 2j (j = 0..5).
  FCW in openpilot: hard-brake-5 > [.05,.05,.15,.15,.15] for the last 5 frames AND hard-brake-3 > [.7,.7]
  for the last 2 frames (uses only the k = 0 entries).
* **desire_state** (8, softmax): the model's belief of the currently executing desire (enum as in 1.1);
  openpilot uses `[3]+[4]` (laneChangeLeft + laneChangeRight) to end lane changes.
* **desire_pred** (4 x 8, softmax over 8 in each row): predicted desire at 4 future horizons.
* **pose** (6): mean[0..2] = translational **velocity** (m/s), mean[3..5] = rotation **rate** (rad/s), in the
  calibrated frame; between the previous and the current frame (it is posenet, published as
  `cameraOdometry.trans/rot`). locationd rotates it to the device frame
  (`trans_device = device_from_calib * trans`) and sanity-checks `|rot| < 10 rad/s`, `|trans| < 200 m/s`;
  calibrationd compares `trans[0]` against a speed in m/s.
* **wide_from_device_euler** (3, rad): estimated rotation of the wide camera relative to the road camera.
* **sim_pose** (6): published as `modelV2.temporalPose` (trans m/s, rot rad/s), a temporally smoothed pose.
* **road_transform** (6): published as `cameraOdometry.roadTransformTrans` = mean[0..2]; calibrationd uses
  `road_transform[2]` as the **camera height above the road** (m).
* **desired_curvature** (1, 1/m): the model's lateral action (`modelV2.action.desiredCurvature`), already
  accounting for `lateral_control_params`. Feeds back into `prev_desired_curv`.
* **hidden_state** (512): unit-norm feature vector; feeds back into `features_buffer`.

`modelV2.confidence` (green/yellow/red) is computed in `fill_model_msg` from the disengage probs every 2 s;
port it only if you need it (code in `src097/selfdrive/modeld/fill_model_msg.py`).

---

## 4. Feedback loop (exact indexing)

After every executed run (not on `prepare_only` frames):

```
features_buffer  (99 x 512, row-major, float):  memmove(rows 0..97 <- rows 1..98); row 98 = out[5992 .. 6503]
prev_desired_curv(100 x 1):                     memmove(rows 0..98 <- rows 1..99); row 99 = out[5990]
```

Before every run:

```
desire (100 x 8): memmove(rows 0..98 <- rows 1..99); row 99 = rising-edge pulse of the one-hot (index 0 forced 0)
input_imgs / big_input_imgs: slot0 (ch 0..5) <- slot1 (ch 6..11); slot1 <- new warped+packed frame
```

What the graph actually reads: `desire` all 100 rows; `features_buffer` rows 94, 89, 84, 79, 74, 69, 64, 59;
`prev_desired_curv` row 99 only.

A C++ ring buffer is fine as long as you materialise the tensor in the order above (or, for
`features_buffer`, only those 8 rows need to be correct).

---

## 5. Runtime notes

### 5.1 Graph content

* Opset 14, default domain only, **no custom ops**. 595 nodes. Op histogram: Mul 132, Constant 95, Add 87,
  Gemm 83, Relu 69, Conv 60, Tanh 19, MatMul 7, Concat 5, ReduceMean 5, Reshape 5, Transpose 4, Sqrt 3,
  Div 3, Flatten 2, ReduceSum 2, Gather 2, Unsqueeze 2, Sub 2, Pow 2, Sigmoid 1, GlobalAveragePool 1, Clip 1,
  Split 1, Softmax 1, Elu 1. All are standard ORT CPU kernels.
* Vision backbone: re-parameterised conv stem (`reparam_conv`, FastViT/MobileOne style) with a GELU-tanh
  approximation expanded into Mul/Add/Tanh; temporal part: Gemm MLPs plus one small attention block.
* Parameters: **25,641,048** (all 313 initializers fp16, 51.3 MB).
* Compute: **~0.84 GMAC (~1.69 GFLOP) per inference**, of which ~0.79 GMAC are convolutions
  (computed from shape inference: Conv + Gemm + MatMul).
* Quantised variants: **none shipped**. v0.9.7 ships only this fp16 ONNX; on the comma device it is
  compiled by tinygrad to a GPU "thneed" (`FLOAT16=1`). No int8/QDQ model exists in the repo (the old
  `.dlc` quantised file was the driver-monitoring model, run on the Qualcomm DSP via SNPE). Any int8
  model would have to be made by you (ORT static quantisation with calibration frames), accuracy unknown.

### 5.2 How openpilot creates the ORT session (CPU path, `runners/onnxmodel.py`)

```python
os.environ["OMP_NUM_THREADS"] = "4"; os.environ["OMP_WAIT_POLICY"] = "PASSIVE"
options.intra_op_num_threads = 2
options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
model_data = convert_fp16_to_fp32(path)     # initializers, I/O and Constant tensors fp16 -> fp32
```

### 5.3 Measured latency on this laptop (Intel i7-8850H, 6C/12T, AVX2; ORT 1.31.0 CPU EP, Python)

Zero inputs, 3 warm-up runs, median of 20 runs, `ORT_ENABLE_ALL`, sequential, memory-capped unit. Output
finite; `|hidden_state| = 1.000`.

| model | 1 thread | 2 threads | 4 threads |
|---|---|---|---|
| supercombo v0.9.7, fp32-converted | 30.7 ms | 18.7 ms | 12.3 ms |
| supercombo v0.9.7, original fp16 I/O | 31.0 ms | 19.2 ms | 12.6 ms |
| v0.11.1 driving_vision (uint8 in) | 34.3 ms | n/a | 15.7 ms |
| v0.11.1 driving_policy | 1.8 ms | n/a | 1.1 ms |
| master driving_supercombo (stateful) | 36.6 ms | n/a | 16.8 ms |

(fp16 on x86 gives no gain: ORT runs fp32 kernels with casts.) Preprocessing (two warps + packing) is
not included; on CPU it is ~0.4 M output pixels per frame, a few ms with NEON.

Extrapolation to Snapdragon 845 (4x Kryo 385 Gold = Cortex-A75 @ 2.8 GHz, 4x A55): the laptop sustains
~55 GFLOP/s single-thread on this graph (1.69 GFLOP / 30.7 ms). An A75 core peaks at ~45 GFLOP/s fp32
(2x 128-bit FMA); at a similar ~45 % efficiency that is ~20 GFLOP/s, i.e. **~80-90 ms single-thread and
roughly 30-50 ms on the 4 big cores**, worse under sustained thermal throttling. 20 Hz (50 ms budget
including preprocessing) is therefore marginal on this model. The A75 supports ARMv8.2 FP16 arithmetic
and dot-product instructions; an ORT build with ARM64 fp16 MLAS kernels or an int8 model could gain
~1.5-2x, but neither is verified here. Measure on the phone before committing.

### 5.4 If 20 Hz is not reachable

The v0.9.7 model hard-codes 20 Hz: the image pair is 50 ms apart and the feature taps are every 5 frames.
Running it at 10 Hz gives pairs 100 ms apart (biases `pose`, plan dynamics) and stretches the feature
history to 4 s. Options: (a) keep warping every frame at 20 Hz (cheap) so the image pair stays correct,
and run the network on as many frames as possible (openpilot's `prepare_only` path); the features FIFO
then advances only on executed frames, which still distorts the history; (b) move to the newer split models
(section 6), which are trained at a 5 Hz context and can be run at 5 Hz with exact input semantics.

---

## 6. Latest master vs v0.9.7

Timeline (checked through the GitHub trees of each tag):

| version | model files in `selfdrive/modeld/models/` | runtime |
|---|---|---|
| v0.9.7 | `supercombo.onnx` (one graph, 7 inputs above) | thneed (GPU) on device, ORT on PC |
| v0.9.8 | `driving_vision.onnx` + `driving_policy.onnx` | tinygrad |
| v0.10.0 .. v0.11.1 | `driving_vision/policy.onnx` + `big_driving_vision/policy.onnx` | tinygrad |
| master (commit 815a90f, 2026-10-21) | `openpilot/selfdrive/modeld/models/driving_supercombo.onnx` (Git LFS, 60,918,562 B, sha256 `65a08adc…8cce`), compiled at build time to `driving_tinygrad.pkl` + warp pkls | tinygrad (`DEV=CPU:LLVM` on PC, `QCOM` on device) |

The models are stored with **Git LFS** in the repo (the raw.githubusercontent URL gives an LFS pointer; the
binary comes from `https://media.githubusercontent.com/media/commaai/openpilot/<ref>/<path>`). They are not
downloaded from a separate URL at runtime. Note master moved the tree under a top-level `openpilot/` directory.

v0.11.1 split models (opset 20, default domain, no custom ops; `Gelu` and `LayerNormalization` need
ORT >= 1.16/1.17):

* `driving_vision.onnx`: inputs `img`, `big_img` [1,12,128,256] **uint8** (normalisation inside the graph);
  output [1,1576]: meta 0..54 (55, new layout), desire_pred 55..86, pose 87..98, wide_from_device_euler
  99..104, road_transform 105..116, lane_lines 117..644, lane_lines_prob 645..652, road_edges 653..916,
  lead 917..1060 (144 = 3 leads x 6 x 4 mean + log-std, no MHP; the parser detects MHP by length), lead_prob 1061..1063, hidden_state 1064..1575.
  23.05 M params.
* `driving_policy.onnx`: inputs `desire_pulse` [1,25,8], `traffic_convention` [1,2], `features_buffer`
  [1,25,512] (all fp16); output [1,1000]: plan 0..989 (single hypothesis, 33x15 mean + 33x15 log-std, no MHP),
  desire_state 990..997. 6.95 M params. No `lateral_control_params`, no `desired_curvature` output; the
  action is derived from the plan (`get_curvature_from_plan`, `get_accel_from_plan`).
* Context is **5 Hz** (`MODEL_FREQ = 20`, `HISTORY_FREQ = 5`, `TEMPORAL_SKIP = 4`): the image pair is the
  current frame and the frame **4 frames (200 ms) earlier**; `features_buffer` is every 4th hidden state
  over 5 s (25 rows); `desire` is the 20 Hz pulse buffer max-pooled over groups of 4 (v0.10.0 `modeld.py`).

Master `driving_supercombo.onnx` is one stateful graph (30.0 M params, opset 20): inputs `new_img`
[2,6,128,256] uint8 (current packed frame for the road and wide camera), `desire` [8], `traffic_convention`
[1,2], `action_t` [1,2] (lateral and longitudinal action delays, s), and the state tensors `state_img_q`
[2,5,6,128,256] uint8, `state_desire_q` [100,1,8], `state_feat_q` [96,1,512]; outputs `outputs` [1,2576]
(vision slices as above + plan 1576..2565 + desire_state 2566..2573) and `next_state_*` to be fed back
unchanged. Convenient for ORT (no host-side FIFO logic), but the state queues are clocked at 20 Hz, so it
must be run every frame. The warp is a separate tinygrad kernel; the packing is the same 6-channel YUV layout.

### Recommendation for a CPU-only Snapdragon 845

* The single-inference cost is about the same in all three generations (12-17 ms on 4 x86 threads here,
  ~0.8-0.9 GMAC). What differs is **how often you must run it**: v0.9.7 and master assume 20 Hz runs; the
  v0.10-v0.11 **split vision + policy models have exact 5 Hz semantics** (pairs 200 ms apart, 5 Hz feature
  history), so running vision+policy only every 4th frame gives the same inputs as openpilot's 20 Hz loop
  (only the desire pulse needs max-pooling). That is a 4x lower sustained CPU load (~5 Hz x ~40-50 ms on the
  A75 cluster), comfortably feasible, versus a marginal 20 Hz for v0.9.7.
* So for a CPU-only phone the **split models of v0.11.1 (or the latest tagged release that still ships
  `driving_vision.onnx` + `driving_policy.onnx`) are the better choice**: uint8 image input, plain ONNX,
  standard ops, 5 Hz capable, newer training. Costs: write a parser for the v0.10+ slice layout (from
  that tag's `parse_model_outputs.py` + `constants.py`), derive curvature/accel from the plan yourself, and
  use ORT >= 1.17.
* Use v0.9.7 (this spec) if you specifically need the in-model `desired_curvature`/lead MHP or want the
  simplest, fully documented starting point, and only if the phone sustains 20 Hz (measure first; consider
  2-3 big-core threads, pinned, and the fp32-converted file).
* Master's merged stateful graph is attractive for its self-contained state, but it is not tagged, is
  20 Hz-clocked and changes often; not recommended as a fixed target.

---

## 7. Checklist for the C++ implementation

1. Load `supercombo-v0.9.7-fp32.onnx` (or the fp16 original with `Ort::Float16_t` buffers); one session,
   `intra_op_num_threads` = number of big cores, `ORT_ENABLE_ALL`, sequential.
2. Per camera frame (20 Hz): NV12 -> warp with `M_med` (Y 512x256) and `M_uv = S_in*M_med*S_out` (U,V 256x128),
   same for `M_big`; pack per 2.3; shift slot1 -> slot0, write slot1.
3. Update `desire` (pulse, shift), `traffic_convention`, `lateral_control_params`.
4. Run. Copy `outputs` (6504 floats).
5. Parse per section 3 (exp for log-stds, sigmoid/softmax as listed, MHP selection for plan and leads).
6. Push `hidden_state` into `features_buffer`, `desired_curvature` mean into `prev_desired_curv`.
7. Test vectors: run `scripts/bench.py`-style zero inputs and compare raw outputs between your C++ and
   Python ORT to ~1e-3; then feed a recorded drive (openpilot route `fcamera.hevc` decoded to NV12 with
   rpyCalib from its `liveCalibration`) and compare against the logged `modelV2` of a v0.9.7 route.

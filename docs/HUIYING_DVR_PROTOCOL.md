# USB DVR camera 1902:3241: protocol reconstructed from DVR.apk

Static analysis only: no device and no traffic capture were available. Sources:

- `DVR.apk`, decompiled with jadx 1.5.6 (official release zip from github.com/skylot/jadx), into `out/`.
- `libuvccamera-lib.so` (ARM32 Thumb-2, built at -O0 with exported symbols), disassembled with capstone. The helper that resolves literal pools, GOT and PLT is `tools/dis.py`.

Confidence levels used below:

- **H**: read directly from code, with no inference.
- **M**: inferred from code; a device test should confirm it.
- **L**: a guess.

## 1. Identification

| Item | Value | Conf |
|---|---|---|
| App package | `com.fvision.camera` (UI); native glue `com.huiying.cameramjpeg.UvcCamera`, lib `com.huiying.uvccameralib` | H |
| Vendor | Huiying (Shenzhen). The strings `HUIYING`, `huiying616.com` and the broadcast extra `mfrs=HUIYING` appear in the app. | H |
| USB IDs the app accepts | VID 6402 = **0x1902**, PID 12865 = **0x3241** or 13107 = **0x3333** (`UvcCamera.initEndPoint`) | H |
| SoC / chip | **Not named** anywhere in the APK or the .so. "Generalplus" is not supported by anything in the code. The info-sector byte 0x34 = 0xE9 marks a "board v1" variant. | — |
| Video | MJPEG. The decode target is fixed at **1280x720** (`framewidth`/`frameheight` in .data, never written), using libyuv `MJPGToI420`. | H |
| Audio | 0x2000-byte PCM blocks (the player uses AudioTrack at 16 kHz, 16-bit) | M |

## 2. Transport

The camera is **only** a USB mass-storage device (BOT, SCSI transparent, interface 0). No UVC interface, vendor control requests or SCSI vendor CDBs are used anywhere. Everything goes through ordinary **READ(10)** and **WRITE(10)** at fixed LBAs. The firmware generates the contents of those LBAs live. They fall inside two files of the FAT volume:

| Region | Absolute LBA, layout A | Absolute LBA, layout B | Offset in file | Purpose | Conf |
|---|---|---|---|---|---|
| info | 0x380 | 0x300 | `VSFILE` + 0x0000 | 512-byte status and frame header | H |
| audio | 0x381 | 0x301 | `VSFILE` + 0x0200 | 0x2000 bytes of audio | H |
| video | 0x391 | 0x311 | `VSFILE` + 0x2200 | current JPEG frame | H |
| cmd write | 0x340 | 0x2C0 | `Android/data/com.fvision.camera/HWC` + 0x000 | command packet | H |
| cmd read | 0x341 | 0x2C1 | `HWC` + 0x200 | command reply | H |

**Layout selection** (`getVerApi`, H): READ LBA 0 (the MBR) and take the u32 at 0x1C6, which is the partition-1 start LBA. If it is 8, use layout A; otherwise use layout B. In file mode the app needs neither: it simply opens the files.

The app has two equivalent access paths. It chooses by Android version (`SharedPreferencesUtil.getUsbProtocol`):

- **Protocol 100, "read sdcard"** (default on SDK < 30):
  - Search the mounts for a file named `VSFILE` and open it with `O_DIRECT|O_NOCTTY|O_RDWR` (0x10102), or `O_RDONLY` on "px3" boards.
  - Open `<root>/Android/data/com.fvision.camera/HWC` with `O_DIRECT|O_NOCTTY|O_RDWR`.
  - Then use `lseek` plus `read`/`write` at the offsets above.
  - `O_DIRECT` is essential: it forces every read through to the device as a fresh SCSI READ.
- **Protocol 200, "endpoint"** (default on SDK >= 30):
  - `UsbManager.openDevice` → `libusb_wrap_sys_device(fd)` → `claimInterface(iface 0, force=true)`. This detaches the kernel usb-storage driver.
  - `libusb_clear_halt(0x01)` and `libusb_clear_halt(0x81)`.
  - From then on the app speaks BOT itself (`libUsbRead`/`libUsbWrite`, H):

```
CBW (31 bytes, to EP 0x01, timeout 1000 ms):
  0  'USBC'            4  tag = 0x74B4EA50 (constant, never incremented)
  8  dCBWDataTransferLength = nsec*512 (LE)
 12  flags = 0x80 (read) / 0x00 (write)   13 LUN 0   14 CB length 10
 15  CDB: 28 (READ10) or 2A (WRITE10), 00, LBA[31:24], LBA[23:16], LBA[15:8], LBA[7:0],
          00, nsec[15:8], nsec[7:0], 00
data:  bulk IN 0x81 / OUT 0x01, timeout 10000 ms, nsec = ceil(len/512)
CSW:   bulk IN 0x81, 16-byte buffer (13 used)
```

If the CBW transfer fails, the app calls `libusb_reset_device` and then re-initialises.

**On the Linux phone, the easiest path is raw-disk mode.** Use `/dev/sdg` with `O_DIRECT` and `pread` at `LBA*512`. It produces the same READ(10) commands, and the kernel driver stays attached. File mode on a mounted `/dev/sdg1` also works. pyusb BOT, which reproduces protocol 200 exactly, is the fallback. It needs the usb-storage driver detached from interface 0, so unmount `sdg1` first. (Conf M: this is equivalent by construction, but untested.)

## 3. Info sector (512 bytes, little-endian)

| Off | Type | Meaning | Conf |
|---|---|---|---|
| 0x00 | u16 | state flags. 0x8000 playback mode, 0x4000 recording, 0x2000 locked, 0x1000 playing, 0x0800 watermark, 0x0400 mute, 0x0200 SD present, 0x0100 picture, 0x0080 motion detect, 0x0040 formatting, 0x0020 file-list update, 0x0010 0 = TF card needs format. **Must be ≠ 0.** | H (bits from Java) |
| 0x02 | s16 | playback total time (s) | H |
| 0x04 | s16 | playback current time (s) | H |
| 0x06 | u8 | g-sensor level | H |
| 0x07 | u8 | save_1 | H |
| 0x08 | s16 | file index / PLIST count | H |
| 0x0C | u32 | **video frame size** in bytes. Valid when 1..0x4B000. | H |
| 0x10 | u32 | **video frame sequence**. A new frame exists when it exceeds the last value seen. | H |
| 0x14 | u32 | audio length, 0 or 0x2000 (any other value = invalid header) | H |
| 0x18 | u32 | audio sequence | H |
| 0x1C | char[12] | firmware version string, e.g. `..._vX.Y...` (the Java side parses `_v` and gates features on ≥ 2.5/2.7/3.1/3.2). A `W` in it selects some UI mode. | H |
| 0x26 | s16 | play speed (4 = 1x) | H |
| 0x2C | u32 | **frame check word**: must equal the u32 at byte offset `((size-8)>>2)*4` of the frame (ignored in playback mode) | H |
| 0x30 | u16 | integrity word, checked by `isAnNum()` (see below). Commands are refused until it passes. | H |
| 0x32 | u8 | ADAS state | H |
| 0x34 | u8 | 0xE9 → "board v1" (16-bit command header) | H |
| 0x38 | … | device version code (Java `getDevVersionCode`) | M |

`isAnNum(v)` returns true when `lo == (((popcount(hi & 0x1F) ^ 0x0C) << 3) | ((hi >> 5) ^ 0x0B)) & 0xFF`, where `lo = v & 0xFF` and `hi = v >> 8` (H).

## 4. Streaming sequence (`getDataThread`, H)

1. Optional: read the MBR to choose the layout. (Raw mode only; it is not a handshake.)
2. Read the info sector with `O_DIRECT`, so nothing is cached. The app polls this in a loop.
3. Reject the header unless all of these hold: `audio_len ∈ {0, 0x2000}`, `0 < size ≤ 0x4B000`, `flags ≠ 0`. Otherwise sleep 10 ms and retry.
4. If `audio_len == 0x2000` and the audio sequence is new, read 0x2000 bytes from the audio region.
5. If the video sequence is greater than the last one seen, and `isAnNum(info[0x30])` passes:
   - Read `(size & ~0x1FF) + 0x200` bytes from the video region. Raw mode does this in **one** READ(10). File mode reads in 64 KiB chunks at increasing offsets.
   - The JPEG is bytes `[0, size)`. It is accepted only if the check word matches; otherwise it is dropped.
   - The data starts directly with the JPEG: there is no header inside the video area.
6. If the sequence did not change, sleep 20 ms. After 20 such polls, unless in playback mode, the app does a dummy "kick" read of `(size & ~0x1FF) + 0x1400` bytes from the video region and discards the data.

**No start or stop command exists.** `startPreview()` only creates threads. The first command the app sends after init is a time sync (opcode 11), 2 s later in endpoint mode. Its purpose appears to be the clock, not streaming. (Conf: M that no command is needed for video.) **Update 2026-11-17 (H, measured):** no command is needed for video, but without the time sync the camera leaves the USB bus once, 28 s after power-up, for about 5 s; with it, it stays. See PHONE_DVR_DEPLOY.md "USB drop-outs after power-up".

Frame rate is not set by the host. It is whatever rate the firmware advances the sequence at. The app derives a frame-cache depth from an observed rate of more than 1000 bytes/ms.

## 5. Commands (`nativeSendCmd`, H)

Precondition: a valid info sector has been seen (the isAnNum flag is set).

A packet is a 512-byte buffer, zero-filled, built as follows:

```
board v1 (info[0x34]==0xE9):  [u16 LE opcode][u16 LE len][len bytes payload]
otherwise:                    [u8 opcode][u8 len][len bytes payload]
```

- If `len > 0x1FC`, the write size becomes `(len & ~0x1FF) + 0x400`, at most `CMD_DATA_LEN` = 20480.
- Write the packet to the cmd-write region:
  - raw mode: `WRITE(10)` at `cmdWriteSectorIndex`;
  - file mode: `lseek(HWC, 0)` then `write`.
- If `opcode >= 23`, read the same size back from the cmd-read region (`HWC` + 0x200 / `cmdReadSectorIndex`). The reply payload is the first `len` bytes, with no header. Below opcode 23 there is no reply.
- There is no checksum, magic or sequence number. The return value is 0 if the write succeeded.

| Op | Name | Payload | Notes |
|---|---|---|---|
| 0 | SET_MODE | 1 B (0 = live, 1 = playback) | |
| 1 | SET_REC | 1 B on/off | |
| 2 | SET_LOCK | 1 B | |
| 3 | SET_PLAY | 1 B play/pause | |
| 6 | SET_MUTE | 1 B | |
| 7 | SET_PICTURE | 1 B = 1 | snapshot |
| 11 | SET_TIME | 7 B: year u16 LE, month, day, hour, min, sec | sent after init |
| 12 | PLAY_FILE | file name / index | |
| 14, 16, 17 | g-sensor / motion detect / password | | |
| 15 | FORMAT TF | 1 B = 1 | **destructive** |
| 18 | DEL_FILE | name | **destructive** |
| 19 | BACK_PLAY_LOCK | name | |
| 20, 21 | FF / FB | 1 B | |
| 22 / 23 | SET / GET ADAS code | 64 B | |
| 24 / 25 | SET / GET ADAS UID | 16 B | |
| 26 | GET_DVR_UID | 16 B reply | |
| 27 / 28 | SET / GET vendor | int32 LE | |
| 29 / 30 | SET / GET video duration | int32 LE | |
| 31 | GET_ADAS_TYPE | 1 B reply | |
| 33, 35, 36, 37, 38 | OTA: open write / write data / close / check file / start upgrade (`DestOtaBin.bin`) | | **never send** |
| 39, 40, 41 | open / read / close a file on the TF card (download) | | |
| 43, 44, 45, 46 | video duration / frame seek / frame data / close | | |
| 47 | SET_SPEED (also used as "is connected") | 1 B | |
| 48 | GET card size | 2 × int32 LE reply | |

`nativeGetFile` (the file list) reads the cmd-read region with no new write.

## 6. Probe script

`proto_probe.py` contains no device-specific secrets and uses only the standard library, except for `--usb`.

```
python3 -I proto_probe.py --block /dev/sdg -o frame.jpg            # default, read-only
python3 -I proto_probe.py --block /dev/sdg -o f.jpg --count 30     # rough fps
python3 -I proto_probe.py --block /dev/sdg --info-only             # dump header
python3 -I proto_probe.py --file /media/sdg1 -o frame.jpg          # mounted FAT
python3 -I proto_probe.py --usb -o frame.jpg                       # pyusb BOT (unmount first)
python3 -I proto_probe.py --block /dev/sdg --cmd get_uid           # ONE 512-B write to HWC sector
```

Safety:

- It checks through sysfs (or pyusb) that the device is 1902:3241, and refuses otherwise.
- It walks the FAT (FAT12/16/32, with long file names) and verifies that the info LBA lies inside `VSFILE`. It refuses to read otherwise, unless `--no-fat-check` is given.
- It writes only with `--cmd`, only whitelisted getters plus `sync_time`, and only to the one cmd-write sector after the FAT proves it belongs to `HWC`.

Requirements:

- Python ≥ 3.8; tested only offline, on 3.14, against a synthetic image (`tools/selftest.py`).
- Root, or the `disk` group, to read `/dev/sdg`.
- For `--usb`: `apk add libusb`, `python3 -m venv v && v/bin/pip install pyusb`, and root.

To feed camerad, run the loop "poll info → read video → JPEG → decode (libjpeg-turbo / libyuv `MJPGToNV12`) → ring".

## 7. Open points, and what a capture would settle

1. **Linear multi-READ of a frame (M).** Raw mode reads a frame in one READ(10) of up to about 600 KB. The kernel splits `O_DIRECT` reads at `max_sectors_kb` (usb-storage defaults to 120 KB), which gives consecutive LBAs. File mode does this too (64 KiB chunks), so the firmware very likely serves linear LBAs. If frames come back corrupt in `--block` mode, either:
   - raise `/sys/block/sdg/queue/max_sectors_kb`, or
   - use `--usb`, which sends one CBW per frame.
2. **Frame size beyond `VSFILE` (M).** `VSFILE` is 196608 bytes, so the video region holds 187904 bytes in file mode. Raw mode allows up to 0x4B000. Large frames therefore need `--block` or `--usb`.
3. **Whether `sync_time` or another command is needed before frames flow (M).** The code suggests not.
4. **Which layout this unit uses, and whether `HWC` exists on it.** The probe prints both on its first run.
5. **The chip.** Unknown statically. `lsusb -v` and the SCSI INQUIRY strings (`sg_inq /dev/sdg`) may name it.

If static plus on-device probing proves insufficient, a usbmon capture of the original app would settle all of the above. It would need an Android ≥ 11 host using protocol 200, or a rooted Android device running `cat /sys/kernel/debug/usb/usbmon/<bus>u`. Capture the following:

- the CBW LBAs and lengths during preview start (to confirm the info / video / cmd LBAs and the per-frame transfer length);
- the order of the first commands (opcode 11 time sync; any others);
- whether an info READ is always followed by a video READ, or whether the app ever re-reads;
- the timing between frames (the fps).

## Verified on hardware (2026-10-22, laptop, /dev/sda)

`tools/dvr_probe.py --block /dev/sda --count 30`: info sector valid (version `0416_v4.8P`, board v1 header, SD card present), 30 frames in 1.48 s = 20.2 fps, frames about 46-64 KB, 1280x720 baseline JPEG with the camera's own date/speed overlay burned into the bottom-left corner (clock unset: 2022/04/26). The JPEG lacks an EOI marker inside `video_size`; decoders tolerate it. The kernel names the device "Buildwin Media-Player", so the controller is a Buildwin chip. Kernel usb-storage stayed attached; no writes were made.

## Streaming daemon

`camerad-dvr` (`camerad-dvr/main.cc`) implements the read-only streaming path of
section 4 in C++ and feeds roadsense's frame ring: raw block device with
`O_DIRECT | O_EXCL`, the probe's FAT/VSFILE validation, info-sector polling,
check-word verification, libjpeg-turbo raw YCbCr decode to NV12, and the
camera's JPEG as `preview.jpg`. Usage, measured numbers and the systemd unit:
[ARCHITECTURE.md, "Alternative camera sources"](ARCHITECTURE.md#alternative-camera-sources).
Measured with it: 19.5 fps over 3206 frames with no sequence gaps; VSFILE on
this unit is only 192 KiB (LBA 0x380..0x500), so frames are read past its end,
as the app does. The bytes after a frame's EOI (still inside `video_size`) can be stale data.

## Latency (2026-10-22, laptop, backlight-toggle test)

Glass-to-host latency measured by toggling the laptop backlight in front of the camera and timing the first frame (centre-of-screen ROI) that shows the change, via the block backend polling the info sector every ~3 ms. Backlight ON: change visible on the second frame after the write, 92-95 ms (n=5, spread 3 ms). Backlight OFF: the sysfs write itself blocks ~355 ms (panel fade), change visible ~42 ms after the write returns. So the camera's exposure + JPEG encode + USB delivery is roughly 45-95 ms, i.e. about one to two frame periods at 20 fps; call it 70 ms ± 25. Script: scratchpad `dvr-latency/latency.py` (not in the repo; needs root for the backlight and the block device).

## Calibration (2026-10-22, checkerboard on the laptop panel)

Setup: 12x6 board of 160 px squares (28.75 mm on the 15.6" 1920x1080 panel) drawn on /dev/fb0, camera 1100 mm in front, board in the lower-centre of the frame (about 300x140 px). All 5 frames detected (cv2 findChessboardCornersSB), corner pitch 24.92 px, reprojection rms 0.23 px.

| quantity | value |
|---|---|
| focal from corner pitch (pinhole, local) | 953 px |
| focal from calibrateCamera (single view, principal point fixed at centre) | 995 px, solved distance 1098 mm |
| radial distortion k1 | -0.34 (barrel; single off-centre view, so treat as indicative) |
| horizontal FOV | about 68 deg (not the 110 deg of the earlier guess) |

`config/camera-dvr.json` now uses focal_px 975. The lens is a moderate wide angle, close to openpilot's road camera, so the supercombo warp works without undistortion for the central crop; undistortion (k1) in camerad-dvr is a possible refinement for the lane-line geometry at the image edges.

## Framebuffer latency test (2026-10-22)

Writing white/black directly to /dev/fb0 on a text VT (i915drmfb) gave 144-253 ms, median 195 ms, versus 92-95 ms with the backlight toggle. The difference is the kernel's fbdev emulation itself: /dev/fb0 writes go through a shadow buffer flushed by deferred I/O, which adds a variable ~100 ms before the pixels reach the panel, so the framebuffer path is the worse reference on this kernel. The backlight PWM responds within a millisecond and remains the trustworthy bound: camera latency about 45-95 ms. A hardware LED driven by a GPIO would be the next step up in rigour.

## Latency, definitive (2026-10-22, LED reference, tools/ledlat)

An RP2040 LED switched over USB serial (1 ms round trip) at random phase to the frame clock, 40 events: **min 30, median 56, max 84 ms** glass-to-host. That is a fixed ~30 ms pipeline delay (exposure end, JPEG encode, USB read) plus 0-50 ms of frame-clock phase. The earlier backlight (92-95 ms) and framebuffer (195 ms) figures were inflated by the display path and by a trigger that was phase-locked to the frames; the LED numbers supersede them.

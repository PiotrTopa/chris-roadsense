#!/usr/bin/env python3
"""proto_probe.py - grab one MJPEG frame from the Huiying/fvision "USB DVR"
camera (USB 1902:3241, mass-storage only), reconstructed statically from
DVR.apk (com.fvision.camera, libuvccamera-lib.so). See PROTOCOL.md.

Transport summary: the camera is a plain USB mass-storage disk. The firmware
serves live data from fixed LBAs that lie inside two files of its FAT volume:
  VSFILE                               info sector, audio, then JPEG frame
  Android/data/com.fvision.camera/HWC  command sector (write), reply sector (read)
Getting video needs only READs. Commands are a 512-byte WRITE to the HWC sector.

Back ends (pick one):
  --block /dev/sdg   raw disk, O_DIRECT reads (default /dev/sdg). Kernel driver stays.
  --file  /mnt/x     mounted FAT root (the "read sdcard" protocol 100 of the app).
  --usb              pyusb, detaches usb-storage from interface 0, speaks BOT itself
                     (the "endpoint" protocol 200 of the app). Unmount sdg1 first.

Safety:
  * refuses to run unless the backing USB device is 1902:3241 (sysfs or pyusb);
  * never writes by default; commands need --cmd and are limited to a whitelist of
    getters plus time sync; a write goes only to the one HWC command sector, and
    only after the FAT on the device confirms that sector belongs to HWC;
  * reads stay inside LBA 0, the FAT metadata, VSFILE and HWC.

Requirements on the phone (aarch64 Alpine, Python 3.14): nothing for --block and
--file (stdlib only; needs read access to /dev/sdg, i.e. root or group disk).
--usb needs:  python3 -m venv ~/dvr-venv && ~/dvr-venv/bin/pip install pyusb
and libusb-1.0 (apk add libusb), run as root (detaching a kernel driver).

Usage examples:
  python3 -I proto_probe.py --block /dev/sdg -o frame.jpg
  python3 -I proto_probe.py --block /dev/sdg -o frame.jpg --count 30   # fps check
  python3 -I proto_probe.py --file /media/sdg1 -o frame.jpg
  python3 -I proto_probe.py --usb -o frame.jpg
  python3 -I proto_probe.py --block /dev/sdg --info-only
  python3 -I proto_probe.py --block /dev/sdg --cmd get_uid      # does ONE sector write
"""
import argparse
import json
import mmap
import os
import stat
import struct
import sys
import time

VID, PID = 0x1902, 0x3241
SECTOR = 512

# getVerApi(): MBR partition-1 start LBA == 8 selects layout A, else layout B.
LAYOUT_A = dict(info=0x380, audio=0x381, video=0x391, cmd_w=0x340, cmd_r=0x341)
LAYOUT_B = dict(info=0x300, audio=0x301, video=0x311, cmd_w=0x2C0, cmd_r=0x2C1)
# Offsets of the same regions inside the files (protocol 100 / --file)
VS_OFF = dict(info=0x0, audio=0x200, video=0x2200)
HWC_OFF = dict(cmd_w=0x0, cmd_r=0x200)

MAX_FRAME = 0x4B000            # getDataThread rejects bigger frames
AUDIO_LEN = 0x2000

# Whitelisted commands: (opcode, payload length, needs reply). Reply is read only
# for opcode >= 23 (nativeSendCmd). Nothing that changes recording or storage.
COMMANDS = {
    "get_uid":       (26, 16),   # CAM_GET_DVR_UID
    "get_adas_uid":  (25, 16),   # CAM_GET_ADAS_UID
    "get_vendor":    (28, 4),    # CAM_GET_VENDOR, int32 LE
    "get_duration":  (30, 4),    # CAM_GET_VIDEO_DURATION, int32 LE
    "get_adas_type": (31, 1),    # CAM_GET_ADAS_TYPE
    "get_card_size": (48, 8),    # two int32 LE
    "sync_time":     (11, 7),    # CAM_SET_TIME: year LE16, month, day, h, m, s
}


def die(msg):
    sys.stderr.write("proto_probe: " + msg + "\n")
    sys.exit(2)


# ---------------------------------------------------------------- device id --
def usb_id_for_devnum(st_dev_or_rdev):
    """Walk sysfs from a block device number up to its USB device node."""
    maj, mnr = os.major(st_dev_or_rdev), os.minor(st_dev_or_rdev)
    p = os.path.realpath("/sys/dev/block/%d:%d" % (maj, mnr))
    while p and p != "/":
        v, d = os.path.join(p, "idVendor"), os.path.join(p, "idProduct")
        if os.path.exists(v) and os.path.exists(d):
            return int(open(v).read(), 16), int(open(d).read(), 16), p
        p = os.path.dirname(p)
    return None


def check_block_id(st_num, what):
    r = usb_id_for_devnum(st_num)
    if r is None:
        die("%s is not backed by a USB device; refusing" % what)
    if (r[0], r[1]) != (VID, PID):
        die("%s belongs to USB %04x:%04x (%s), not %04x:%04x; refusing"
            % (what, r[0], r[1], r[2], VID, PID))
    return r[2]


# ---------------------------------------------------------------- back ends --
class AlignedIO:
    """O_DIRECT helper: page-aligned bounce buffer, 512-byte multiples."""

    def __init__(self, fd):
        self.fd = fd
        self.buf = mmap.mmap(-1, 1 << 20)

    def pread(self, off, n):
        if off % SECTOR:
            raise ValueError("unaligned offset")
        n_al = (n + SECTOR - 1) // SECTOR * SECTOR
        if n_al > len(self.buf):
            self.buf = mmap.mmap(-1, (n_al + 4095) // 4096 * 4096)
        mv = memoryview(self.buf)[:n_al]
        got = os.preadv(self.fd, [mv], off)
        data = bytes(mv[:min(got, n)])
        mv.release()
        return data

    def pwrite(self, off, data):
        if off % SECTOR or len(data) % SECTOR:
            raise ValueError("unaligned write")
        self.buf[:len(data)] = data
        mv = memoryview(self.buf)[:len(data)]
        n = os.pwritev(self.fd, [mv], off)
        mv.release()
        os.fsync(self.fd)
        return n


class BlockBackend:
    """Raw disk (/dev/sdg). LBAs are absolute disk LBAs, as in the app's raw mode."""

    def __init__(self, path):
        st = os.stat(path)
        if not stat.S_ISBLK(st.st_mode):
            die("%s is not a block device" % path)
        self.path = path
        self.sysfs = check_block_id(st.st_rdev, path)
        lbs = "/sys/dev/block/%d:%d/queue/logical_block_size" % (
            os.major(st.st_rdev), os.minor(st.st_rdev))
        if os.path.exists(lbs) and int(open(lbs).read()) != SECTOR:
            die("logical block size is not 512")
        self.fd = os.open(path, os.O_RDONLY | os.O_DIRECT)
        self.io = AlignedIO(self.fd)
        self.wfd = None

    def read_lba(self, lba, n):
        return self.io.pread(lba * SECTOR, n)

    def write_sector(self, lba, data):
        if self.wfd is None:
            self.wfd = os.open(self.path, os.O_RDWR | os.O_DIRECT | os.O_SYNC)
            self.wio = AlignedIO(self.wfd)
        return self.wio.pwrite(lba * SECTOR, data)

    def close(self):
        os.close(self.fd)
        if self.wfd is not None:
            os.close(self.wfd)


class UsbBackend:
    """pyusb Bulk-Only Transport: same CBWs as libUsbRead/libUsbWrite in the .so."""

    def __init__(self):
        try:
            import usb.core
            import usb.util
        except ImportError:
            die("pyusb missing: python3 -m venv v && v/bin/pip install pyusb")
        self.usb = usb
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            die("no USB device %04x:%04x" % (VID, PID))
        self.dev = dev
        self.reattach = False
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
            self.reattach = True
        usb.util.claim_interface(dev, 0)
        intf = dev.get_active_configuration()[(0, 0)]
        if (intf.bInterfaceClass, intf.bInterfaceSubClass, intf.bInterfaceProtocol) != (8, 6, 0x50):
            self.close()
            die("interface 0 is not mass-storage SCSI/BOT")
        eps = {e.bEndpointAddress for e in intf}
        if not {0x01, 0x81} <= eps:   # the app hard-codes OUT 0x01 / IN 0x81
            self.close()
            die("expected endpoints 0x01/0x81, found %s" % sorted(hex(e) for e in eps))
        for ep in (0x01, 0x81):
            try:
                dev.clear_halt(ep)
            except usb.core.USBError:
                pass
        self.tag = 0x74B4EA50   # tag value used by the app

    def _cbw(self, opcode, lba, nbytes, data_in):
        nsec = (nbytes + SECTOR - 1) // SECTOR
        cdb = struct.pack(">BBIBHB", opcode, 0, lba, 0, nsec, 0)
        self.tag = (self.tag + 1) & 0xFFFFFFFF
        cbw = struct.pack("<4sIIBBB", b"USBC", self.tag, nsec * SECTOR,
                          0x80 if data_in else 0x00, 0, len(cdb)) + cdb.ljust(16, b"\0")
        assert len(cbw) == 31
        self.dev.write(0x01, cbw, timeout=1000)
        return nsec * SECTOR

    def _csw(self):
        csw = bytes(self.dev.read(0x81, 13, timeout=10000))
        if len(csw) != 13 or csw[:4] != b"USBS":
            raise IOError("bad CSW %r" % csw)
        sig, tag, residue, status = struct.unpack("<4sIIB", csw)
        if status != 0:
            raise IOError("CSW status %d" % status)

    def read_lba(self, lba, n):
        total = self._cbw(0x28, lba, n, True)       # READ(10)
        data = bytes(self.dev.read(0x81, total, timeout=10000))
        self._csw()
        return data[:n]

    def write_sector(self, lba, data):
        total = self._cbw(0x2A, lba, len(data), False)  # WRITE(10)
        assert total == len(data)
        self.dev.write(0x01, data, timeout=10000)
        self._csw()
        return len(data)

    def close(self):
        u = self.usb.util
        try:
            u.release_interface(self.dev, 0)
        finally:
            if self.reattach:
                try:
                    self.dev.attach_kernel_driver(0)
                except self.usb.core.USBError:
                    pass
            u.dispose_resources(self.dev)


class FileBackend:
    """Mounted FAT: VSFILE + Android/data/com.fvision.camera/HWC with O_DIRECT."""

    def __init__(self, root):
        self.vs = os.path.join(root, "VSFILE")
        self.hwc = os.path.join(root, "Android", "data", "com.fvision.camera", "HWC")
        if not os.path.isfile(self.vs):
            die("no VSFILE under %s" % root)
        check_block_id(os.stat(self.vs).st_dev, self.vs)
        self.fd = os.open(self.vs, os.O_RDONLY | os.O_DIRECT)
        self.io = AlignedIO(self.fd)
        self.wfd = None

    def read_region(self, region, n, extra_off=0):
        if region in VS_OFF:
            return self.io.pread(VS_OFF[region] + extra_off, n)
        if region == "cmd_r":
            fd = self._hwc()
            return AlignedIO(fd).pread(HWC_OFF["cmd_r"], n)
        raise KeyError(region)

    def _hwc(self):
        if self.wfd is None:
            if not os.path.isfile(self.hwc):
                die("HWC command file missing: %s" % self.hwc)
            self.wfd = os.open(self.hwc, os.O_RDWR | os.O_DIRECT | os.O_SYNC)
        return self.wfd

    def write_cmd(self, data):
        return AlignedIO(self._hwc()).pwrite(HWC_OFF["cmd_w"], data)

    def close(self):
        os.close(self.fd)
        if self.wfd is not None:
            os.close(self.wfd)


# ------------------------------------------------------------- FAT verifier --
class Fat:
    """Minimal read-only FAT12/16/32 walker, used to prove which file owns an LBA."""

    def __init__(self, rd, part_lba):
        self.rd, self.p = rd, part_lba
        b = rd(part_lba, SECTOR)
        if b[510:512] != b"\x55\xaa":
            raise ValueError("no boot sector signature at LBA %d" % part_lba)
        bps, self.spc, res, nf, rootent, ts16, _, fsz16 = struct.unpack_from("<HBHBHHBH", b, 11)
        ts32, fsz32 = struct.unpack_from("<II", b, 32)
        if bps != SECTOR:
            raise ValueError("bytes/sector %d" % bps)
        fsz = fsz16 or fsz32
        self.fat_lba = part_lba + res
        root_secs = (rootent * 32 + SECTOR - 1) // SECTOR
        self.root_lba = self.fat_lba + nf * fsz
        self.data_lba = self.root_lba + root_secs
        clusters = ((ts16 or ts32) - (self.data_lba - part_lba)) // self.spc
        self.kind = 12 if clusters < 4085 else 16 if clusters < 65525 else 32
        self.root_clus = struct.unpack_from("<I", b, 44)[0] if self.kind == 32 else None
        self.root_secs = root_secs

    def clus_lba(self, c):
        return self.data_lba + (c - 2) * self.spc

    def next_clus(self, c):
        if self.kind == 16:
            off = c * 2
        elif self.kind == 32:
            off = c * 4
        else:
            off = c + c // 2
        sec = self.rd(self.fat_lba + off // SECTOR, 2 * SECTOR)
        o = off % SECTOR
        if self.kind == 16:
            v = struct.unpack_from("<H", sec, o)[0]
            return None if v >= 0xFFF8 else v
        if self.kind == 32:
            v = struct.unpack_from("<I", sec, o)[0] & 0x0FFFFFFF
            return None if v >= 0x0FFFFFF8 else v
        v = struct.unpack_from("<H", sec, o)[0]
        v = v >> 4 if c & 1 else v & 0xFFF
        return None if v >= 0xFF8 else v

    def chain(self, c, limit=4096):
        out = []
        while c is not None and c >= 2 and len(out) < limit:
            out.append(c)
            c = self.next_clus(c)
        return out

    def dir_bytes(self, clus):
        if clus is None:           # FAT12/16 fixed root
            return self.rd(self.root_lba, self.root_secs * SECTOR)
        return b"".join(self.rd(self.clus_lba(c), self.spc * SECTOR) for c in self.chain(clus, 64))

    def listdir(self, clus):
        d, lfn, out = self.dir_bytes(clus), [], {}
        for i in range(0, len(d), 32):
            e = d[i:i + 32]
            if e[0] == 0:
                break
            if e[0] == 0xE5:
                lfn = []
                continue
            if e[11] == 0x0F:
                part = e[1:11] + e[14:26] + e[28:32]
                lfn.insert(0, part.decode("utf-16-le", "ignore").split("\x00")[0].rstrip("￿"))
                continue
            short = e[0:8].decode("ascii", "replace").rstrip() + (
                "." + e[8:11].decode("ascii", "replace").rstrip() if e[8:11].strip() else "")
            name = "".join(lfn) or short
            lfn = []
            hi, lo = struct.unpack_from("<H", e, 20)[0], struct.unpack_from("<H", e, 26)[0]
            out[name.lower()] = dict(clus=(hi << 16 | lo) if self.kind == 32 else lo,
                                     size=struct.unpack_from("<I", e, 28)[0],
                                     dir=bool(e[11] & 0x10))
        return out

    def lookup(self, path):
        cur = self.root_clus
        ent = None
        for comp in path:
            ent = self.listdir(cur).get(comp.lower())
            if ent is None:
                return None
            cur = ent["clus"]
        return ent

    def file_lbas(self, ent):
        """List of (first_lba, nsectors) runs covering the file."""
        runs = []
        for c in self.chain(ent["clus"]):
            l = self.clus_lba(c)
            if runs and runs[-1][0] + runs[-1][1] == l:
                runs[-1][1] += self.spc
            else:
                runs.append([l, self.spc])
        return runs


def in_runs(runs, lba, n):
    return any(r0 <= lba and lba + n <= r0 + rn for r0, rn in runs)


# ---------------------------------------------------------- protocol logic --
def is_an_num(v):
    """isAnNum(): integrity check of the u16 at info+0x30 (LE)."""
    lo, hi = v & 0xFF, (v >> 8) & 0xFF
    r = ((hi >> 5) ^ 0x0B) & 0xFF
    cnt = sum((hi >> i) & 1 for i in range(5))
    r = (r | ((cnt ^ 0x0C) << 3)) & 0xFF
    return r == lo


def parse_info(b):
    flags, = struct.unpack_from("<H", b, 0)
    total_t, cur_t = struct.unpack_from("<hh", b, 2)
    vsize, vseq, alen, aseq = struct.unpack_from("<IIII", b, 0x0C)
    chk, = struct.unpack_from("<I", b, 0x2C)
    an, = struct.unpack_from("<H", b, 0x30)
    h = dict(
        flags=flags,
        playback_mode=bool(flags & 0x8000), recording=bool(flags & 0x4000),
        locked=bool(flags & 0x2000), playing=bool(flags & 0x1000),
        watermark=bool(flags & 0x0800), mute=bool(flags & 0x0400),
        sd_present=bool(flags & 0x0200),
        total_time=total_t, cur_time=cur_t, gsensor=b[6], save_1=b[7],
        file_index=struct.unpack_from("<h", b, 8)[0],
        video_size=vsize, video_seq=vseq, audio_len=alen, audio_seq=aseq,
        version=b[0x1C:0x28].split(b"\0")[0].decode("latin-1"),
        play_speed=struct.unpack_from("<h", b, 0x26)[0],
        frame_check=chk, an_word=an, an_ok=is_an_num(an),
        adas_state=b[0x32], board_byte=b[0x34], board_v1=(b[0x34] == 0xE9),
    )
    h["valid"] = (alen in (0, AUDIO_LEN) and 0 < vsize <= MAX_FRAME and flags != 0)
    return h


class Camera:
    def __init__(self, be, layout_override=None, no_fat_check=False):
        self.be = be
        self.layout = None
        self.cmd_ok = False
        if isinstance(be, FileBackend):
            self.cmd_ok = os.path.isfile(be.hwc)
            return
        mbr = be.read_lba(0, SECTOR)
        if mbr[510:512] != b"\x55\xaa":
            die("LBA 0 has no MBR signature")
        part = struct.unpack_from("<I", mbr, 0x1C6)[0]
        self.layout = dict(LAYOUT_A if part == 8 else LAYOUT_B)
        self.part = part
        if layout_override:
            self.layout = dict(LAYOUT_A if layout_override == "A" else LAYOUT_B)
        sys.stderr.write("partition 1 at LBA %d -> layout %s\n"
                         % (part, "A" if self.layout == LAYOUT_A else "B"))
        try:
            fat = Fat(be.read_lba, part)
            vs = fat.lookup(["VSFILE"])
            hwc = fat.lookup(["Android", "data", "com.fvision.camera", "HWC"])
            vs_runs = fat.file_lbas(vs) if vs else []
            hwc_runs = fat.file_lbas(hwc) if hwc else []
            sys.stderr.write("FAT%d: VSFILE %s runs=%s ; HWC %s runs=%s\n" % (
                fat.kind, vs and vs["size"], vs_runs[:4], hwc and hwc["size"], hwc_runs[:4]))
            L = self.layout
            vs_ok = bool(vs) and in_runs(vs_runs, L["info"], 1 + 16 + 1)
            self.cmd_ok = bool(hwc) and in_runs(hwc_runs, L["cmd_w"], 1) and in_runs(hwc_runs, L["cmd_r"], 1)
            if not vs_ok:
                msg = "VSFILE does not cover info LBA 0x%x" % L["info"]
                if vs_runs:
                    msg += " (VSFILE starts at LBA 0x%x)" % vs_runs[0][0]
                if not no_fat_check:
                    die(msg + "; use --no-fat-check to read anyway (reads only)")
                sys.stderr.write("WARNING: " + msg + "\n")
            if not self.cmd_ok:
                sys.stderr.write("HWC does not cover cmd LBAs 0x%x/0x%x: commands disabled\n"
                                 % (L["cmd_w"], L["cmd_r"]))
        except Exception as e:  # noqa: BLE001
            if not no_fat_check:
                die("FAT verification failed: %s (use --no-fat-check for reads only)" % e)
            sys.stderr.write("WARNING: FAT check skipped: %s\n" % e)
            self.cmd_ok = False

    # region reads -------------------------------------------------------
    def read(self, region, n, extra=0):
        if isinstance(self.be, FileBackend):
            return self.be.read_region(region, n, extra)
        return self.be.read_lba(self.layout[region] + extra // SECTOR, n)

    def info(self):
        return parse_info(self.read("info", SECTOR))

    def read_frame(self, h):
        size = h["video_size"]
        n = (size & ~0x1FF) + 0x200          # exactly what getDataThread reads
        data = self.read("video", n)
        if len(data) < size:
            return None, "short read %d < %d" % (len(data), size)
        frame = data[:size]
        off = ((size - 8) >> 2) * 4
        word = struct.unpack_from("<I", data, off)[0]
        if word != h["frame_check"] and not h["playback_mode"]:
            return None, "check word mismatch 0x%08x != 0x%08x" % (word, h["frame_check"])
        return frame, None

    def kick(self, h):
        """Stall path of getDataThread: dummy read of size+0x1400 from the video area."""
        n = (h["video_size"] & ~0x1FF) + 0x1400
        self.read("video", min(n, MAX_FRAME + 0x1400))

    # commands -----------------------------------------------------------
    def send_cmd(self, name, board_v1):
        op, ln = COMMANDS[name]
        if not self.cmd_ok:
            die("command path not verified (HWC sectors); refusing to write")
        if name == "sync_time":
            t = time.localtime()
            payload = struct.pack("<HBBBBB", t.tm_year, t.tm_mon, t.tm_mday,
                                  t.tm_hour, t.tm_min, t.tm_sec)
        else:
            payload = bytes(ln)
        if board_v1:
            pkt = struct.pack("<HH", op, ln) + payload
        else:
            pkt = struct.pack("<BB", op, ln) + payload
        pkt = pkt.ljust(SECTOR, b"\0")       # len <= 0x1fc -> one 512-byte sector
        if isinstance(self.be, FileBackend):
            self.be.write_cmd(pkt)
        else:
            self.be.write_sector(self.layout["cmd_w"], pkt)
        if op < 23:
            return None
        return self.read("cmd_r", SECTOR)[:ln]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--block", metavar="DEV", help="raw disk, default /dev/sdg")
    g.add_argument("--file", metavar="MOUNTPOINT", help="mounted FAT root holding VSFILE")
    g.add_argument("--usb", action="store_true", help="pyusb BOT, detaches usb-storage")
    ap.add_argument("-o", "--out", default="frame.jpg")
    ap.add_argument("--count", type=int, default=1, help="frames to grab (last one saved)")
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--info-only", action="store_true")
    ap.add_argument("--layout", choices=["A", "B"], help="override MBR-based layout choice")
    ap.add_argument("--no-fat-check", action="store_true", help="read even if FAT check fails")
    ap.add_argument("--cmd", choices=sorted(COMMANDS), help="send ONE whitelisted command")
    a = ap.parse_args()

    if a.usb:
        be = UsbBackend()
    elif a.file:
        be = FileBackend(a.file)
    else:
        be = BlockBackend(a.block or "/dev/sdg")
    try:
        cam = Camera(be, a.layout, a.no_fat_check)
        t_end = time.time() + a.timeout
        h = cam.info()
        while not (h["valid"] and h["an_ok"]) and time.time() < t_end:
            time.sleep(0.02)
            h = cam.info()
        print(json.dumps({k: v for k, v in h.items()}, indent=1))
        if not (h["valid"] and h["an_ok"]):
            die("no valid info header within %.1fs" % a.timeout)
        if a.info_only:
            return
        if a.cmd:
            r = cam.send_cmd(a.cmd, h["board_v1"])
            print("cmd %s reply: %s" % (a.cmd, r.hex() if r is not None else "(none: opcode < 23)"))
            return
        last_seq, got, stale, t0 = 0, 0, 0, time.time()
        frame = None
        while got < a.count and time.time() < t_end + a.count * 0.2:
            h = cam.info()
            if not (h["valid"] and h["an_ok"]):
                time.sleep(0.01)
                continue
            if h["video_seq"] != last_seq:
                f, err = cam.read_frame(h)
                if f is None:
                    sys.stderr.write("frame seq %d rejected: %s\n" % (h["video_seq"], err))
                    continue
                last_seq, frame, got, stale = h["video_seq"], f, got + 1, 0
                continue
            time.sleep(0.02)
            stale += 1
            if stale > 20 and not h["playback_mode"]:
                cam.kick(h)
                stale = 0
        if frame is None:
            die("no frame captured")
        dt = time.time() - t0
        with open(a.out, "wb") as fo:
            fo.write(frame)
        print("saved %s: %d bytes, SOI=%s EOI=%s, seq=%d, %d frames in %.2fs (%.1f fps)" % (
            a.out, len(frame), frame[:2] == b"\xff\xd8", frame.rstrip(b"\0")[-2:] == b"\xff\xd9",
            last_seq, got, dt, got / dt if dt > 0 else 0))
    finally:
        be.close()


if __name__ == "__main__":
    main()

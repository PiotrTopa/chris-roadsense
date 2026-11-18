#!/usr/bin/env python3
"""roadsense-dvr-timesync: send the vendor app's time sync to a USB DVR camera (1902:3241).

Why: the camera expects the first command its own app sends after connecting
(CAM_SET_TIME, opcode 11). Without it the camera leaves the USB bus once, 28 s
after power-up, for about 5 s; with it, it stays (docs/PHONE_DVR_DEPLOY.md,
"USB drop-outs after power-up"). Started by udev for every such disk that
appears (systemd/roadsense-dvr-timesync@.service, 60-roadsense-dvr.rules), so
the camera is handled once, whoever reads it afterwards (camerad-dvr, a
preview reader, nobody).

    roadsense-dvr-timesync /dev/sdX     (root or group disk)

It writes exactly one 512-byte sector, the command sector of the file
Android/data/com.fvision.camera/HWC, and only after the FAT on the device
confirms that sector belongs to that file. Any other device, layout or doubt:
no write, exit 1. Standard library only.
"""
import mmap
import os
import sys
import time

SECTOR = 512
VID, PID = "1902", "3241"
OP_SET_TIME = 11
BOARD_V1 = 0xE9
LAYOUTS = {True: dict(info=0x380, cmd_w=0x340), False: dict(info=0x300, cmd_w=0x2C0)}  # part at LBA 8 / else
HWC_PATH = ("Android", "data", "com.fvision.camera", "HWC")
AUDIO_LEN, MAX_FRAME = 0x2000, 0x4B000
WAIT_INFO_S = 15.0


def log(msg):
    sys.stderr.write("roadsense-dvr-timesync: %s\n" % msg)


def u16(b, o):
    return int.from_bytes(b[o:o + 2], "little")


def u32(b, o):
    return int.from_bytes(b[o:o + 4], "little")


def usb_id(dev):
    """(vendor, product) of the USB device behind a whole-disk node, or None."""
    name = os.path.basename(os.path.realpath(dev))
    p = os.path.realpath("/sys/class/block/%s/device" % name)
    while p and p != "/":
        try:
            return open(p + "/idVendor").read().strip(), open(p + "/idProduct").read().strip()
        except OSError:
            p = os.path.dirname(p)
    return None


class Disk:
    def __init__(self, path):
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_DIRECT | os.O_CLOEXEC)
        self.buf = mmap.mmap(-1, 1 << 20)

    def read(self, lba, n):
        n = (n + SECTOR - 1) // SECTOR * SECTOR
        view = memoryview(self.buf)[:n]
        try:
            if os.preadv(self.fd, [view], lba * SECTOR) != n:
                raise OSError(5, "short read")
            return bytes(view)
        finally:
            view.release()

    def write_sector(self, lba, data):
        assert len(data) == SECTOR
        self.buf[:SECTOR] = data
        fd = os.open(self.path, os.O_WRONLY | os.O_DIRECT | os.O_SYNC | os.O_CLOEXEC)
        try:
            view = memoryview(self.buf)[:SECTOR]
            try:
                if os.pwritev(fd, [view], lba * SECTOR) != SECTOR:
                    raise OSError(5, "short write")
            finally:
                view.release()
        finally:
            os.close(fd)


class Fat:
    def __init__(self, read, part):
        self.read = read
        b = read(part, SECTOR)
        if b[510:512] != b"\x55\xaa":
            raise ValueError("no FAT boot sector at LBA %d" % part)
        bps, self.spc, res, nf = u16(b, 11), b[13], u16(b, 14), b[16]
        ts = u16(b, 19) or u32(b, 32)
        fsz = u16(b, 22) or u32(b, 36)
        if bps != SECTOR or self.spc == 0:
            raise ValueError("unexpected FAT geometry")
        self.fat_lba = part + res
        self.root_secs = (u16(b, 17) * 32 + SECTOR - 1) // SECTOR
        self.root_lba = self.fat_lba + nf * fsz
        self.data_lba = self.root_lba + self.root_secs
        clusters = (ts - (self.data_lba - part)) // self.spc
        self.kind = 12 if clusters < 4085 else 16 if clusters < 65525 else 32
        self.root_clus = u32(b, 44) if self.kind == 32 else 0

    def next(self, c):
        off = c * 2 if self.kind == 16 else c * 4 if self.kind == 32 else c + c // 2
        s = self.read(self.fat_lba + off // SECTOR, 2 * SECTOR)
        o = off % SECTOR
        if self.kind == 16:
            v = u16(s, o)
            return -1 if v >= 0xFFF8 else v
        if self.kind == 32:
            v = u32(s, o) & 0x0FFFFFFF
            return -1 if v >= 0x0FFFFFF8 else v
        v = u16(s, o)
        v = v >> 4 if c & 1 else v & 0xFFF
        return -1 if v >= 0xFF8 else v

    def chain(self, c, limit=4096):
        out = []
        while c >= 2 and len(out) < limit:
            out.append(c)
            c = self.next(c)
        return out

    def lba(self, c):
        return self.data_lba + (c - 2) * self.spc

    def read_dir(self, clus):
        if clus < 2:
            return self.read(self.root_lba, self.root_secs * SECTOR) if self.kind != 32 else b""
        return b"".join(self.read(self.lba(c), self.spc * SECTOR) for c in self.chain(clus, 64))

    def lookup(self, d, name):
        want, lfn = name.lower(), ""
        for i in range(0, len(d) - 31, 32):
            e = d[i:i + 32]
            if e[0] == 0:
                break
            if e[0] == 0xE5:
                lfn = ""
                continue
            if e[11] == 0x0F:                       # long-name part, last part first
                part = ""
                for k in (1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30):
                    ch = u16(e, k)
                    if ch in (0, 0xFFFF):
                        break
                    part += chr(ch) if ch < 0x80 else "?"
                lfn = part + lfn
                continue
            if e[11] & 0x08:
                lfn = ""
                continue
            base, ext = e[0:8].decode("latin-1").rstrip(), e[8:11].decode("latin-1").rstrip()
            short = base + ("." + ext if ext else "")
            hit = (lfn and lfn.lower() == want) or short.lower() == want
            lfn = ""
            if hit:
                return (u16(e, 20) << 16 if self.kind == 32 else 0) | u16(e, 26), bool(e[11] & 0x10)
        return -1, False


def hwc_covers(read, part, lba):
    """True when the HWC command file exists and contains sector `lba`."""
    f = Fat(read, part)
    clus, is_dir = (f.root_clus if f.kind == 32 else 0), True
    for comp in HWC_PATH:
        if not is_dir:
            return False
        clus, is_dir = f.lookup(f.read_dir(clus), comp)
        if clus < 2:
            return False
    return (not is_dir) and any(f.lba(c) <= lba < f.lba(c) + f.spc for c in f.chain(clus))


def is_an_num(v):
    lo, hi = v & 0xFF, (v >> 8) & 0xFF
    return lo == (((bin(hi & 0x1F).count("1") ^ 0x0C) << 3) | ((hi >> 5) ^ 0x0B)) & 0xFF


def info_valid(b):
    return (u32(b, 0x14) in (0, AUDIO_LEN) and 0 < u32(b, 0x0C) <= MAX_FRAME and u16(b, 0) != 0
            and is_an_num(u16(b, 0x30)))


def time_packet(board_v1, t=None):
    t = t or time.localtime()
    payload = bytes([t.tm_year & 0xFF, t.tm_year >> 8, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec])
    head = bytes([OP_SET_TIME, 0, 7, 0]) if board_v1 else bytes([OP_SET_TIME, 7])
    return (head + payload).ljust(SECTOR, b"\0")


def main(argv):
    if len(argv) != 2 or argv[1].startswith("-"):
        sys.stderr.write(__doc__)
        return 2
    dev = argv[1]
    t0 = time.monotonic()
    if usb_id(dev) != (VID, PID):
        log("%s is not a %s:%s USB disk; not touching it" % (dev, VID, PID))
        return 1
    try:
        disk = Disk(dev)
        mbr = disk.read(0, SECTOR)
        if mbr[510:512] != b"\x55\xaa":
            log("%s: no MBR" % dev)
            return 1
        part = u32(mbr, 0x1C6)
        lay = LAYOUTS[part == 8]
        if not hwc_covers(disk.read, part, lay["cmd_w"]):
            log("%s: command sector 0x%x is not inside %s; not writing" % (dev, lay["cmd_w"], "/".join(HWC_PATH)))
            return 1
        info = b""
        while time.monotonic() - t0 < WAIT_INFO_S:          # zeros right after power-up
            info = disk.read(lay["info"], SECTOR)
            if info_valid(info):
                break
            time.sleep(0.2)
        else:
            log("%s: no valid info sector within %.0f s; not writing" % (dev, WAIT_INFO_S))
            return 1
        board_v1 = info[0x34] == BOARD_V1
        disk.write_sector(lay["cmd_w"], time_packet(board_v1))
    except (OSError, ValueError) as exc:
        log("%s: %s" % (dev, exc))
        return 1
    log("%s: time sync sent (board %s, %.1f s after start)" % (dev, "v1" if board_v1 else "v0", time.monotonic() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

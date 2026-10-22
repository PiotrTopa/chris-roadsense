# ledlat: LED latency reference on an RP2040-Zero

Firmware (`main.c`, Pico SDK 2.2.0, `-DPICO_BOARD=waveshare_rp2040_zero`): the WS2812 on
GP16 is switched by one serial byte (`1` on, `0` off, `p` 30 ms pulse, `B` BOOTSEL) and each
command is answered `ok <us>`; host round trip about 1 ms. `ledlat_test.py` fires LED-on
events at random phase relative to the camera's frame clock and times the first frame in
which a cluster of pixels brightened (`tools/dvr_probe.py` block backend, root needed).

Result for the Huiying/Buildwin USB DVR camera (2026-10-22, 40 events):
min 30, p25 47, median 56, p75 64, max 84 ms. The spread is one frame period (50 ms at
20 fps) plus jitter, so the camera's fixed pipeline delay is about 30 ms and a scene
change is in the host's hands 30-85 ms later, 55 ms on average.

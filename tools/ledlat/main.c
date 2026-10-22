// ledlat: camera-latency reference. Serial (USB CDC) commands, each answered with "ok <us>\n"
// where <us> is the Pico's microsecond clock right after the LED was updated:
//   '1' LED white on, '0' LED off, 'p' 30 ms white pulse (on, wait, off), 'B' reboot to BOOTSEL.
// WS2812 on GP16 (RP2040-Zero), RGB order (this board), copied from linscan.
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

static void led_init(void) {
    uint off = pio_add_program(pio0, &ws2812_program);
    pio_gpio_init(pio0, 16);
    pio_sm_set_consecutive_pindirs(pio0, 0, 16, 1, true);
    pio_sm_config c = ws2812_program_get_default_config(off);
    sm_config_set_sideset_pins(&c, 16);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    int cyc = ws2812_T1 + ws2812_T2 + ws2812_T3;
    sm_config_set_clkdiv(&c, clock_get_hz(clk_sys) / (800000.0f * cyc));
    pio_sm_init(pio0, 0, off, &c);
    pio_sm_set_enabled(pio0, 0, true);
}
static void led(uint8_t r, uint8_t g, uint8_t b) {
    pio_sm_put_blocking(pio0, 0, ((uint32_t)r << 24) | ((uint32_t)g << 16) | ((uint32_t)b << 8));
}
int main(void) {
    stdio_init_all();
    led_init();
    led(0, 0, 0);
    for (;;) {
        int c = getchar_timeout_us(100000);
        if (c == PICO_ERROR_TIMEOUT) continue;
        if (c == '1') { led(255, 255, 255); printf("ok %llu\n", time_us_64()); }
        else if (c == '0') { led(0, 0, 0); printf("ok %llu\n", time_us_64()); }
        else if (c == 'p') { led(255, 255, 255); uint64_t t = time_us_64(); sleep_ms(30); led(0, 0, 0); printf("ok %llu\n", t); }
        else if (c == 'B') { printf("bootsel\n"); sleep_ms(50); reset_usb_boot(0, 0); }
    }
}

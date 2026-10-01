#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/gpio.h"

#define PIN_PHI2      0
#define PIN_WR        1
#define PIN_WR_HI     35
#define PIN_PHI2_HI   36

static void configure_input(uint pin, bool pull_up) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
    if (pull_up) gpio_pull_up(pin);
}

typedef struct {
    uint32_t phi2_edges_lo;
    uint32_t phi2_edges_hi;
    uint32_t wr_edges_lo;
    uint32_t wr_edges_hi;
    uint32_t wr_low_samples_lo;
    uint32_t wr_low_samples_hi;
    uint32_t samples;
    uint8_t start_phi2_lo, end_phi2_lo;
    uint8_t start_phi2_hi, end_phi2_hi;
    uint8_t start_wr_lo, end_wr_lo;
    uint8_t start_wr_hi, end_wr_hi;
} diag_t;

static diag_t sample_window_us(uint32_t duration_us) {
    diag_t d = {0};
    uint64_t v = gpio_get_all64();
    uint8_t p0 = (v >> PIN_PHI2) & 1u;
    uint8_t p36 = (v >> PIN_PHI2_HI) & 1u;
    uint8_t w1 = (v >> PIN_WR) & 1u;
    uint8_t w35 = (v >> PIN_WR_HI) & 1u;

    d.start_phi2_lo = p0;
    d.start_phi2_hi = p36;
    d.start_wr_lo = w1;
    d.start_wr_hi = w35;

    uint64_t deadline = time_us_64() + duration_us;
    while (time_us_64() < deadline) {
        v = gpio_get_all64();
        uint8_t n0 = (v >> PIN_PHI2) & 1u;
        uint8_t n36 = (v >> PIN_PHI2_HI) & 1u;
        uint8_t n1 = (v >> PIN_WR) & 1u;
        uint8_t n35 = (v >> PIN_WR_HI) & 1u;

        d.phi2_edges_lo += (n0 != p0);
        d.phi2_edges_hi += (n36 != p36);
        d.wr_edges_lo += (n1 != w1);
        d.wr_edges_hi += (n35 != w35);
        d.wr_low_samples_lo += !n1;
        d.wr_low_samples_hi += !n35;
        d.samples++;

        p0 = n0;
        p36 = n36;
        w1 = n1;
        w35 = n35;
    }

    d.end_phi2_lo = p0;
    d.end_phi2_hi = p36;
    d.end_wr_lo = w1;
    d.end_wr_hi = w35;
    return d;
}

int main(void) {
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(50);
    sleep_ms(200);

    configure_input(PIN_PHI2, false);
    configure_input(PIN_WR, true);
    configure_input(PIN_WR_HI, true);
    configure_input(PIN_PHI2_HI, false);

    printf("\n=== SNES RP2350B SIO SIGNAL DIAGNOSTIC v0.5 ===\n");
    printf("This version DOES NOT use PIO or DMA. It only watches the raw GPIO pads.\n");
    printf("Expected: GP0=PHI2, GP1=/WR, GP36=PHI2 jumper copy, GP35=/WR jumper copy.\n");
    printf("Keep RP2350B powered before the SNES. Turn the SNES on now.\n\n");
    fflush(stdout);

    uint64_t iter = 0;
    while (true) {
        ++iter;
        diag_t d = sample_window_us(100000); // 100 ms busy sample window

        double wr_low_pct_lo = d.samples ? (100.0 * d.wr_low_samples_lo / d.samples) : 0.0;
        double wr_low_pct_hi = d.samples ? (100.0 * d.wr_low_samples_hi / d.samples) : 0.0;

        printf("#%" PRIu64 "  100ms samples=%lu\n", iter, (unsigned long)d.samples);
        printf(" PHI2: GP0 edges=%lu (%u->%u) | GP36 edges=%lu (%u->%u)\n",
               (unsigned long)d.phi2_edges_lo, d.start_phi2_lo, d.end_phi2_lo,
               (unsigned long)d.phi2_edges_hi, d.start_phi2_hi, d.end_phi2_hi);
        printf(" /WR : GP1 edges=%lu low=%.2f%% (%u->%u) | GP35 edges=%lu low=%.2f%% (%u->%u)\n\n",
               (unsigned long)d.wr_edges_lo, wr_low_pct_lo, d.start_wr_lo, d.end_wr_lo,
               (unsigned long)d.wr_edges_hi, wr_low_pct_hi, d.start_wr_hi, d.end_wr_hi);
        fflush(stdout);
        sleep_ms(400);
    }
}

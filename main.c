#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "capture_low.pio.h"
#include "capture_high.pio.h"

#define PIN_PHI2       0
#define PIN_WR         1
#define PIN_DATA_BASE  2
#define PIN_WR_HI      35
#define PIN_PHI2_HI    36

#define SAMPLE_COUNT   64

static uint32_t low_samples[SAMPLE_COUNT];
static uint32_t high_samples[SAMPLE_COUNT];

static void configure_input(uint pin, bool pull_up) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
    if (pull_up) gpio_pull_up(pin);
}

static uint32_t unpack_low_word(uint32_t raw) {
    return raw >> 2; // 30 captured bits occupy bits 31:2
}

static uint32_t unpack_high_word(uint32_t raw) {
    return raw >> 29; // 3 captured bits occupy bits 31:29
}

static uint32_t reconstruct_address(uint32_t low30, uint32_t high3) {
    const uint32_t a0_a9   = (low30 >> 8)  & 0x3ffu;
    const uint32_t a10_a20 = (low30 >> 19) & 0x7ffu;
    const uint32_t a21_a23 = high3 & 0x7u;
    return a0_a9 | (a10_a20 << 10) | (a21_a23 << 21);
}

static uint32_t dma_remaining(uint channel) {
    // RP2350: low 28 bits hold the normal transfer count.
    return dma_channel_hw_addr(channel)->transfer_count & 0x0fffffffu;
}

int main(void) {
    stdio_init_all();

    // Do not throw away the boot messages. Wait for a terminal (PuTTY, TeraTerm, etc.)
    // to actually open the CDC serial port.
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    sleep_ms(250);

    printf("\n=== SNES RP2350B bus capture DIAGNOSTIC v0.4 INFINITE ===\n");
    printf("USB serial connected. Keep the RP2350B powered before powering the SNES.\n");
    printf("Expected jumpers: GP0->GP36 (PHI2), GP1->GP35 (/WR).\n\n");
    fflush(stdout);

    for (uint pin = 0; pin <= 36; ++pin) {
        if (pin == 20) continue; // onboard WS2812
        configure_input(pin, pin == PIN_WR || pin == PIN_WR_HI);
    }

    PIO pio_lo = pio0;
    const uint sm_lo = 0;
    int base_lo_rc = pio_set_gpio_base(pio_lo, 0);

    // IMPORTANT: route the physical pads to PIO0. gpio_init() above leaves them
    // on the SIO function; PIO cannot sample them until their function select
    // is changed to this PIO instance. Keep every line input-only.
    pio_gpio_init(pio_lo, PIN_PHI2);
    pio_gpio_init(pio_lo, PIN_WR);
    for (uint pin = 2; pin <= 19; ++pin) pio_gpio_init(pio_lo, pin);
    for (uint pin = 21; pin <= 31; ++pin) pio_gpio_init(pio_lo, pin);
    pio_sm_set_consecutive_pindirs(pio_lo, sm_lo, 0, 20, false);
    pio_sm_set_consecutive_pindirs(pio_lo, sm_lo, 21, 11, false);

    uint off_lo = pio_add_program(pio_lo, &snes_capture_low_program);
    pio_sm_config c_lo = snes_capture_low_program_get_default_config(off_lo);
    sm_config_set_in_pins(&c_lo, PIN_DATA_BASE);
    sm_config_set_jmp_pin(&c_lo, PIN_WR);
    sm_config_set_in_shift(&c_lo, true, false, 32);
    sm_config_set_fifo_join(&c_lo, PIO_FIFO_JOIN_RX);
    int init_lo_rc = pio_sm_init(pio_lo, sm_lo, off_lo, &c_lo);

    PIO pio_hi = pio1;
    const uint sm_hi = 0;
    int base_hi_rc = pio_set_gpio_base(pio_hi, 16);

    // Route A21-A23 and the duplicated /WR + PHI2 lines to PIO1.
    for (uint pin = 32; pin <= 36; ++pin) pio_gpio_init(pio_hi, pin);
    pio_sm_set_consecutive_pindirs(pio_hi, sm_hi, 32, 5, false);

    uint off_hi = pio_add_program(pio_hi, &snes_capture_high_program);
    pio_sm_config c_hi = snes_capture_high_program_get_default_config(off_hi);
    sm_config_set_in_pins(&c_hi, 32);
    sm_config_set_jmp_pin(&c_hi, PIN_WR_HI);
    sm_config_set_in_shift(&c_hi, true, false, 32);
    sm_config_set_fifo_join(&c_hi, PIO_FIFO_JOIN_RX);
    int init_hi_rc = pio_sm_init(pio_hi, sm_hi, off_hi, &c_hi);

    printf("PIO pads routed to PIO0/PIO1.\n");
    printf("PIO init: base_lo=%d init_lo=%d | base_hi=%d init_hi=%d\n",
           base_lo_rc, init_lo_rc, base_hi_rc, init_hi_rc);
    if (base_lo_rc || init_lo_rc || base_hi_rc || init_hi_rc) {
        printf("ERROR: a PIO configuration call failed. Do not power the SNES yet.\n");
        fflush(stdout);
        while (true) sleep_ms(1000);
    }

    pio_sm_clear_fifos(pio_lo, sm_lo);
    pio_sm_clear_fifos(pio_hi, sm_hi);

    int dma_lo = dma_claim_unused_channel(true);
    int dma_hi = dma_claim_unused_channel(true);

    dma_channel_config dc_lo = dma_channel_get_default_config(dma_lo);
    channel_config_set_transfer_data_size(&dc_lo, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_lo, false);
    channel_config_set_write_increment(&dc_lo, true);
    channel_config_set_dreq(&dc_lo, pio_get_dreq(pio_lo, sm_lo, false));

    dma_channel_config dc_hi = dma_channel_get_default_config(dma_hi);
    channel_config_set_transfer_data_size(&dc_hi, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_hi, false);
    channel_config_set_write_increment(&dc_hi, true);
    channel_config_set_dreq(&dc_hi, pio_get_dreq(pio_hi, sm_hi, false));

    dma_channel_configure(dma_lo, &dc_lo, low_samples, &pio_lo->rxf[sm_lo], SAMPLE_COUNT, false);
    dma_channel_configure(dma_hi, &dc_hi, high_samples, &pio_hi->rxf[sm_hi], SAMPLE_COUNT, false);

    printf("READY. Turn the SNES on now.\n");
    printf("I will keep waiting indefinitely and print capture progress every 500 ms.\n\n");
    fflush(stdout);

    dma_start_channel_mask((1u << dma_lo) | (1u << dma_hi));
    pio_sm_set_enabled(pio_lo, sm_lo, true);
    pio_sm_set_enabled(pio_hi, sm_hi, true);

    uint32_t last_lo = SAMPLE_COUNT + 1;
    uint32_t last_hi = SAMPLE_COUNT + 1;
    uint64_t heartbeat_count = 0;

    // No timeout in v0.4. This loop waits forever until both halves have
    // captured SAMPLE_COUNT write cycles. If the SNES is off or a signal is
    // missing, the heartbeat continues indefinitely instead of aborting.
    while (true) {
        uint32_t rem_lo = dma_remaining(dma_lo);
        uint32_t rem_hi = dma_remaining(dma_hi);

        if (rem_lo != last_lo || rem_hi != last_hi) {
            printf("progress: LOW %lu/%d   HIGH %lu/%d   /WR GP1=%d GP35=%d\n",
                   (unsigned long)(SAMPLE_COUNT - rem_lo), SAMPLE_COUNT,
                   (unsigned long)(SAMPLE_COUNT - rem_hi), SAMPLE_COUNT,
                   gpio_get(PIN_WR), gpio_get(PIN_WR_HI));
            fflush(stdout);
            last_lo = rem_lo;
            last_hi = rem_hi;
        } else {
            ++heartbeat_count;
            printf("heartbeat #%llu: LOW %lu/%d   HIGH %lu/%d   /WR GP1=%d GP35=%d\n",
                   (unsigned long long)heartbeat_count,
                   (unsigned long)(SAMPLE_COUNT - rem_lo), SAMPLE_COUNT,
                   (unsigned long)(SAMPLE_COUNT - rem_hi), SAMPLE_COUNT,
                   gpio_get(PIN_WR), gpio_get(PIN_WR_HI));
            fflush(stdout);
        }

        if (rem_lo == 0 && rem_hi == 0) break;

        sleep_ms(500);
    }

    pio_sm_set_enabled(pio_lo, sm_lo, false);
    pio_sm_set_enabled(pio_hi, sm_hi, false);

    printf("\nCaptured %d paired write cycles:\n\n", SAMPLE_COUNT);

    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        uint32_t low30 = unpack_low_word(low_samples[i]);
        uint32_t high3 = unpack_high_word(high_samples[i]);
        uint8_t data = (uint8_t)(low30 & 0xffu);
        uint32_t address = reconstruct_address(low30, high3) & 0xffffffu;
        uint8_t bank = (uint8_t)(address >> 16);
        uint16_t addr = (uint16_t)address;

        printf("WRITE $%02X:%04X = %02X", bank, addr, data);
        if (bank == 0x7e || bank == 0x7f) {
            uint32_t wram_offset = ((uint32_t)(bank - 0x7e) << 16) | addr;
            printf("   [WRAM +0x%05lX]", (unsigned long)wram_offset);
        }
        printf("\n");
    }
    fflush(stdout);

    printf("\nCapture complete. Power the SNES off before disconnecting the RP2350B.\n");
    printf("This v0.4 waits indefinitely for a capture, but records one 64-write batch per boot.\n");
    fflush(stdout);
    while (true) sleep_ms(1000);
}

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
#define PIN_HIGH_BASE  18

#define SAMPLE_COUNT   64

static uint32_t low_samples[SAMPLE_COUNT];
static uint32_t high_samples[SAMPLE_COUNT];

static void configure_input(uint pin, bool pull_up) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
    if (pull_up) gpio_pull_up(pin);
}

static uint32_t dma_remaining(uint channel) {
    return dma_channel_hw_addr(channel)->transfer_count & 0x0fffffffu;
}

static uint16_t unpack_low16(uint32_t raw) {
    // SHIFT_RIGHT + IN PINS,16 -> captured bits occupy 31:16.
    return (uint16_t)(raw >> 16);
}

static uint32_t unpack_high17(uint32_t raw) {
    // SHIFT_RIGHT + IN PINS,17 -> captured bits occupy 31:15.
    return (raw >> 15) & 0x1ffffu;
}

static uint32_t reconstruct_address(uint16_t low16, uint32_t high17) {
    uint32_t a0_a7 = (low16 >> 8) & 0xffu;

    // high17 layout, LSB first:
    // bit 0 = GP18 = A8
    // bit 1 = GP19 = A9
    // bit 2 = GP20 = dummy
    // bits 3..16 = GP21..34 = A10..A23
    uint32_t a8_a9 = high17 & 0x3u;
    uint32_t a10_a23 = (high17 >> 3) & 0x3fffu;
    uint32_t a8_a23 = a8_a9 | (a10_a23 << 2);

    return a0_a7 | (a8_a23 << 8);
}

static void arm_dma_channel(int channel, const dma_channel_config *cfg,
                            volatile void *write_addr, const volatile void *read_addr) {
    dma_channel_configure(channel, cfg, write_addr, read_addr, SAMPLE_COUNT, false);
}

int main(void) {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    sleep_ms(250);

    printf("\n=== SNES RP2350B capture v0.6 - NO JUMPERS ===\n");
    printf("PHI2=GP0, /WR=GP1, D0-D7=GP2-9, A0-A9=GP10-19, GP20 skipped, A10-A23=GP21-34.\n");
    printf("Cross-PIO synchronization uses RP2350 IRQ NEXT internally; GP35/GP36 are not used.\n");
    printf("Keep the RP2350B powered before powering the SNES.\n\n");
    fflush(stdout);

    // All SNES bus pins are inputs only. GP20 is a dummy bit inside the high
    // capture span; route it to PIO1 too, but never drive it.
    for (uint pin = 0; pin <= 34; ++pin) {
        configure_input(pin, pin == PIN_WR);
    }

    // ---------- PIO0: timing + D0-D7 + A0-A7 ----------
    PIO pio_lo = pio0;
    const uint sm_lo = 0;
    int base_lo_rc = pio_set_gpio_base(pio_lo, 0);

    for (uint pin = 0; pin <= 17; ++pin) pio_gpio_init(pio_lo, pin);
    pio_sm_set_consecutive_pindirs(pio_lo, sm_lo, 0, 18, false);

    uint off_lo = pio_add_program(pio_lo, &snes_capture_low_program);
    pio_sm_config c_lo = snes_capture_low_program_get_default_config(off_lo);
    sm_config_set_in_pins(&c_lo, PIN_DATA_BASE);
    sm_config_set_jmp_pin(&c_lo, PIN_WR);
    sm_config_set_in_shift(&c_lo, true, false, 32);
    sm_config_set_fifo_join(&c_lo, PIO_FIFO_JOIN_RX);
    int init_lo_rc = pio_sm_init(pio_lo, sm_lo, off_lo, &c_lo);

    // ---------- PIO1: A8-A23 ----------
    PIO pio_hi = pio1;
    const uint sm_hi = 0;
    int base_hi_rc = pio_set_gpio_base(pio_hi, 16);

    for (uint pin = 18; pin <= 34; ++pin) pio_gpio_init(pio_hi, pin);
    pio_sm_set_consecutive_pindirs(pio_hi, sm_hi, 18, 17, false);

    uint off_hi = pio_add_program(pio_hi, &snes_capture_high_program);
    pio_sm_config c_hi = snes_capture_high_program_get_default_config(off_hi);
    sm_config_set_in_pins(&c_hi, PIN_HIGH_BASE);
    sm_config_set_in_shift(&c_hi, true, false, 32);
    sm_config_set_fifo_join(&c_hi, PIO_FIFO_JOIN_RX);
    int init_hi_rc = pio_sm_init(pio_hi, sm_hi, off_hi, &c_hi);

    printf("PIO init: base_lo=%d init_lo=%d | base_hi=%d init_hi=%d\n",
           base_lo_rc, init_lo_rc, base_hi_rc, init_hi_rc);
    if (base_lo_rc || init_lo_rc || base_hi_rc || init_hi_rc) {
        printf("ERROR: PIO configuration failed. Leave the SNES off.\n");
        fflush(stdout);
        while (true) sleep_ms(1000);
    }

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

    printf("READY. Turn the SNES on now.\n");
    printf("This diagnostic captures 64 writes, prints them, then automatically rearms forever.\n\n");
    fflush(stdout);

    uint32_t batch = 0;

    while (true) {
        ++batch;

        pio_sm_set_enabled(pio_lo, sm_lo, false);
        pio_sm_set_enabled(pio_hi, sm_hi, false);
        pio_sm_restart(pio_lo, sm_lo);
        pio_sm_restart(pio_hi, sm_hi);
        pio_sm_clear_fifos(pio_lo, sm_lo);
        pio_sm_clear_fifos(pio_hi, sm_hi);

        // Clear any stale IRQ0 in PIO1 before rearming the pair.
        pio_interrupt_clear(pio_hi, 0);

        arm_dma_channel(dma_lo, &dc_lo, low_samples, &pio_lo->rxf[sm_lo]);
        arm_dma_channel(dma_hi, &dc_hi, high_samples, &pio_hi->rxf[sm_hi]);
        dma_start_channel_mask((1u << dma_lo) | (1u << dma_hi));

        // Start the receiver first so it is already waiting when PIO0 signals it.
        pio_sm_set_enabled(pio_hi, sm_hi, true);
        pio_sm_set_enabled(pio_lo, sm_lo, true);

        uint32_t last_lo = SAMPLE_COUNT + 1;
        uint32_t last_hi = SAMPLE_COUNT + 1;
        uint32_t heartbeat = 0;

        while (true) {
            uint32_t rem_lo = dma_remaining(dma_lo);
            uint32_t rem_hi = dma_remaining(dma_hi);

            if (rem_lo == 0 && rem_hi == 0) break;

            if (rem_lo != last_lo || rem_hi != last_hi) {
                printf("batch %lu progress: LOW %lu/%d HIGH %lu/%d\n",
                       (unsigned long)batch,
                       (unsigned long)(SAMPLE_COUNT - rem_lo), SAMPLE_COUNT,
                       (unsigned long)(SAMPLE_COUNT - rem_hi), SAMPLE_COUNT);
                last_lo = rem_lo;
                last_hi = rem_hi;
            } else if ((heartbeat++ % 4u) == 0u) {
                printf("batch %lu heartbeat: LOW %lu/%d HIGH %lu/%d\n",
                       (unsigned long)batch,
                       (unsigned long)(SAMPLE_COUNT - rem_lo), SAMPLE_COUNT,
                       (unsigned long)(SAMPLE_COUNT - rem_hi), SAMPLE_COUNT);
            }
            fflush(stdout);
            sleep_ms(250);
        }

        pio_sm_set_enabled(pio_lo, sm_lo, false);
        pio_sm_set_enabled(pio_hi, sm_hi, false);

        printf("\n--- batch %lu: %d paired write cycles ---\n",
               (unsigned long)batch, SAMPLE_COUNT);

        for (int i = 0; i < SAMPLE_COUNT; ++i) {
            uint16_t low16 = unpack_low16(low_samples[i]);
            uint32_t high17 = unpack_high17(high_samples[i]);
            uint8_t data = (uint8_t)(low16 & 0xffu);
            uint32_t address = reconstruct_address(low16, high17) & 0xffffffu;
            uint8_t bank = (uint8_t)(address >> 16);
            uint16_t addr = (uint16_t)address;

            printf("WRITE $%02X:%04X = %02X", bank, addr, data);
            if (bank == 0x7e || bank == 0x7f) {
                uint32_t wram_offset = ((uint32_t)(bank - 0x7e) << 16) | addr;
                printf("   [WRAM +0x%05lX]", (unsigned long)wram_offset);
            }
            printf("\n");
        }

        printf("--- end batch %lu; rearming ---\n\n", (unsigned long)batch);
        fflush(stdout);
        sleep_ms(100);
    }
}

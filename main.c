#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "capture_low.pio.h"
#include "capture_high.pio.h"

#define PIN_PHI2       0
#define PIN_WR         1
#define PIN_DATA_BASE  2
#define PIN_ADDR0_BASE 10
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
    // PIO shifts RIGHT: 30 sampled bits occupy raw[31:2].
    return raw >> 2;
}

static uint32_t unpack_high_word(uint32_t raw) {
    // PIO shifts RIGHT: 3 sampled bits occupy raw[31:29].
    return raw >> 29;
}

static uint32_t reconstruct_address(uint32_t low30, uint32_t high3) {
    // low30 bit layout, sourced from GPIO2..31:
    // [7:0]   D0-D7
    // [17:8]  A0-A9
    // [18]    GPIO20 dummy
    // [29:19] A10-A20
    const uint32_t a0_a9   = (low30 >> 8)  & 0x3ffu;
    const uint32_t a10_a20 = (low30 >> 19) & 0x7ffu;
    const uint32_t a21_a23 = high3 & 0x7u;

    return a0_a9 | (a10_a20 << 10) | (a21_a23 << 21);
}

int main(void) {
    stdio_init_all();

    // Give USB CDC time to enumerate.
    sleep_ms(1800);

    printf("\nSNES RP2350B bus capture v0.1\n");
    printf("Board target: SpotPear RP2350B-MINI-A (48 GPIO)\n");
    printf("IMPORTANT: SNES should be OFF while this message appears.\n\n");

    // Everything attached to the SNES is input-only.
    for (uint pin = 0; pin <= 36; ++pin) {
        if (pin == 20) continue; // onboard WS2812; not part of SNES bus
        configure_input(pin, pin == PIN_WR || pin == PIN_WR_HI);
    }

    // PIO0 sees GPIO0..31.
    PIO pio_lo = pio0;
    const uint sm_lo = 0;
    pio_set_gpio_base(pio_lo, 0);
    uint off_lo = pio_add_program(pio_lo, &snes_capture_low_program);
    pio_sm_config c_lo = snes_capture_low_program_get_default_config(off_lo);
    sm_config_set_in_pins(&c_lo, PIN_DATA_BASE);
    sm_config_set_jmp_pin(&c_lo, PIN_WR);
    sm_config_set_in_shift(&c_lo, true, false, 32); // shift right, manual push
    sm_config_set_fifo_join(&c_lo, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio_lo, sm_lo, off_lo, &c_lo);

    // PIO1 sees GPIO16..47. We duplicate /WR onto GP35 and PHI2 onto GP36.
    PIO pio_hi = pio1;
    const uint sm_hi = 0;
    pio_set_gpio_base(pio_hi, 16);
    uint off_hi = pio_add_program(pio_hi, &snes_capture_high_program);
    pio_sm_config c_hi = snes_capture_high_program_get_default_config(off_hi);
    sm_config_set_in_pins(&c_hi, 32);
    sm_config_set_jmp_pin(&c_hi, PIN_WR_HI);
    sm_config_set_in_shift(&c_hi, true, false, 32);
    sm_config_set_fifo_join(&c_hi, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio_hi, sm_hi, off_hi, &c_hi);

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

    dma_channel_configure(
        dma_lo, &dc_lo,
        low_samples,
        &pio_lo->rxf[sm_lo],
        SAMPLE_COUNT,
        false
    );

    dma_channel_configure(
        dma_hi, &dc_hi,
        high_samples,
        &pio_hi->rxf[sm_hi],
        SAMPLE_COUNT,
        false
    );

    printf("READY. Now turn on the SNES. Waiting for %d write cycles...\n", SAMPLE_COUNT);
    fflush(stdout);

    // Start DMA first, then state machines. Since SNES is OFF until READY,
    // both PIOs will synchronise to the same first PHI2 cycle.
    dma_start_channel_mask((1u << dma_lo) | (1u << dma_hi));
    pio_sm_set_enabled(pio_lo, sm_lo, true);
    pio_sm_set_enabled(pio_hi, sm_hi, true);

    dma_channel_wait_for_finish_blocking(dma_lo);
    dma_channel_wait_for_finish_blocking(dma_hi);

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

        // Mark direct accesses to the two full WRAM banks.
        if (bank == 0x7e || bank == 0x7f) {
            uint32_t wram_offset = ((uint32_t)(bank - 0x7e) << 16) | addr;
            printf("   [WRAM +0x%05lX]", (unsigned long)wram_offset);
        }
        printf("\n");
    }

    printf("\nDone. Power the SNES off before disconnecting the RP2350B.\n");
    printf("Reset the RP2350B to capture another batch.\n");

    while (true) {
        tight_loop_contents();
    }
}

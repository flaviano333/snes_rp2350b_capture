#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "capture_atomic_read.pio.h"
#include "capture_read_trigger.pio.h"

#define PIN_PHI2       0
#define PIN_DATA_BASE  2
#define PIN_RD         35
#define SAMPLE_COUNT   256
#define CMD_BUF_SIZE   128
#define TRACE_MAX      4096u

static uint32_t samples[SAMPLE_COUNT];
static char cmd_buf[CMD_BUF_SIZE];
static size_t cmd_len = 0;
static uint32_t trace_remaining = 0;
static uint64_t total_read_cycles = 0;
static uint64_t total_rom_candidates = 0;
static uint64_t total_batches = 0;

// Logical A0..A19 -> measured physical GPIO.
// These are all inside GP10..GP31, so one PIO0 word contains them atomically.
static const uint8_t ADDRESS_GPIO_A0_A19[20] = {
    12, 11, 10, 13, 14, 16, 15, 17,  // A0..A7
    19, 21, 22, 23, 24, 25, 26, 27,  // A8..A15
    28, 29, 30, 31                    // A16..A19
};

static void configure_input(uint pin, bool pull_up) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
    if (pull_up) gpio_pull_up(pin);
}

static uint32_t dma_remaining(uint channel) {
    return dma_channel_hw_addr(channel)->transfer_count & 0x0fffffffu;
}

static void arm_dma_channel(int channel, const dma_channel_config *cfg,
                            volatile void *write_addr, const volatile void *read_addr) {
    dma_channel_configure(channel, cfg, write_addr, read_addr, SAMPLE_COUNT, false);
}

static inline uint32_t pins30_from_raw(uint32_t raw) {
    // SHIFT_RIGHT + IN PINS,30 places the 30 sampled bits in raw[31:2].
    // After >>2: bit0=GP2, bit1=GP3, ... bit29=GP31.
    return raw >> 2;
}

static inline uint32_t physical_gpio_bit(uint32_t pins30, uint gpio) {
    if (gpio < 2 || gpio > 31) return 0;
    return (pins30 >> (gpio - 2u)) & 1u;
}

static uint32_t reconstruct_a0_a19(uint32_t pins30) {
    uint32_t a = 0;
    for (uint bit = 0; bit < 20; ++bit) {
        a |= physical_gpio_bit(pins30, ADDRESS_GPIO_A0_A19[bit]) << bit;
    }
    return a;
}

static void print_help(void) {
    printf("Commands:\n");
    printf("  INFO\n");
    printf("  ATOMICREAD <1-4096>   print that many candidate LoROM reads\n");
    printf("  OFF                   stop printing\n");
    printf("  PING\n");
}

static void execute_command(char *line) {
    while (*line == ' ' || *line == '\t') ++line;
    if (!*line) return;

    char *rest = line;
    while (*rest && *rest != ' ' && *rest != '\t') ++rest;
    if (*rest) {
        *rest++ = '\0';
        while (*rest == ' ' || *rest == '\t') ++rest;
    } else {
        rest = NULL;
    }

    for (char *p = line; *p; ++p) {
        if (*p >= 'a' && *p <= 'z') *p -= ('a' - 'A');
    }

    if (!strcmp(line, "HELP") || !strcmp(line, "?")) {
        print_help();
    } else if (!strcmp(line, "PING")) {
        printf("PONG\n");
    } else if (!strcmp(line, "INFO")) {
        printf("INFO batches=%llu read_cycles=%llu rom_candidates=%llu trace_remaining=%lu\n",
               (unsigned long long)total_batches,
               (unsigned long long)total_read_cycles,
               (unsigned long long)total_rom_candidates,
               (unsigned long)trace_remaining);
    } else if (!strcmp(line, "OFF")) {
        trace_remaining = 0;
        printf("OK OFF\n");
    } else if (!strcmp(line, "ATOMICREAD")) {
        if (!rest || !*rest) {
            printf("ATOMICREAD remaining=%lu\n", (unsigned long)trace_remaining);
        } else {
            char *end = NULL;
            unsigned long n = strtoul(rest, &end, 10);
            if (end == rest || *end != '\0' || n < 1 || n > TRACE_MAX) {
                printf("ERR ATOMICREAD count must be decimal 1-4096\n");
            } else {
                trace_remaining = (uint32_t)n;
                printf("OK ATOMICREAD %lu\n", n);
            }
        }
    } else {
        printf("ERR unknown command '%s'\n", line);
    }
    fflush(stdout);
}

static void poll_serial_commands(void) {
    while (true) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) break;
        if (c == '\r' || c == '\n') {
            if (cmd_len) {
                cmd_buf[cmd_len] = '\0';
                execute_command(cmd_buf);
                cmd_len = 0;
            }
            continue;
        }
        if ((c == 8 || c == 127) && cmd_len) {
            --cmd_len;
            continue;
        }
        if (c >= 32 && c <= 126 && cmd_len + 1 < CMD_BUF_SIZE) {
            cmd_buf[cmd_len++] = (char)c;
        }
    }
}

static void process_batch(void) {
    ++total_batches;
    for (uint i = 0; i < SAMPLE_COUNT; ++i) {
        uint32_t pins30 = pins30_from_raw(samples[i]);
        uint8_t data = (uint8_t)(pins30 & 0xffu);
        uint32_t a = reconstruct_a0_a19(pins30);
        ++total_read_cycles;

        // For ordinary LoROM, upper-half CPU reads (A15=1) are ROM candidates.
        // Tom & Jerry USA is 512 KiB, so the physical ROM offset uses A0..A14
        // and A16..A19. A20+ are mirrors and are intentionally not needed here.
        if (((a >> 15) & 1u) == 0) continue;

        uint32_t bank_low4 = (a >> 16) & 0x0fu;
        uint32_t offset = (bank_low4 << 15) | (a & 0x7fffu);
        ++total_rom_candidates;

        if (trace_remaining) {
            printf("ATOM P=%08lX D=%02X O=%05lX A20=%05lX\n",
                   (unsigned long)(pins30 & 0x3fffffffu),
                   data,
                   (unsigned long)(offset & 0x7ffffu),
                   (unsigned long)(a & 0xfffffu));
            --trace_remaining;
            if (trace_remaining == 0) {
                printf("ATOMIC DONE\n");
            }
        }
    }
    if (stdio_usb_connected()) fflush(stdout);
}

int main(void) {
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(50);
    sleep_ms(250);

    printf("\n=== SNES RP2350B v1.5 SINGLE-PIO ROM DIAGNOSTIC ===\n");
    printf("Diagnostic-only firmware: atomic GP2..GP31 capture on CPU reads.\n");
    printf("PHI2=GP0 /RD=GP35 D0-D7=GP2-9. No wiring change required.\n");
    printf("For the 512 KiB Tom & Jerry LoROM, A20-A23 are not needed to compute physical ROM offset.\n");
    printf("Keep RP2350B powered before SNES. Type HELP for commands.\n\n");
    fflush(stdout);

    for (uint pin = 0; pin <= 35; ++pin) {
        configure_input(pin, pin == PIN_RD);
    }

    PIO pio_cap = pio0;
    const uint sm_cap = 0;
    int base_cap_rc = pio_set_gpio_base(pio_cap, 0);
    for (uint pin = 0; pin <= 31; ++pin) pio_gpio_init(pio_cap, pin);
    pio_sm_set_consecutive_pindirs(pio_cap, sm_cap, 0, 32, false);

    uint off_cap = pio_add_program(pio_cap, &snes_capture_atomic_read_program);
    pio_sm_config c_cap = snes_capture_atomic_read_program_get_default_config(off_cap);
    sm_config_set_in_pins(&c_cap, PIN_DATA_BASE);
    sm_config_set_in_shift(&c_cap, true, false, 32);
    sm_config_set_fifo_join(&c_cap, PIO_FIFO_JOIN_RX);
    int init_cap_rc = pio_sm_init(pio_cap, sm_cap, off_cap, &c_cap);

    PIO pio_rd = pio2;
    const uint sm_rd = 0;
    int base_rd_rc = pio_set_gpio_base(pio_rd, 16);
    pio_gpio_init(pio_rd, PIN_RD);
    pio_sm_set_consecutive_pindirs(pio_rd, sm_rd, PIN_RD, 1, false);
    uint off_rd = pio_add_program(pio_rd, &snes_read_trigger_program);
    pio_sm_config c_rd = snes_read_trigger_program_get_default_config(off_rd);
    int init_rd_rc = pio_sm_init(pio_rd, sm_rd, off_rd, &c_rd);

    printf("PIO init: base_cap=%d init_cap=%d base_rd=%d init_rd=%d\n",
           base_cap_rc, init_cap_rc, base_rd_rc, init_rd_rc);
    if (base_cap_rc || init_cap_rc || base_rd_rc || init_rd_rc) {
        printf("ERROR PIO init failed; leave SNES off.\n");
        while (true) { poll_serial_commands(); sleep_ms(10); }
    }

    int dma_cap = dma_claim_unused_channel(true);
    dma_channel_config dc = dma_channel_get_default_config(dma_cap);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(pio_cap, sm_cap, false));

    pio_sm_set_enabled(pio_rd, sm_rd, false);
    pio_sm_set_enabled(pio_cap, sm_cap, false);
    pio_sm_restart(pio_rd, sm_rd);
    pio_sm_restart(pio_cap, sm_cap);
    pio_sm_clear_fifos(pio_cap, sm_cap);
    pio_interrupt_clear(pio_cap, 1);
    arm_dma_channel(dma_cap, &dc, samples, &pio_cap->rxf[sm_cap]);
    dma_start_channel_mask(1u << dma_cap);
    pio_sm_set_enabled(pio_cap, sm_cap, true);
    pio_sm_set_enabled(pio_rd, sm_rd, true);

    printf("READY. Run atomic_rom_verify.py from the PC.\n\n");
    fflush(stdout);

    while (true) {
        if (dma_remaining(dma_cap) == 0) {
            pio_sm_set_enabled(pio_rd, sm_rd, false);
            pio_sm_set_enabled(pio_cap, sm_cap, false);
            process_batch();

            pio_sm_restart(pio_rd, sm_rd);
            pio_sm_restart(pio_cap, sm_cap);
            pio_sm_clear_fifos(pio_cap, sm_cap);
            pio_interrupt_clear(pio_cap, 1);
            arm_dma_channel(dma_cap, &dc, samples, &pio_cap->rxf[sm_cap]);
            dma_start_channel_mask(1u << dma_cap);
            pio_sm_set_enabled(pio_cap, sm_cap, true);
            pio_sm_set_enabled(pio_rd, sm_rd, true);
        }

        poll_serial_commands();
        tight_loop_contents();
    }
}

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
#include "hardware/clocks.h"

#include "capture_timing_sweep.pio.h"
#include "capture_read_trigger.pio.h"

#define PIN_RD          35
#define SLOT_COUNT      16u
#define EVENT_COUNT     16u
#define WORD_COUNT      (SLOT_COUNT * EVENT_COUNT)   // 256 DMA words
#define CMD_BUF_SIZE    128
#define TRACE_MAX       4096u

static uint32_t words[WORD_COUNT];
static char cmd_buf[CMD_BUF_SIZE];
static size_t cmd_len = 0;

static uint32_t trace_events_remaining = 0;
static uint64_t total_events = 0;
static uint64_t total_batches = 0;

static void configure_input(uint pin, bool pull_up) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
    if (pull_up) gpio_pull_up(pin);
}

static uint32_t dma_remaining(uint channel) {
    return dma_channel_hw_addr(channel)->transfer_count & 0x0fffffffu;
}

static void arm_dma_channel(int channel, const dma_channel_config *cfg) {
    dma_channel_configure(channel, cfg, words, &pio0->rxf[0], WORD_COUNT, false);
}

static void print_help(void) {
    printf("Commands:\n");
    printf("  INFO\n");
    printf("  SWEEPREAD <1-4096>   print that many /RD events, each with 8 timing slots\n");
    printf("  OFF                  stop printing\n");
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
        uint32_t sys_hz = clock_get_hz(clk_sys);
        printf("INFO batches=%llu events=%llu trace_events=%lu sys_hz=%lu slots=%u\n",
               (unsigned long long)total_batches,
               (unsigned long long)total_events,
               (unsigned long)trace_events_remaining,
               (unsigned long)sys_hz,
               (unsigned)SLOT_COUNT);
    } else if (!strcmp(line, "OFF")) {
        trace_events_remaining = 0;
        printf("OK OFF\n");
    } else if (!strcmp(line, "SWEEPREAD")) {
        if (!rest || !*rest) {
            printf("SWEEPREAD remaining=%lu\n",
                   (unsigned long)trace_events_remaining);
        } else {
            char *end = NULL;
            unsigned long n = strtoul(rest, &end, 10);
            if (end == rest || *end != '\0' || n < 1 || n > TRACE_MAX) {
                printf("ERR SWEEPREAD count must be decimal 1-4096\n");
            } else {
                trace_events_remaining = (uint32_t)n;
                printf("OK SWEEPREAD %lu\n", n);
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

    for (uint e = 0; e < EVENT_COUNT; ++e) {
        ++total_events;

        if (trace_events_remaining) {
            const uint32_t *s = &words[e * SLOT_COUNT];
            printf("SWEEP");
            for (uint slot = 0; slot < SLOT_COUNT; ++slot) {
                printf(" %08lX", (unsigned long)s[slot]);
            }
            printf("\n");

            --trace_events_remaining;
            if (trace_events_remaining == 0) {
                printf("SWEEP DONE\n");
            }
        }
    }

    if (stdio_usb_connected()) fflush(stdout);
}

int main(void) {
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(50);
    sleep_ms(250);

    printf("\n=== SNES RP2350B v1.7 WIDE TIMING SWEEP ===\n");
    printf("Diagnostic-only firmware. No wiring changes required.\n");
    printf("Each /RD event produces 16 atomic GP0..GP31 snapshots.\n");
    printf("Capture SM clkdiv=2; consecutive slots are ~53 ns apart at 150 MHz.\n");
    printf("GP0=PHI2 GP1=/WR GP2-9=D0-7; GP10-31 include A0-A19 wiring.\n");
    printf("Keep RP2350B powered before SNES. Type HELP for commands.\n\n");
    fflush(stdout);

    // Inputs only. GP0..39 on RP2350B are the 5V-tolerant bank when IOVDD=3.3V.
    for (uint pin = 0; pin <= 35; ++pin) {
        configure_input(pin, pin == PIN_RD);
    }

    // PIO0 captures one 32-bit word containing GP0..GP31.
    PIO pio_cap = pio0;
    const uint sm_cap = 0;
    int base_cap_rc = pio_set_gpio_base(pio_cap, 0);

    for (uint pin = 0; pin <= 31; ++pin) pio_gpio_init(pio_cap, pin);
    pio_sm_set_consecutive_pindirs(pio_cap, sm_cap, 0, 32, false);

    uint off_cap = pio_add_program(pio_cap, &snes_wide_timing_sweep_program);
    pio_sm_config c_cap = snes_wide_timing_sweep_program_get_default_config(off_cap);
    sm_config_set_in_pins(&c_cap, 0);
    sm_config_set_in_shift(&c_cap, true, true, 32); // autopush each IN PINS,32
    sm_config_set_clkdiv(&c_cap, 2.0f); // wide sweep: ~53.3 ns between slots at 150 MHz sysclk
    sm_config_set_fifo_join(&c_cap, PIO_FIFO_JOIN_RX);
    int init_cap_rc = pio_sm_init(pio_cap, sm_cap, off_cap, &c_cap);

    // PIO2 watches /RD on GP35 and raises cross-PIO IRQ1 in PIO0.
    PIO pio_rd = pio2;
    const uint sm_rd = 0;
    int base_rd_rc = pio_set_gpio_base(pio_rd, 16);
    pio_gpio_init(pio_rd, PIN_RD);
    pio_sm_set_consecutive_pindirs(pio_rd, sm_rd, PIN_RD, 1, false);

    uint off_rd = pio_add_program(pio_rd, &snes_read_trigger_program);
    pio_sm_config c_rd = snes_read_trigger_program_get_default_config(off_rd);
    int init_rd_rc = pio_sm_init(pio_rd, sm_rd, off_rd, &c_rd);

    printf("PIO init: base_cap=%d init_cap=%d base_rd=%d init_rd=%d sys_hz=%lu\n",
           base_cap_rc, init_cap_rc, base_rd_rc, init_rd_rc,
           (unsigned long)clock_get_hz(clk_sys));

    if (base_cap_rc || init_cap_rc || base_rd_rc || init_rd_rc) {
        printf("ERROR PIO init failed; leave SNES off.\n");
        while (true) {
            poll_serial_commands();
            sleep_ms(10);
        }
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

    arm_dma_channel(dma_cap, &dc);
    dma_start_channel_mask(1u << dma_cap);

    pio_sm_set_enabled(pio_cap, sm_cap, true);
    pio_sm_set_enabled(pio_rd, sm_rd, true);

    printf("READY. Run wide_timing_sweep_verify.py from the PC.\n\n");
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

            arm_dma_channel(dma_cap, &dc);
            dma_start_channel_mask(1u << dma_cap);

            pio_sm_set_enabled(pio_cap, sm_cap, true);
            pio_sm_set_enabled(pio_rd, sm_rd, true);
        }

        poll_serial_commands();
        tight_loop_contents();
    }
}

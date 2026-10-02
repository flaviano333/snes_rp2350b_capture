#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "capture_low.pio.h"
#include "capture_high.pio.h"
#include "capture_read_trigger.pio.h"
#include "capture_read_low.pio.h"
#include "capture_read_high.pio.h"

#define PIN_PHI2       0
#define PIN_WR         1
#define PIN_DATA_BASE  2
#define PIN_HIGH_BASE  18
#define PIN_RD         35

#define SAMPLE_COUNT   256
#define WRAM_SIZE      (128u * 1024u)
#define VALID_SIZE     (WRAM_SIZE / 8u)
#define CMD_BUF_SIZE   128

static uint32_t low_samples[SAMPLE_COUNT];
static uint32_t high_samples[SAMPLE_COUNT];
static uint32_t read_low_samples[SAMPLE_COUNT];
static uint32_t read_high_samples[SAMPLE_COUNT];

// Passive WRAM mirror. A byte is marked valid only after this adapter
// has actually observed a write to it.
static uint8_t wram[WRAM_SIZE];
static uint8_t wram_valid[VALID_SIZE];
static uint32_t known_wram_bytes = 0;

static uint64_t total_bus_writes = 0;
static uint64_t total_wram_writes = 0;
static uint64_t total_direct_writes = 0;
static uint64_t total_mirror_writes = 0;
static uint64_t total_bus_reads = 0;
static uint64_t total_wram_reads = 0;
static uint64_t total_direct_reads = 0;
static uint64_t total_mirror_reads = 0;
static uint32_t completed_batches = 0;

static char cmd_buf[CMD_BUF_SIZE];
static size_t cmd_len = 0;

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
    return (uint16_t)(raw >> 16);
}

static uint32_t unpack_high17(uint32_t raw) {
    return (raw >> 15) & 0x1ffffu;
}

static uint32_t reconstruct_address(uint16_t low16, uint32_t high17) {
    uint32_t a0_a7 = (low16 >> 8) & 0xffu;
    uint32_t a8_a9 = high17 & 0x3u;
    uint32_t a10_a23 = (high17 >> 3) & 0x3fffu;
    uint32_t a8_a23 = a8_a9 | (a10_a23 << 2);
    return a0_a7 | (a8_a23 << 8);
}

static void arm_dma_channel(int channel, const dma_channel_config *cfg,
                            volatile void *write_addr, const volatile void *read_addr) {
    dma_channel_configure(channel, cfg, write_addr, read_addr, SAMPLE_COUNT, false);
}

// Translate a 24-bit SNES A-bus address to the canonical WRAM offset.
static bool map_address_to_wram(uint32_t address, uint32_t *offset, bool *direct) {
    uint8_t bank = (uint8_t)(address >> 16);
    uint16_t addr = (uint16_t)address;

    if (bank == 0x7e || bank == 0x7f) {
        *offset = ((uint32_t)(bank - 0x7e) << 16) | addr;
        *direct = true;
        return true;
    }

    bool mirror_bank = (bank <= 0x3f) || (bank >= 0x80 && bank <= 0xbf);
    if (mirror_bank && addr <= 0x1fffu) {
        *offset = addr;
        *direct = false;
        return true;
    }

    return false;
}

static bool wram_byte_known(uint32_t offset) {
    return (wram_valid[offset >> 3] >> (offset & 7u)) & 1u;
}

static void mark_wram_known(uint32_t offset) {
    uint8_t mask = (uint8_t)(1u << (offset & 7u));
    uint8_t *slot = &wram_valid[offset >> 3];
    if ((*slot & mask) == 0) {
        *slot |= mask;
        ++known_wram_bytes;
    }
}

static bool parse_hex_u32(const char *s, uint32_t *out) {
    if (!s || !*s) return false;
    while (*s == ' ' || *s == '\t') ++s;
    if (*s == '$') ++s;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) s += 2;
    if (!isxdigit((unsigned char)*s)) return false;

    char *end = NULL;
    unsigned long v = strtoul(s, &end, 16);
    if (end == s) return false;
    while (*end == ' ' || *end == '\t') ++end;
    if (*end != '\0') return false;
    *out = (uint32_t)v;
    return true;
}

static void strtoupper_inplace(char *s) {
    while (*s) {
        *s = (char)toupper((unsigned char)*s);
        ++s;
    }
}

static void print_help(void) {
    printf("Commands:\n");
    printf("  HELP                    show this help\n");
    printf("  INFO                    capture/mirror statistics\n");
    printf("  READ <offset>           read canonical WRAM offset 00000-1FFFF\n");
    printf("  READSNES <BB:AAAA>      read a SNES address if it maps to WRAM\n");
    printf("  HEX [offset] [length]   human-readable WRAM dump (max 4096 bytes)\n");
    printf("  DUMPBIN                 binary snapshot: 128 KiB WRAM + 16 KiB valid bitmap\n");
    printf("  RBIN <off> <len>        raw WRAM range for PC bridge (hex; max 1000 bytes)\n");
    printf("  CLEAR                   clear the software mirror/known bitmap\n");
    printf("  PING                    reply PONG\n");
    printf("Mirror source: observed WRAM reads + writes.\n");
}

static void print_info(void) {
    printf("INFO batches=%lu bus_writes=%llu wram_writes=%llu direct_w=%llu mirror_w=%llu "
           "bus_reads=%llu wram_reads=%llu direct_r=%llu mirror_r=%llu known=%lu/%u (%.2f%%)\n",
           (unsigned long)completed_batches,
           (unsigned long long)total_bus_writes,
           (unsigned long long)total_wram_writes,
           (unsigned long long)total_direct_writes,
           (unsigned long long)total_mirror_writes,
           (unsigned long long)total_bus_reads,
           (unsigned long long)total_wram_reads,
           (unsigned long long)total_direct_reads,
           (unsigned long long)total_mirror_reads,
           (unsigned long)known_wram_bytes,
           (unsigned)WRAM_SIZE,
           (double)known_wram_bytes * 100.0 / (double)WRAM_SIZE);
}

static void command_read_offset(const char *arg) {
    uint32_t off;
    if (!parse_hex_u32(arg, &off) || off >= WRAM_SIZE) {
        printf("ERR READ expects hexadecimal offset 00000-1FFFF\n");
        return;
    }

    if (wram_byte_known(off)) {
        printf("READ %05lX = %02X KNOWN\n", (unsigned long)off, wram[off]);
    } else {
        printf("READ %05lX = ?? UNKNOWN\n", (unsigned long)off);
    }
}

static void command_read_snes(const char *arg) {
    if (!arg) {
        printf("ERR READSNES expects BB:AAAA, e.g. 00:13FB or 7E:1234\n");
        return;
    }

    char temp[32];
    strncpy(temp, arg, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    char *colon = strchr(temp, ':');
    if (!colon) {
        printf("ERR READSNES expects BB:AAAA\n");
        return;
    }
    *colon = '\0';

    uint32_t bank, addr;
    if (!parse_hex_u32(temp, &bank) || !parse_hex_u32(colon + 1, &addr) || bank > 0xffu || addr > 0xffffu) {
        printf("ERR READSNES expects BB:AAAA in hex\n");
        return;
    }

    uint32_t full = (bank << 16) | addr;
    uint32_t off;
    bool direct;
    if (!map_address_to_wram(full, &off, &direct)) {
        printf("READSNES $%02lX:%04lX = NOT_WRAM\n", (unsigned long)bank, (unsigned long)addr);
        return;
    }

    if (wram_byte_known(off)) {
        printf("READSNES $%02lX:%04lX -> WRAM %05lX = %02X KNOWN %s\n",
               (unsigned long)bank, (unsigned long)addr, (unsigned long)off,
               wram[off], direct ? "DIRECT" : "MIRROR");
    } else {
        printf("READSNES $%02lX:%04lX -> WRAM %05lX = ?? UNKNOWN %s\n",
               (unsigned long)bank, (unsigned long)addr, (unsigned long)off,
               direct ? "DIRECT" : "MIRROR");
    }
}

static void command_hex(char *args) {
    uint32_t start = 0;
    uint32_t len = 256;

    if (args && *args) {
        char *a = strtok(args, " \t");
        char *b = strtok(NULL, " \t");
        if (a && !parse_hex_u32(a, &start)) {
            printf("ERR HEX offset must be hexadecimal\n");
            return;
        }
        if (b && !parse_hex_u32(b, &len)) {
            printf("ERR HEX length must be hexadecimal\n");
            return;
        }
    }

    if (start >= WRAM_SIZE) {
        printf("ERR HEX offset out of range\n");
        return;
    }
    if (len == 0) len = 1;
    if (len > 4096) len = 4096;
    if (start + len > WRAM_SIZE) len = WRAM_SIZE - start;

    printf("HEX %05lX %lX\n", (unsigned long)start, (unsigned long)len);
    for (uint32_t row = 0; row < len; row += 16) {
        uint32_t off = start + row;
        printf("%05lX:", (unsigned long)off);
        uint32_t row_len = (len - row > 16) ? 16 : (len - row);
        for (uint32_t i = 0; i < row_len; ++i) {
            uint32_t p = off + i;
            if (wram_byte_known(p)) printf(" %02X", wram[p]);
            else printf(" ??");
        }
        printf("\n");
    }
    printf("ENDHEX\n");
}

static void command_dumpbin(void) {
    // Machine-readable framing. Raw payload is sent with CR/LF translation disabled.
    // Payload = 131072 WRAM bytes + 16384 validity-bitmap bytes.
    printf("BIN1 %u %u %lu %lu\n",
           (unsigned)WRAM_SIZE,
           (unsigned)VALID_SIZE,
           (unsigned long)known_wram_bytes,
           (unsigned long)completed_batches);
    fflush(stdout);

    const size_t chunk = 1024;
    for (size_t off = 0; off < WRAM_SIZE; off += chunk) {
        size_t n = WRAM_SIZE - off;
        if (n > chunk) n = chunk;
        stdio_put_string((const char *)&wram[off], (int)n, false, false);
    }
    for (size_t off = 0; off < VALID_SIZE; off += chunk) {
        size_t n = VALID_SIZE - off;
        if (n > chunk) n = chunk;
        stdio_put_string((const char *)&wram_valid[off], (int)n, false, false);
    }
    stdio_flush();
    printf("\nEND BIN1\n");
    fflush(stdout);
}


static void command_rbin(char *args) {
    if (!args || !*args) {
        printf("ERR RBIN expects <offset> <length> in hex\n");
        return;
    }

    char temp[64];
    strncpy(temp, args, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    char *a = strtok(temp, " \t");
    char *b = strtok(NULL, " \t");
    uint32_t start = 0, len = 0;
    if (!a || !b || !parse_hex_u32(a, &start) || !parse_hex_u32(b, &len)) {
        printf("ERR RBIN expects hexadecimal <offset> <length>\n");
        return;
    }
    if (start >= WRAM_SIZE || len == 0 || len > 0x1000u || start + len > WRAM_SIZE) {
        printf("ERR RBIN range out of bounds (WRAM 00000-1FFFF, len 1-1000)\n");
        return;
    }

    uint32_t unknown = 0;
    for (uint32_t i = 0; i < len; ++i) {
        if (!wram_byte_known(start + i)) ++unknown;
    }

    printf("RBIN1 %lu %lu\n", (unsigned long)len, (unsigned long)unknown);
    fflush(stdout);
    stdio_put_string((const char *)&wram[start], (int)len, false, false);
    stdio_flush();
    printf("\nEND RBIN1\n");
    fflush(stdout);
}

static void execute_command(char *line) {
    while (*line == ' ' || *line == '\t') ++line;
    if (!*line) return;

    char *cmd = line;
    char *rest = line;
    while (*rest && *rest != ' ' && *rest != '\t') ++rest;
    if (*rest) {
        *rest++ = '\0';
        while (*rest == ' ' || *rest == '\t') ++rest;
        if (!*rest) rest = NULL;
    } else {
        rest = NULL;
    }
    strtoupper_inplace(cmd);

    if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) {
        print_help();
    } else if (!strcmp(cmd, "INFO")) {
        print_info();
    } else if (!strcmp(cmd, "PING")) {
        printf("PONG\n");
    } else if (!strcmp(cmd, "READ")) {
        command_read_offset(rest);
    } else if (!strcmp(cmd, "READSNES")) {
        command_read_snes(rest);
    } else if (!strcmp(cmd, "HEX")) {
        command_hex(rest);
    } else if (!strcmp(cmd, "DUMPBIN")) {
        command_dumpbin();
    } else if (!strcmp(cmd, "RBIN")) {
        command_rbin(rest);
    } else if (!strcmp(cmd, "CLEAR")) {
        memset(wram, 0, sizeof(wram));
        memset(wram_valid, 0, sizeof(wram_valid));
        known_wram_bytes = 0;
        printf("OK mirror cleared\n");
    } else {
        printf("ERR unknown command '%s' (type HELP)\n", cmd);
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

        if (c == 8 || c == 127) {
            if (cmd_len) --cmd_len;
            continue;
        }

        if (cmd_len + 1 < CMD_BUF_SIZE && c >= 32 && c <= 126) {
            cmd_buf[cmd_len++] = (char)c;
        }
    }
}

static void process_write_batch(void) {
    ++completed_batches;

    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        uint16_t low16 = unpack_low16(low_samples[i]);
        uint32_t high17 = unpack_high17(high_samples[i]);
        uint8_t data = (uint8_t)(low16 & 0xffu);
        uint32_t address = reconstruct_address(low16, high17) & 0xffffffu;

        ++total_bus_writes;

        uint32_t wram_offset;
        bool direct;
        if (!map_address_to_wram(address, &wram_offset, &direct)) continue;

        ++total_wram_writes;
        if (direct) ++total_direct_writes;
        else ++total_mirror_writes;

        wram[wram_offset] = data;
        mark_wram_known(wram_offset);
    }
}

static void process_read_batch(void) {
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        uint16_t low16 = unpack_low16(read_low_samples[i]);
        uint32_t high17 = unpack_high17(read_high_samples[i]);
        uint8_t data = (uint8_t)(low16 & 0xffu);
        uint32_t address = reconstruct_address(low16, high17) & 0xffffffu;

        ++total_bus_reads;

        uint32_t wram_offset;
        bool direct;
        if (!map_address_to_wram(address, &wram_offset, &direct)) continue;

        ++total_wram_reads;
        if (direct) ++total_direct_reads;
        else ++total_mirror_reads;

        // A read gives us the actual byte currently present in WRAM, so it can
        // initialise an UNKNOWN byte and repair a stale value if a write was missed.
        wram[wram_offset] = data;
        mark_wram_known(wram_offset);
    }
}

int main(void) {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    sleep_ms(250);

    memset(wram, 0, sizeof(wram));
    memset(wram_valid, 0, sizeof(wram_valid));

    printf("\n=== SNES RP2350B RA Bridge Firmware v1.0 RW ===\n");
    printf("Passive A-bus read+write monitor; /RD=GP35; no PHI2/WR duplicate jumpers.\n");
    printf("PHI2=GP0 /WR=GP1 /RD=GP35 D0-D7=GP2-9 A0-A9=GP10-19 GP20 skipped A10-A23=GP21-34.\n");
    printf("The 128 KiB mirror learns from observed WRAM reads and writes; unseen bytes remain UNKNOWN.\n");
    printf("Keep the RP2350B powered before powering the SNES.\n");
    printf("Type HELP for commands.\n\n");
    fflush(stdout);

    for (uint pin = 0; pin <= 35; ++pin) {
        configure_input(pin, pin == PIN_WR || pin == PIN_RD);
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

    // ---------- PIO2: /RD detector ----------
    PIO pio_rd = pio2;
    const uint sm_rd_trigger = 0;
    int base_rd_rc = pio_set_gpio_base(pio_rd, 16);
    pio_gpio_init(pio_rd, PIN_RD);
    pio_sm_set_consecutive_pindirs(pio_rd, sm_rd_trigger, PIN_RD, 1, false);
    uint off_rd_trigger = pio_add_program(pio_rd, &snes_read_trigger_program);
    pio_sm_config c_rd_trigger = snes_read_trigger_program_get_default_config(off_rd_trigger);
    int init_rd_trigger_rc = pio_sm_init(pio_rd, sm_rd_trigger, off_rd_trigger, &c_rd_trigger);

    // ---------- PIO0 SM1: D0-D7 + A0-A7 on reads ----------
    const uint sm_read_lo = 1;
    uint off_read_lo = pio_add_program(pio_lo, &snes_capture_read_low_program);
    pio_sm_config c_read_lo = snes_capture_read_low_program_get_default_config(off_read_lo);
    sm_config_set_in_pins(&c_read_lo, PIN_DATA_BASE);
    sm_config_set_in_shift(&c_read_lo, true, false, 32);
    sm_config_set_fifo_join(&c_read_lo, PIO_FIFO_JOIN_RX);
    int init_read_lo_rc = pio_sm_init(pio_lo, sm_read_lo, off_read_lo, &c_read_lo);

    // ---------- PIO1 SM1: A8-A23 on reads ----------
    const uint sm_read_hi = 1;
    uint off_read_hi = pio_add_program(pio_hi, &snes_capture_read_high_program);
    pio_sm_config c_read_hi = snes_capture_read_high_program_get_default_config(off_read_hi);
    sm_config_set_in_pins(&c_read_hi, PIN_HIGH_BASE);
    sm_config_set_in_shift(&c_read_hi, true, false, 32);
    sm_config_set_fifo_join(&c_read_hi, PIO_FIFO_JOIN_RX);
    int init_read_hi_rc = pio_sm_init(pio_hi, sm_read_hi, off_read_hi, &c_read_hi);

    printf("PIO init: base_lo=%d init_lo=%d | base_hi=%d init_hi=%d | base_rd=%d trig=%d read_lo=%d read_hi=%d\n",
           base_lo_rc, init_lo_rc, base_hi_rc, init_hi_rc,
           base_rd_rc, init_rd_trigger_rc, init_read_lo_rc, init_read_hi_rc);
    if (base_lo_rc || init_lo_rc || base_hi_rc || init_hi_rc ||
        base_rd_rc || init_rd_trigger_rc || init_read_lo_rc || init_read_hi_rc) {
        printf("ERROR: PIO configuration failed. Leave the SNES off.\n");
        fflush(stdout);
        while (true) {
            poll_serial_commands();
            sleep_ms(10);
        }
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

    int dma_read_lo = dma_claim_unused_channel(true);
    int dma_read_hi = dma_claim_unused_channel(true);

    dma_channel_config dc_read_lo = dma_channel_get_default_config(dma_read_lo);
    channel_config_set_transfer_data_size(&dc_read_lo, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_read_lo, false);
    channel_config_set_write_increment(&dc_read_lo, true);
    channel_config_set_dreq(&dc_read_lo, pio_get_dreq(pio_lo, sm_read_lo, false));

    dma_channel_config dc_read_hi = dma_channel_get_default_config(dma_read_hi);
    channel_config_set_transfer_data_size(&dc_read_hi, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_read_hi, false);
    channel_config_set_write_increment(&dc_read_hi, true);
    channel_config_set_dreq(&dc_read_hi, pio_get_dreq(pio_hi, sm_read_hi, false));

    printf("READY. Read+write capture runs quietly in the background.\n");
    printf("Use INFO, READ, READSNES, HEX, DUMPBIN or RBIN.\n\n");
    fflush(stdout);

    // Start both capture pipelines. Each pipeline is independently rearmed when its
    // low+high DMA pair fills, so read traffic does not have to wait for writes and vice versa.

    // Write pipeline initial arm.
    pio_sm_set_enabled(pio_lo, sm_lo, false);
    pio_sm_set_enabled(pio_hi, sm_hi, false);
    pio_sm_restart(pio_lo, sm_lo);
    pio_sm_restart(pio_hi, sm_hi);
    pio_sm_clear_fifos(pio_lo, sm_lo);
    pio_sm_clear_fifos(pio_hi, sm_hi);
    pio_interrupt_clear(pio_hi, 0);
    arm_dma_channel(dma_lo, &dc_lo, low_samples, &pio_lo->rxf[sm_lo]);
    arm_dma_channel(dma_hi, &dc_hi, high_samples, &pio_hi->rxf[sm_hi]);
    dma_start_channel_mask((1u << dma_lo) | (1u << dma_hi));
    pio_sm_set_enabled(pio_hi, sm_hi, true);
    pio_sm_set_enabled(pio_lo, sm_lo, true);

    // Read pipeline initial arm.
    pio_sm_set_enabled(pio_rd, sm_rd_trigger, false);
    pio_sm_set_enabled(pio_lo, sm_read_lo, false);
    pio_sm_set_enabled(pio_hi, sm_read_hi, false);
    pio_sm_restart(pio_rd, sm_rd_trigger);
    pio_sm_restart(pio_lo, sm_read_lo);
    pio_sm_restart(pio_hi, sm_read_hi);
    pio_sm_clear_fifos(pio_lo, sm_read_lo);
    pio_sm_clear_fifos(pio_hi, sm_read_hi);
    pio_interrupt_clear(pio_lo, 1);
    pio_interrupt_clear(pio_hi, 1);
    arm_dma_channel(dma_read_lo, &dc_read_lo, read_low_samples, &pio_lo->rxf[sm_read_lo]);
    arm_dma_channel(dma_read_hi, &dc_read_hi, read_high_samples, &pio_hi->rxf[sm_read_hi]);
    dma_start_channel_mask((1u << dma_read_lo) | (1u << dma_read_hi));
    pio_sm_set_enabled(pio_hi, sm_read_hi, true);
    pio_sm_set_enabled(pio_lo, sm_read_lo, true);
    pio_sm_set_enabled(pio_rd, sm_rd_trigger, true);

    while (true) {
        poll_serial_commands();

        if (dma_remaining(dma_lo) == 0 && dma_remaining(dma_hi) == 0) {
            pio_sm_set_enabled(pio_lo, sm_lo, false);
            pio_sm_set_enabled(pio_hi, sm_hi, false);
            process_write_batch();

            pio_sm_restart(pio_lo, sm_lo);
            pio_sm_restart(pio_hi, sm_hi);
            pio_sm_clear_fifos(pio_lo, sm_lo);
            pio_sm_clear_fifos(pio_hi, sm_hi);
            pio_interrupt_clear(pio_hi, 0);
            arm_dma_channel(dma_lo, &dc_lo, low_samples, &pio_lo->rxf[sm_lo]);
            arm_dma_channel(dma_hi, &dc_hi, high_samples, &pio_hi->rxf[sm_hi]);
            dma_start_channel_mask((1u << dma_lo) | (1u << dma_hi));
            pio_sm_set_enabled(pio_hi, sm_hi, true);
            pio_sm_set_enabled(pio_lo, sm_lo, true);
        }

        if (dma_remaining(dma_read_lo) == 0 && dma_remaining(dma_read_hi) == 0) {
            pio_sm_set_enabled(pio_rd, sm_rd_trigger, false);
            pio_sm_set_enabled(pio_lo, sm_read_lo, false);
            pio_sm_set_enabled(pio_hi, sm_read_hi, false);
            process_read_batch();

            pio_sm_restart(pio_rd, sm_rd_trigger);
            pio_sm_restart(pio_lo, sm_read_lo);
            pio_sm_restart(pio_hi, sm_read_hi);
            pio_sm_clear_fifos(pio_lo, sm_read_lo);
            pio_sm_clear_fifos(pio_hi, sm_read_hi);
            pio_interrupt_clear(pio_lo, 1);
            pio_interrupt_clear(pio_hi, 1);
            arm_dma_channel(dma_read_lo, &dc_read_lo, read_low_samples, &pio_lo->rxf[sm_read_lo]);
            arm_dma_channel(dma_read_hi, &dc_read_hi, read_high_samples, &pio_hi->rxf[sm_read_hi]);
            dma_start_channel_mask((1u << dma_read_lo) | (1u << dma_read_hi));
            pio_sm_set_enabled(pio_hi, sm_read_hi, true);
            pio_sm_set_enabled(pio_lo, sm_read_lo, true);
            pio_sm_set_enabled(pio_rd, sm_rd_trigger, true);
        }

        tight_loop_contents();
    }

}

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
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
#define CMD_BUF_SIZE   4096
#define DEBUG_MAX_LINES 4096u
#define SNAP_MAX_RANGES 256u
#define SNAP_MAX_BYTES  4096u

static uint32_t low_samples[SAMPLE_COUNT];
static uint32_t high_samples[SAMPLE_COUNT];
static uint32_t read_low_samples[SAMPLE_COUNT];
static uint32_t read_high_samples[SAMPLE_COUNT];

// Passive WRAM mirror. A byte is marked valid after this adapter
// has observed a WRAM read or write carrying that byte.
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
static uint64_t total_wmaddr_reg_writes = 0;
static uint64_t total_wmdata_writes = 0;
static uint64_t total_wmdata_reads = 0;
static uint64_t total_wmdata_reconstructed_writes = 0;
static uint64_t total_wmdata_reconstructed_reads = 0;
static uint64_t total_wmdata_unknown_pointer = 0;
static uint32_t completed_batches = 0;
static uint64_t total_snapshots = 0;
static uint32_t last_snapshot_ranges = 0;
static uint32_t last_snapshot_bytes = 0;
static uint32_t last_snapshot_unknown = 0;

// CPU-side WRAM port ($2180-$2183) shadow.  The three WMADD registers form
// a 17-bit pointer.  This is useful when a game accesses WRAM above $1FFF
// through WMDATA instead of direct $7E/$7F A-bus cycles.
static uint32_t wmadd_shadow = 0;
static uint8_t wmadd_known_mask = 0; // bit0=$2181, bit1=$2182, bit2=$2183

static char cmd_buf[CMD_BUF_SIZE];
static size_t cmd_len = 0;

// Machine-readable atomic snapshot staging. The mirror is single-threaded: while
// command_snap() copies these bytes, process_read/write_batch() cannot modify WRAM.
// DMA can continue collecting bus samples in the background, so all returned
// bytes represent one coherent software-mirror instant.
static uint8_t snap_data[SNAP_MAX_BYTES];
static uint8_t snap_known[SNAP_MAX_BYTES];

// Physical wiring actually measured on the prototype.
// Index = logical SNES address bit A0..A23, value = RP2350B GPIO.
// The PIO programs capture raw GPIO windows; address bits are reordered here.
static const uint8_t ADDRESS_GPIO[24] = {
    12, 11, 10, 13, 14, 16, 15, 17,  // A0..A7
    19, 21, 22, 23, 24, 25, 26, 27,  // A8..A15
    28, 29, 30, 31, 32, 33, 34, 18   // A16..A23
};

typedef enum {
    DEBUG_OFF = 0,
    DEBUG_ALL,
    DEBUG_WRAM,
    DEBUG_READ,
    DEBUG_WRITE,
    DEBUG_WMDATA,
    DEBUG_RAWREAD
} debug_mode_t;

static debug_mode_t debug_mode = DEBUG_OFF;
static uint32_t debug_remaining = 0;
static uint64_t bank_write_counts[256];
static uint64_t bank_read_counts[256];

static void strtoupper_inplace(char *s);
static void mark_wram_known(uint32_t offset);

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

static inline uint32_t sampled_gpio_bit(uint16_t low16, uint32_t high17, uint gpio) {
    // low16 bit 0 is GP2 and bit 15 is GP17.
    if (gpio >= 10 && gpio <= 17) {
        return (low16 >> (gpio - 2u)) & 1u;
    }
    // high17 bit 0 is GP18 and bit 16 is GP34.
    if (gpio >= 18 && gpio <= 34) {
        return (high17 >> (gpio - 18u)) & 1u;
    }
    return 0;
}

static uint32_t reconstruct_address(uint16_t low16, uint32_t high17) {
    uint32_t address = 0;
    for (uint bit = 0; bit < 24; ++bit) {
        address |= sampled_gpio_bit(low16, high17, ADDRESS_GPIO[bit]) << bit;
    }
    return address;
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

static const char *debug_mode_name(debug_mode_t mode) {
    switch (mode) {
        case DEBUG_ALL: return "ALL";
        case DEBUG_WRAM: return "WRAM";
        case DEBUG_READ: return "READ";
        case DEBUG_WRITE: return "WRITE";
        case DEBUG_WMDATA: return "WMDATA";
        case DEBUG_RAWREAD: return "RAWREAD";
        default: return "OFF";
    }
}

static bool parse_debug_count(const char *s, uint32_t *out) {
    if (!s || !*s) return false;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end != '\0' || v == 0 || v > DEBUG_MAX_LINES) return false;
    *out = (uint32_t)v;
    return true;
}

static void maybe_debug_access(bool is_read, uint32_t address, uint8_t data,
                               bool is_wram, uint32_t wram_offset, bool direct) {
    if (debug_mode == DEBUG_OFF || debug_remaining == 0) return;

    bool match = false;
    switch (debug_mode) {
        case DEBUG_ALL: match = true; break;
        case DEBUG_WRAM: match = is_wram; break;
        case DEBUG_READ: match = is_read; break;
        case DEBUG_WRITE: match = !is_read; break;
        case DEBUG_WMDATA: match = false; break;
        case DEBUG_RAWREAD: match = false; break;
        default: break;
    }
    if (!match) return;

    uint8_t bank = (uint8_t)(address >> 16);
    uint16_t addr = (uint16_t)address;
    printf("%s $%02X:%04X = %02X", is_read ? "READ " : "WRITE", bank, addr, data);
    if (is_wram) {
        printf("   [WRAM +0x%05lX %s]", (unsigned long)wram_offset,
               direct ? "DIRECT" : "MIRROR");
    }
    printf("\n");
    fflush(stdout);

    --debug_remaining;
    if (debug_remaining == 0) {
        debug_mode = DEBUG_OFF;
        printf("DEBUG DONE (auto-off)\n");
        fflush(stdout);
    }
}

static void maybe_debug_raw_read(uint16_t low16, uint32_t high17, uint8_t data, uint32_t reconstructed) {
    if (debug_mode != DEBUG_RAWREAD || debug_remaining == 0) return;

    // L is the raw state of GP10..GP17: bit0=GP10 ... bit7=GP17.
    // H is the raw state of GP18..GP34: bit0=GP18 ... bit16=GP34.
    // These are captured before logical A0..A23 remapping.  The PC-side mapper
    // can therefore test alternate wiring permutations without reflashing.
    uint8_t raw_l = (uint8_t)((low16 >> 8) & 0xffu);
    printf("RAWREAD L=%02X H=%05lX D=%02X A=%06lX\n",
           raw_l, (unsigned long)(high17 & 0x1ffffu), data,
           (unsigned long)(reconstructed & 0xffffffu));
    fflush(stdout);

    --debug_remaining;
    if (debug_remaining == 0) {
        debug_mode = DEBUG_OFF;
        printf("DEBUG DONE (auto-off)\n");
        fflush(stdout);
    }
}

static void command_debug(char *args) {
    if (!args || !*args) {
        printf("DEBUG mode=%s remaining=%lu\n", debug_mode_name(debug_mode),
               (unsigned long)debug_remaining);
        return;
    }

    char temp[64];
    strncpy(temp, args, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    char *mode = strtok(temp, " \t");
    char *count_s = strtok(NULL, " \t");
    strtoupper_inplace(mode);

    if (!strcmp(mode, "OFF")) {
        debug_mode = DEBUG_OFF;
        debug_remaining = 0;
        printf("OK DEBUG OFF\n");
        return;
    }

    debug_mode_t new_mode;
    if (!strcmp(mode, "ALL")) new_mode = DEBUG_ALL;
    else if (!strcmp(mode, "WRAM")) new_mode = DEBUG_WRAM;
    else if (!strcmp(mode, "READ")) new_mode = DEBUG_READ;
    else if (!strcmp(mode, "WRITE")) new_mode = DEBUG_WRITE;
    else if (!strcmp(mode, "WMDATA")) new_mode = DEBUG_WMDATA;
    else if (!strcmp(mode, "RAWREAD")) new_mode = DEBUG_RAWREAD;
    else {
        printf("ERR DEBUG expects OFF, ALL, WRAM, READ, WRITE, WMDATA or RAWREAD\n");
        return;
    }

    uint32_t count = 64;
    if (count_s && !parse_debug_count(count_s, &count)) {
        printf("ERR DEBUG count must be decimal 1-%u\n", (unsigned)DEBUG_MAX_LINES);
        return;
    }

    debug_mode = new_mode;
    debug_remaining = count;
    printf("OK DEBUG %s %lu (auto-off after matching lines)\n",
           debug_mode_name(debug_mode), (unsigned long)debug_remaining);
}

static bool is_wram_port_address(uint32_t address, uint16_t *reg_out) {
    uint8_t bank = (uint8_t)(address >> 16);
    uint16_t addr = (uint16_t)address;
    bool io_bank = (bank <= 0x3f) || (bank >= 0x80 && bank <= 0xbf);
    if (!io_bank || addr < 0x2180u || addr > 0x2183u) return false;
    if (reg_out) *reg_out = addr;
    return true;
}

static bool wmadd_is_known(void) {
    return wmadd_known_mask == 0x07u;
}

static void debug_wm_event(const char *fmt, ...) {
    if (debug_mode != DEBUG_WMDATA || debug_remaining == 0) return;

    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);

    --debug_remaining;
    if (debug_remaining == 0) {
        debug_mode = DEBUG_OFF;
        printf("DEBUG DONE (auto-off)\n");
        fflush(stdout);
    }
}

static void process_wram_port_write(uint32_t address, uint8_t data) {
    uint16_t reg;
    if (!is_wram_port_address(address, &reg)) return;

    uint8_t bank = (uint8_t)(address >> 16);
    switch (reg) {
        case 0x2181u: // WMADDL
            wmadd_shadow = (wmadd_shadow & 0x1ff00u) | (uint32_t)data;
            wmadd_known_mask |= 0x01u;
            ++total_wmaddr_reg_writes;
            debug_wm_event("WMADDL WRITE $%02X:2181 = %02X -> WMADD=%05lX mask=%u/7%s",
                           bank, data, (unsigned long)wmadd_shadow,
                           (unsigned)wmadd_known_mask, wmadd_is_known() ? " VALID" : "");
            break;

        case 0x2182u: // WMADDM
            wmadd_shadow = (wmadd_shadow & 0x100ffu) | ((uint32_t)data << 8);
            wmadd_known_mask |= 0x02u;
            ++total_wmaddr_reg_writes;
            debug_wm_event("WMADDM WRITE $%02X:2182 = %02X -> WMADD=%05lX mask=%u/7%s",
                           bank, data, (unsigned long)wmadd_shadow,
                           (unsigned)wmadd_known_mask, wmadd_is_known() ? " VALID" : "");
            break;

        case 0x2183u: // WMADDH (only bit 0 participates in the 17-bit WRAM pointer)
            wmadd_shadow = (wmadd_shadow & 0x0ffffu) | (((uint32_t)data & 1u) << 16);
            wmadd_known_mask |= 0x04u;
            ++total_wmaddr_reg_writes;
            debug_wm_event("WMADDH WRITE $%02X:2183 = %02X -> WMADD=%05lX mask=%u/7%s",
                           bank, data, (unsigned long)wmadd_shadow,
                           (unsigned)wmadd_known_mask, wmadd_is_known() ? " VALID" : "");
            break;

        case 0x2180u: { // WMDATA write, then 17-bit pointer increments
            ++total_wmdata_writes;
            if (!wmadd_is_known()) {
                ++total_wmdata_unknown_pointer;
                debug_wm_event("WMDATA WRITE $%02X:2180 = %02X -> pointer UNKNOWN (mask=%u/7)",
                               bank, data, (unsigned)wmadd_known_mask);
                return;
            }

            uint32_t off = wmadd_shadow & 0x1ffffu;
            wram[off] = data;
            mark_wram_known(off);
            ++total_wmdata_reconstructed_writes;
            uint32_t next = (off + 1u) & 0x1ffffu;
            debug_wm_event("WMDATA WRITE $%02X:2180 = %02X -> WRAM[%05lX], next=%05lX",
                           bank, data, (unsigned long)off, (unsigned long)next);
            wmadd_shadow = next;
            break;
        }
    }
}

static void process_wram_port_read(uint32_t address, uint8_t data) {
    uint16_t reg;
    if (!is_wram_port_address(address, &reg) || reg != 0x2180u) return;

    ++total_wmdata_reads;
    uint8_t bank = (uint8_t)(address >> 16);
    if (!wmadd_is_known()) {
        ++total_wmdata_unknown_pointer;
        debug_wm_event("WMDATA READ  $%02X:2180 = %02X -> pointer UNKNOWN (mask=%u/7)",
                       bank, data, (unsigned)wmadd_known_mask);
        return;
    }

    // Experimental: the read and write capture pipelines are buffered independently,
    // so their software processing order is not guaranteed to be identical to bus order.
    // We still expose the candidate and use it to improve the mirror, but DEBUG output
    // labels it EXPERIMENTAL.  A future unified event stream can remove this caveat.
    uint32_t off = wmadd_shadow & 0x1ffffu;
    wram[off] = data;
    mark_wram_known(off);
    ++total_wmdata_reconstructed_reads;
    uint32_t next = (off + 1u) & 0x1ffffu;
    debug_wm_event("WMDATA READ  $%02X:2180 = %02X -> WRAM[%05lX], next=%05lX [EXPERIMENTAL ORDER]",
                   bank, data, (unsigned long)off, (unsigned long)next);
    wmadd_shadow = next;
}

static void command_wmstate(void) {
    printf("WMSTATE addr=%05lX mask=%u/7 valid=%s addr_reg_w=%llu data_w=%llu data_r=%llu "
           "recon_w=%llu recon_r=%llu unknown_ptr=%llu\n",
           (unsigned long)(wmadd_shadow & 0x1ffffu), (unsigned)wmadd_known_mask,
           wmadd_is_known() ? "YES" : "NO",
           (unsigned long long)total_wmaddr_reg_writes,
           (unsigned long long)total_wmdata_writes,
           (unsigned long long)total_wmdata_reads,
           (unsigned long long)total_wmdata_reconstructed_writes,
           (unsigned long long)total_wmdata_reconstructed_reads,
           (unsigned long long)total_wmdata_unknown_pointer);
}

static void command_banks(void) {
    printf("BANKS nonzero reconstructed A-bus banks:\n");
    uint32_t shown = 0;
    for (uint32_t b = 0; b < 256; ++b) {
        if (bank_write_counts[b] || bank_read_counts[b]) {
            printf("BANK %02lX W=%llu R=%llu\n", (unsigned long)b,
                   (unsigned long long)bank_write_counts[b],
                   (unsigned long long)bank_read_counts[b]);
            ++shown;
        }
    }
    if (!shown) printf("(none yet)\n");
    printf("END BANKS\n");
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
    printf("  SNAP o:l [o:l ...]      atomic multi-range WRAM snapshot for RA bridge\n");
    printf("  CLEAR                   clear the software mirror/known bitmap\n");
    printf("  DEBUG                   show debug status\n");
    printf("  DEBUG WRAM [N]          print next N WRAM accesses (default 64)\n");
    printf("  DEBUG ALL [N]           print next N A-bus accesses\n");
    printf("  DEBUG READ [N]          print next N reads\n");
    printf("  DEBUG WRITE [N]         print next N writes\n");
    printf("  DEBUG WMDATA [N]        print $2180-$2183 WRAM-port activity\n");
    printf("  DEBUG RAWREAD [N]       print raw GP10-17/GP18-34 read samples for auto-mapper\n");
    printf("  DEBUG OFF               stop debug output\n");
    printf("  BANKS                   show per-bank read/write counters\n");
    printf("  WMSTATE                 show $2180-$2183 pointer/counters\n");
    printf("  PING                    reply PONG\n");
    printf("Mirror source: observed WRAM reads+writes plus reconstructed CPU WMDATA ($2180) traffic.\n");
    printf("DEBUG is diagnostic only: use it in PuTTY with the Python RA bridge closed.\n");
}

static void print_info(void) {
    printf("INFO batches=%lu bus_writes=%llu wram_writes=%llu direct_w=%llu mirror_w=%llu "
           "bus_reads=%llu wram_reads=%llu direct_r=%llu mirror_r=%llu "
           "wmaddr_w=%llu wmdata_w=%llu wmdata_r=%llu wm_recon_w=%llu wm_recon_r=%llu "
           "wm_unknown=%llu snap_count=%llu snap_ranges=%lu snap_bytes=%lu snap_unknown=%lu "
           "known=%lu/%u (%.2f%%)\n",
           (unsigned long)completed_batches,
           (unsigned long long)total_bus_writes,
           (unsigned long long)total_wram_writes,
           (unsigned long long)total_direct_writes,
           (unsigned long long)total_mirror_writes,
           (unsigned long long)total_bus_reads,
           (unsigned long long)total_wram_reads,
           (unsigned long long)total_direct_reads,
           (unsigned long long)total_mirror_reads,
           (unsigned long long)total_wmaddr_reg_writes,
           (unsigned long long)total_wmdata_writes,
           (unsigned long long)total_wmdata_reads,
           (unsigned long long)total_wmdata_reconstructed_writes,
           (unsigned long long)total_wmdata_reconstructed_reads,
           (unsigned long long)total_wmdata_unknown_pointer,
           (unsigned long long)total_snapshots,
           (unsigned long)last_snapshot_ranges,
           (unsigned long)last_snapshot_bytes,
           (unsigned long)last_snapshot_unknown,
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


static void command_snap(char *args) {
    // Syntax: SNAP OOOOO:L OOOOO:L ...
    // Offsets and lengths are hexadecimal. Response payload is:
    //   SNAP1 <ranges> <bytes> <unknown> <batch>\n
    //   <bytes raw WRAM data>
    //   <bytes raw known flags: 1=known, 0=unknown>
    //   \nEND SNAP1\n
    // Data/known bytes are copied before any USB payload is emitted, so a single
    // SNAP response cannot mix values from different mirror updates.
    if (!args || !*args) {
        printf("ERR SNAP expects <offset:length> [offset:length ...] in hex\n");
        return;
    }

    uint32_t total_len = 0;
    uint32_t unknown = 0;
    uint32_t ranges = 0;

    for (char *tok = strtok(args, " \t"); tok; tok = strtok(NULL, " \t")) {
        if (ranges >= SNAP_MAX_RANGES) {
            printf("ERR SNAP too many ranges (max %u)\n", (unsigned)SNAP_MAX_RANGES);
            return;
        }

        char *colon = strchr(tok, ':');
        if (!colon) {
            printf("ERR SNAP token '%s' expects offset:length\n", tok);
            return;
        }
        *colon = '\0';
        const char *off_s = tok;
        const char *len_s = colon + 1;
        uint32_t off = 0, len = 0;
        if (!parse_hex_u32(off_s, &off) || !parse_hex_u32(len_s, &len)) {
            printf("ERR SNAP expects hexadecimal offset:length tokens\n");
            return;
        }
        if (off >= WRAM_SIZE || len == 0 || off + len > WRAM_SIZE) {
            printf("ERR SNAP range out of bounds at %05lX:%lX\n",
                   (unsigned long)off, (unsigned long)len);
            return;
        }
        if (total_len + len > SNAP_MAX_BYTES) {
            printf("ERR SNAP total payload too large (max %u bytes)\n", (unsigned)SNAP_MAX_BYTES);
            return;
        }

        for (uint32_t i = 0; i < len; ++i) {
            uint32_t src = off + i;
            bool known = wram_byte_known(src);
            snap_data[total_len] = wram[src];
            snap_known[total_len] = known ? 1u : 0u;
            if (!known) ++unknown;
            ++total_len;
        }
        ++ranges;
    }

    if (ranges == 0 || total_len == 0) {
        printf("ERR SNAP empty request\n");
        return;
    }

    // Everything above this point is the atomic copy. From here on, only the
    // staging buffers are transmitted; changes in WRAM no longer affect this response.
    ++total_snapshots;
    last_snapshot_ranges = ranges;
    last_snapshot_bytes = total_len;
    last_snapshot_unknown = unknown;
    uint32_t batch_stamp = completed_batches;
    printf("SNAP1 %lu %lu %lu %lu\n",
           (unsigned long)ranges,
           (unsigned long)total_len,
           (unsigned long)unknown,
           (unsigned long)batch_stamp);
    fflush(stdout);
    stdio_put_string((const char *)snap_data, (int)total_len, false, false);
    stdio_put_string((const char *)snap_known, (int)total_len, false, false);
    stdio_flush();
    printf("\nEND SNAP1\n");
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
    } else if (!strcmp(cmd, "SNAP")) {
        command_snap(rest);
    } else if (!strcmp(cmd, "CLEAR")) {
        memset(wram, 0, sizeof(wram));
        memset(wram_valid, 0, sizeof(wram_valid));
        known_wram_bytes = 0;
        wmadd_shadow = 0;
        wmadd_known_mask = 0;
        printf("OK mirror and WMADD state cleared\n");
    } else if (!strcmp(cmd, "DEBUG")) {
        command_debug(rest);
    } else if (!strcmp(cmd, "BANKS")) {
        command_banks();
    } else if (!strcmp(cmd, "WMSTATE")) {
        command_wmstate();
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
        uint8_t bank = (uint8_t)(address >> 16);

        ++total_bus_writes;
        ++bank_write_counts[bank];

        // CPU-side WRAM data/address ports live at $2180-$2183 in I/O banks.
        // Handle these even though they are not themselves ordinary WRAM addresses.
        process_wram_port_write(address, data);

        uint32_t wram_offset = 0;
        bool direct = false;
        bool is_wram = map_address_to_wram(address, &wram_offset, &direct);
        maybe_debug_access(false, address, data, is_wram, wram_offset, direct);
        if (!is_wram) continue;

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
        maybe_debug_raw_read(low16, high17, data, address);
        uint8_t bank = (uint8_t)(address >> 16);

        ++total_bus_reads;
        ++bank_read_counts[bank];

        process_wram_port_read(address, data);

        uint32_t wram_offset = 0;
        bool direct = false;
        bool is_wram = map_address_to_wram(address, &wram_offset, &direct);
        maybe_debug_access(true, address, data, is_wram, wram_offset, direct);
        if (!is_wram) continue;

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

    printf("\n=== SNES RP2350B RA Bridge Firmware v1.4 RAW ADDRESS MAPPER ===\n");
    printf("Passive A-bus monitor with address remap + atomic RA snapshots + raw-address diagnostics.\n");
    printf("PHI2=GP0 /WR=GP1 /RD=GP35 D0-D7=GP2-9; address GPIO order is remapped in firmware.\n");
    printf("The 128 KiB mirror learns from direct/mirror WRAM traffic and CPU-side WMDATA accesses; unseen bytes remain UNKNOWN.\n");
    printf("Keep the RP2350B powered before powering the SNES.\n");
    printf("Measured map: A0=12 A1=11 A2=10 A3=13 A4=14 A5=16 A6=15 A7=17\n");
    printf("              A8=19 A9=21 A10=22 ... A22=34 A23=18 (GP20 skipped)\n");
    printf("Type HELP for commands; DEBUG RAWREAD is used by infer_address_map.py.\n\n");
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

    printf("READY. Read+write capture runs quietly in the background (v1.4 raw mapper).\n");
    printf("Use INFO, BANKS, WMSTATE, DEBUG, READ, READSNES, HEX, DUMPBIN, RBIN or SNAP.\n\n");
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
        // Process completed capture batches before servicing serial commands so an
        // atomic SNAP sees the freshest software mirror available at that boundary.
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

        poll_serial_commands();

        tight_loop_contents();
    }

}

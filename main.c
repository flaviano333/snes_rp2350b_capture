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
#define PIN_HIGH_BASE  21
#define PIN_RD         35
#define PIN_ROMSEL     38   // active-low ROM select; diagnostic in v2.2 AUTO-ROM
#define PIN_WRAMSEL    39   // active-low WRAM select, used by v1.8+

#define SAMPLE_COUNT   1024
#define WRAM_SIZE      (128u * 1024u)
#define VALID_SIZE     (WRAM_SIZE / 8u)
#define CMD_BUF_SIZE   4096
#define DEBUG_MAX_LINES 4096u
#define SNAP_MAX_RANGES 256u
#define SNAP_MAX_BYTES  4096u
#define ROMFP_MAX_RECORDS 1024u


// v1.6 uses two buffers per capture stream. As soon as one DMA batch fills,
 // DMA is rearmed into the other buffer BEFORE the completed batch is processed.
 // This removes the long blind interval present in v1.3.x, where PIO was stopped
 // while 256 samples were decoded.
static uint32_t low_samples[2][SAMPLE_COUNT];
static uint32_t high_samples[2][SAMPLE_COUNT];
static uint32_t read_low_samples[2][SAMPLE_COUNT];
static uint32_t read_high_samples[2][SAMPLE_COUNT];

// Passive WRAM mirror. In v2.0, writes remain authoritative and are
// qualified by /WRAMSEL. Qualified A-bus reads can also refresh KNOWN bytes.
// This READ-REPAIR path fixes stale counters when an occasional write sample
// is missed: a later physical WRAM read restores the mirror to the bus value.
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
static uint64_t total_wramsel_write_ok = 0;
static uint64_t total_wramsel_write_reject = 0;
static uint64_t total_wramsel_read_seed = 0;
static uint64_t total_wramsel_read_same = 0;
static uint64_t total_wramsel_read_repair = 0;
static uint64_t total_wramsel_read_ignored = 0;
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

// v2.2 AUTO-ROM fingerprint.
// The v2.1 detector trusted /ROMSEL at the exact high-window sample instant. On real
// hardware that proved too timing-sensitive: a valid cartridge could yield essentially
// random ROM matches. v2.2 therefore fingerprints only a conservative A-bus window that
// is ROM for both LoROM and HiROM: banks $00-$3F/$80-$BF at $8000-$FFFF.
// GP38=/ROMSEL is still sampled and counted as a diagnostic, but it is no longer required
// for a sample to enter the fingerprint. This does NOT touch the proven v2.0 WRAM path.
static uint32_t romfp_addr[ROMFP_MAX_RECORDS];
static uint8_t romfp_data[ROMFP_MAX_RECORDS];
static uint32_t romfp_count = 0;
static uint64_t total_romsel_reads = 0;
static uint64_t total_rom_window_reads = 0;
static uint64_t total_rom_window_romsel_low = 0;
static uint32_t romfp_last_addr = 0xffffffffu;
static uint8_t romfp_last_data = 0u;


// Tom and Jerry (USA) targeted diagnostic. Known Action Replay codes for
// 99 cheese bits write 0x63 to WRAM $7E:1558 and $7E:155C. These counters
// let the PC bridge prove whether the physical bus mirror is actually moving.
#define CHEESE_OFF_A 0x1558u
#define CHEESE_OFF_B 0x155Cu
static uint64_t cheese_write_seen[2] = {0, 0};
static uint64_t cheese_read_seen[2] = {0, 0};
static uint64_t cheese_repair_seen[2] = {0, 0};

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
    11, 12, 13, 14, 15, 16, 17, 18,  // A0..A7
    19, 21, 40, 23, 24, 25, 27, 28,  // A8..A15; A10 moved to GP40
    29, 30, 31, 32, 33, 34, 36, 37   // A16..A23
};

typedef enum {
    DEBUG_OFF = 0,
    DEBUG_ALL,
    DEBUG_WRAM,
    DEBUG_READ,
    DEBUG_WRITE,
    DEBUG_WMDATA
} debug_mode_t;

static debug_mode_t debug_mode = DEBUG_OFF;
static uint32_t debug_remaining = 0;
static uint64_t bank_write_counts[256];
static uint64_t bank_read_counts[256];

static void strtoupper_inplace(char *s);
static void mark_wram_known(uint32_t offset);
static bool wram_byte_known(uint32_t offset);
static bool seed_wram_if_unknown(uint32_t offset, uint8_t data);
static int refresh_wram_from_read(uint32_t offset, uint8_t data);
static int cheese_slot(uint32_t offset);

static void configure_input(uint pin, bool pull_up) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
    if (pull_up) gpio_pull_up(pin);
}

static uint32_t dma_remaining(uint channel) {
    return dma_channel_hw_addr(channel)->transfer_count & 0x0fffffffu;
}

static uint32_t unpack_low18(uint32_t raw) {
    // PIO "in pins, 18" with right-shift leaves captured bits in raw[31:14].
    return (raw >> 14) & 0x3ffffu;
}

static uint32_t unpack_high20(uint32_t raw) {
    // PIO "in pins, 20" with right-shift leaves captured bits in raw[31:12].
    return (raw >> 12) & 0xfffffu;
}

static inline uint8_t reconstruct_data(uint32_t low18) {
    // low18 starts at GP2:
    // bit0=GP2=D0, bit1=GP3=D1, bit2=GP4=D2
    // bit3=GP5=dummy/unusable
    // bit4..8=GP6..GP10=D3..D7
    return (uint8_t)(
        (low18 & 0x07u) |
        ((low18 >> 1) & 0xf8u)
    );
}

static inline bool sampled_wramsel_active(uint32_t high20) {
    // HIGH window starts at GP21, therefore GP39 is bit 18.
    // /WRAMSEL is active low on the SNES bus.
    return ((high20 >> (PIN_WRAMSEL - PIN_HIGH_BASE)) & 1u) == 0u;
}

static inline bool sampled_romsel_active(uint32_t high20) {
    // HIGH window starts at GP21, therefore GP38 is bit 17.
    // /ROMSEL (/CART) is active low on cartridge-ROM accesses.
    return ((high20 >> (PIN_ROMSEL - PIN_HIGH_BASE)) & 1u) == 0u;
}

static inline bool is_safe_rom_fingerprint_window(uint32_t address) {
    uint8_t bank = (uint8_t)(address >> 16);
    uint16_t addr = (uint16_t)address;
    bool low_or_mirror_bank = (bank <= 0x3fu) || (bank >= 0x80u && bank <= 0xbfu);
    return low_or_mirror_bank && addr >= 0x8000u;
}

static inline void record_rom_fingerprint(uint32_t address, uint8_t data, uint32_t high20) {
    bool romsel = sampled_romsel_active(high20);
    if (romsel) ++total_romsel_reads;

    // v2.2 SAFE-WINDOW: do not depend on the exact /ROMSEL sampling instant.
    // $00-$3F/$80-$BF:$8000-$FFFF is cartridge ROM space for both conventional
    // LoROM and HiROM mappings, so these reads are suitable for mapper scoring.
    if (!is_safe_rom_fingerprint_window(address)) return;
    ++total_rom_window_reads;
    if (romsel) ++total_rom_window_romsel_low;

    // Keep the first diverse address/data stream. Consecutive identical reads add no
    // identification information and are skipped. Once full, the table stays stable
    // so a later ROMFP query cannot mix two different moments during one comparison.
    if (romfp_count >= ROMFP_MAX_RECORDS) return;
    if (address == romfp_last_addr && data == romfp_last_data) return;

    romfp_addr[romfp_count] = address & 0x00ffffffu;
    romfp_data[romfp_count] = data;
    ++romfp_count;
    romfp_last_addr = address & 0x00ffffffu;
    romfp_last_data = data;
}

static inline uint32_t sampled_gpio_bit(uint32_t low18, uint32_t high20, uint gpio) {
    // low18 bit0 is GP2 and bit17 is GP19.
    if (gpio >= 11 && gpio <= 19) {
        return (low18 >> (gpio - 2u)) & 1u;
    }

    // high20 bit0 is GP21 and bit19 is GP40.
    if (gpio >= 21 && gpio <= 40) {
        return (high20 >> (gpio - 21u)) & 1u;
    }

    return 0;
}

static inline uint32_t reconstruct_address(uint32_t low18, uint32_t high20) {
    // v1.6O3d ORDERED:
    //
    // LOW window GP2..GP19:
    // GP2..4   = D0..D2
    // GP5      = dummy (unusable)
    // GP6..10  = D3..D7
    // GP11..19 = A0..A8
    //
    // HIGH window GP21..GP40:
    // GP21     = A9
    // GP22     = dummy / unused (A10 moved away)
    // GP23..25 = A11..A13
    // GP26     = dummy (unusable)
    // GP27..34 = A14..A21
    // GP35     = /RD dummy in sampled window
    // GP36..37 = A22..A23
    // GP38     = /ROMSEL (active-low diagnostic; AUTO-ROM matching uses safe address window)
    // GP39     = /WRAMSEL (active-low qualifier)
    // GP40     = A10

    uint32_t low_addr = (low18 >> 9) & 0x1ffu;  // A0..A8

    // high20 bit mapping (starting at GP21):
    // bit0  = GP21 = A9
    // bit1  = GP22 dummy
    // bit2  = GP23 = A11
    // bit3  = GP24 = A12
    // bit4  = GP25 = A13
    // bit5  = GP26 dummy
    // bit6..13  = GP27..34 = A14..A21
    // bit14 = GP35 /RD dummy
    // bit15 = GP36 = A22
    // bit16 = GP37 = A23
    // bit17 = GP38 /ROMSEL diagnostic
    // bit18 = GP39 /WRAMSEL (active-low qualifier)
    // bit19 = GP40 = A10

    uint32_t high_addr =
        (((high20 >> 0) & 0x1u) << 9) |      // A9
        (((high20 >> 19) & 0x1u) << 10) |    // A10 from GP40
        (((high20 >> 2) & 0x7u) << 11) |     // A11..A13
        (((high20 >> 6) & 0xffu) << 14) |    // A14..A21
        (((high20 >> 15) & 0x1u) << 22) |    // A22
        (((high20 >> 16) & 0x1u) << 23);     // A23

    return low_addr | high_addr;
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
    else {
        printf("ERR DEBUG expects OFF, ALL, WRAM, READ, WRITE or WMDATA\n");
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

    // v2.0 keeps WMDATA reads conservative/seed-only. The read/write capture pipelines
    // are buffered independently, so a read must never "repair" a KNOWN byte
    // with a potentially reordered/stale sample. $2180 remains a separate path,
    // but obeys the same UNKNOWN-only seeding rule.
    uint32_t off = wmadd_shadow & 0x1ffffu;
    bool seeded = seed_wram_if_unknown(off, data);
    ++total_wmdata_reconstructed_reads;
    uint32_t next = (off + 1u) & 0x1ffffu;
    debug_wm_event(
        "WMDATA READ  $%02X:2180 = %02X -> WRAM[%05lX] %s, next=%05lX [WMDATA-SEED-ONLY]",
        bank, data, (unsigned long)off, seeded ? "SEEDED" : "IGNORED_KNOWN",
        (unsigned long)next);
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

static bool seed_wram_if_unknown(uint32_t offset, uint8_t data) {
    if (wram_byte_known(offset)) return false;
    wram[offset] = data;
    mark_wram_known(offset);
    return true;
}

// Return: 0=seeded UNKNOWN, 1=KNOWN and unchanged, 2=KNOWN and repaired.
static int refresh_wram_from_read(uint32_t offset, uint8_t data) {
    if (!wram_byte_known(offset)) {
        wram[offset] = data;
        mark_wram_known(offset);
        return 0;
    }
    if (wram[offset] == data) return 1;
    wram[offset] = data;
    return 2;
}

static int cheese_slot(uint32_t offset) {
    if (offset == CHEESE_OFF_A) return 0;
    if (offset == CHEESE_OFF_B) return 1;
    return -1;
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
    printf("  DEBUG OFF               stop debug output\n");
    printf("  BANKS                   show per-bank read/write counters\n");
    printf("  WMSTATE                 show $2180-$2183 pointer/counters\n");
    printf("  WRAMSEL                 show /WRAMSEL + READ-REPAIR counters\n");
    printf("  ROMFP                   binary safe-window cartridge-ROM fingerprint for AUTO-ROM bridge\n");
    printf("  ROMFPCLEAR              clear captured ROM fingerprint samples\n");
    printf("  CHEESE                  show legacy $1558/$155C targeted counters\n");
    printf("  PING                    reply PONG\n");
    printf("Mirror source: /WRAMSEL-qualified writes + qualified A-bus READ-REPAIR + conservative WMDATA ($2180).\n");
    printf("DEBUG is diagnostic only: use it in PuTTY with the Python RA bridge closed.\n");
}

static void command_wramsel(void) {
    printf("WRAMSEL write_ok=%llu write_reject=%llu read_seed=%llu read_same=%llu read_repair=%llu read_ignored=%llu known=%lu/%u\n",
           (unsigned long long)total_wramsel_write_ok,
           (unsigned long long)total_wramsel_write_reject,
           (unsigned long long)total_wramsel_read_seed,
           (unsigned long long)total_wramsel_read_same,
           (unsigned long long)total_wramsel_read_repair,
           (unsigned long long)total_wramsel_read_ignored,
           (unsigned long)known_wram_bytes,
           (unsigned)WRAM_SIZE);
}

static void command_cheese(void) {
    bool ka = wram_byte_known(CHEESE_OFF_A);
    bool kb = wram_byte_known(CHEESE_OFF_B);
    printf("CHEESE A=%02X A_known=%u A_w=%llu A_r=%llu A_repair=%llu "
           "B=%02X B_known=%u B_w=%llu B_r=%llu B_repair=%llu\n",
           ka ? wram[CHEESE_OFF_A] : 0u, ka ? 1u : 0u,
           (unsigned long long)cheese_write_seen[0],
           (unsigned long long)cheese_read_seen[0],
           (unsigned long long)cheese_repair_seen[0],
           kb ? wram[CHEESE_OFF_B] : 0u, kb ? 1u : 0u,
           (unsigned long long)cheese_write_seen[1],
           (unsigned long long)cheese_read_seen[1],
           (unsigned long long)cheese_repair_seen[1]);
}

static void command_romfp(void) {
    // Binary record format (4 bytes each): addr_lo, addr_mid, addr_hi, data.
    // Addresses are physical SNES A-bus addresses observed in the v2.2 safe ROM window.
    printf("ROMFP2 %lu %llu %llu %llu\n",
           (unsigned long)romfp_count,
           (unsigned long long)total_rom_window_reads,
           (unsigned long long)total_rom_window_romsel_low,
           (unsigned long long)total_romsel_reads);
    fflush(stdout);

    uint8_t rec[4];
    for (uint32_t i = 0; i < romfp_count; ++i) {
        uint32_t a = romfp_addr[i] & 0x00ffffffu;
        rec[0] = (uint8_t)(a & 0xffu);
        rec[1] = (uint8_t)((a >> 8) & 0xffu);
        rec[2] = (uint8_t)((a >> 16) & 0xffu);
        rec[3] = romfp_data[i];
        stdio_put_string((const char *)rec, 4, false, false);
    }
    stdio_flush();
    printf("\nEND ROMFP2\n");
    fflush(stdout);
}

static void command_romfp_clear(void) {
    romfp_count = 0;
    total_romsel_reads = 0;
    total_rom_window_reads = 0;
    total_rom_window_romsel_low = 0;
    romfp_last_addr = 0xffffffffu;
    romfp_last_data = 0u;
    printf("OK ROMFP cleared\n");
}

static void print_info(void) {
    printf("INFO batches=%lu bus_writes=%llu wram_writes=%llu direct_w=%llu mirror_w=%llu "
           "bus_reads=%llu wram_reads=%llu direct_r=%llu mirror_r=%llu "
           "write_ok=%llu write_reject=%llu read_seed=%llu read_same=%llu read_repair=%llu read_ignored=%llu "
           "wmaddr_w=%llu wmdata_w=%llu wmdata_r=%llu wm_recon_w=%llu wm_recon_r=%llu "
           "wm_unknown=%llu snap_count=%llu snap_ranges=%lu snap_bytes=%lu snap_unknown=%lu "
           "romsel_reads=%llu romwin_reads=%llu romwin_romsel_low=%llu romfp=%lu/%u known=%lu/%u (%.2f%%)\n",
           (unsigned long)completed_batches,
           (unsigned long long)total_bus_writes,
           (unsigned long long)total_wram_writes,
           (unsigned long long)total_direct_writes,
           (unsigned long long)total_mirror_writes,
           (unsigned long long)total_bus_reads,
           (unsigned long long)total_wram_reads,
           (unsigned long long)total_direct_reads,
           (unsigned long long)total_mirror_reads,
           (unsigned long long)total_wramsel_write_ok,
           (unsigned long long)total_wramsel_write_reject,
           (unsigned long long)total_wramsel_read_seed,
           (unsigned long long)total_wramsel_read_same,
           (unsigned long long)total_wramsel_read_repair,
           (unsigned long long)total_wramsel_read_ignored,
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
           (unsigned long long)total_romsel_reads,
           (unsigned long long)total_rom_window_reads,
           (unsigned long long)total_rom_window_romsel_low,
           (unsigned long)romfp_count,
           (unsigned)ROMFP_MAX_RECORDS,
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
    } else if (!strcmp(cmd, "WRAMSEL")) {
        command_wramsel();
    } else if (!strcmp(cmd, "ROMFP")) {
        command_romfp();
    } else if (!strcmp(cmd, "ROMFPCLEAR")) {
        command_romfp_clear();
    } else if (!strcmp(cmd, "CHEESE")) {
        command_cheese();
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

static void process_write_batch(const uint32_t *low_buf, const uint32_t *high_buf) {
    ++completed_batches;

    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        uint32_t low18 = unpack_low18(low_buf[i]);
        uint32_t high20 = unpack_high20(high_buf[i]);
        uint8_t data = reconstruct_data(low18);
        uint32_t address = reconstruct_address(low18, high20) & 0xffffffu;
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

        // v1.8 WRAMSEL: address decoding alone is not enough. Accept a WRAM
        // write only when the SNES also asserts /WRAMSEL (active low).
        if (!sampled_wramsel_active(high20)) {
            ++total_wramsel_write_reject;
            continue;
        }

        ++total_wramsel_write_ok;
        ++total_wram_writes;
        if (direct) ++total_direct_writes;
        else ++total_mirror_writes;

        wram[wram_offset] = data;
        mark_wram_known(wram_offset);
        int cslot = cheese_slot(wram_offset);
        if (cslot >= 0) ++cheese_write_seen[cslot];
    }
}

static void process_read_batch(const uint32_t *low_buf, const uint32_t *high_buf) {
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        uint32_t low18 = unpack_low18(low_buf[i]);
        uint32_t high20 = unpack_high20(high_buf[i]);
        uint8_t data = reconstruct_data(low18);
        uint32_t address = reconstruct_address(low18, high20) & 0xffffffu;
        uint8_t bank = (uint8_t)(address >> 16);

        ++total_bus_reads;
        ++bank_read_counts[bank];

        // v2.2 AUTO-ROM SAFE-WINDOW: collect real cartridge ROM reads in parallel with WRAM capture.
        // This is intentionally passive and does not alter the proven v2.0 WRAM path.
        record_rom_fingerprint(address, data, high20);

        process_wram_port_read(address, data);

        uint32_t wram_offset = 0;
        bool direct = false;
        bool is_wram = map_address_to_wram(address, &wram_offset, &direct);
        maybe_debug_access(true, address, data, is_wram, wram_offset, direct);
        if (!is_wram) continue;

        // The read state machine is already triggered by /RD=LOW. v2.0 keeps
        // /WRAMSEL=LOW as the second qualifier. Qualified reads can repair stale bytes.
        if (!sampled_wramsel_active(high20)) {
            ++total_wramsel_read_ignored;
            continue;
        }

        ++total_wram_reads;
        if (direct) ++total_direct_reads;
        else ++total_mirror_reads;

        // v2.0 READ-REPAIR: a qualified physical WRAM read is an observation
        // of the current byte. It may seed UNKNOWN memory or repair a stale
        // KNOWN value left behind when a write sample was missed.
        int refresh = refresh_wram_from_read(wram_offset, data);
        if (refresh == 0) ++total_wramsel_read_seed;
        else if (refresh == 1) ++total_wramsel_read_same;
        else ++total_wramsel_read_repair;

        int cslot = cheese_slot(wram_offset);
        if (cslot >= 0) {
            ++cheese_read_seen[cslot];
            if (refresh == 2) ++cheese_repair_seen[cslot];
        }
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

    printf("\n=== SNES RP2350B RA Firmware v2.2 WRAMSEL + READ-REPAIR + AUTO-ROM SAFE-WINDOW ===\n");
    printf("Passive A-bus monitor: /WRAMSEL-qualified WRAM + safe-window ROM fingerprint.\n");
    printf("PHI2=GP0 /WR=GP1; D0-D2=GP2-4; GP5=SKIP; D3-D7=GP6-10.\n");
    printf("A0..A8=GP11..19; GP20=SKIP; A9=GP21; GP22=SKIP; A10=GP40.\n");
    printf("A11..A13=GP23..25; GP26=SKIP; A14..A21=GP27..34; /RD=GP35.\n");
    printf("A22=GP36 A23=GP37 /ROMSEL=GP38 /WRAMSEL=GP39.\n");
    printf("GP38=/ROMSEL is diagnostic; AUTO-ROM uses the safe ROM address window. GP39=/WRAMSEL remains active; GP40=A10.\n");
    printf("Type HELP for commands; CHEESE probes $1558/$155C; SNAP provides coherent RA reads.\n\n");
    fflush(stdout);

    for (uint pin = 0; pin <= 40; ++pin) {
        configure_input(
            pin,
            pin == PIN_WR || pin == PIN_RD ||
            pin == PIN_ROMSEL || pin == PIN_WRAMSEL
        );
    }

    // ---------- PIO0: timing + low GPIO window GP2..GP19 ----------
    // GP5 is an intentional dummy. Data and A0..A8 are reconstructed in C.
    PIO pio_lo = pio0;
    const uint sm_lo = 0;
    int base_lo_rc = pio_set_gpio_base(pio_lo, 0);

    for (uint pin = 0; pin <= 19; ++pin) pio_gpio_init(pio_lo, pin);
    pio_sm_set_consecutive_pindirs(pio_lo, sm_lo, 0, 20, false);

    uint off_lo = pio_add_program(pio_lo, &snes_capture_low_program);
    pio_sm_config c_lo = snes_capture_low_program_get_default_config(off_lo);
    sm_config_set_in_pins(&c_lo, PIN_DATA_BASE);
    sm_config_set_jmp_pin(&c_lo, PIN_WR);
    sm_config_set_in_shift(&c_lo, true, false, 32);
    sm_config_set_fifo_join(&c_lo, PIO_FIFO_JOIN_RX);
    int init_lo_rc = pio_sm_init(pio_lo, sm_lo, off_lo, &c_lo);

    // ---------- PIO1: high GPIO window GP21..GP40 ----------
    // GP22, GP26 and GP35 are ignored for address reconstruction. GP38=/ROMSEL is diagnostic; GP39=/WRAMSEL qualifies WRAM.
    // GP40 now carries A10.
    PIO pio_hi = pio1;
    const uint sm_hi = 0;
    int base_hi_rc = pio_set_gpio_base(pio_hi, 16);

    for (uint pin = 21; pin <= 40; ++pin) pio_gpio_init(pio_hi, pin);
    pio_sm_set_consecutive_pindirs(pio_hi, sm_hi, 21, 20, false);

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

    // ---------- PIO0 SM1: GP2..GP19 low window on reads ----------
    const uint sm_read_lo = 1;
    uint off_read_lo = pio_add_program(pio_lo, &snes_capture_read_low_program);
    pio_sm_config c_read_lo = snes_capture_read_low_program_get_default_config(off_read_lo);
    sm_config_set_in_pins(&c_read_lo, PIN_DATA_BASE);
    sm_config_set_in_shift(&c_read_lo, true, false, 32);
    sm_config_set_fifo_join(&c_read_lo, PIO_FIFO_JOIN_RX);
    int init_read_lo_rc = pio_sm_init(pio_lo, sm_read_lo, off_read_lo, &c_read_lo);

    // ---------- PIO1 SM1: GP21..GP40 high window on reads ----------
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

    printf("READY. v2.2 keeps READ-REPAIR and uses SAFE-WINDOW AUTO-ROM fingerprint capture.\n");
    printf("Use INFO, WRAMSEL, ROMFP, ROMFPCLEAR, CHEESE, BANKS, WMSTATE, DEBUG, READ, READSNES, HEX, DUMPBIN, RBIN or SNAP.\n\n");
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
    arm_dma_channel(dma_lo, &dc_lo, low_samples[0], &pio_lo->rxf[sm_lo]);
    arm_dma_channel(dma_hi, &dc_hi, high_samples[0], &pio_hi->rxf[sm_hi]);
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
    arm_dma_channel(dma_read_lo, &dc_read_lo, read_low_samples[0], &pio_lo->rxf[sm_read_lo]);
    arm_dma_channel(dma_read_hi, &dc_read_hi, read_high_samples[0], &pio_hi->rxf[sm_read_hi]);
    dma_start_channel_mask((1u << dma_read_lo) | (1u << dma_read_hi));
    pio_sm_set_enabled(pio_hi, sm_read_hi, true);
    pio_sm_set_enabled(pio_lo, sm_read_lo, true);
    pio_sm_set_enabled(pio_rd, sm_rd_trigger, true);

    uint write_buf = 0;
    uint read_buf = 0;

    while (true) {
        // v1.6: when a pair fills, switch DMA to the other buffer immediately.
        // PIO remains enabled, so its FIFO only has to cover the few register writes
        // needed to rearm DMA instead of the entire software decoding loop.
        // atomic SNAP sees the freshest software mirror available at that boundary.
        if (dma_remaining(dma_lo) == 0 && dma_remaining(dma_hi) == 0) {
            uint done = write_buf;
            uint next = done ^ 1u;

            // Rearm first. Do not stop/restart/clear the PIO state machines.
            dma_channel_set_write_addr(dma_lo, low_samples[next], false);
            dma_channel_set_trans_count(dma_lo, SAMPLE_COUNT, false);
            dma_channel_set_write_addr(dma_hi, high_samples[next], false);
            dma_channel_set_trans_count(dma_hi, SAMPLE_COUNT, false);
            dma_start_channel_mask((1u << dma_lo) | (1u << dma_hi));
            write_buf = next;

            // Decode the completed buffer while hardware fills the alternate one.
            process_write_batch(low_samples[done], high_samples[done]);
        }

        if (dma_remaining(dma_read_lo) == 0 && dma_remaining(dma_read_hi) == 0) {
            uint done = read_buf;
            uint next = done ^ 1u;

            dma_channel_set_write_addr(dma_read_lo, read_low_samples[next], false);
            dma_channel_set_trans_count(dma_read_lo, SAMPLE_COUNT, false);
            dma_channel_set_write_addr(dma_read_hi, read_high_samples[next], false);
            dma_channel_set_trans_count(dma_read_hi, SAMPLE_COUNT, false);
            dma_start_channel_mask((1u << dma_read_lo) | (1u << dma_read_hi));
            read_buf = next;

            process_read_batch(read_low_samples[done], read_high_samples[done]);
        }

        poll_serial_commands();

        tight_loop_contents();
    }

}

#!/usr/bin/env python3
"""
RP2350B SNES -> usb2snes/RA2Snes bridge v2.2 WRAMSEL + READ-REPAIR + AUTO-ROM SAFE-WINDOW.

Design:
- AUTO-ROM identifies the physical cartridge by comparing safe-window A-bus ROM reads
  against .sfc/.smc reference ROMs in --rom-dir. --rom remains an optional manual override.
- live achievement memory comes from the RP2350B WRAM mirror.
- firmware v2.2 keeps the proven GP39=/WRAMSEL + READ-REPAIR path unchanged and
  fingerprints a conservative cartridge-ROM address window. GP38=/ROMSEL remains
  diagnostic instead of being required for AUTO-ROM.
- HYBRID-POLL starts a fresh atomic SNAP when either a quiet GetAddress gap is
  detected, the first request signature of the prior cycle repeats, or a maximum
  snapshot age is reached. This prevents a continuously busy RA client from
  reusing one stale snapshot forever.
- the gap timer starts only after the previous GetAddress response was sent, so
  firmware SNAP time is not counted as RA idle time.
"""
import argparse
import asyncio
import hashlib
import json
import threading
import time
from pathlib import Path

import serial
import websockets

USB2SNES_WRAM_BASE = 0xF50000
WRAM_SIZE = 128 * 1024
CHEESE_OFFSETS = (0x1558, 0x155C)
DEVICE_NAME = "RP2350B SNES Universal RA Bridge v2.2 AUTO-ROM SAFE-WINDOW"
CONFIG_YML = (
    "EnableCheats: false\n"
    "EnableIngameSavestate: 0\n"
    "EnableIngameHook: false\n"
    "SGBEnableState: false\n"
    "SGBEnableIngameHook: false\n"
).encode("utf-8")

ROM_EXTENSIONS = {".sfc", ".smc"}
AUTO_MIN_UNIQUE = 48
AUTO_MIN_MATCHES = 40
AUTO_MIN_SCORE = 0.90
AUTO_MIN_MARGIN = 0.08


def cartridge_bytes(path: Path) -> bytes:
    raw = path.read_bytes()
    return raw[512:] if (len(raw) % 1024) == 512 else raw


def mirror_rom_offset(address: int, size: int):
    """Mirror a linear ROM address into an arbitrary SNES ROM size."""
    if size <= 0:
        return None
    address = int(address)
    if address < size:
        return address
    base = 0
    mask = 1 << 23
    while address >= size:
        while mask and not (address & mask):
            mask >>= 1
        if not mask:
            return address % size
        address -= mask
        if size > mask:
            size -= mask
            base += mask
        mask >>= 1
    return base + address


def bus_to_rom_offset(address: int, mode: str, rom_size: int):
    """Translate a cartridge A-bus access to a candidate ROM-file offset."""
    bank = (address >> 16) & 0xFF
    addr = address & 0xFFFF

    # Only conventional cartridge ROM windows are mapped. Keeping this check
    # makes noisy samples harmless instead of forcing them into a mapper.
    in_low_banks = (bank <= 0x3F) or (0x80 <= bank <= 0xBF)
    in_full_banks = (0x40 <= bank <= 0x7D) or (0xC0 <= bank <= 0xFF)
    if not ((in_low_banks and addr >= 0x8000) or in_full_banks):
        return None

    if mode == "LOROM":
        linear = ((bank & 0x7F) << 15) | (addr & 0x7FFF)
    elif mode == "HIROM":
        linear = ((bank & 0x3F) << 16) | addr
    elif mode == "EXHIROM":
        linear = ((bank & 0x3F) << 16) | addr
        if (bank & 0x80) == 0:
            linear += 0x400000
    else:
        return None
    return mirror_rom_offset(linear, rom_size)


def normalize_fingerprint(records, max_unique=768, data_lag=0):
    """Collapse observations to one majority byte per address, optionally compensating a small
    constant address/data pipeline lag. data_lag=+1 pairs address[i] with data[i+1]."""
    counts = {}
    order = []
    n = len(records)
    for i, (address, _data) in enumerate(records):
        j = i + int(data_lag)
        if j < 0 or j >= n:
            continue
        data = records[j][1]
        if address not in counts:
            counts[address] = {}
            order.append(address)
        d = counts[address]
        d[data] = d.get(data, 0) + 1

    out = []
    for address in order:
        vals = counts[address]
        data, votes = max(vals.items(), key=lambda kv: kv[1])
        total = sum(vals.values())
        if total > 2 and votes / total < 0.67:
            continue
        out.append((address, data))
        if len(out) >= max_unique:
            break
    return out


def build_lagged_fingerprints(records):
    # Lag 0 is the expected path. Small alternatives make AUTO-ROM resilient to a
    # constant one/few-record skew between the two independently DMA'd PIO streams.
    return [(lag, normalize_fingerprint(records, data_lag=lag)) for lag in range(-3, 4)]


def score_rom_candidate(path: Path, lagged_records):
    rom = cartridge_bytes(path)
    if not rom:
        return None
    best = None
    for lag, records in lagged_records:
        for mode in ("LOROM", "HIROM", "EXHIROM"):
            matches = 0
            usable = 0
            for address, data in records:
                off = bus_to_rom_offset(address, mode, len(rom))
                if off is None or off < 0 or off >= len(rom):
                    continue
                usable += 1
                if rom[off] == data:
                    matches += 1
            score = (matches / usable) if usable else 0.0
            result = {
                "path": path,
                "mode": mode,
                "lag": lag,
                "matches": matches,
                "usable": usable,
                "score": score,
                "md5": hashlib.md5(rom).hexdigest(),
                "size": len(rom),
            }
            if best is None or (score, matches, usable, -abs(lag)) > (best["score"], best["matches"], best["usable"], -abs(best["lag"])):
                best = result
    return best


def rank_rom_candidates(paths, lagged_records):
    ranked = []
    for path in paths:
        try:
            result = score_rom_candidate(path, lagged_records)
        except OSError as e:
            print(f"[AUTO-ROM] ignoring unreadable file {path.name}: {e}", flush=True)
            continue
        if result is not None:
            ranked.append(result)
    ranked.sort(key=lambda r: (r["score"], r["matches"], r["usable"], -abs(r["lag"])), reverse=True)
    return ranked


def auto_result_is_confident(ranked):
    if not ranked:
        return False, "no candidates"
    top = ranked[0]
    if top["usable"] < AUTO_MIN_UNIQUE:
        return False, f"only {top['usable']} usable unique samples"
    if top["matches"] < AUTO_MIN_MATCHES:
        return False, f"only {top['matches']} matching samples"
    if top["score"] < AUTO_MIN_SCORE:
        return False, f"best score {top['score']:.1%} is below {AUTO_MIN_SCORE:.0%}"

    # Identical dump duplicates are harmless: either file presents the same game.
    second_distinct = next((r for r in ranked[1:] if r["md5"] != top["md5"]), None)
    if second_distinct is not None:
        margin = top["score"] - second_distinct["score"]
        if margin < AUTO_MIN_MARGIN and top["score"] < 0.995:
            return False, f"top-two confidence margin is only {margin:.1%}"
    return True, "confident"


class SerialWRAM:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baudrate=baud, timeout=2.0, write_timeout=2.0)
        self.ser.dtr = True
        self.ser.rts = True
        self.lock = threading.Lock()
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def _send_line(self, line: str):
        self.ser.write((line.rstrip() + "\n").encode("ascii"))
        self.ser.flush()

    def _readline_until(self, prefix: bytes, timeout: float = 2.0) -> bytes:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.ser.readline()
            if line.startswith(prefix):
                return line.strip()
        raise TimeoutError(f"serial timeout waiting for {prefix!r}")

    def _read_exact(self, length: int, label: str) -> bytes:
        data = bytearray()
        while len(data) < length:
            chunk = self.ser.read(length - len(data))
            if not chunk:
                raise TimeoutError(f"{label} timeout ({len(data)}/{length})")
            data.extend(chunk)
        return bytes(data)

    def ping(self):
        with self.lock:
            # Debug text would corrupt machine-readable binary framing.
            self.ser.reset_input_buffer()
            self._send_line("DEBUG OFF")
            try:
                self._readline_until(b"OK DEBUG OFF", 0.75)
            except TimeoutError:
                pass
            self.ser.reset_input_buffer()
            self._send_line("PING")
            line = self._readline_until(b"PONG", 2.0)
            return line == b"PONG"

    def read_range(self, offset: int, length: int):
        """Fallback v1.2-compatible single-range read."""
        if not (0 <= offset < WRAM_SIZE and 1 <= length <= 0x1000 and offset + length <= WRAM_SIZE):
            raise ValueError("WRAM range out of bounds")

        with self.lock:
            self._send_line(f"RBIN {offset:X} {length:X}")
            header = self._readline_until(b"RBIN1 ", 2.0).decode("ascii", errors="replace")
            parts = header.split()
            if len(parts) != 3:
                raise RuntimeError(f"bad RBIN header: {header!r}")
            got_len = int(parts[1])
            unknown = int(parts[2])
            if got_len != length:
                raise RuntimeError(f"RBIN length mismatch: {got_len} != {length}")

            data = self._read_exact(length, "RBIN payload")
            self._readline_until(b"END RBIN1", 1.0)
            return data, unknown

    def snapshot_ranges(self, ranges):
        """
        Ask firmware v1.3 to atomically copy all requested WRAM ranges.

        ranges: [(offset, length), ...]
        returns: (data, known_flags, unknown_count, batch_stamp)
        where data/known_flags are concatenated in the same range order.
        """
        if not ranges:
            raise ValueError("SNAP requires at least one range")
        total = 0
        for off, length in ranges:
            if not (0 <= off < WRAM_SIZE and length >= 1 and off + length <= WRAM_SIZE):
                raise ValueError(f"SNAP range out of bounds: {off:X}:{length:X}")
            total += length
        if len(ranges) > 256 or total > 4096:
            raise ValueError(f"SNAP request too large: {len(ranges)} ranges, {total} bytes")

        args = " ".join(f"{off:X}:{length:X}" for off, length in ranges)
        with self.lock:
            self._send_line("SNAP " + args)
            header = self._readline_until(b"SNAP1 ", 2.0).decode("ascii", errors="replace")
            parts = header.split()
            if len(parts) != 5:
                raise RuntimeError(f"bad SNAP header: {header!r}")
            got_ranges = int(parts[1])
            got_total = int(parts[2])
            unknown = int(parts[3])
            batch_stamp = int(parts[4])
            if got_ranges != len(ranges):
                raise RuntimeError(f"SNAP range-count mismatch: {got_ranges} != {len(ranges)}")
            if got_total != total:
                raise RuntimeError(f"SNAP length mismatch: {got_total} != {total}")

            data = self._read_exact(total, "SNAP data")
            known = self._read_exact(total, "SNAP known-map")
            self._readline_until(b"END SNAP1", 1.0)
            if any(v not in (0, 1) for v in known):
                raise RuntimeError("SNAP known-map contains invalid byte")
            if sum(1 for v in known if not v) != unknown:
                raise RuntimeError("SNAP unknown-count does not match known-map")
            return data, known, unknown, batch_stamp

    def wramsel_stats(self):
        """Read the compact v1.8 firmware qualification counters."""
        with self.lock:
            self._send_line("WRAMSEL")
            line = self._readline_until(b"WRAMSEL ", 1.0).decode("ascii", errors="replace")

        stats = {}
        for token in line.split()[1:]:
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            stats[key] = value

        required = {"write_ok", "write_reject", "read_seed", "read_ignored", "known"}
        if not required.issubset(stats):
            raise RuntimeError(f"bad WRAMSEL response: {line!r}")
        return stats

    def cheese_stats(self):
        """Read v2.0 targeted $1558/$155C diagnostic values/counters."""
        with self.lock:
            self._send_line("CHEESE")
            line = self._readline_until(b"CHEESE ", 1.0).decode("ascii", errors="replace")
        stats = {}
        for token in line.split()[1:]:
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            stats[key] = value
        required = {"A", "A_known", "A_w", "A_r", "A_repair", "B", "B_known", "B_w", "B_r", "B_repair"}
        if not required.issubset(stats):
            raise RuntimeError(f"bad CHEESE response: {line!r}")
        return stats


    def rom_fingerprint(self):
        """Fetch v2.2 safe-window address/data observations plus /ROMSEL diagnostics."""
        with self.lock:
            self._send_line("ROMFP")
            header = self._readline_until(b"ROMFP2 ", 2.0).decode("ascii", errors="replace")
            parts = header.split()
            if len(parts) != 5:
                raise RuntimeError(f"bad ROMFP2 header: {header!r}")
            count = int(parts[1])
            window_reads = int(parts[2])
            window_romsel_low = int(parts[3])
            total_romsel_low = int(parts[4])
            if count < 0 or count > 1024:
                raise RuntimeError(f"ROMFP count out of range: {count}")
            payload = self._read_exact(count * 4, "ROMFP2 payload")
            self._readline_until(b"END ROMFP2", 1.0)

        records = []
        for i in range(count):
            p = i * 4
            address = payload[p] | (payload[p + 1] << 8) | (payload[p + 2] << 16)
            data = payload[p + 3]
            records.append((address, data))
        stats = {
            "window_reads": window_reads,
            "window_romsel_low": window_romsel_low,
            "total_romsel_low": total_romsel_low,
        }
        return records, stats

    def clear_rom_fingerprint(self):
        with self.lock:
            self._send_line("ROMFPCLEAR")
            line = self._readline_until(b"OK ROMFP cleared", 1.0)
            return line.startswith(b"OK ROMFP cleared")


class BridgeServer:
    def __init__(self, serial_wram: SerialWRAM, rom_path: Path,
                 verbose=False, trace_ra=False, trace_all=False, trace_snapshots=False,
                 poll_gap_ms=8.0, max_snapshot_age_ms=50.0, trace_polls=False, trace_wramsel=False, trace_cheese=False):
        self.serial_wram = serial_wram
        self.rom_path = rom_path
        self.rom_file = rom_path.read_bytes()
        self.rom_mem = cartridge_bytes(rom_path)
        self.rom_virtual_path = "/games/" + rom_path.name
        self.verbose = verbose
        self.trace_ra = trace_ra
        self.trace_all = trace_all
        self.trace_snapshots = trace_snapshots
        self.poll_gap_ms = float(poll_gap_ms)
        self.max_snapshot_age_ms = float(max_snapshot_age_ms)
        self.trace_polls = trace_polls
        self.trace_wramsel = trace_wramsel
        self.trace_cheese = trace_cheese
        self.clients = 0

        # Address-set learning and coherent snapshot cache.
        self.ra_ranges = {}          # ordered set: (off, size) -> None
        self.saw_ra_getaddress = False
        self.snapshot_values = {}    # off -> byte
        self.snapshot_known = {}     # off -> bool
        self.snapshot_generation = 0
        self.snapshot_batch_stamp = 0
        self.snapshot_active = False
        self._printed_snapshot_activation = False
        self._last_snapshot_summary = None
        self._printed_fully_known = False

        # Change-only trace state.
        self._trace_last = {}
        self._unknown_warn_last = {}

        # v1.8 TIMED-POLL:
        # A new polling pass is detected by a quiet interval between the previous
        # GetAddress response and the next GetAddress request. The timestamp is set
        # only AFTER ws.send(payload), so SNAP/serial cost is never counted as idle.
        self._last_getaddress_done = None
        self._poll_number = 0
        self._timed_poll_refreshes = 0
        self._info_requests = 0
        self._printed_timed_poll = False
        self._snapshot_completed_at = None
        self._cycle_first_signature = None
        self._cycle_request_count = 0
        self._cheese_ra_seen = {off: False for off in CHEESE_OFFSETS}
        self._cheese_last_stats = None

    def log(self, *a):
        if self.verbose:
            print("[bridge]", *a, flush=True)

    def _register_ra_range(self, off: int, size: int):
        key = (off, size)
        if key not in self.ra_ranges:
            self.ra_ranges[key] = None
            if self.verbose:
                print(f"[bridge] learned RA WRAM range +{off:05X} len={size:X}", flush=True)
            return True
        return False

    def _merged_ra_ranges(self):
        """Merge overlapping/adjacent requested ranges while preserving all bytes."""
        ranges = sorted(self.ra_ranges.keys())
        if not ranges:
            return []
        merged = []
        cur_off, cur_len = ranges[0]
        cur_end = cur_off + cur_len
        for off, length in ranges[1:]:
            end = off + length
            if off <= cur_end:  # overlap or adjacent
                if end > cur_end:
                    cur_end = end
            else:
                merged.append((cur_off, cur_end - cur_off))
                cur_off, cur_end = off, end
        merged.append((cur_off, cur_end - cur_off))
        return merged

    async def refresh_snapshot(self, reason="manual"):
        """Take one firmware-side coherent image of every RA-requested WRAM byte."""
        if not self.saw_ra_getaddress or not self.ra_ranges:
            return
        ranges = self._merged_ra_ranges()
        total = sum(length for _, length in ranges)
        if len(ranges) > 256 or total > 4096:
            # Do not silently pretend a chunked read is atomic across chunks.
            if self.snapshot_active:
                print("[snapshot] disabled: RA working set exceeds firmware atomic SNAP limits", flush=True)
            self.snapshot_active = False
            self.snapshot_values.clear()
            self.snapshot_known.clear()
            return

        try:
            data, known, unknown, batch_stamp = await asyncio.to_thread(
                self.serial_wram.snapshot_ranges, ranges
            )
        except Exception as e:
            self.snapshot_active = False
            self.snapshot_values.clear()
            self.snapshot_known.clear()
            print(f"[snapshot] ERROR: {e}; falling back to individual RBIN reads", flush=True)
            return

        values = {}
        flags = {}
        pos = 0
        for off, length in ranges:
            for i in range(length):
                values[off + i] = data[pos]
                flags[off + i] = bool(known[pos])
                pos += 1

        self.snapshot_values = values
        self.snapshot_known = flags
        self.snapshot_generation += 1
        self.snapshot_batch_stamp = batch_stamp
        self.snapshot_active = True
        self._snapshot_completed_at = time.monotonic()

        if reason in ("timed-poll", "hybrid-poll"):
            self._timed_poll_refreshes += 1
            if not self._printed_timed_poll:
                print(
                    f"HYBRID-POLL snapshot mode ACTIVE: gap > {self.poll_gap_ms:g} ms, "
                    f"cycle-signature repeat, or age > {self.max_snapshot_age_ms:g} ms refreshes WRAM.",
                    flush=True,
                )
                self._printed_timed_poll = True

        if not self._printed_snapshot_activation:
            print(
                f"Atomic RA snapshot ACTIVE: {len(ranges)} merged ranges / {total} bytes. "
                "All covered GetAddress groups reuse the current mirror instant until a hybrid poll boundary.",
                flush=True,
            )
            self._printed_snapshot_activation = True

        summary = (len(ranges), total, unknown)
        if self.trace_snapshots and (self.trace_all or summary != self._last_snapshot_summary):
            print(
                f"[SNAP] gen={self.snapshot_generation} batch={batch_stamp} "
                f"ranges={len(ranges)} bytes={total} unknown={unknown}",
                flush=True,
            )

        if unknown == 0 and not self._printed_fully_known:
            print(
                f"RA WRAM working set FULLY KNOWN: {len(ranges)} merged ranges / {total} bytes.",
                flush=True,
            )
            self._printed_fully_known = True

        self._last_snapshot_summary = summary

    def _snapshot_read(self, off: int, size: int):
        if not self.snapshot_active:
            return None
        if any((off + i) not in self.snapshot_values for i in range(size)):
            return None
        data = bytes(self.snapshot_values[off + i] for i in range(size))
        known_flags = [self.snapshot_known.get(off + i, False) for i in range(size)]
        unknown = sum(1 for k in known_flags if not k)
        return data, unknown, self.snapshot_generation

    def _trace_memory(self, off: int, size: int, data: bytes, unknown: int, source: str, generation=None):
        if not self.trace_ra:
            return
        key = (off, size)
        state = (bytes(data), unknown)
        if self.trace_all or self._trace_last.get(key) != state:
            hexdata = bytes(data).hex(" ").upper()
            if unknown == 0:
                status = "KNOWN"
            elif unknown == size:
                status = "UNKNOWN"
            else:
                status = f"PARTIAL {size-unknown}/{size}"
            suffix = f" snap={generation}" if source == "SNAP" and generation is not None else " direct"
            print(f"[RA-MEM] +{off:05X} len={size:X} data={hexdata} {status}{suffix}", flush=True)
        self._trace_last[key] = state

    def _prelearn_getaddress_ranges(self, obj):
        """Register all WRAM ranges in one GetAddress message before snapshotting."""
        operands = obj.get("Operands") or []
        space = (obj.get("Space") or "SNES").upper()
        if space != "SNES" or len(operands) % 2:
            return False

        learned = False
        for i in range(0, len(operands), 2):
            address = int(str(operands[i]), 16)
            size = int(str(operands[i + 1]), 16)
            if size <= 0:
                continue
            if USB2SNES_WRAM_BASE <= address < USB2SNES_WRAM_BASE + WRAM_SIZE:
                off = address - USB2SNES_WRAM_BASE
                size = min(size, WRAM_SIZE - off)
                if size > 0:
                    learned = self._register_ra_range(off, size) or learned
                    self.saw_ra_getaddress = True
                    for target in CHEESE_OFFSETS:
                        if off <= target < off + size and not self._cheese_ra_seen[target]:
                            self._cheese_ra_seen[target] = True
                            print(f"[CHEESE-RA] RA requested WRAM +{target:05X} inside +{off:05X} len={size:X}", flush=True)
        return learned

    def _getaddress_signature(self, obj):
        """Stable signature for one RA memory-request group."""
        space = (obj.get("Space") or "SNES").upper()
        operands = tuple(str(v).upper() for v in (obj.get("Operands") or []))
        return (space, operands)

    async def _print_wramsel_stats(self):
        if not self.trace_wramsel:
            return
        try:
            stats = await asyncio.to_thread(self.serial_wram.wramsel_stats)
            print(
                "[WRAMSEL] "
                f"write_ok={stats['write_ok']} write_reject={stats['write_reject']} "
                f"read_seed={stats['read_seed']} read_same={stats.get('read_same', '?')} "
                f"read_repair={stats.get('read_repair', '?')} read_ignored={stats['read_ignored']} "
                f"known={stats['known']}",
                flush=True,
            )
        except Exception as e:
            print(f"[WRAMSEL] ERROR: {e}", flush=True)

    async def _print_cheese_stats(self, force=False):
        if not self.trace_cheese:
            return
        try:
            stats = await asyncio.to_thread(self.serial_wram.cheese_stats)
            state = tuple(stats[k] for k in (
                "A", "A_known", "A_w", "A_r", "A_repair",
                "B", "B_known", "B_w", "B_r", "B_repair"
            ))
            if force or state != self._cheese_last_stats:
                print(
                    "[CHEESE] "
                    f"+01558={stats['A']} known={stats['A_known']} W={stats['A_w']} R={stats['A_r']} repair={stats['A_repair']} | "
                    f"+0155C={stats['B']} known={stats['B_known']} W={stats['B_w']} R={stats['B_r']} repair={stats['B_repair']}",
                    flush=True,
                )
            self._cheese_last_stats = state
        except Exception as e:
            print(f"[CHEESE] ERROR: {e}", flush=True)

    async def prepare_getaddress_poll(self, obj):
        """
        v1.9 HYBRID-POLL boundary detector.

        A fresh SNAP is taken when any of these happens:
        1) first GetAddress request;
        2) idle gap after the previous response exceeds --poll-gap-ms;
        3) the first GetAddress signature of the current cycle appears again;
        4) the active snapshot exceeds --max-snapshot-age-ms.

        Rule 3 fixes the v1.8 failure mode where RA2Snes can issue consecutive
        polling cycles with no >8 ms quiet gap, which otherwise freezes the
        snapshot forever. Rule 4 is a fail-safe for clients whose grouping/order
        changes dynamically.
        """
        learned = self._prelearn_getaddress_ranges(obj)
        sig = self._getaddress_signature(obj)
        now = time.monotonic()

        gap_ms = None if self._last_getaddress_done is None else (now - self._last_getaddress_done) * 1000.0
        age_ms = None if self._snapshot_completed_at is None else (now - self._snapshot_completed_at) * 1000.0

        first = self._last_getaddress_done is None or self._cycle_first_signature is None
        gap_boundary = gap_ms is not None and gap_ms > self.poll_gap_ms
        repeat_boundary = (
            not first
            and self._cycle_request_count > 0
            and sig == self._cycle_first_signature
        )
        age_boundary = (
            not first
            and age_ms is not None
            and age_ms > self.max_snapshot_age_ms
        )

        new_poll = first or gap_boundary or repeat_boundary or age_boundary
        if new_poll:
            if first:
                boundary_reason = "FIRST"
            elif gap_boundary:
                boundary_reason = "GAP"
            elif repeat_boundary:
                boundary_reason = "REPEAT"
            else:
                boundary_reason = "AGE"

            self._poll_number += 1
            self._cycle_first_signature = sig
            self._cycle_request_count = 0
            before_gen = self.snapshot_generation
            await self.refresh_snapshot(reason="hybrid-poll")
            snap_text = str(self.snapshot_generation) if self.snapshot_generation != before_gen else "none"

            if self.trace_polls:
                gap_text = "FIRST" if gap_ms is None else f"{gap_ms:.3f}ms"
                age_text = "n/a" if age_ms is None else f"{age_ms:.3f}ms"
                print(
                    f"[POLL {self._poll_number}] reason={boundary_reason} gap={gap_text} "
                    f"age={age_text} threshold={self.poll_gap_ms:g}ms/"
                    f"{self.max_snapshot_age_ms:g}ms snap={snap_text}",
                    flush=True,
                )
            await self._print_wramsel_stats()
            await self._print_cheese_stats(force=(self._poll_number == 1))
        elif learned and self.verbose:
            print(
                "[bridge] new WRAM range learned inside current poll; "
                "it will join the next atomic HYBRID-POLL SNAP",
                flush=True,
            )

        self._cycle_request_count += 1

    def mark_getaddress_done(self):
        # Deliberately called only after ws.send(payload) completes.
        self._last_getaddress_done = time.monotonic()

    async def read_snes_space(self, address: int, size: int) -> bytes:
        if size <= 0:
            return b""

        if USB2SNES_WRAM_BASE <= address < USB2SNES_WRAM_BASE + WRAM_SIZE:
            off = address - USB2SNES_WRAM_BASE
            if off + size > WRAM_SIZE:
                size = WRAM_SIZE - off
            self._register_ra_range(off, size)
            self.saw_ra_getaddress = True

            snap = self._snapshot_read(off, size)
            if snap is not None:
                data, unknown, generation = snap
                self._trace_memory(off, size, data, unknown, "SNAP", generation)
                return data

            # First discovery poll or a newly appearing range: direct fallback only
            # until the next timed poll creates a complete working-set snapshot.
            data, unknown = await asyncio.to_thread(self.serial_wram.read_range, off, size)
            warn_key = (off, size)
            if self.verbose and unknown and self._unknown_warn_last.get(warn_key) != unknown:
                print(f"[bridge] WARNING: {unknown}/{size} requested WRAM bytes are not yet known at {off:05X}", flush=True)
            self._unknown_warn_last[warn_key] = unknown
            self._trace_memory(off, size, data, unknown, "DIRECT")
            return data

        if 0 <= address < len(self.rom_mem):
            data = self.rom_mem[address:address + size]
            if len(data) < size:
                data += bytes(size - len(data))
            return data

        return bytes(size)

    async def handle_get_address(self, obj):
        operands = obj.get("Operands") or []
        space = (obj.get("Space") or "SNES").upper()
        if len(operands) % 2:
            raise ValueError("GetAddress operands must be address/size pairs")

        out = bytearray()
        for i in range(0, len(operands), 2):
            address = int(str(operands[i]), 16)
            size = int(str(operands[i + 1]), 16)
            if space == "SNES":
                out.extend(await self.read_snes_space(address, size))
            else:
                out.extend(bytes(size))
        return bytes(out)

    async def send_file(self, ws, path: str):
        if path == "/sd2snes/config.yml":
            payload = CONFIG_YML
        else:
            payload = self.rom_file

        await ws.send(json.dumps({"Results": [format(len(payload), "X")]}))
        for off in range(0, len(payload), 1024):
            await ws.send(payload[off:off + 1024])

    async def handler(self, ws):
        self.clients += 1
        print(f"RA client connected ({self.clients})", flush=True)
        try:
            async for message in ws:
                if isinstance(message, bytes):
                    self.log("ignoring binary client message", len(message))
                    continue

                try:
                    obj = json.loads(message)
                    opcode = obj.get("Opcode", "")
                    operands = obj.get("Operands") or []
                    self.log("<-", opcode, operands, obj.get("Space"))

                    if opcode == "DeviceList":
                        await ws.send(json.dumps({"Results": [DEVICE_NAME]}))
                    elif opcode == "Attach":
                        pass
                    elif opcode == "Name":
                        pass
                    elif opcode == "Info":
                        # v1.8 intentionally does not refresh on Info. TIMED-POLL owns
                        # the snapshot boundary so protocol chatter cannot split a burst.
                        self._info_requests += 1
                        await ws.send(json.dumps({
                            "Results": [
                                "1.11.0",
                                "SD2SNES",
                                self.rom_virtual_path,
                                "NO_CONTROL_CMD",
                                "NO_ROM_WRITE"
                            ]
                        }))
                    elif opcode == "AppVersion":
                        await ws.send(json.dumps({"Results": ["0.7.20"]}))
                    elif opcode == "GetFile":
                        path = str(operands[0]) if operands else ""
                        await self.send_file(ws, path)
                    elif opcode == "GetAddress":
                        await self.prepare_getaddress_poll(obj)
                        payload = await self.handle_get_address(obj)
                        await ws.send(payload)
                        self.mark_getaddress_done()
                    elif opcode == "PutAddress":
                        self.log("PutAddress ignored (passive bridge)")
                    elif opcode in ("Reset", "Menu", "Boot", "PutFile", "PutIPS"):
                        self.log(opcode, "ignored (passive bridge)")
                    else:
                        self.log("unsupported opcode", opcode)
                        await ws.send(json.dumps({"Results": []}))
                except Exception as e:
                    print(f"Request error: {e}", flush=True)
                    if self.verbose:
                        import traceback
                        traceback.print_exc()
        finally:
            self.clients -= 1
            print("RA client disconnected", flush=True)


async def detect_reference_rom(sw: SerialWRAM, rom_dir: Path):
    rom_dir = rom_dir.expanduser().resolve()
    if not rom_dir.is_dir():
        raise FileNotFoundError(f"AUTO-ROM folder does not exist: {rom_dir}")

    candidates = sorted(
        (p for p in rom_dir.rglob("*") if p.is_file() and p.suffix.lower() in ROM_EXTENSIONS),
        key=lambda p: str(p).lower(),
    )
    if not candidates:
        raise RuntimeError(
            f"AUTO-ROM found no .sfc/.smc reference files in {rom_dir}. "
            "Put your RetroAchievements-compatible ROM references in that folder."
        )

    print(f"AUTO-ROM: {len(candidates)} reference ROM(s) found in {rom_dir}", flush=True)
    # Clear twice with a short settling interval. This also discards a DMA batch
    # that may have been queued before a reconnect or cartridge reset.
    await asyncio.to_thread(sw.clear_rom_fingerprint)
    await asyncio.sleep(0.10)
    await asyncio.to_thread(sw.clear_rom_fingerprint)
    print("AUTO-ROM: fingerprint cleared; collecting physical cartridge reads from the safe ROM window...", flush=True)

    last_count = -1
    unchanged_full_rounds = 0
    collect_started = time.monotonic()
    while True:
        records, fpstats = await asyncio.to_thread(sw.rom_fingerprint)
        lagged = build_lagged_fingerprints(records)
        normalized = next(fp for lag, fp in lagged if lag == 0)

        if len(normalized) < AUTO_MIN_UNIQUE:
            if len(records) != last_count:
                print(
                    f"AUTO-ROM: collecting safe-window reads: {len(records)} records / "
                    f"{len(normalized)} unique (need at least {AUTO_MIN_UNIQUE}); "
                    f"/ROMSEL low in {fpstats['window_romsel_low']}/{fpstats['window_reads']} window reads...",
                    flush=True,
                )
            if len(records) >= 1024:
                raise RuntimeError(
                    f"AUTO-ROM captured 1024 samples but only {len(normalized)} unique safe-window addresses. "
                    "Reset/unpause the game and retry; no uncertain ROM will be selected."
                )
            if time.monotonic() - collect_started > 10.0:
                raise RuntimeError(
                    f"AUTO-ROM did not collect enough ROM reads in 10 seconds ({len(records)} records / {len(normalized)} unique). "
                    "Reset or unpause the SNES and retry."
                )
            last_count = len(records)
            await asyncio.sleep(0.20)
            continue

        # Re-rank when the fingerprint grows. 1024 records usually fill almost
        # instantly on a running SNES, so this is normally a one-pass operation.
        ranked = await asyncio.to_thread(rank_rom_candidates, candidates, lagged)
        ok, reason = auto_result_is_confident(ranked)
        if ranked:
            top = ranked[0]
            print(
                f"AUTO-ROM candidate: {top['path'].name} [{top['mode']}] "
                f"lag={top['lag']:+d} {top['matches']}/{top['usable']} = {top['score']:.1%}",
                flush=True,
            )
        if ok:
            top = ranked[0]
            print(
                f"AUTO-ROM DETECTED: {top['path'].name} | mapper={top['mode']} | lag={top['lag']:+d} | "
                f"confidence={top['score']:.1%} | MD5={top['md5']}",
                flush=True,
            )
            return top["path"], top, ranked[:5]

        if len(records) >= 1024:
            unchanged_full_rounds += 1
            if unchanged_full_rounds >= 2:
                details = "; ".join(
                    f"{r['path'].name} [{r['mode']}] lag={r['lag']:+d} {r['score']:.1%} ({r['matches']}/{r['usable']})"
                    for r in ranked[:5]
                ) or "no scorable candidates"
                raise RuntimeError(
                    "AUTO-ROM could not identify the cartridge safely: " + reason + ". Top candidates: " + details
                )
        else:
            unchanged_full_rounds = 0

        last_count = len(records)
        await asyncio.sleep(0.25)


async def amain(args):
    rom = Path(args.rom).expanduser().resolve() if args.rom else None
    if rom is not None and not rom.is_file():
        raise FileNotFoundError(rom)

    sw = SerialWRAM(args.port, args.baud)
    try:
        if not sw.ping():
            raise RuntimeError("RP2350B did not answer PING. Close PuTTY/PowerShell serial and verify COM port.")

        # v2.2 requires the proven READ-REPAIR path plus ROMFP2 safe-window framing.
        try:
            fw_stats = await asyncio.to_thread(sw.wramsel_stats)
            if "read_repair" not in fw_stats:
                raise RuntimeError("WRAMSEL response has no read_repair counter")
            await asyncio.to_thread(sw.rom_fingerprint)
        except Exception as e:
            raise RuntimeError(
                "Firmware v2.2 WRAMSEL+READ-REPAIR+AUTO-ROM SAFE-WINDOW not detected. "
                "Flash/rebuild the v2.2 firmware first. "
                f"Details: {e}"
            ) from e

        # Verify atomic SNAP framing before RA2Snes connects.
        try:
            await asyncio.to_thread(sw.snapshot_ranges, [(0, 1)])
        except Exception as e:
            raise RuntimeError(
                "Firmware does not support the required atomic SNAP framing. Flash/rebuild firmware v1.8. "
                f"Details: {e}"
            ) from e

        detection = None
        if rom is None:
            rom, detection, _top = await detect_reference_rom(sw, Path(args.rom_dir))
        else:
            print(f"AUTO-ROM bypassed by manual --rom: {rom.name}", flush=True)

        bridge = BridgeServer(
            sw, rom, args.verbose, args.trace_ra, args.trace_all, args.trace_snapshots,
            args.poll_gap_ms, args.max_snapshot_age_ms, args.trace_polls, args.trace_wramsel, args.trace_cheese
        )
        md5 = hashlib.md5(bridge.rom_mem).hexdigest()
        print("RP2350B serial: OK (v2.2 WRAMSEL+READ-REPAIR + SAFE-WINDOW AUTO-ROM verified)")
        print(
            "WRAMSEL firmware counters: "
            f"write_ok={fw_stats['write_ok']} write_reject={fw_stats['write_reject']} "
            f"read_seed={fw_stats['read_seed']} read_same={fw_stats.get('read_same','?')} read_repair={fw_stats.get('read_repair','?')} read_ignored={fw_stats['read_ignored']} "
            f"known={fw_stats['known']}"
        )
        print(f"ROM presented to RA2Snes: {rom.name} ({len(bridge.rom_mem)} bytes cartridge data, MD5 {md5})")
        print("NOTE: selected reference ROM controls RA game identification; live achievement memory still comes only from physical SNES WRAM.")
        print("usb2snes compatibility server: ws://127.0.0.1:23074")
        print("Do NOT run QUsb2Snes at the same time (same TCP port).")
        print("Use SOFTCORE for these tests. Keep the RP powered before the SNES.")
        print(
            f"v2.2 HYBRID-POLL: refresh on gap > {args.poll_gap_ms:g} ms, repeated cycle-start signature, "
            f"or snapshot age > {args.max_snapshot_age_ms:g} ms."
        )
        print("HYBRID-POLL prevents stale snapshots; firmware READ-REPAIR now also heals missed write samples.")
        print("Firmware v2.2 is REQUIRED for AUTO-ROM. GP38=/ROMSEL is diagnostic; GP39=/WRAMSEL and GP40=A10 stay unchanged.")

        async with websockets.serve(bridge.handler, "127.0.0.1", args.ws_port, max_size=None):
            await asyncio.Future()
    finally:
        sw.close()


def main():
    ap = argparse.ArgumentParser(description="RP2350B SNES -> usb2snes/RA2Snes bridge v2.2 AUTO-ROM SAFE-WINDOW + WRAMSEL + READ-REPAIR")
    ap.add_argument("--port", required=True, help="RP2350B serial port, e.g. COM7")
    ap.add_argument("--rom", help="manual reference ROM override; omit to use AUTO-ROM")
    ap.add_argument("--rom-dir", default="ROMs", help="folder recursively scanned for .sfc/.smc references (default: ROMs)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--ws-port", type=int, default=23074)
    ap.add_argument("-v", "--verbose", action="store_true", help="show protocol requests (very noisy)")
    ap.add_argument("--trace-ra", action="store_true", help="print RA-requested WRAM values only when data/status changes")
    ap.add_argument("--trace-all", action="store_true", help="with --trace-ra, print every RA WRAM read")
    ap.add_argument("--trace-snapshots", action="store_true", help="print snapshot summary when range/unknown totals change")
    ap.add_argument("--poll-gap-ms", type=float, default=8.0,
                    help="idle gap in milliseconds that starts a new atomic poll (default: 8.0)")
    ap.add_argument("--max-snapshot-age-ms", type=float, default=50.0,
                    help="force a fresh snapshot when the current one exceeds this age (default: 50.0)")
    ap.add_argument("--trace-polls", action="store_true", help="print [POLL n] reason/gap/age/snapshot diagnostics")
    ap.add_argument("--trace-wramsel", action="store_true", help="print firmware WRAMSEL/READ-REPAIR counters at each detected poll")
    ap.add_argument("--trace-cheese", action="store_true", help="track Tom & Jerry WRAM +01558/+0155C and show whether RA requests them")
    args = ap.parse_args()
    if args.poll_gap_ms <= 0:
        ap.error("--poll-gap-ms must be > 0")
    if args.max_snapshot_age_ms <= 0:
        ap.error("--max-snapshot-age-ms must be > 0")
    try:
        asyncio.run(amain(args))
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\nERROR: {e}", flush=True)
        print("No ROM was selected. The bridge did not start with an uncertain match.", flush=True)


if __name__ == "__main__":
    main()

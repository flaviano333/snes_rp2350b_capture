#!/usr/bin/env python3
"""
RP2350B v1.3 atomic-snapshot WRAM -> usb2snes compatibility bridge for RA2Snes.

Important design point:
- --rom is still the manually supplied ROM used by RA2Snes for game identification.
- The RP2350B supplies live WRAM data from the physical SNES.
- The bridge cannot yet verify that the physical cartridge matches --rom.

v1.3 adds coherent multi-range snapshots. After the first RA polling cycle learns
which WRAM ranges RA2Snes requests, each following RA cycle is served from one
firmware SNAP command. This prevents the different GetAddress groups in one RA
cycle from being read from the RP2350B at different moments.
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

# Tom & Jerry final-USA RA -> physical BETA/Nightfall WRAM translation.
# Generated automatically from the two ROM builds.
from tomjerry_address_map import (
    EXPLICIT as BETA_WRAM_EXPLICIT,
    RANGE_RULES as BETA_WRAM_RANGE_RULES,
    translate_wram_offset,
)

DEVICE_NAME = "RP2350B RA Bridge v1.6 TomJerry Exact-Map"
CONFIG_YML = (
    "EnableCheats: false\n"
    "EnableIngameSavestate: 0\n"
    "EnableIngameHook: false\n"
    "SGBEnableState: false\n"
    "SGBEnableIngameHook: false\n"
).encode("utf-8")


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


class BridgeServer:
    def __init__(self, serial_wram: SerialWRAM, rom_path: Path,
                 verbose=False, trace_ra=False, trace_all=False, trace_snapshots=False,
                 beta_translate=False):
        self.serial_wram = serial_wram
        self.rom_path = rom_path
        self.rom_file = rom_path.read_bytes()
        self.rom_mem = self.rom_file[512:] if (len(self.rom_file) % 1024) == 512 else self.rom_file
        self.rom_virtual_path = "/games/" + rom_path.name
        self.verbose = verbose
        self.trace_ra = trace_ra
        self.trace_all = trace_all
        self.trace_snapshots = trace_snapshots
        self.beta_translate = beta_translate
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
        self._last_unknown_ra = None

        # Change-only trace state.
        self._trace_last = {}
        self._unknown_warn_last = {}

    def log(self, *a):
        if self.verbose:
            print("[bridge]", *a, flush=True)

    def _translate_wram_byte(self, ra_off: int) -> int:
        if not self.beta_translate:
            return ra_off
        return translate_wram_offset(ra_off)

    def _translate_range_bytes(self, off: int, size: int):
        return [self._translate_wram_byte(off + i) for i in range(size)]

    def _physical_snapshot_ranges(self):
        phys = set()
        for off, size in self.ra_ranges.keys():
            phys.update(self._translate_range_bytes(off, size))
        if not phys:
            return []

        ordered = sorted(phys)
        out = []
        start = prev = ordered[0]
        for p in ordered[1:]:
            if p == prev + 1:
                prev = p
                continue
            out.append((start, prev - start + 1))
            start = prev = p
        out.append((start, prev - start + 1))
        return out

    async def _read_translated_direct(self, off: int, size: int):
        phys = self._translate_range_bytes(off, size)
        data_out = bytearray()
        unknown_total = 0
        i = 0
        while i < size:
            start_phys = phys[i]
            j = i + 1
            while j < size and phys[j] == phys[j - 1] + 1:
                j += 1
            length = j - i
            data, unknown = await asyncio.to_thread(
                self.serial_wram.read_range, start_phys, length
            )
            data_out.extend(data)
            unknown_total += unknown
            i = j
        return bytes(data_out), unknown_total

    def _register_ra_range(self, off: int, size: int):
        key = (off, size)
        if key not in self.ra_ranges:
            self.ra_ranges[key] = None
            if self.verbose:
                print(f"[bridge] learned RA WRAM range +{off:05X} len={size:X}", flush=True)

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

    async def refresh_snapshot(self):
        """Take one firmware-side coherent image of every RA-requested WRAM byte."""
        if not self.saw_ra_getaddress or not self.ra_ranges:
            return
        ranges = self._physical_snapshot_ranges() if self.beta_translate else self._merged_ra_ranges()
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

        # Translate physical unknown flags back to the RA-requested addresses.
        unknown_ra = []
        for ra_off, ra_len in self.ra_ranges.keys():
            for i in range(ra_len):
                ra_byte = ra_off + i
                phys = self._translate_wram_byte(ra_byte)
                if not flags.get(phys, False):
                    unknown_ra.append(ra_byte)
        unknown_ra = tuple(sorted(set(unknown_ra)))
        if unknown_ra != self._last_unknown_ra:
            if unknown_ra:
                shown = " ".join(f"+{x:05X}" for x in unknown_ra)
                print(f"[UNKNOWN-RA] {len(unknown_ra)} byte(s) still unseen by the passive mirror: {shown}", flush=True)
            else:
                print("[UNKNOWN-RA] none; every RA-requested byte has been observed at least once.", flush=True)
            self._last_unknown_ra = unknown_ra

        self.snapshot_values = values
        self.snapshot_known = flags
        self.snapshot_generation += 1
        self.snapshot_batch_stamp = batch_stamp
        self.snapshot_active = True

        if not self._printed_snapshot_activation:
            print(
                f"Atomic RA snapshot ACTIVE: {len(ranges)} merged ranges / {total} bytes. "
                "All GetAddress groups after each RA Info poll use the same mirror instant.",
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
        phys = self._translate_range_bytes(off, size)
        if any(p not in self.snapshot_values for p in phys):
            return None
        data = bytes(self.snapshot_values[p] for p in phys)
        known_flags = [self.snapshot_known.get(p, False) for p in phys]
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
            if self.beta_translate:
                phys = self._translate_range_bytes(off, size)
                if phys == list(range(off, off + size)):
                    map_text = ""
                elif len(phys) == 1:
                    map_text = f" map=+{phys[0]:05X}"
                else:
                    map_text = " map=" + ",".join(f"+{p:05X}" for p in phys)
            else:
                map_text = ""
            print(f"[RA-MEM] +{off:05X} len={size:X} data={hexdata} {status}{suffix}{map_text}", flush=True)
        self._trace_last[key] = state

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

            # First discovery cycle or a newly appearing range: fallback only until
            # the next Info boundary creates a complete working-set snapshot.
            if self.beta_translate:
                data, unknown = await self._read_translated_direct(off, size)
            else:
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
                        # In RA2Snes polling, Info is the boundary immediately before
                        # the GetAddress groups. Refresh once here so all groups in this
                        # cycle are served from the exact same firmware-side snapshot.
                        await self.refresh_snapshot()
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
                        payload = await self.handle_get_address(obj)
                        await ws.send(payload)
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


async def amain(args):
    rom = Path(args.rom).expanduser().resolve()
    if not rom.is_file():
        raise FileNotFoundError(rom)

    sw = SerialWRAM(args.port, args.baud)
    try:
        if not sw.ping():
            raise RuntimeError("RP2350B did not answer PING. Flash v1.3 firmware and close PuTTY/PowerShell serial first.")

        # Verify the new firmware command before RA2Snes connects.
        try:
            await asyncio.to_thread(sw.snapshot_ranges, [(0, 1)])
        except Exception as e:
            raise RuntimeError(
                "Firmware does not appear to support v1.3 SNAP framing. Flash the v1.3 UF2 first. "
                f"Details: {e}"
            ) from e

        bridge = BridgeServer(
            sw, rom, args.verbose, args.trace_ra, args.trace_all, args.trace_snapshots,
            args.beta_translate
        )
        md5 = hashlib.md5(bridge.rom_mem).hexdigest()
        print("RP2350B serial: OK (atomic SNAP verified)")
        print(f"ROM presented to RA2Snes: {rom.name} ({len(bridge.rom_mem)} bytes cartridge data, MD5 {md5})")
        print("NOTE: --rom controls RA game identification; live achievement memory comes from the physical SNES WRAM mirror.")
        print("usb2snes compatibility server: ws://127.0.0.1:23074")
        print("Do NOT run QUsb2Snes at the same time (same TCP port).")
        print("Use SOFTCORE for these tests. Keep the RP powered before the SNES.")
        print("First RA polling cycle learns addresses; coherent atomic snapshots begin on the following Info/GetAddress cycle.")
        print("With corrected A23 on GP18, direct $7E/$7F WRAM can now populate addresses above $1FFF.")
        if args.beta_translate:
            print("BETA WRAM TRANSLATION: ENABLED")
            print("RA2Snes identifies the final USA ROM; final-build WRAM requests are redirected to beta-build equivalents.")
            print(f"Exact map: {len(BETA_WRAM_EXPLICIT)} explicit WRAM references (no broad range translation)")
            if BETA_WRAM_RANGE_RULES:
                for start, end, delta in BETA_WRAM_RANGE_RULES:
                    sign = "+" if delta >= 0 else ""
                    print(f"  range +{start:05X}..+{end:05X}: beta = final {sign}{delta}")
            print("  addresses without explicit evidence remain unchanged")
        else:
            print("BETA WRAM TRANSLATION: disabled")

        async with websockets.serve(bridge.handler, "127.0.0.1", args.ws_port, max_size=None):
            await asyncio.Future()
    finally:
        sw.close()


def main():
    ap = argparse.ArgumentParser(description="RP2350B SNES -> usb2snes/RA2Snes bridge v1.6 with Tom & Jerry BETA exact map")
    ap.add_argument("--port", required=True, help="RP2350B serial port, e.g. COM7")
    ap.add_argument("--rom", required=True, help="ROM file used ONLY for RA2Snes game identification")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--ws-port", type=int, default=23074)
    ap.add_argument("-v", "--verbose", action="store_true", help="show protocol requests (very noisy)")
    ap.add_argument("--trace-ra", action="store_true", help="print RA-requested WRAM values only when data/status changes")
    ap.add_argument("--trace-all", action="store_true", help="with --trace-ra, print every RA WRAM read")
    ap.add_argument("--trace-snapshots", action="store_true", help="print snapshot summary when range/unknown totals change")
    ap.add_argument("--beta-translate", action="store_true",
                    help="use the generated final-USA -> BETA Tom & Jerry WRAM auto-map")
    args = ap.parse_args()
    try:
        asyncio.run(amain(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

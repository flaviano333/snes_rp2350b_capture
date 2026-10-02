#!/usr/bin/env python3
"""
SNES RP2350B raw A-bus mapper v1.4

Captures raw GPIO states from firmware command DEBUG RAWREAD and brute-forces
A14..A21 across the remaining physical GPIOs GP26..GP33.

No soldering changes are made. The result is a proposed ADDRESS_GPIO[] remap
for the firmware.
"""

from __future__ import annotations

import argparse
import collections
import csv
import hashlib
import itertools
import re
import time
from pathlib import Path

try:
    import serial
    from serial import SerialException
except ImportError:  # lets the offline analyzer functions be imported for testing
    serial = None
    class SerialException(Exception):
        pass

RAW_RE = re.compile(
    r"^RAWREAD\s+L=([0-9A-Fa-f]{2})\s+H=([0-9A-Fa-f]{1,5})\s+D=([0-9A-Fa-f]{2})(?:\s+A=([0-9A-Fa-f]{6}))?"
)
DEBUG_STATUS_RE = re.compile(r"^DEBUG\s+mode=([A-Z]+)\s+remaining=(\d+)", re.I)

# Lines already physically checked or strongly established in the prototype.
KNOWN_ADDRESS_GPIO = {
    0: 12, 1: 11, 2: 10, 3: 13, 4: 14, 5: 16, 6: 15, 7: 17,
    8: 19, 9: 21, 10: 22, 11: 23, 12: 24, 13: 25,
    22: 34, 23: 18,
}
UNKNOWN_LOGICAL_BITS = tuple(range(14, 22))
CANDIDATE_GPIOS = tuple(range(26, 34))
CURRENT_MAP = tuple(range(26, 34))  # A14->GP26 ... A21->GP33


def strip_copier_header(blob: bytes):
    if len(blob) % 1024 == 512:
        return blob[512:], 512
    return blob, 0


def u16le(b: bytes, off: int) -> int:
    if off + 1 >= len(b):
        return 0
    return b[off] | (b[off + 1] << 8)


def header_score(rom: bytes, base: int, mapper: str) -> int:
    if base < 0 or base + 0x40 > len(rom):
        return -999
    score = 0
    title = rom[base:base + 21]
    score += sum(1 for c in title if c == 0x20 or 0x21 <= c <= 0x7E)
    mode = rom[base + 0x15]
    if mapper == "lorom" and (mode & 1) == 0:
        score += 8
    if mapper == "hirom" and (mode & 1) == 1:
        score += 8
    comp = u16le(rom, base + 0x1C)
    csum = u16le(rom, base + 0x1E)
    if (csum ^ comp) == 0xFFFF and csum not in (0, 0xFFFF):
        score += 12
    if u16le(rom, base + 0x3C) >= 0x8000:
        score += 4
    return score


def detect_mapper(rom: bytes):
    lo = header_score(rom, 0x7FC0, "lorom")
    hi = header_score(rom, 0xFFC0, "hirom")
    return ("lorom" if lo >= hi else "hirom"), lo, hi


def rom_title(rom: bytes, mapper: str) -> str:
    base = 0x7FC0 if mapper == "lorom" else 0xFFC0
    if base + 21 > len(rom):
        return "(header unavailable)"
    return rom[base:base + 21].decode("ascii", errors="replace").rstrip(" \x00")


def cpu_to_rom_offset(address: int, mapper: str, rom_size: int):
    bank = (address >> 16) & 0xFF
    offs = address & 0xFFFF
    if bank in (0x7E, 0x7F) or rom_size <= 0:
        return None

    if mapper == "lorom":
        # Conventional LoROM ROM windows. Banks 40-7D/C0-FF expose ROM across
        # the full bank; in other banks only $8000-$FFFF is ROM.
        full_bank_rom = (0x40 <= bank <= 0x7D) or (0xC0 <= bank <= 0xFF)
        if offs < 0x8000 and not full_bank_rom:
            return None
        raw = ((bank & 0x7F) << 15) | (offs & 0x7FFF)
        return raw % rom_size

    if mapper == "hirom":
        if 0xC0 <= bank <= 0xFF:
            return (((bank & 0x3F) << 16) | offs) % rom_size
        if ((0x00 <= bank <= 0x3F) or (0x80 <= bank <= 0xBF)) and offs >= 0x8000:
            return (((bank & 0x3F) << 16) | offs) % rom_size
        return None

    return None


def raw_gpio_bit(raw_l: int, raw_h: int, gpio: int) -> int:
    if 10 <= gpio <= 17:
        return (raw_l >> (gpio - 10)) & 1
    if 18 <= gpio <= 34:
        return (raw_h >> (gpio - 18)) & 1
    raise ValueError(f"GPIO {gpio} is outside raw address capture windows")


def known_base_address(raw_l: int, raw_h: int) -> int:
    addr = 0
    for logical_bit, gpio in KNOWN_ADDRESS_GPIO.items():
        addr |= raw_gpio_bit(raw_l, raw_h, gpio) << logical_bit
    return addr


def physical_pattern_26_33(raw_h: int) -> int:
    # q bit 0 corresponds to GP26, bit 7 to GP33.
    p = 0
    for j, gpio in enumerate(CANDIDATE_GPIOS):
        p |= raw_gpio_bit(0, raw_h, gpio) << j
    return p


def permute_pattern(p: int, mapping: tuple[int, ...]) -> int:
    # mapping[i] is the physical GPIO carrying logical A(14+i).
    q = 0
    for i, gpio in enumerate(mapping):
        q |= ((p >> (gpio - 26)) & 1) << i
    return q


def open_serial(port: str, baud: int):
    if serial is None:
        raise RuntimeError("pyserial is not installed. Run: python -m pip install -r requirements.txt")
    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = 0.10
    s.write_timeout = 1.0
    s.dtr = True
    s.rts = True
    s.open()
    time.sleep(0.15)
    s.reset_input_buffer()
    return s


def send_line(ser, text: str):
    ser.write((text.rstrip() + "\n").encode("ascii"))
    ser.flush()


def arm_raw(ser, chunk: int):
    send_line(ser, f"DEBUG RAWREAD {chunk}")


def collect_raw(args):
    observations: dict[tuple[int, int], collections.Counter[int]] = {}
    ser = None
    rearms = 0
    reconnects = 0
    raw_lines = 0
    start = time.monotonic()
    next_report = start + args.report_every
    last_raw = start
    waiting_status = False
    last_watchdog_query = 0.0

    def restart():
        nonlocal ser, rearms, last_raw
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        ser = open_serial(args.port, args.baud)
        send_line(ser, "DEBUG OFF")
        time.sleep(0.03)
        ser.reset_input_buffer()
        arm_raw(ser, args.chunk)
        rearms += 1
        last_raw = time.monotonic()

    restart()
    print(f"Capturing RAWREAD for {args.seconds:.0f}s. Play normally while this runs. Ctrl+C to stop early.\n")

    try:
        while time.monotonic() - start < args.seconds:
            try:
                raw = ser.readline()
            except SerialException as e:
                reconnects += 1
                print(f"[USB] {e}; reopening COM...", flush=True)
                time.sleep(0.4)
                while True:
                    try:
                        restart()
                        break
                    except (SerialException, OSError):
                        time.sleep(0.5)
                continue

            if raw:
                line = raw.decode("ascii", errors="replace").strip()
                m = RAW_RE.match(line)
                if m:
                    l = int(m.group(1), 16)
                    h = int(m.group(2), 16) & 0x1FFFF
                    d = int(m.group(3), 16)
                    observations.setdefault((l, h), collections.Counter())[d] += 1
                    raw_lines += 1
                    last_raw = time.monotonic()
                    continue

                if line.startswith("DEBUG DONE"):
                    arm_raw(ser, args.chunk)
                    rearms += 1
                    last_raw = time.monotonic()
                    waiting_status = False
                    continue

                sm = DEBUG_STATUS_RE.match(line)
                if sm:
                    waiting_status = False
                    mode = sm.group(1).upper()
                    remaining = int(sm.group(2))
                    if mode == "OFF" or remaining == 0:
                        arm_raw(ser, args.chunk)
                        rearms += 1
                        last_raw = time.monotonic()
                    continue

                if line.startswith("ERR DEBUG") and "RAWREAD" in line.upper():
                    raise RuntimeError("Firmware does not support DEBUG RAWREAD. Flash the v1.4 UF2 first.")

            now = time.monotonic()
            if now - last_raw >= args.watchdog and not waiting_status:
                send_line(ser, "DEBUG")
                waiting_status = True
                last_watchdog_query = now
            if waiting_status and now - last_watchdog_query >= 0.5:
                arm_raw(ser, args.chunk)
                rearms += 1
                last_raw = now
                waiting_status = False

            if now >= next_report:
                stable = 0
                for ctr in observations.values():
                    hits = sum(ctr.values())
                    maj = ctr.most_common(1)[0][1]
                    if hits >= args.min_hits and maj / hits >= args.purity:
                        stable += 1
                print(
                    f"[CAPTURE] {now-start:5.1f}s raw={raw_lines} unique_physical={len(observations)} "
                    f"stable={stable} rearms={rearms} reconnects={reconnects}",
                    flush=True,
                )
                next_report = now + args.report_every
    except KeyboardInterrupt:
        pass
    finally:
        if ser is not None:
            try:
                send_line(ser, "DEBUG OFF")
            except Exception:
                pass
            try:
                ser.close()
            except Exception:
                pass

    return observations, raw_lines, rearms, reconnects


def stable_samples(observations, min_hits: int, purity: float):
    out = []
    for (l, h), ctr in observations.items():
        hits = sum(ctr.values())
        data, majority_hits = ctr.most_common(1)[0]
        pur = majority_hits / hits
        if hits >= min_hits and pur >= purity:
            out.append((l, h, data, hits, pur))
    return out


def analyze(samples, rom: bytes, mapper: str, top_n: int):
    # Aggregate match/comparable contributions by physical GP26..GP33 bit pattern
    # and by every possible logical A14..A21 bit pattern. This makes the final
    # 8! brute force cheap and deterministic.
    match = [[0] * 256 for _ in range(256)]
    comparable = [[0] * 256 for _ in range(256)]

    for l, h, data, _hits, _pur in samples:
        base = known_base_address(l, h)
        p = physical_pattern_26_33(h)
        for q in range(256):
            address = base | (q << 14)
            off = cpu_to_rom_offset(address, mapper, len(rom))
            if off is None:
                continue
            comparable[p][q] += 1
            if rom[off] == data:
                match[p][q] += 1

    active_p = [p for p in range(256) if any(comparable[p])]

    # Cache transformed q for every permutation and active p, then score.
    results = []
    for perm in itertools.permutations(CANDIDATE_GPIOS):
        m = 0
        c = 0
        for p in active_p:
            q = permute_pattern(p, perm)
            m += match[p][q]
            c += comparable[p][q]
        rate = (m / c) if c else 0.0
        # Primary ranking is number of matching UNIQUE physical signatures.
        # This prevents a candidate from looking good simply by classifying few
        # samples as ROM. Match rate breaks ties.
        results.append((m, rate, c, perm))

    results.sort(key=lambda x: (x[0], x[1], x[2]), reverse=True)

    def score_mapping(mapping):
        m = c = 0
        for p in active_p:
            q = permute_pattern(p, mapping)
            m += match[p][q]
            c += comparable[p][q]
        return m, (m / c if c else 0.0), c

    current = score_mapping(CURRENT_MAP)
    top = results[:top_n]
    if results:
        bm, br, bc, _ = results[0]
        # Exact-score equivalence group. With small ROMs, high CPU address bits can
        # legitimately be invisible because the cartridge mirrors them.
        best_group = [r for r in results if r[0] == bm and r[2] == bc and abs(r[1] - br) < 1e-15]
    else:
        best_group = []
    return top, current, best_group


def save_raw_csv(path: Path, observations):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["raw_L_GP10_17", "raw_H_GP18_34", "majority_data", "hits", "majority_hits", "purity", "observed_values"])
        for (l, h), ctr in sorted(observations.items()):
            hits = sum(ctr.values())
            data, maj = ctr.most_common(1)[0]
            vals = " ".join(f"{v:02X}:{n}" for v, n in sorted(ctr.items()))
            w.writerow([f"{l:02X}", f"{h:05X}", f"{data:02X}", hits, maj, f"{maj/hits:.6f}", vals])


def format_map(perm: tuple[int, ...]) -> str:
    return " ".join(f"A{14+i}->GP{gpio}" for i, gpio in enumerate(perm))


def full_initializer(perm: tuple[int, ...]):
    arr = [None] * 24
    for bit, gpio in KNOWN_ADDRESS_GPIO.items():
        arr[bit] = gpio
    for i, gpio in enumerate(perm):
        arr[14 + i] = gpio
    return arr


def main():
    ap = argparse.ArgumentParser(description="Infer SNES A14-A21 physical GPIO order from raw bus captures")
    ap.add_argument("--port", required=True)
    ap.add_argument("--rom", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--chunk", type=int, default=128)
    ap.add_argument("--watchdog", type=float, default=1.0)
    ap.add_argument("--report-every", type=float, default=5.0)
    ap.add_argument("--min-hits", type=int, default=2)
    ap.add_argument("--purity", type=float, default=0.98)
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--mapper", choices=["auto", "lorom", "hirom"], default="auto")
    ap.add_argument("--csv", default="raw_mapper_capture.csv", help="save aggregated raw physical samples")
    args = ap.parse_args()

    if not 1 <= args.chunk <= 4096:
        ap.error("--chunk must be 1..4096")
    if not 0.5 <= args.purity <= 1.0:
        ap.error("--purity must be 0.5..1.0")

    rom_path = Path(args.rom).expanduser().resolve()
    blob = rom_path.read_bytes()
    rom, header_bytes = strip_copier_header(blob)
    if args.mapper == "auto":
        mapper, lo_score, hi_score = detect_mapper(rom)
    else:
        mapper, lo_score, hi_score = args.mapper, 0, 0

    print(f"Reference: {rom_path.name}")
    print(f"ROM data: {len(rom)} bytes" + (f" (stripped {header_bytes}-byte header)" if header_bytes else ""))
    print(f"MD5: {hashlib.md5(rom).hexdigest()}")
    if args.mapper == "auto":
        print(f"Mapper: {mapper.upper()} (LoROM={lo_score}, HiROM={hi_score})")
    else:
        print(f"Mapper: {mapper.upper()} (forced)")
    print(f"Title: {rom_title(rom, mapper)}")
    print("Unknown block under test: A14..A21 <-> GP26..GP33 (8! = 40320 permutations)\n")

    observations, raw_lines, rearms, reconnects = collect_raw(args)
    samples = stable_samples(observations, args.min_hits, args.purity)

    print("\n--- CAPTURE SUMMARY ---")
    print(f"raw lines: {raw_lines}")
    print(f"unique physical addresses: {len(observations)}")
    print(f"stable samples used (hits>={args.min_hits}, purity>={args.purity:.2f}): {len(samples)}")
    print(f"rearms: {rearms}   reconnects: {reconnects}")
    if args.csv:
        csv_path = Path(args.csv).expanduser().resolve()
        save_raw_csv(csv_path, observations)
        print(f"raw capture CSV: {csv_path}")

    if len(samples) < 100:
        print("\nNot enough stable samples to infer a reliable map. Play longer and try again.")
        return

    print("\nTesting all 40,320 A14-A21 permutations...")
    top, current, best_group = analyze(samples, rom, mapper, args.top)

    print("\n--- CURRENT FIRMWARE ASSUMPTION ---")
    print(format_map(CURRENT_MAP))
    print(f"matches={current[0]} comparable={current[2]} rate={current[1]*100:.3f}%")

    print("\n--- BEST CANDIDATES ---")
    for rank, (m, rate, c, perm) in enumerate(top, 1):
        marker = "  <== BEST" if rank == 1 else ""
        print(f"#{rank}: matches={m} comparable={c} rate={rate*100:.3f}%{marker}")
        print("    " + format_map(perm))

    best_m, best_rate, best_c, best_perm = top[0]
    second_m = top[1][0] if len(top) > 1 else -1
    improvement = best_m - current[0]
    margin = best_m - second_m

    best_m, best_rate, best_c, best_perm = top[0]
    second_m = top[1][0] if len(top) > 1 else -1
    improvement = best_m - current[0]
    margin = best_m - second_m

    print("\n--- IDENTIFIABILITY ---")
    print(f"exactly tied best mappings: {len(best_group)}")
    if len(best_group) > 1:
        perms = [r[3] for r in best_group]
        print("Some address lines cannot be uniquely distinguished by this ROM/capture.")
        print("Consensus across all tied best mappings:")
        for i in range(8):
            vals = sorted({perm[i] for perm in perms})
            if len(vals) == 1:
                print(f"  A{14+i} -> GP{vals[0]}  FIXED")
            else:
                print(f"  A{14+i} -> " + "/".join(f"GP{x}" for x in vals) + "  AMBIGUOUS")

    unique_enough = len(best_group) == 1 and margin > 10
    if unique_enough:
        print("\n--- RECOMMENDED ADDRESS_GPIO[] ---")
        arr = full_initializer(best_perm)
        print("static const uint8_t ADDRESS_GPIO[24] = {")
        print("    " + ", ".join(str(x) for x in arr[:8]) + ",  // A0..A7")
        print("    " + ", ".join(str(x) for x in arr[8:16]) + ",  // A8..A15")
        print("    " + ", ".join(str(x) for x in arr[16:24]) + "   // A16..A23")
        print("};")
    else:
        print("\nNo automatic full remap is recommended yet because the best mapping is not unique enough.")

    print("\n--- INTERPRETATION ---")
    print(f"best-vs-current additional matching signatures: {improvement:+d}")
    print(f"best-vs-second-place match margin: {margin:+d}")
    if best_perm == CURRENT_MAP and len(best_group) == 1:
        print("The current A14-A21 order is already the unique best of all 40,320 permutations.")
        print("If the ROM match is still poor, the cause is elsewhere (different ROM, other address lines, capture pairing, etc.).")
    elif unique_enough and improvement > max(50, int(0.05 * max(1, current[0]))):
        print("A different A14-A21 wiring order is strongly preferred by the captured ROM data.")
        print("Do NOT resolder; use the printed ADDRESS_GPIO[] as the next software remap candidate.")
    elif len(best_group) > 1:
        print("The capture found an equivalence class rather than one exact wiring map.")
        print("This is expected when the reference ROM is too small to decode some high address bits (mirrors).")
        print("The FIXED lines above can still be remapped in software; ambiguous lines need another constraint/test.")
    else:
        print("A different mapping ranked first, but the evidence is not decisive yet.")
        print("Capture a longer/more varied gameplay segment before changing the firmware map.")



if __name__ == "__main__":
    main()

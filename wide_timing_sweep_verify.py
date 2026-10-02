#!/usr/bin/env python3
"""
SNES RP2350B v1.7 wide timing sweep verifier.

Requires v1.7 wide timing-sweep firmware.
For every /RD event the firmware sends 16 snapshots of GP0..GP31 taken at
successive time slots. This script compares each slot independently against
the reference 512 KiB LoROM and shows which sampling window is most coherent.
"""

import argparse
import collections
import hashlib
import re
import time
from pathlib import Path

import serial
from serial import SerialException

SWEEP_RE = re.compile(
    r"^SWEEP"
    + r"(?:\s+([0-9A-Fa-f]{8}))" * 16
    + r"$"
)

# Logical A0..A19 -> measured physical GPIO.
ADDR_GPIO = [
    12,11,10,13,14,16,15,17,
    19,21,22,23,24,25,26,27,
    28,29,30,31
]

SLOTS = 16

def strip_header(blob):
    if len(blob) % 1024 == 512:
        return blob[512:], 512
    return blob, 0

def bit(raw, gpio):
    return (raw >> gpio) & 1

def decode(raw):
    data = (raw >> 2) & 0xFF
    a = 0
    for logical, gpio in enumerate(ADDR_GPIO):
        a |= bit(raw, gpio) << logical

    phi2 = bit(raw, 0)
    wr_n = bit(raw, 1)

    # Candidate upper-half LoROM read. Physical 512 KiB offset depends on
    # A0..A14 and A16..A19; A20+ select mirrors and are not needed for offset.
    if ((a >> 15) & 1) == 0:
        return data, a, None, phi2, wr_n

    off = (((a >> 16) & 0x0F) << 15) | (a & 0x7FFF)
    return data, a, off, phi2, wr_n

def open_serial(port, baud):
    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = 0.10
    s.write_timeout = 1.0
    s.dtr = True
    s.rts = True
    s.open()
    time.sleep(0.20)
    s.reset_input_buffer()
    return s

def send(s, cmd):
    s.write((cmd.rstrip() + "\n").encode("ascii"))
    s.flush()

class SlotStats:
    def __init__(self):
        self.by_offset = {}  # off -> Counter(data)
        self.events = 0
        self.event_matches = 0
        self.phi2_high = 0
        self.wr_high = 0

    def add(self, raw, rom):
        data, a, off, phi2, wr_n = decode(raw)
        self.phi2_high += phi2
        self.wr_high += wr_n
        if off is None or off >= len(rom):
            return
        self.events += 1
        if rom[off] == data:
            self.event_matches += 1
        self.by_offset.setdefault(off, collections.Counter())[data] += 1

    def summary(self, min_hits=2, min_purity=0.98):
        all_rows = []
        for off, ctr in self.by_offset.items():
            seen, maj = ctr.most_common(1)[0]
            hits = sum(ctr.values())
            purity = maj / hits
            all_rows.append((off, seen, hits, purity, seen == None))

        all_match = 0
        trusted_match = 0
        trusted_total = 0
        unstable = 0

        # expected byte is supplied externally in calc below
        return all_rows, all_match, trusted_match, trusted_total, unstable

def calc_slot(stats, rom, min_hits=2, min_purity=0.98):
    total = len(stats.by_offset)
    match = 0
    trusted = 0
    tmatch = 0
    unstable = 0

    mismatches = []
    for off, ctr in stats.by_offset.items():
        seen, maj = ctr.most_common(1)[0]
        hits = sum(ctr.values())
        purity = maj / hits
        if len(ctr) > 1:
            unstable += 1
        good = (seen == rom[off])
        match += int(good)
        if hits >= min_hits and purity >= min_purity:
            trusted += 1
            tmatch += int(good)
            if not good:
                mismatches.append((hits, purity, off, rom[off], seen))

    all_pct = 100.0 * match / total if total else 0.0
    tpct = 100.0 * tmatch / trusted if trusted else 0.0
    epct = 100.0 * stats.event_matches / stats.events if stats.events else 0.0
    return {
        "unique": total,
        "match": match,
        "all_pct": all_pct,
        "trusted": trusted,
        "tmatch": tmatch,
        "trusted_pct": tpct,
        "unstable": unstable,
        "event_pct": epct,
        "events": stats.events,
        "mismatches": sorted(mismatches, reverse=True),
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--rom", required=True)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--chunk", type=int, default=32,
                    help="/RD events per firmware burst (default 32)")
    ap.add_argument("--min-hits", type=int, default=2)
    ap.add_argument("--min-purity", type=float, default=0.98)
    args = ap.parse_args()

    rom_path = Path(args.rom).resolve()
    blob = rom_path.read_bytes()
    rom, stripped = strip_header(blob)

    print(f"Reference: {rom_path.name}")
    print(f"ROM data: {len(rom)} bytes" +
          (f" (stripped {stripped}-byte copier header)" if stripped else ""))
    print(f"MD5: {hashlib.md5(rom).hexdigest()}")
    print("Firmware required: v1.7 WIDE TIMING SWEEP")
    print("Sixteen slots are sampled after /RD assertion without waiting for PHI2.")
    print("Slots are ~53 ns apart at the default 150 MHz system clock; GP0 shows PHI2 state.\n")

    stats = [SlotStats() for _ in range(SLOTS)]
    ser = None
    rearms = 0
    reconnects = 0
    sweep_lines = 0
    start = time.monotonic()
    next_report = start + 5.0
    last_line = start

    def reopen():
        nonlocal ser, rearms, reconnects, last_line
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        while True:
            try:
                ser = open_serial(args.port, args.baud)
                break
            except (SerialException, OSError):
                reconnects += 1
                time.sleep(0.5)
        send(ser, "OFF")
        time.sleep(0.03)
        ser.reset_input_buffer()
        send(ser, f"SWEEPREAD {args.chunk}")
        rearms += 1
        last_line = time.monotonic()

    reopen()

    try:
        while time.monotonic() - start < args.seconds:
            try:
                rawline = ser.readline()
            except SerialException:
                reconnects += 1
                reopen()
                continue

            if rawline:
                line = rawline.decode("ascii", errors="replace").strip()
                m = SWEEP_RE.match(line)
                if m:
                    words = [int(x, 16) for x in m.groups()]
                    for i, word in enumerate(words):
                        stats[i].add(word, rom)
                    sweep_lines += 1
                    last_line = time.monotonic()
                    continue

                if line.startswith("SWEEP DONE"):
                    send(ser, f"SWEEPREAD {args.chunk}")
                    rearms += 1
                    last_line = time.monotonic()
                    continue

            now = time.monotonic()
            if now - last_line > 1.0:
                send(ser, f"SWEEPREAD {args.chunk}")
                rearms += 1
                last_line = now

            if now >= next_report:
                vals = [calc_slot(s, rom, args.min_hits, args.min_purity) for s in stats]
                best = max(range(SLOTS),
                           key=lambda i: (vals[i]["trusted_pct"], vals[i]["trusted"]))
                v = vals[best]
                print(
                    f"[LIVE] {now-start:5.1f}s events={sweep_lines} rearms={rearms} "
                    f"best=S{best} trusted={v['tmatch']}/{v['trusted']} "
                    f"{v['trusted_pct']:.3f}% event={v['event_pct']:.3f}%",
                    flush=True
                )
                next_report = now + 5.0

    except KeyboardInterrupt:
        pass
    finally:
        if ser is not None:
            try:
                send(ser, "OFF")
            except Exception:
                pass
            try:
                ser.close()
            except Exception:
                pass

    vals = [calc_slot(s, rom, args.min_hits, args.min_purity) for s in stats]

    print("\n--- FINAL TIMING SWEEP ---")
    print(f"events={sweep_lines} rearms={rearms} reconnects={reconnects}")
    print()
    print("slot | PHI2 high | unique all-match | trusted match | event match")
    print("-----+-----------+------------------+---------------+------------")
    for i, (s, v) in enumerate(zip(stats, vals)):
        phi2_pct = 100.0 * s.phi2_high / sweep_lines if sweep_lines else 0.0
        print(
            f"S{i:<3} | {phi2_pct:8.2f}% | "
            f"{v['match']:5d}/{v['unique']:<5d} {v['all_pct']:7.3f}% | "
            f"{v['tmatch']:5d}/{v['trusted']:<5d} {v['trusted_pct']:7.3f}% | "
            f"{v['event_pct']:7.3f}%"
        )

    eligible = [i for i,v in enumerate(vals) if v["trusted"] >= 100]
    if eligible:
        best = max(eligible, key=lambda i:(vals[i]["trusted_pct"], vals[i]["trusted"]))
    else:
        best = max(range(SLOTS), key=lambda i:(vals[i]["trusted_pct"], vals[i]["trusted"]))

    bv = vals[best]
    print(f"\nBEST SLOT: S{best}")
    print(f"trusted={bv['tmatch']}/{bv['trusted']} = {bv['trusted_pct']:.3f}%")
    print(f"event-weighted={bv['event_pct']:.3f}%")

    print("\nFirst trusted mismatches at best slot:")
    for hits, purity, off, expected, seen in bv["mismatches"][:16]:
        print(
            f"  ROM+{off:05X}: expected={expected:02X} seen={seen:02X} "
            f"hits={hits} purity={purity:.4f}"
        )

    print("\n--- INTERPRETATION ---")
    phi = [
        (100.0 * s.phi2_high / sweep_lines if sweep_lines else 0.0)
        for s in stats
    ]
    crossed = (max(phi) - min(phi)) >= 25.0

    if not crossed:
        print("WARNING: even this wider sweep did not clearly cross a PHI2 transition.")
        print("Do not use it to rule timing in or out; send the table to ChatGPT.")
    elif bv["trusted_pct"] >= 90.0:
        print("A timing window is dramatically better than the old ~50% result.")
        print("The coherent bus window has been found; use the BEST SLOT timing in the bridge.")
    elif bv["trusted_pct"] >= 70.0:
        print("Timing has a strong effect, but the best window is not yet clean enough.")
        print("A finer scan around the BEST SLOT is warranted.")
    else:
        print("The sweep crossed PHI2 but no slot became highly coherent.")
        print("That makes a simple timing-offset explanation much less likely.")
        print("Next priority: atomically retain enough upper bank context to exclude non-ROM reads,")
        print("then compare only unquestionably valid LoROM cycles.")

if __name__ == "__main__":
    main()

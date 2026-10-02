#!/usr/bin/env python3
"""
SNES RP2350B v1.8 /ROMSEL-qualified verifier.

Requires v1.8 firmware and ONE extra wire:
  SNES cartridge connector pin 49 (/ROMSEL) -> RP2350 GP36

The firmware only emits events where CPU /RD=0 and /ROMSEL=0, so every event is
a genuine cartridge-ROM read. This removes WRAM/MMIO/open-bus ambiguity before
comparing the physical cartridge against the reference .sfc.

For each 16-slot event, the script finds the first PHI2 falling edge and tests
address/data sample positions around it.
"""

import argparse
import collections
import csv
import hashlib
import re
import time
from pathlib import Path
import serial
from serial import SerialException

SLOTS = 16
SWEEP_RE = re.compile(r"^SWEEP" + r"(?:\s+([0-9A-Fa-f]{8}))" * SLOTS + r"$")

ADDR_GPIO = [
    12,11,10,13,14,16,15,17,
    19,21,22,23,24,25,26,27,
    28,29,30,31
]

ADDR_RELS = (-3, -2, -1, 0, 1)
DATA_RELS = (-2, -1, 0, 1, 2)

def strip_header(blob):
    if len(blob) % 1024 == 512:
        return blob[512:], 512
    return blob, 0

def bit(raw, gpio):
    return (raw >> gpio) & 1

def decode_address(raw):
    a = 0
    for logical, gpio in enumerate(ADDR_GPIO):
        a |= bit(raw, gpio) << logical
    return a

def decode_data(raw):
    return (raw >> 2) & 0xFF

def lorom_512k_offset(a):
    # For a 512 KiB LoROM, physical ROM A0..A18 are:
    # CPU A0..A14 + CPU A16..A19. CPU A15 is not part of the ROM offset.
    return (((a >> 16) & 0x0F) << 15) | (a & 0x7FFF)

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

class Combo:
    def __init__(self, ar, dr):
        self.ar = ar
        self.dr = dr
        self.by_offset = {}
        self.events = 0
        self.event_matches = 0

    def add(self, words, edge, rom):
        ai = edge + self.ar
        di = edge + self.dr
        if not (0 <= ai < SLOTS and 0 <= di < SLOTS):
            return

        a = decode_address(words[ai])
        data = decode_data(words[di])
        off = lorom_512k_offset(a)
        if off >= len(rom):
            return

        self.events += 1
        if rom[off] == data:
            self.event_matches += 1
        self.by_offset.setdefault(off, collections.Counter())[data] += 1

    def stats(self, rom, min_hits, min_purity):
        unique = len(self.by_offset)
        all_match = 0
        trusted = 0
        trusted_match = 0
        mismatches = []

        for off, ctr in self.by_offset.items():
            seen, maj = ctr.most_common(1)[0]
            hits = sum(ctr.values())
            purity = maj / hits
            good = (seen == rom[off])
            all_match += int(good)

            if hits >= min_hits and purity >= min_purity:
                trusted += 1
                trusted_match += int(good)
                if not good:
                    mismatches.append((hits, purity, off, rom[off], seen))

        return {
            "ar": self.ar, "dr": self.dr,
            "unique": unique,
            "all_match": all_match,
            "all_pct": (100.0 * all_match / unique if unique else 0.0),
            "trusted": trusted,
            "trusted_match": trusted_match,
            "trusted_pct": (100.0 * trusted_match / trusted if trusted else 0.0),
            "events": self.events,
            "event_pct": (100.0 * self.event_matches / self.events if self.events else 0.0),
            "mismatches": sorted(mismatches, reverse=True),
        }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--rom", required=True)
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--chunk", type=int, default=32)
    ap.add_argument("--min-hits", type=int, default=2)
    ap.add_argument("--min-purity", type=float, default=0.98)
    ap.add_argument("--csv", default="romsel_edge_results.csv")
    args = ap.parse_args()

    rom_path = Path(args.rom).resolve()
    blob = rom_path.read_bytes()
    rom, stripped = strip_header(blob)

    print(f"Reference: {rom_path.name}")
    print(f"ROM data: {len(rom)} bytes" + (f" (stripped {stripped}-byte header)" if stripped else ""))
    print(f"MD5: {hashlib.md5(rom).hexdigest()}")
    print("Firmware: v1.8 /ROMSEL-qualified")
    print("Every emitted event is qualified by /RD=0 AND /ROMSEL=0.\n")

    combos = {(ar,dr): Combo(ar,dr) for ar in ADDR_RELS for dr in DATA_RELS}
    edge_hist = collections.Counter()

    ser = None
    reconnects = 0
    rearms = 0
    total = 0
    edged = 0
    no_edge = 0
    start = time.monotonic()
    next_report = start + 5
    last = start

    def reopen():
        nonlocal ser, reconnects, rearms, last
        if ser:
            try: ser.close()
            except Exception: pass
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
        last = time.monotonic()

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
                    words = [int(x,16) for x in m.groups()]
                    total += 1
                    ph = [bit(w,0) for w in words]

                    edge = None
                    for i in range(1,SLOTS):
                        if ph[i-1] == 1 and ph[i] == 0:
                            edge = i
                            break

                    if edge is None:
                        no_edge += 1
                    else:
                        edged += 1
                        edge_hist[edge] += 1
                        for c in combos.values():
                            c.add(words, edge, rom)
                    last = time.monotonic()
                    continue

                if line.startswith("SWEEP DONE"):
                    send(ser, f"SWEEPREAD {args.chunk}")
                    rearms += 1
                    last = time.monotonic()
                    continue

            now = time.monotonic()
            if now-last > 1.0:
                send(ser, f"SWEEPREAD {args.chunk}")
                rearms += 1
                last = now

            if now >= next_report:
                ss = [c.stats(rom,args.min_hits,args.min_purity) for c in combos.values()]
                eligible = [s for s in ss if s["trusted"] >= 50]
                best = max(eligible or ss, key=lambda s:(s["trusted_pct"],s["trusted"],s["event_pct"]))
                print(
                    f"[LIVE] {now-start:5.1f}s ROM-events={total} edges={edged} "
                    f"best=A{best['ar']:+d}/D{best['dr']:+d} "
                    f"trusted={best['trusted_match']}/{best['trusted']} "
                    f"{best['trusted_pct']:.3f}% rearms={rearms}",
                    flush=True
                )
                next_report = now + 5

    except KeyboardInterrupt:
        pass
    finally:
        if ser:
            try: send(ser, "OFF")
            except Exception: pass
            try: ser.close()
            except Exception: pass

    ss = [c.stats(rom,args.min_hits,args.min_purity) for c in combos.values()]
    ss.sort(key=lambda s:(s["trusted_pct"],s["trusted"],s["event_pct"]), reverse=True)

    print("\n--- ROMSEL-QUALIFIED SUMMARY ---")
    print(f"qualified ROM events={total}")
    print(f"with PHI2 falling edge={edged}")
    print(f"without edge={no_edge}")
    print(f"rearms={rearms} reconnects={reconnects}")

    print("\n--- BEST ADDRESS/DATA COMBINATIONS ---")
    print("rank | address | data | trusted match | all match | event match")
    print("-----+---------+------+---------------+-----------+------------")
    for rank,s in enumerate(ss[:15],1):
        print(
            f"{rank:>4} | {s['ar']:+7d} | {s['dr']:+4d} | "
            f"{s['trusted_match']:5d}/{s['trusted']:<5d} {s['trusted_pct']:7.3f}% | "
            f"{s['all_match']:5d}/{s['unique']:<5d} {s['all_pct']:7.3f}% | "
            f"{s['event_pct']:7.3f}%"
        )

    best = ss[0]
    expected = next(s for s in ss if s["ar"] == -1 and s["dr"] == 0)

    print("\n--- PHYSICALLY EXPECTED ---")
    print(f"A-1/D+0 trusted={expected['trusted_match']}/{expected['trusted']} = {expected['trusted_pct']:.3f}%")
    print("\n--- BEST ---")
    print(f"A{best['ar']:+d}/D{best['dr']:+d} trusted={best['trusted_match']}/{best['trusted']} = {best['trusted_pct']:.3f}%")
    print(f"event-weighted={best['event_pct']:.3f}%")

    print("\nFirst trusted mismatches:")
    for hits,purity,off,exp,seen in best["mismatches"][:20]:
        print(f"  ROM+{off:05X}: expected={exp:02X} seen={seen:02X} hits={hits} purity={purity:.4f}")

    out = Path(args.csv).resolve()
    with out.open("w",newline="",encoding="utf-8") as f:
        w=csv.writer(f)
        w.writerow(["rank","address_rel","data_rel","trusted_match","trusted_total","trusted_pct",
                    "all_match","unique","all_pct","event_pct"])
        for rank,s in enumerate(ss,1):
            w.writerow([rank,s["ar"],s["dr"],s["trusted_match"],s["trusted"],
                        f"{s['trusted_pct']:.6f}",s["all_match"],s["unique"],
                        f"{s['all_pct']:.6f}",f"{s['event_pct']:.6f}"])
    print(f"\nCSV saved: {out}")

    print("\n--- INTERPRETATION ---")
    if best["trusted"] >= 100 and best["trusted_pct"] >= 90:
        print("Genuine /ROMSEL ROM cycles now match the reference very closely.")
        print("The previous low score was mostly contamination by non-ROM bus reads.")
    elif best["trusted"] >= 100 and best["trusted_pct"] >= 70:
        print("/ROMSEL qualification helped substantially, but another timing/map issue remains.")
    else:
        print("Even genuine /ROMSEL-qualified ROM reads remain far from the reference ROM.")
        print("At that point a different physical ROM/revision becomes a strong hypothesis.")
        print("The next decisive step is an offline cartridge dump or comparison against other regional/revision ROMs.")

if __name__ == "__main__":
    main()

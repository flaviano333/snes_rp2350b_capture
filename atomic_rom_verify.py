#!/usr/bin/env python3
import argparse
import collections
import csv
import hashlib
import re
import time
from pathlib import Path

import serial
from serial import SerialException

ATOM_RE = re.compile(r"^ATOM\s+P=([0-9A-Fa-f]{8})\s+D=([0-9A-Fa-f]{2})\s+O=([0-9A-Fa-f]{5})\s+A20=([0-9A-Fa-f]{5})")


def strip_header(blob):
    if len(blob) % 1024 == 512:
        return blob[512:], 512
    return blob, 0


def open_serial(port, baud):
    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = 0.10
    s.write_timeout = 1.0
    s.dtr = True
    s.rts = True
    s.open()
    time.sleep(0.2)
    s.reset_input_buffer()
    return s


def send(s, cmd):
    s.write((cmd.rstrip() + "\n").encode("ascii"))
    s.flush()


def stats(obs, rom):
    rows = []
    for off, ctr in obs.items():
        val, majority = ctr.most_common(1)[0]
        hits = sum(ctr.values())
        purity = majority / hits
        exp = rom[off] if off < len(rom) else None
        rows.append((off, exp, val, hits, majority, purity, len(ctr) == 1, exp == val if exp is not None else False, dict(ctr)))
    comparable = [r for r in rows if r[1] is not None]
    match = sum(r[7] for r in comparable)
    trusted = [r for r in comparable if r[3] >= 2 and r[5] >= 0.98]
    tmatch = sum(r[7] for r in trusted)
    return rows, comparable, match, trusted, tmatch


def main():
    ap = argparse.ArgumentParser(description="Single-PIO atomic SNES ROM verifier")
    ap.add_argument("--port", required=True)
    ap.add_argument("--rom", required=True)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--chunk", type=int, default=128)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--csv", default="atomic_rom_verify.csv")
    args = ap.parse_args()

    if not (16 <= args.chunk <= 4096):
        ap.error("--chunk must be 16..4096")

    p = Path(args.rom).resolve()
    blob = p.read_bytes()
    rom, stripped = strip_header(blob)
    print(f"Reference: {p.name}")
    print(f"ROM data: {len(rom)} bytes" + (f" (stripped {stripped}-byte header)" if stripped else ""))
    print(f"MD5: {hashlib.md5(rom).hexdigest()}")
    if len(rom) != 524288:
        print("WARNING: this v1.5 diagnostic is optimized for the 512 KiB Tom & Jerry LoROM.")
    print("Capture path: ONE PIO word = GP2..GP31 = data + all address bits needed for 512 KiB LoROM.")
    print("If this jumps near 100%, the old low/high PIO pairing was the problem.\n")

    obs = {}
    start = time.monotonic()
    next_report = start + 5.0
    last_atom = start
    rearms = 0
    reconnects = 0
    lines = 0
    ser = None

    def reopen():
        nonlocal ser, reconnects, rearms, last_atom
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
        send(ser, f"ATOMICREAD {args.chunk}")
        rearms += 1
        last_atom = time.monotonic()

    reopen()

    try:
        while time.monotonic() - start < args.seconds:
            try:
                raw = ser.readline()
            except SerialException:
                reconnects += 1
                reopen()
                continue

            if raw:
                line = raw.decode("ascii", errors="replace").strip()
                m = ATOM_RE.match(line)
                if m:
                    data = int(m.group(2), 16)
                    off = int(m.group(3), 16)
                    obs.setdefault(off, collections.Counter())[data] += 1
                    lines += 1
                    last_atom = time.monotonic()
                    continue
                if line.startswith("ATOMIC DONE"):
                    send(ser, f"ATOMICREAD {args.chunk}")
                    rearms += 1
                    last_atom = time.monotonic()
                    continue

            now = time.monotonic()
            # Lost DONE / USB burst recovery.
            if now - last_atom > 1.0:
                send(ser, f"ATOMICREAD {args.chunk}")
                rearms += 1
                last_atom = now

            if now >= next_report:
                rows, comp, match, trusted, tmatch = stats(obs, rom)
                pct = 100 * match / len(comp) if comp else 0.0
                tpct = 100 * tmatch / len(trusted) if trusted else 0.0
                print(f"[LIVE] {now-start:5.1f}s lines={lines} unique_offsets={len(obs)} "
                      f"all={match}/{len(comp)} {pct:.3f}% "
                      f"trusted={tmatch}/{len(trusted)} {tpct:.3f}% "
                      f"rearms={rearms} reconnects={reconnects}", flush=True)
                next_report = now + 5.0
    except KeyboardInterrupt:
        pass
    finally:
        if ser:
            try: send(ser, "OFF")
            except Exception: pass
            try: ser.close()
            except Exception: pass

    rows, comp, match, trusted, tmatch = stats(obs, rom)
    pct = 100 * match / len(comp) if comp else 0.0
    tpct = 100 * tmatch / len(trusted) if trusted else 0.0
    print("\n--- FINAL ---")
    print(f"lines={lines} unique_offsets={len(obs)} rearms={rearms} reconnects={reconnects}")
    print(f"all:     {match}/{len(comp)} = {pct:.3f}%")
    print(f"trusted: {tmatch}/{len(trusted)} = {tpct:.3f}%  (hits>=2, purity>=0.98)")

    mism = [r for r in trusted if not r[7]]
    if mism:
        print("\nFirst trusted mismatches:")
        for r in sorted(mism, key=lambda x: (-x[3], x[0]))[:20]:
            print(f"  ROM+{r[0]:05X}: expected={r[1]:02X} seen={r[2]:02X} hits={r[3]} purity={r[5]:.4f}")

    out = Path(args.csv).resolve()
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["rom_offset","reference","physical_majority","match","stable","hits","majority_hits","purity","observed_values"])
        for r in sorted(rows, key=lambda x: x[0]):
            vals = " ".join(f"{k:02X}:{v}" for k,v in sorted(r[8].items()))
            w.writerow([f"{r[0]:05X}", f"{r[1]:02X}" if r[1] is not None else "", f"{r[2]:02X}", int(r[7]), int(r[6]), r[3], r[4], f"{r[5]:.6f}", vals])
    print(f"CSV saved: {out}")

    print("\nInterpretation:")
    if len(trusted) >= 100 and tpct >= 95:
        print("  Strong result: single-PIO atomic capture matches the reference ROM closely.")
        print("  This strongly implicates the previous two-PIO low/high pairing.")
    elif len(trusted) >= 100 and tpct <= 70:
        print("  Match is still low even with atomic data+address capture.")
        print("  Next suspects are ROM/revision mismatch or a remaining A0-A19 wiring/map issue.")
    else:
        print("  Not enough trusted samples for a strong conclusion; play longer and repeat.")


if __name__ == "__main__":
    main()

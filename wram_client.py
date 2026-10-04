#!/usr/bin/env python3
import argparse
import serial
import sys
import time

WRAM_SIZE = 131072
VALID_SIZE = 16384


def open_serial(port):
    ser = serial.Serial(port, 115200, timeout=3)
    time.sleep(0.25)
    ser.reset_input_buffer()
    return ser


def send_line(ser, line):
    ser.write((line.rstrip() + "\n").encode("ascii"))
    ser.flush()


def read_text_response(ser, seconds=1.0):
    deadline = time.time() + seconds
    lines = []
    while time.time() < deadline:
        line = ser.readline()
        if line:
            lines.append(line.decode("utf-8", errors="replace").rstrip())
        else:
            break
    return lines


def read_exact(ser, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise TimeoutError(f"Timeout: received {len(buf)}/{n} bytes")
        buf.extend(chunk)
    return bytes(buf)


def cmd_simple(port, command):
    with open_serial(port) as ser:
        send_line(ser, command)
        for line in read_text_response(ser, 1.5):
            print(line)


def cmd_dump(port, path):
    with open_serial(port) as ser:
        send_line(ser, "DUMPBIN")

        header = None
        deadline = time.time() + 5
        while time.time() < deadline:
            line = ser.readline()
            if not line:
                continue
            text = line.decode("ascii", errors="ignore").strip()
            if text.startswith("BIN1 "):
                header = text
                break

        if header is None:
            raise RuntimeError("Did not receive BIN1 header")

        parts = header.split()
        if len(parts) < 5:
            raise RuntimeError(f"Bad header: {header}")

        wram_len = int(parts[1])
        valid_len = int(parts[2])
        known = int(parts[3])
        batches = int(parts[4])

        if wram_len != WRAM_SIZE or valid_len != VALID_SIZE:
            raise RuntimeError(f"Unexpected sizes in header: {header}")

        data = read_exact(ser, wram_len)
        valid = read_exact(ser, valid_len)

        with open(path, "wb") as f:
            f.write(data)
        valid_path = path + ".valid"
        with open(valid_path, "wb") as f:
            f.write(valid)

        print(f"Saved {len(data)} WRAM bytes to {path}")
        print(f"Saved {len(valid)} validity bytes to {valid_path}")
        print(f"Known bytes at snapshot: {known}/{WRAM_SIZE} ({known * 100 / WRAM_SIZE:.2f}%)")
        print(f"Completed capture batches: {batches}")



def cmd_snap(port, ranges):
    parsed = []
    total = 0
    for token in ranges:
        if ":" not in token:
            raise ValueError(f"Bad range {token!r}; use OFFSET:LENGTH in hex")
        a, b = token.split(":", 1)
        off = int(a, 16)
        length = int(b, 16)
        parsed.append((off, length))
        total += length

    with open_serial(port) as ser:
        send_line(ser, "SNAP " + " ".join(ranges))
        header = None
        deadline = time.time() + 3
        while time.time() < deadline:
            line = ser.readline()
            if not line:
                continue
            text = line.decode("ascii", errors="ignore").strip()
            if text.startswith("SNAP1 "):
                header = text
                break
            if text.startswith("ERR "):
                raise RuntimeError(text)
        if header is None:
            raise RuntimeError("Did not receive SNAP1 header")

        parts = header.split()
        if len(parts) != 5:
            raise RuntimeError(f"Bad SNAP header: {header}")
        count, got_total, unknown, batch = map(int, parts[1:])
        if count != len(parsed) or got_total != total:
            raise RuntimeError(f"SNAP shape mismatch: {header}")
        data = read_exact(ser, total)
        known = read_exact(ser, total)

        pos = 0
        print(f"SNAP ranges={count} bytes={total} unknown={unknown} batch={batch}")
        for off, length in parsed:
            vals = data[pos:pos+length]
            flags = known[pos:pos+length]
            chunks = []
            for v, k in zip(vals, flags):
                chunks.append(f"{v:02X}" if k else "??")
            print(f"{off:05X}:{length:X}  " + " ".join(chunks))
            pos += length


def main():
    ap = argparse.ArgumentParser(description="SNES RP2350B WRAM Bridge v1.8 client")
    ap.add_argument("port", help="serial port, e.g. COM7 or /dev/ttyACM0")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("info")
    sub.add_parser("help")
    sub.add_parser("ping")
    sub.add_parser("clear")
    sub.add_parser("wramsel")

    p_read = sub.add_parser("read")
    p_read.add_argument("offset", help="canonical WRAM offset in hex, e.g. 013FB")

    p_rs = sub.add_parser("read-snes")
    p_rs.add_argument("address", help="SNES address BB:AAAA, e.g. 00:13FB")

    p_hex = sub.add_parser("hex")
    p_hex.add_argument("offset", nargs="?", default="0")
    p_hex.add_argument("length", nargs="?", default="100")

    p_dump = sub.add_parser("dump")
    p_dump.add_argument("path", nargs="?", default="wram.bin")

    p_snap = sub.add_parser("snap")
    p_snap.add_argument("ranges", nargs="+", help="hex OFFSET:LENGTH pairs, e.g. 00026:1 00AE8:2")

    args = ap.parse_args()

    if args.cmd == "info":
        cmd_simple(args.port, "INFO")
    elif args.cmd == "help":
        cmd_simple(args.port, "HELP")
    elif args.cmd == "ping":
        cmd_simple(args.port, "PING")
    elif args.cmd == "clear":
        cmd_simple(args.port, "CLEAR")
    elif args.cmd == "wramsel":
        cmd_simple(args.port, "WRAMSEL")
    elif args.cmd == "read":
        cmd_simple(args.port, f"READ {args.offset}")
    elif args.cmd == "read-snes":
        cmd_simple(args.port, f"READSNES {args.address}")
    elif args.cmd == "hex":
        cmd_simple(args.port, f"HEX {args.offset} {args.length}")
    elif args.cmd == "dump":
        cmd_dump(args.port, args.path)
    elif args.cmd == "snap":
        cmd_snap(args.port, args.ranges)
    else:
        ap.error("unknown command")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)

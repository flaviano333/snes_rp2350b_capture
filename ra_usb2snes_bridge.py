#!/usr/bin/env python3
"""
RP2350B -> usb2snes compatibility bridge for RA2Snes.

This program exposes a small subset of the QUsb2Snes WebSocket protocol on
ws://127.0.0.1:23074 and maps usb2snes WRAM reads (0xF50000+) to the
RP2350B firmware's passive WRAM mirror.

It also serves a local SNES ROM file so RA2Snes can identify/hash the game.
First target: RA2Snes Softcore mode.
"""
import argparse
import asyncio
import json
import os
import threading
import time
from pathlib import Path

import serial
import websockets

USB2SNES_WRAM_BASE = 0xF50000
WRAM_SIZE = 128 * 1024
USB2SNES_SRAM_BASE = 0xE00000
DEVICE_NAME = "RP2350B RA Bridge"
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

    def ping(self):
        with self.lock:
            self.ser.reset_input_buffer()
            self._send_line("PING")
            line = self._readline_until(b"PONG", 2.0)
            return line == b"PONG"

    def read_range(self, offset: int, length: int):
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

            data = self.ser.read(length)
            while len(data) < length:
                chunk = self.ser.read(length - len(data))
                if not chunk:
                    raise TimeoutError(f"RBIN payload timeout ({len(data)}/{length})")
                data += chunk

            # Consume newline + END marker. We deliberately tolerate stray blank lines.
            self._readline_until(b"END RBIN1", 1.0)
            return bytes(data), unknown


class BridgeServer:
    def __init__(self, serial_wram: SerialWRAM, rom_path: Path, verbose=False):
        self.serial_wram = serial_wram
        self.rom_path = rom_path
        self.rom_file = rom_path.read_bytes()
        # usb2snes ROM address 0 maps to actual cartridge ROM, not copier header.
        self.rom_mem = self.rom_file[512:] if (len(self.rom_file) % 1024) == 512 else self.rom_file
        self.rom_virtual_path = "/games/" + rom_path.name
        self.verbose = verbose
        self.clients = 0
        self.unknown_reads = 0

    def log(self, *a):
        if self.verbose:
            print("[bridge]", *a, flush=True)

    async def read_snes_space(self, address: int, size: int) -> bytes:
        if size <= 0:
            return b""

        # WRAM exposed by usb2snes at F50000-F6FFFF.
        if USB2SNES_WRAM_BASE <= address < USB2SNES_WRAM_BASE + WRAM_SIZE:
            off = address - USB2SNES_WRAM_BASE
            if off + size > WRAM_SIZE:
                size = WRAM_SIZE - off
            data, unknown = await asyncio.to_thread(self.serial_wram.read_range, off, size)
            if unknown:
                self.unknown_reads += 1
                if self.verbose and self.unknown_reads <= 20:
                    print(f"[bridge] WARNING: {unknown}/{size} requested WRAM bytes are not yet known at {off:05X}")
            return data

        # Cartridge ROM space in usb2snes storage address space starts at 0.
        if 0 <= address < len(self.rom_mem):
            data = self.rom_mem[address:address + size]
            if len(data) < size:
                data += bytes(size - len(data))
            return data

        # SRAM/other storage regions aren't required for the first Softcore test.
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
                # CMD reads are used for SD2SNES-specific state/patch checks.
                # Initial bridge target is Softcore, so return an unpatched/zero state.
                out.extend(bytes(size))
        return bytes(out)

    async def send_file(self, ws, path: str):
        if path == "/sd2snes/config.yml":
            payload = CONFIG_YML
        else:
            # RA2Snes asks for the path returned by Info. Serve the selected ROM.
            payload = self.rom_file

        await ws.send(json.dumps({"Results": [format(len(payload), "X")] }))
        # QUsb2Snes/usb2snes commonly chunks binary at 1024 bytes.
        for off in range(0, len(payload), 1024):
            await ws.send(payload[off:off + 1024])

    async def handler(self, ws):
        self.clients += 1
        print(f"RA client connected ({self.clients})", flush=True)
        try:
            async for message in ws:
                if isinstance(message, bytes):
                    # PutAddress/PutFile isn't supported in this first passive bridge.
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
                        # Protocol specifies no reply on successful attach.
                        pass
                    elif opcode == "Name":
                        pass
                    elif opcode == "Info":
                        await ws.send(json.dumps({
                            "Results": [
                                "1.11.0",              # plausible official-style SD2SNES firmware string
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
                        # Passive hardware bridge: no writes into the SNES.
                        # Softcore should not require this.
                        self.log("PutAddress ignored (passive bridge)")
                    elif opcode in ("Reset", "Menu", "Boot", "PutFile", "PutIPS"):
                        self.log(opcode, "ignored (passive bridge)")
                    else:
                        self.log("unsupported opcode", opcode)
                        # For benign metadata commands, an empty Results reply is friendlier than disconnecting.
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
            raise RuntimeError("RP2350B did not answer PING. Flash v0.9 firmware and close PuTTY first.")

        bridge = BridgeServer(sw, rom, args.verbose)
        print("RP2350B serial: OK")
        print(f"ROM: {rom.name} ({len(bridge.rom_mem)} bytes cartridge data)")
        print("usb2snes compatibility server: ws://127.0.0.1:23074")
        print("Do NOT run QUsb2Snes at the same time (same TCP port).")
        print("Start RA2Snes and use SOFTCORE for this first test.")
        print("Keep the RP powered before the SNES, and ideally boot the game before starting RA2Snes.")

        async with websockets.serve(bridge.handler, "127.0.0.1", args.ws_port, max_size=None):
            await asyncio.Future()
    finally:
        sw.close()


def main():
    ap = argparse.ArgumentParser(description="RP2350B SNES -> usb2snes/RA2Snes bridge")
    ap.add_argument("--port", required=True, help="RP2350B serial port, e.g. COM7")
    ap.add_argument("--rom", required=True, help="ROM file for game identification, e.g. game.sfc")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--ws-port", type=int, default=23074)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    try:
        asyncio.run(amain(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

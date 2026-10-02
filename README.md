# SNES RP2350B RA Bridge v0.9

Proof-of-concept bridge from a passive SNES cartridge-bus monitor to RA2Snes.

## Hardware wiring

Same as v0.8. No GP35/GP36 jumpers.

- GP0 = PHI2
- GP1 = /WR
- GP2..GP9 = D0..D7
- GP10..GP19 = A0..A9
- GP20 unused by SNES wiring
- GP21..GP34 = A10..A23
- common GND

Keep RP2350B powered before powering the SNES.

## What changed in firmware

Adds a machine-readable command:

`RBIN <WRAM offset hex> <length hex>`

Maximum length per request is 0x1000 bytes.

Response framing:

`RBIN1 <decimal length> <decimal unknown-byte-count>\n`

then raw bytes, then:

`END RBIN1`

The existing INFO, READ, READSNES, HEX and DUMPBIN commands remain available.

## PC-side RA2Snes compatibility bridge

`ra_usb2snes_bridge.py`:

- opens the RP2350B serial port;
- exposes a usb2snes-compatible WebSocket server on `ws://127.0.0.1:23074`;
- maps usb2snes WRAM `0xF50000..0xF6FFFF` to the RP2350B WRAM mirror;
- serves a local ROM file to RA2Snes for game identification/hash;
- emulates the minimum DeviceList / Attach / Info / AppVersion / GetFile / GetAddress commands needed for a first RA2Snes Softcore test.

This is a passive bridge. PutAddress/control commands are not implemented. Use **Softcore** for the first tests.

## Build firmware

Use the included GitHub Actions workflow as with the previous versions. Flash the generated `.uf2` to the RP2350B.

## Install PC dependencies

```powershell
py -m pip install -r requirements.txt
```

or:

```powershell
py -m pip install pyserial "websockets>=13,<16"
```

## Run

Close PuTTY first so it releases the COM port.

Use a ROM file that matches the cartridge/game being played:

```powershell
py ra_usb2snes_bridge.py --port COM7 --rom "C:\ROMs\Tom and Jerry.sfc"
```

Expected output:

```text
RP2350B serial: OK
ROM: Tom and Jerry.sfc (... bytes cartridge data)
usb2snes compatibility server: ws://127.0.0.1:23074
Do NOT run QUsb2Snes at the same time (same TCP port).
Start RA2Snes and use SOFTCORE for this first test.
```

Then launch RA2Snes. Do not launch QUsb2Snes; this Python program is taking its place.

For protocol debugging:

```powershell
py ra_usb2snes_bridge.py --port COM7 --rom "C:\ROMs\game.sfc" --verbose
```

## Important current limitations

1. The WRAM mirror is write-observed. Bytes not written since the RP started remain unknown internally; the compatibility layer currently returns the mirror byte value to RA2Snes.
2. Capture still rearms DMA in batches, so there are small gaps between batches. Tight one-frame conditions may be missed.
3. This version is intended for Softcore proof-of-concept testing. Hardcore integrity checks and SD2SNES write/hook commands are not implemented.
4. The supplied ROM file is used for RA2Snes game identification. It must correspond to the game actually running.

The next engineering step after proving RA2Snes connectivity is ping-pong/ring DMA for gapless capture.

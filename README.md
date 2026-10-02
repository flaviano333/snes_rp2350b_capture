# SNES RP2350B RA Bridge v1.3 — Atomic Snapshot

This version keeps the v1.2 wiring/remap and adds a firmware-side **atomic multi-range WRAM snapshot** for RA2Snes.

## Why v1.3 exists

RA2Snes asks for several memory ranges in multiple `GetAddress` messages. In v1.2 the Python bridge forwarded each range to the RP2350B separately with `RBIN`, so rapidly changing values could come from different moments of the game.

v1.3 adds the `SNAP` command. The firmware copies every RA-requested byte into staging buffers first, before returning anything over USB. The Python bridge learns RA2Snes's requested addresses during the first polling cycle. Starting with the next `Info` polling boundary, all `GetAddress` groups in that cycle are served from the **same firmware-side mirror instant**.

This improves consistency, but it does **not** make the passive capture frame-perfect and it does not solve currently unseen WRAM bytes such as high-WRAM addresses that remain `UNKNOWN`.

## Important ROM-identification limitation

The physical cartridge and the ROM passed to `--rom` are still independent:

- RP2350B = live WRAM from the physical SNES/cart.
- `--rom` = file served to RA2Snes for game identification/hash.

The bridge cannot yet prove that the physical cartridge is the same region/revision as the supplied ROM. The script prints the selected ROM's MD5 so the file being presented to RA2Snes is explicit.

## Existing wiring (unchanged)

```
GP0   = PHI2
GP1   = /WR
GP2-9 = D0-D7
GP35  = /RD
GP20  = unused (board WS2812)
```

Measured logical address remap used by firmware:

```
A0  -> GP12     A8  -> GP19     A16 -> GP28
A1  -> GP11     A9  -> GP21     A17 -> GP29
A2  -> GP10     A10 -> GP22     A18 -> GP30
A3  -> GP13     A11 -> GP23     A19 -> GP31
A4  -> GP14     A12 -> GP24     A20 -> GP32
A5  -> GP16     A13 -> GP25     A21 -> GP33
A6  -> GP15     A14 -> GP26     A22 -> GP34
A7  -> GP17     A15 -> GP27     A23 -> GP18
```

## Build the UF2

The included GitHub Actions workflow builds with Pico SDK 2.2.0. Upload the contents of this folder to the repository root and run **Build UF2**. Flash the resulting `snes_rp2350b_capture.uf2` to the RP2350B.

## New firmware command

Example:

```
SNAP 00026:1 001C1:1 00AE8:2 0157A:1
```

The firmware first copies all requested bytes and known/unknown flags into staging buffers, then emits a binary `SNAP1` response. Maximum: 256 ranges and 4096 total bytes.

`INFO` now also includes:

```
snap_count=
snap_ranges=
snap_bytes=
snap_unknown=
```

For a manual test from Python:

```
python wram_client.py COM7 snap 00026:1 001C1:1 00AE8:2 0157A:1 0287A:1 02AF2:1
```

## RA2Snes trace test

Install dependencies once:

```
python -m pip install -r requirements.txt
```

Close PuTTY/other programs that own COM7, then run:

```
python .\ra_usb2snes_bridge_trace.py --port COM7 --rom "C:\path\Tom and Jerry (USA).sfc" --trace-ra
```

Do **not** add `--verbose` unless protocol spam is intentionally wanted.

At startup the script verifies the new `SNAP` command. The first RA polling cycle is used to learn the memory working set. Then it prints:

```
Atomic RA snapshot ACTIVE: ...
```

From then on, the multiple `GetAddress` calls belonging to one RA polling interval are served from one snapshot. `--trace-ra` only prints a range again when its data or KNOWN/UNKNOWN status changes.

Optional snapshot diagnostics:

```
--trace-snapshots
```

This prints snapshot summaries only when the working-set shape or unknown count changes. `--trace-all` remains intentionally noisy.

## Current known limitations

- Unknown bytes are still returned as the current mirror byte (usually zero) because usb2snes has no `UNKNOWN` state.
- WRAM above `$1FFF` may remain unavailable with the current A-bus-only cartridge connector setup.
- The experimental v1.2 `$2180-$2183` reconstruction code is retained for diagnostics but is not a substitute for observing the real B-bus `/PWR` and `/PRD` signals.
- Physical cartridge identity/region/revision is not verified against `--rom`.
- This is still a passive experimental prototype, not a drop-in FXPak Pro replacement.

## Electrical reminder

The prototype directly uses only RP2350B FT GPIOs for SNES 5 V signals. Keep the RP powered before the SNES, turn the SNES off before disconnecting the RP, keep all SNES-facing GPIOs as inputs, and maintain common ground.

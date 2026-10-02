# SNES RP2350B RA Bridge v1.1 — address remap + debug

This version keeps the v1.0 A-bus read/write capture and fixes the address reconstruction to match the **actual wiring measured on this prototype**. No address wires need to be resoldered just to put them in numerical order: the firmware reorders the sampled GPIO bits in software.

## Physical wiring used by v1.1

Timing/data signals are unchanged:

- GP0 = PHI2
- GP1 = CPU /WR
- GP2..GP9 = D0..D7
- GP35 = CPU /RD
- common GND

Measured address wiring:

| SNES | RP2350B |
|---|---:|
| A0 | GP12 |
| A1 | GP11 |
| A2 | GP10 |
| A3 | GP13 |
| A4 | GP14 |
| A5 | GP16 |
| A6 | GP15 |
| A7 | GP17 |
| A8 | GP19 |
| A9 | GP21 |
| A10 | GP22 |
| A11 | GP23 |
| A12 | GP24 |
| A13 | GP25 |
| A14 | GP26 |
| A15 | GP27 |
| A16 | GP28 |
| A17 | GP29 |
| A18 | GP30 |
| A19 | GP31 |
| A20 | GP32 |
| A21 | GP33 |
| A22 | GP34 |
| A23 | GP18 |

GP20 remains unused by the SNES wiring because it is associated with the board's onboard RGB LED.

A0/A1/A2/A3/A4/A5/A6/A7, A8..A13, A22 and A23 were checked during diagnosis. A14..A21 follow the continuous sequence between the confirmed endpoints; if the corrected bank counters still look impossible, verify those eight wires individually.

## Why this matters

The older firmware assumed the logical address bits were wired in numerical GPIO order. In the actual prototype, A0/A2 and A5/A6 are rearranged, and the high block is rotated so A23 is on GP18 while A8..A22 continue from GP19/GP21..GP34.

That corrupts the reconstructed 24-bit SNES address even though the console itself continues to run normally. In particular, bad A16..A23 reconstruction can make real bank `$7E/$7F` WRAM traffic look like unrelated banks, which can explain `direct_r=0` and `direct_w=0` in v1.0.

## New debug commands

The normal firmware is quiet so it can be used by the Python RA bridge. For diagnostics in PuTTY, v1.1 can print captured bus traffic in the same style as the earlier capture firmware.

```text
DEBUG WRAM 64
```

Prints the next 64 reconstructed WRAM accesses, then turns debug off automatically. Example:

```text
WRITE $7E:287A = 3C   [WRAM +0x0287A DIRECT]
READ  $00:13FB = F4   [WRAM +0x013FB MIRROR]
DEBUG DONE (auto-off)
```

Other modes:

```text
DEBUG ALL 64
DEBUG READ 64
DEBUG WRITE 64
DEBUG OFF
DEBUG
```

The number is decimal, defaults to 64, and can be 1..4096. Debug printing intentionally slows the main loop and can create additional capture gaps, so use it only for short diagnostics. **Do not leave debug enabled while running the Python/RA2Snes bridge.** The Python v1.1 bridge also sends `DEBUG OFF` when it opens the serial port as a safeguard.

## Bank diagnostic

The new command:

```text
BANKS
```

shows reconstructed A-bus traffic per bank:

```text
BANK 00 W=12345 R=45678
BANK 7E W=2345 R=9876
BANK 7F W=120 R=540
END BANKS
```

After the wiring remap, seeing activity on `7E` and/or `7F` is the key test. `INFO` should then also begin showing nonzero `direct_w` and/or `direct_r`.

## Recommended first test

1. Flash the v1.1 UF2.
2. Power the RP2350B first, then the SNES.
3. Open PuTTY and let the game run for a few seconds.
4. Run:

```text
INFO
BANKS
DEBUG WRAM 64
```

5. Look especially for `$7E:xxxx` / `$7F:xxxx` entries marked `DIRECT`.
6. Close PuTTY before starting the Python bridge.

Then run the RA bridge as before:

```powershell
python ra_usb2snes_bridge.py --port COM7 --rom "C:\ROMs\Tom and Jerry (USA).sfc" --verbose
```

Start RA2Snes in Softcore mode.

## Build

The included GitHub Actions workflow builds with Pico SDK 2.2.0 and uploads `snes_rp2350b_capture.uf2` as the workflow artifact `snes-rp2350b-ra-bridge-v1.1-uf2`.

## Remaining limitations

- WRAM values are still learned passively from observed A-bus reads and writes.
- B-bus/WMDATA (`$2180`) traffic is not directly reconstructed yet.
- DMA capture is still rearmed in batches, leaving short gaps.
- Heavy debug printing makes those gaps worse; use it only temporarily.
- Hardcore SD2SNES control/patch behavior is not implemented.

## Electrical safety

Keep the same prototype precautions: use only the RP2350B GPIOs already chosen for the 5 V SNES signals, keep all SNES-connected GPIOs as inputs, share GND, power the RP2350B before the SNES, and turn the SNES off before unplugging the RP2350B.

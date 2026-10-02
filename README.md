# SNES RP2350B RA Bridge v1.2 — WMDATA reconstruction + debug

v1.2 keeps the v1.1 address remap and A-bus read/write capture, and adds tracking for the SNES CPU-side WRAM port at `$2180-$2183`.

This matters because a game does not have to access all 128 KiB of WRAM through direct `$7E/$7F` A-bus addresses. CPU code can set a 17-bit WRAM pointer with `WMADDL/WMADDM/WMADDH` and then transfer bytes through `WMDATA`.

## Physical wiring

No new wire is required for v1.2. It uses the same prototype wiring as v1.1:

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

GP20 remains unused.

A14..A21 are still inferred from the continuous sequence between the checked A13 and A22 endpoints. If bank reconstruction remains implausible, verify those eight physically.

## What v1.2 adds

The firmware recognizes normal CPU accesses in I/O banks to:

```text
$2180  WMDATA
$2181  WMADDL
$2182  WMADDM
$2183  WMADDH
```

When all three WMADD components have been observed, a CPU write sequence such as:

```text
WRITE $00:2181 = F2
WRITE $00:2182 = 2A
WRITE $00:2183 = 00
WRITE $00:2180 = 37
```

is reconstructed as:

```text
WRAM[$02AF2] = 37
WMADD -> $02AF3
```

The reconstructed byte is inserted into the same 128 KiB software WRAM mirror served to RA2Snes.

WMDATA reads are also detected and currently used experimentally. There is one important limitation: read and write traffic are captured in separate DMA pipelines, so their processing order is not guaranteed to exactly match bus chronology. The debug output labels reconstructed reads with `[EXPERIMENTAL ORDER]`. CPU-side WMDATA writes are more useful for this experiment because WMADD setup and WMDATA writes live in the same write stream and retain their relative order there.

DMA that accesses WMDATA through the SNES B-bus is still not directly visible with this wiring. If Tom and Jerry relies on that path, PA0..PA7 plus `/PRD`/`/PWR` will eventually be required.

## New diagnostics

### WMDATA live debug

In PuTTY, with the Python bridge closed:

```text
DEBUG WMDATA 128
```

Example output:

```text
WMADDL WRITE $00:2181 = F2 -> WMADD=000F2 mask=1/7
WMADDM WRITE $00:2182 = 2A -> WMADD=02AF2 mask=3/7
WMADDH WRITE $00:2183 = 00 -> WMADD=02AF2 mask=7/7 VALID
WMDATA WRITE $00:2180 = 37 -> WRAM[02AF2], next=02AF3
```

The number is decimal, defaults to 64, and can be 1..4096. Debug automatically switches off after that many matching lines.

Existing modes remain available:

```text
DEBUG WRAM 64
DEBUG ALL 64
DEBUG READ 64
DEBUG WRITE 64
DEBUG OFF
BANKS
```

### WMDATA state/counters

```text
WMSTATE
```

Example:

```text
WMSTATE addr=02AF3 mask=7/7 valid=YES addr_reg_w=123 data_w=456 data_r=0 recon_w=450 recon_r=0 unknown_ptr=6
```

`INFO` now also includes:

```text
wmaddr_w=
wmdata_w=
wmdata_r=
wm_recon_w=
wm_recon_r=
wm_unknown=
```

The most useful numbers are `wmdata_w`/`wmdata_r` (whether the game is using `$2180`) and `wm_recon_w`/`wm_recon_r` (how many of those accesses could be mapped into the 128 KiB mirror).

## Important: an idle game can hide activity

Yes. The RP2350B does not magically know the whole WRAM image; it learns bytes only when the SNES produces observable bus traffic. Some variables are updated constantly, but others are touched only during events such as:

- entering or finishing a stage;
- spawning/killing an enemy;
- collecting an item;
- taking damage/dying;
- opening a menu or changing screens;
- loading/decompressing level data.

RA2Snes can keep asking for an address even while the game itself is not currently touching that address. Therefore a quiet `DEBUG WMDATA` while standing still does not prove that the game never uses WMDATA.

For diagnosis, start `DEBUG WMDATA 128` and then actively play, change rooms/stages, die/restart, and trigger events. Compare `WMSTATE` and `INFO` before and after.

## Recommended test

1. Flash the v1.2 UF2.
2. Power the RP2350B first, then the SNES.
3. Open PuTTY and run:

```text
CLEAR
INFO
WMSTATE
DEBUG WMDATA 128
```

4. While the debug is waiting, actively play Tom and Jerry and trigger a level transition or other obvious event.
5. Then run:

```text
WMSTATE
INFO
READ 0287A
READ 02AF2
```

If either `$0287A` or `$02AF2` changes from `UNKNOWN` to `KNOWN`, this is exactly the path we were trying to recover.

6. Close PuTTY before starting the Python bridge.

Run the bridge as before:

```powershell
python ra_usb2snes_bridge.py --port COM7 --rom "C:\ROMs\Tom and Jerry (USA).sfc" --verbose
```

Use RA2Snes in Softcore for now.

The Python bridge sends `DEBUG OFF` when it opens the serial port so debug text cannot corrupt binary `RBIN` replies.

## Build

The included GitHub Actions workflow uses Pico SDK 2.2.0 and uploads `snes_rp2350b_capture.uf2` as the artifact:

```text
snes-rp2350b-ra-bridge-v1.2-uf2
```

## Remaining limitations

- A14..A21 have not yet been individually continuity-checked.
- WMDATA reads are experimental because read/write DMA streams are processed separately.
- B-bus DMA to/from WMDATA is not reconstructed yet.
- Capture buffers are rearmed in batches, leaving short gaps.
- Heavy debug printing increases those gaps; use debug only briefly.
- The mirror contains only bytes actually learned by the adapter; `UNKNOWN` is not equivalent to zero.
- Hardcore SD2SNES-specific patch/control behavior is not implemented.

## Electrical safety

Keep all SNES-connected GPIOs as inputs, share GND, power the RP2350B before powering the SNES, and turn the SNES off before unplugging the RP2350B. Do not route SNES 5 V into arbitrary RP2350B GPIOs or power pins.

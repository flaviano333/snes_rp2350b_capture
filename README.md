# SNES RP2350B RA Bridge v1.0 — WRAM reads + writes

This version is aimed at the first reliable RetroAchievements tests.

## One new wire

Keep all existing wiring from v0.9 and add:

- SNES cartridge pin **23 (CPU /RD)** -> **GP35** on the RP2350B

The normal cartridge connector exposes CPU /RD on pin 23, so no extra side-tab contact is required.

Existing wiring remains:

- GP0 = PHI2 (cart pin 57)
- GP1 = /WR (cart pin 54)
- GP2..GP9 = D0..D7
- GP10..GP19 = A0..A9
- GP20 unused by SNES wiring
- GP21..GP34 = A10..A23
- GP35 = /RD (cart pin 23) **NEW**
- common GND

Keep the RP2350B powered before powering the SNES.

## Why this version exists

v0.9 only learned bytes when the SNES *wrote* them after the adapter started. That leaves stale/unknown bytes, especially above WRAM $1FFF. RA2Snes was observed requesting offsets such as $287A and $2AF2.

v1.0 also passively captures A-bus **reads**. When the CPU reads WRAM, the actual byte on D0-D7 is used to initialise or repair the software mirror. This should greatly increase the number of WRAM bytes that RA2Snes can read correctly.

This still does not monitor the B-bus / $2180 WRAM port directly, and DMA capture is still batched, so this is not yet guaranteed 100% coverage.

## Firmware architecture

- PIO0 SM0 + PIO1 SM0: A-bus writes (existing path)
- PIO2 SM0: watches /RD on GP35 and signals PIO0
- PIO0 SM1 + PIO1 SM1: A-bus reads
- four DMA channels collect paired low/high samples
- WRAM mirror is updated from both reads and writes

## Build

Use the included GitHub Actions workflow exactly as in previous versions. Flash the generated `.uf2`.

## Quick validation

With PuTTY open, run:

```text
INFO
```

You should now see both write and read counters, for example:

```text
INFO ... bus_writes=... wram_writes=... bus_reads=... wram_reads=... direct_r=... known=.../131072 (...)
```

The important signs are `bus_reads > 0`, `wram_reads > 0`, and ideally `direct_r > 0`.

Then close PuTTY and run the Python bridge as before:

```powershell
python ra_usb2snes_bridge.py --port COM7 --rom "C:\ROMs\Tom and Jerry (USA).sfc" --verbose
```

Launch RA2Snes in Softcore mode.

## Current limitations

1. The adapter is still passive and cannot force a read of arbitrary WRAM.
2. WRAM accesses made only through the B-bus/WMDATA path can still be missed until their values are later observed on A-bus reads/writes.
3. DMA buffers are still rearmed in batches, so there are short capture gaps.
4. Hardcore SD2SNES control/patch features are not implemented.

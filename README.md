# SNES RP2350B v1.5 — Single-PIO ROM diagnostic

This is a **diagnostic firmware**, not the RA bridge firmware.

It tests whether the previous ~45–50% ROM match was caused by pairing two independently captured PIO streams.

## Important clarification about `FIXED`

In v1.4, output such as:

```
A14 -> GP26 FIXED
A15 -> GP27 FIXED
```

means **the mapper determined that this assignment is fixed/certain among the best candidates**. It does *not* mean "wrong" or "needs fixing". In both v1.4 runs the current A14..A17 assignments were already the best mapping.

## What changes in v1.5

The old read path built one CPU read from two independent words:

- PIO0: D0-D7 + lower address
- PIO1: upper address

v1.5 uses a diagnostic path where **one PIO0 instruction captures GP2..GP31 in one ISR word**.

For the 512 KiB Tom & Jerry LoROM, that one word contains all bits needed to compute the physical ROM offset:

- D0-D7
- A0-A19
- A15 to identify the upper-half LoROM window

A20-A23 are not needed for a 512 KiB physical ROM offset because those high address bits only select mirrors of the same 512 KiB image.

No wiring changes are required.

## Build

Upload this folder to GitHub and run the included **Build UF2** action. Flash:

`build/snes_rp2350b_single_pio_romdiag.uf2`

Keep the RP2350B powered before the SNES, as before.

## Test

Close PuTTY, RA2Snes and all other COM7 users, then run:

```powershell
python .\atomic_rom_verify.py --port COM7 --rom "C:\Users\flavi\Downloads\RA2Snes-windows-x64\Tom and Jerry (USA).sfc" --seconds 60
```

Play normally for the minute.

### How to interpret

If `trusted` jumps from the old ~45–50% to roughly 95–100%, the old dual-PIO pairing was the main problem.

If it remains around ~45–50%, then pairing is not the explanation; the next suspects are an actual ROM/revision mismatch or a remaining error among the A0-A19 physical mapping.

After this diagnostic, flash the normal RA bridge firmware again before using RA2Snes.

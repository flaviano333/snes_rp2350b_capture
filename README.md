# SNES RP2350B v1.7 — Wide Timing Sweep

The v1.6 result showed an important flaw in the previous timing diagnostic:
**all eight slots were still ~98% PHI2-high**. That means the old sweep covered
too little time and never really crossed the bus phase transition.

v1.7 fixes that.

## What changes

- 16 snapshots per `/RD` event instead of 8.
- Capture PIO runs at `clkdiv=2.0`.
- Consecutive snapshots are about **53 ns apart** at the normal 150 MHz system clock.
- S0 through S15 span about **800 ns**.
- GP0 is still captured, so the result table shows exactly where PHI2 changes.

No wiring changes are required.

## Build and flash

This is a **new firmware**. Upload the project contents to GitHub and run the build workflow.

The generated UF2 is:

```text
snes_rp2350b_wide_timing_sweep.uf2
```

The workflow uses `build/*.uf2`.

## Run

Close PuTTY, RA2Snes and anything else using COM7:

```powershell
python .\wide_timing_sweep_verify.py --port COM7 --rom "C:\Users\flavi\Downloads\RA2Snes-windows-x64\Tom and Jerry (USA).sfc" --seconds 60
```

Play normally.

Send the entire `--- FINAL TIMING SWEEP ---` table and `BEST SLOT`.

The key check is no longer only the match percentage. The `PHI2 high` column
must visibly change across the 16 slots. If it does not, the sweep still did not
cover the relevant transition and we should not draw conclusions about timing.

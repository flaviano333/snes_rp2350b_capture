# SNES RP2350B v1.6 — Timing Sweep

This is a **diagnostic firmware**.

The v1.5 single-PIO test stayed near 50% ROM match, and the pinmap probe found no
A0-A19 swap that improved the result. v1.6 therefore tests the next hypothesis:
the previous sample instant may be outside the period where the SNES address and
ROM data are valid together.

## What v1.6 changes

For every `/RD` assertion, one PIO state machine captures **eight complete
GP0..GP31 snapshots** in succession.

It does **not** wait for the PHI2 falling edge.

The snapshots include:

- GP0 = PHI2
- GP1 = /WR
- GP2..GP9 = D0..D7
- GP10..GP31 = the physical address wiring used for A0..A19

This lets the PC compare the exact same read event at several timing positions.

## Build / flash

Upload the folder contents to the GitHub repository and run **Build UF2**.

The workflow deliberately uploads:

```text
build/*.uf2
```

so it will not repeat the old artifact-name problem.

Flash the generated:

```text
snes_rp2350b_timing_sweep.uf2
```

## Run

Close PuTTY, RA2Snes and anything else using COM7.

```powershell
python .\timing_sweep_verify.py --port COM7 --rom "C:\Users\flavi\Downloads\RA2Snes-windows-x64\Tom and Jerry (USA).sfc" --seconds 60
```

Play normally during the test.

Send ChatGPT the complete `--- FINAL TIMING SWEEP ---` table and the `BEST SLOT`.

## Interpretation

- If one slot jumps close to 100%, the old sample timing was the problem.
- If one slot improves substantially but is not clean, we can make a finer timing
  scan around that slot.
- If all eight stay near ~50%, timing alone is not sufficient and the next test
  should include the full bank-selection context or independently identify/read
  the physical cartridge ROM.

No wiring changes are required.

# SNES RP2350B SIO diagnostic v0.5

This diagnostic intentionally does **not** use PIO or DMA. It samples the raw GPIO pads for 100 ms every 500 ms and reports observed transitions on:

- GP0 = PHI2 from SNES
- GP1 = /WR from SNES
- GP36 = jumper copy of PHI2 (GP0 -> GP36)
- GP35 = jumper copy of /WR (GP1 -> GP35)

Use it to distinguish wiring/jumper issues from PIO/DMA firmware issues.

Expected with SNES running: PHI2 edge counts should be clearly non-zero. /WR should also normally show non-zero edges and some LOW samples. The duplicated pins should broadly track their originals (counts need not be identical because this is software polling, not exact measurement).

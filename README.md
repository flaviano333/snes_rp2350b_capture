# SNES RP2350B Capture v0.6 — no external synchronization jumpers

This diagnostic uses the RP2350 PIO v1 cross-PIO IRQ feature to synchronize PIO0 and PIO1 internally.

## Wiring

- GP0 = PHI2
- GP1 = /WR
- GP2..GP9 = D0..D7
- GP10..GP19 = A0..A9
- GP20 = not a SNES signal (dummy gap / onboard WS2812 pin)
- GP21..GP34 = A10..A23
- SNES GND = RP2350B GND

**No GP0→GP36 or GP1→GP35 jumper is used in v0.6. GP35 and GP36 can remain disconnected.**

## How synchronization works

PIO0 sees GP0/GP1 and the low data/address pins. On a write cycle while PHI2 is high it executes `irq next 0`. On RP2350, from PIO0 this sets IRQ0 in PIO1. PIO1 is waiting on its own IRQ0, snapshots GP18..GP34, and pushes that high address sample. PIO0 then waits for PHI2 to fall and snapshots D0-D7 + A0-A7.

The firmware uses DMA for both PIO RX FIFOs, reconstructs the 24-bit SNES address, prints 64 paired writes, then rearms automatically forever.

## Build

Use the bundled GitHub Actions workflow (`Actions` → `Build UF2`), then download the artifact `snes-rp2350b-capture-uf2`.

## Power-up order

1. SNES OFF.
2. Power/program the RP2350B over USB.
3. Open the serial terminal.
4. Wait for `READY`.
5. Turn the SNES ON.

Power down in the reverse order: SNES first, RP2350B second.

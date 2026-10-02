# SNES RP2350B WRAM capture v0.7

Diagnostic/experimental firmware for the SpotPear RP2350B-MINI-A used as a passive SNES bus monitor.

## Wiring

- GP0  <- SNES PHI2
- GP1  <- SNES /WR
- GP2..GP9 <- D0..D7
- GP10..GP19 <- A0..A9
- GP20 is skipped
- GP21..GP34 <- A10..A23
- SNES GND -> RP2350B GND

No GP35/GP36 jumpers are required.

## What v0.7 changes

This version keeps the v0.6 two-PIO capture design, but filters the output so only accesses that map to SNES WRAM are printed:

- direct WRAM: `$7E:0000-$7F:FFFF`
- WRAM mirrors: `$00-$3F:0000-$1FFF` and `$80-$BF:0000-$1FFF`

It also maintains a 128 KiB *partial* software mirror. Bytes are marked as known only after an observed write, because this passive A-bus prototype does not know the power-on WRAM contents and may not yet observe every possible path that can modify WRAM.

Output example:

```
WRAM $013FB <- F4   via $00:13FB mirror (first seen)
WRAM $11234 <- CD   via $7F:1234 direct (was 00)
summary: direct=10 mirror=43 other=203 new_known=12
totals: bus_writes=4096 wram_writes=731 known_WRAM=284/131072 bytes (0.22%)
```

## Important

Fix any cartridge/connector mechanical contact issue before treating captured addresses as trustworthy.

Keep the RP2350B powered before powering the SNES when directly connecting SNES bus signals to RP2350B GPIOs.

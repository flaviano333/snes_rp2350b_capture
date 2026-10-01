# SNES RP2350B Bus Capture v0.1

Diagnostic firmware source for the SpotPear RP2350B-MINI-A used as a passive SNES cartridge-bus sniffer.

## Wiring expected

- GPIO0  <- SNES PHI2
- GPIO1  <- SNES /WR
- GPIO2..9 <- D0..D7
- GPIO10..19 <- A0..A9
- GPIO20 unused (on-board WS2812)
- GPIO21..34 <- A10..A23
- **Jumper GPIO1 -> GPIO35** (duplicates /WR for the high GPIO PIO window)
- **Jumper GPIO0 -> GPIO36** (duplicates PHI2 for the high GPIO PIO window)
- SNES GND <-> RP2350B GND

All SNES-connected GPIOs are configured INPUT ONLY.

## Important power order

Because the prototype connects SNES logic directly to RP2350B GPIOs:

1. Power/connect the RP2350B by USB first.
2. Wait for the serial terminal to print `READY`.
3. Turn on the SNES.
4. When finished, turn off the SNES first.
5. Only then disconnect USB from the RP2350B.

## Build requirements

- Pico SDK 2.2.x or newer
- ARM GNU toolchain supported by the Pico SDK
- CMake + Ninja/Make

Set `PICO_SDK_PATH` and run:

```sh
cmake -S . -B build -G Ninja
cmake --build build
```

The resulting file is:

`build/snes_rp2350b_capture.uf2`

## USB output

The firmware enables USB CDC stdio. Open the serial port at any baud rate (USB CDC ignores the configured baud) and reset the board with the SNES OFF. It prints `READY`; then turn on the console. It records the first 64 write cycles using two PIO blocks plus DMA, then prints full 24-bit address and 8-bit data values.

This is a diagnostic first version. It does not yet reconstruct writes performed through all SNES B-bus/WRAM mechanisms.

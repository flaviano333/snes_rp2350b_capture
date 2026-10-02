# SNES RP2350B v1.8 — /ROMSEL Qualified Diagnostic

This test adds **one wire** so the RP2350 can distinguish genuine cartridge-ROM
reads from every other A-bus read.

## New wire

With the SNES powered OFF:

```text
SNES cartridge connector pin 49  /ROMSEL  ->  RP2350 GP36
```

Keep all existing wiring unchanged.

**Do not use GP20 for this.** On the SpotPear RP2350B MINI-A it is connected to
the onboard WS2812/RGB LED.

GP36 is used only as an input.

## Why this is useful

The SNES asserts `/ROMSEL` specifically for cartridge-ROM accesses. The v1.8
trigger only records a cycle when:

```text
/RD = LOW
AND
/ROMSEL = LOW
```

That removes WRAM, MMIO and unrelated bus reads from the ROM comparison.

For a 512 KiB LoROM, once a cycle is known to be a ROM cycle, the physical ROM
offset is fully determined by CPU A0..A14 and A16..A19, all of which are already
inside the atomic GP0..GP31 capture.

## Safety

- RP2350 powered before the SNES.
- Common ground.
- GP36 is input-only.
- Do not feed SNES +5 V into the RP board power rail.
- Turn the SNES off before unplugging the RP2350.

## Build / flash

This is a **new UF2**.

Upload the project to GitHub and run the workflow. It uploads `build/*.uf2`.

Expected firmware:

```text
snes_rp2350b_romsel_qualified.uf2
```

## Run

```powershell
python .\romsel_edge_verify.py --port COM7 --rom "C:\Users\flavi\Downloads\RA2Snes-windows-x64\Tom and Jerry (USA).sfc" --seconds 60
```

Send ChatGPT:
- `ROMSEL-QUALIFIED SUMMARY`
- `BEST ADDRESS/DATA COMBINATIONS`
- `PHYSICALLY EXPECTED`
- `BEST`
- `INTERPRETATION`

# SNES RP2350B RA Bridge v1.4 — Raw Address Mapper

This is based on v1.3 Atomic Snapshot. It keeps the RA bridge/snapshot features and adds a diagnostic mode that exposes the **raw physical address GPIO state before `ADDRESS_GPIO[]` remapping**.

The goal is to infer the real order of the still-unverified address lines **A14..A21** automatically, without resoldering.

## Existing measured wiring kept in v1.4

- PHI2 = GP0
- /WR = GP1
- D0-D7 = GP2-GP9
- /RD = GP35
- GP20 remains unused
- A0 -> GP12
- A1 -> GP11
- A2 -> GP10
- A3 -> GP13
- A4 -> GP14
- A5 -> GP16
- A6 -> GP15
- A7 -> GP17
- A8 -> GP19
- A9 -> GP21
- A10 -> GP22
- A11 -> GP23
- A12 -> GP24
- A13 -> GP25
- A22 -> GP34
- A23 -> GP18

The mapper tests all `8! = 40,320` assignments of **A14..A21 to GP26..GP33**.

## New firmware command

```text
DEBUG RAWREAD 128
```

Example output:

```text
RAWREAD L=9A H=12345 D=CD A=00B28C
```

- `L` is raw GP10..GP17 (`bit0=GP10 ... bit7=GP17`)
- `H` is raw GP18..GP34 (`bit0=GP18 ... bit16=GP34`)
- `D` is the captured data byte
- `A` is the address reconstructed by the *current* firmware map, only for reference

The Python mapper uses L/H/D, not A.

## 1. Build and flash the v1.4 UF2

Upload the project to GitHub as before. The included workflow builds the UF2. Flash that UF2 to the RP2350B.

No wiring changes are needed.

## 2. Install Python requirements

```powershell
python -m pip install -r requirements.txt
```

## 3. Close anything using COM7

Close PuTTY and the RA2Snes Python bridge. Only the mapper should own COM7 during this test.

## 4. Run the automatic mapper

```powershell
python .\infer_address_map.py --port COM7 --rom "C:\Users\flavi\Downloads\RA2Snes-windows-x64\Tom and Jerry (USA).sfc" --seconds 60
```

Play normally while it runs. Moving through different parts of a stage is useful because it makes the CPU execute/read more ROM locations.

The script:

1. continuously arms `DEBUG RAWREAD` in small chunks;
2. aggregates repeated raw physical addresses;
3. keeps only stable observations by default (`>=2` hits and `>=98%` purity);
4. compares them with the reference ROM;
5. brute-forces all 40,320 possible A14..A21 / GP26..GP33 permutations;
6. prints the current firmware score and the best candidate maps;
7. saves the raw capture as `raw_mapper_capture.csv`.

## Reading the result

A decisive result may look like:

```text
--- CURRENT FIRMWARE ASSUMPTION ---
A14->GP26 A15->GP27 ...
matches=1800 ...

--- BEST CANDIDATES ---
#1: matches=6200 comparable=6400 rate=96.875%  <== BEST
    A14->GP29 A15->GP27 A16->GP26 ...
```

If one mapping clearly wins, the script prints a complete replacement:

```c
static const uint8_t ADDRESS_GPIO[24] = {
    ...
};
```

That can be applied entirely in software. **Do not resolder first.**

## Important: ambiguous high bits are possible

Tom and Jerry (USA) is a 512 KiB LoROM. A ROM that small does not necessarily decode every high CPU address bit; some banks are mirrors. Therefore two or more physical mappings can be indistinguishable from ROM data alone, especially for high bank bits.

v1.4 detects this. If multiple maps tie exactly, it prints a consensus like:

```text
A14 -> GP29 FIXED
A15 -> GP27 FIXED
...
A20 -> GP32/GP33 AMBIGUOUS
A21 -> GP32/GP33 AMBIGUOUS
```

In that situation we can still software-remap the lines that are uniquely identified, then design a second constraint/test only for the ambiguous ones.

## Useful options

Capture longer:

```powershell
python .\infer_address_map.py --port COM7 --rom "C:\path\Tom and Jerry (USA).sfc" --seconds 120
```

Require more repeated observations:

```powershell
python .\infer_address_map.py --port COM7 --rom "C:\path\Tom and Jerry (USA).sfc" --seconds 90 --min-hits 3 --purity 0.99
```

## RA2Snes compatibility

The v1.4 firmware retains the v1.3 `SNAP`, `RBIN`, WRAM mirror and other commands, so the included `ra_usb2snes_bridge.py` and `ra_usb2snes_bridge_trace.py` remain usable after the mapping test.

## Electrical safety

Same prototype precautions as before:

- only the RP2350B 5-V-tolerant GPIO range being used for SNES signals;
- RP powered before SNES;
- SNES off before unplugging RP USB;
- common ground;
- SNES-connected pins remain inputs only.

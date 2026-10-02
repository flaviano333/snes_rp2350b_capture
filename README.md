# SNES RP2350B v1.8.2 — /ROMSEL Qualified Diagnostic

This fixes an important RP2350B PIO addressing bug in v1.8/v1.8.1.

PIO2 is configured with:

```text
GPIO base = 16
```

PIO instructions such as `WAIT GPIO n` use `n` **relative to that base**.
Therefore physical GP35 must be referenced in PIO assembly as:

```text
35 - 16 = 19
```

The incorrect builds used `wait gpio 35`, so the trigger was not actually
waiting on physical GP35 as intended.

v1.8.2 uses:

```pio
wait 0 gpio 19
...
wait 1 gpio 19
```

while `JMP PIN` remains configured through the SDK for physical GP36 (/ROMSEL).

## Wiring

```text
A23      -> GP18
/RD      -> GP35
/ROMSEL  -> GP36
```

## Extra firmware verification

The Python verifier now asks the RP2350 for `INFO` before starting. It must see:

```text
version=1.8.2
rd=GP35
romsel=GP36
pio2_base=16
rd_wait_index=19
```

If you accidentally flash an older UF2, the test will stop instead of silently
producing misleading numbers.

## Build / flash

Build a new UF2 from this project and flash it.

Then run:

```powershell
python .\romsel_edge_verify.py --port COM7 --rom "C:\Users\flavi\Downloads\RA2Snes-windows-x64\Tom and Jerry (USA).sfc" --seconds 60
```

At startup, first look for:

```text
Firmware handshake OK:
  INFO version=1.8.2 ...
```

Only trust the ROM comparison if that appears.

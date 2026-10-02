# SNES RP2350B WRAM Bridge v0.8

Experimental firmware for the SpotPear RP2350B-MINI-A used as a passive SNES bus monitor.

This version turns the v0.7 WRAM mirror into a simple USB serial memory bridge.

## Wiring

Unchanged from v0.6/v0.7:

- GP0 <- SNES PHI2
- GP1 <- SNES /WR
- GP2..GP9 <- D0..D7
- GP10..GP19 <- A0..A9
- GP20 is skipped
- GP21..GP34 <- A10..A23
- SNES GND -> RP2350B GND

No GP35/GP36 jumpers are required.

## Commands

Open the USB CDC COM port (PuTTY works for text commands) and type:

```text
HELP
INFO
READ 013FB
READSNES 00:13FB
HEX 01300 100
CLEAR
PING
```

Hexadecimal is used for addresses and lengths.

`READ` uses the canonical 128 KiB WRAM offset (`00000` to `1FFFF`).

`READSNES` accepts a SNES address and maps direct `$7E/$7F` accesses or the low-bank 8 KiB WRAM mirrors to the canonical WRAM offset.

`HEX` prints a human-readable region. Unknown bytes are displayed as `??`.

## DUMPBIN protocol

`DUMPBIN` is intended for software, not PuTTY. The device sends:

```text
BIN1 131072 16384 <known_bytes> <completed_batches>\n
```

followed immediately by:

1. 131072 raw WRAM bytes
2. 16384 raw bytes containing the validity bitmap (1 bit per WRAM byte)
3. `\nEND BIN1\n`

The raw payload is sent with CR/LF translation disabled.

A helper script `wram_client.py` is included.

Examples:

```powershell
pip install pyserial
python wram_client.py COM7 info
python wram_client.py COM7 read 013FB
python wram_client.py COM7 read-snes 00:13FB
python wram_client.py COM7 dump wram.bin
```

The `dump` command creates `wram.bin` and `wram.bin.valid`.

## Important limitation

This is still a **partial mirror**. A byte becomes known only after the adapter observes a write to it. This A-bus prototype may also miss WRAM modifications made through paths not yet monitored, and the current capture engine rearms in finite DMA batches rather than using a lossless ping-pong ring.

So this v0.8 is a bridge/protocol milestone, not yet the final RetroAchievements transport.

Fix any cartridge/connector contact issue before treating captured data as trustworthy.

Keep the RP2350B powered before powering the SNES when directly connecting SNES bus signals to RP2350B GPIOs.

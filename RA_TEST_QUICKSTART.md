# RA2Snes test — corrected A23 setup

Current wiring:

```text
A23     -> GP18
/RD     -> GP35
/ROMSEL -> GP36
```

Your manual test already showed direct `$7E/$7F` WRAM traffic and:

```text
0287A = KNOWN
02AF2 = KNOWN
```

So the next step is the actual RA2Snes bridge test.

## If v1.3.1 fixed firmware is already flashed

Do **not** reflash anything.

1. Close the PowerShell serial object if it is still open:

```powershell
$port.Close()
```

2. In this folder run:

```powershell
.\start_ra_test.ps1
```

Or manually:

```powershell
python -m pip install -r .\requirements.txt
python .\ra_usb2snes_bridge.py --port COM7 --rom "C:\Users\flavi\Downloads\RA2Snes-windows-x64\Tom and Jerry (USA).sfc" --trace-ra --trace-snapshots
```

3. Leave that terminal open.
4. Start RA2Snes v1.1.2.
5. Use Softcore.
6. Play normally.

Expected bridge messages include:

```text
RA client connected
Atomic RA snapshot ACTIVE: ...
[SNAP] ... unknown=0
RA WRAM working set FULLY KNOWN: ...
```

The important addresses should appear as `KNOWN` in `[RA-MEM]`, including
`+0287A` and `+02AF2` if RA2Snes requests them.

Do not run QUsb2Snes or SNI simultaneously because this bridge owns
`ws://127.0.0.1:23074`.

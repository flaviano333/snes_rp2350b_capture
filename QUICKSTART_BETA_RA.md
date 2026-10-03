# Quick test

With the corrected v1.3.1 firmware already flashed:

```powershell
python .\ra_usb2snes_bridge_beta.py --port COM7 --rom "C:\Users\flavi\Downloads\RA2Snes-windows-x64\Tom and Jerry (USA).sfc" --beta-translate --trace-ra --trace-snapshots
```

Then open RA2Snes v1.1.2 in Softcore and start the game from the title screen.

Look especially for:

```text
+01558 ... KNOWN ... map=+0155A
+0155C ... KNOWN ... map=+0155E
+0155E ... KNOWN ... map=+01560
+01E45 ... KNOWN ... map=+01E47
```

and for `unknown=0`.

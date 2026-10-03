# Tom & Jerry BETA compatibility layer — auto-map v1.5

The bridge presents the supported final USA ROM to RA2Snes for identification,
while live WRAM comes from the physical SNES running the BETA/Nightfall build.

ROM pair used to generate the map:

- Final USA MD5: `bc16be2e9c7e170f7cd10da919f3e099`
- BETA MD5: `b690ff0d506fc20faf670cd42c3dc01f`

Static 65C816 analysis found 85 explicit changed WRAM references. They all support
one non-conflicting range rule in the mapped code region:

```text
FINAL +00EA6 .. +01E69
BETA = FINAL + 2
```

Examples:

```text
F51242 -> F51244
F51564 -> F51566
F51E69 -> F51E6B
F5287A -> F5287A   (outside mapped range)
F52AF2 -> F52AF2   (outside mapped range)
```

The implementation is in `tomjerry_address_map.py`. The original recovered v1.4
bridge is preserved as `ra_usb2snes_bridge_beta_original_v1.4.py`.

Run the updated bridge with `INICIAR_BRIDGE.bat` or:

```powershell
python .\ra_usb2snes_bridge_beta.py --port COM7 --rom "C:\path\Tom and Jerry (USA).sfc" --beta-translate --trace-ra --trace-snapshots
```

The range rule is a strongly supported static inference, not a dynamic proof of
every individual byte in the entire interval. Trace output remains available for
spot-checking ambiguous behavior during real gameplay.

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$Port = "COM7"
$Rom = "C:\Users\flavi\Downloads\RA2Snes-windows-x64\Tom and Jerry (USA).sfc"

Write-Host ""
Write-Host "=== RP2350B -> RA2Snes test ==="
Write-Host "Port: $Port"
Write-Host "ROM : $Rom"
Write-Host ""
Write-Host "Close any PowerShell/PuTTY/serial monitor that currently owns COM7."
Write-Host "Do not run QUsb2Snes/SNI on port 23074 at the same time."
Write-Host ""

python -m pip install -r .\requirements.txt
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

python .\ra_usb2snes_bridge.py --port $Port --rom $Rom --trace-ra --trace-snapshots

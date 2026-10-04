$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$port = "COM7"
$romDir = Join-Path $PSScriptRoot "ROMs"

if (-not (Test-Path $romDir)) {
    New-Item -ItemType Directory -Path $romDir | Out-Null
}

$roms = @(Get-ChildItem -Path $romDir -Recurse -File | Where-Object { $_.Extension -ieq ".sfc" -or $_.Extension -ieq ".smc" })
if ($roms.Count -eq 0) {
    Write-Host "ERRO: a pasta ROMs nao possui nenhuma referencia .sfc/.smc." -ForegroundColor Red
    Read-Host "Enter para sair"
    exit 1
}

Write-Host "SNES RA v2.2 DIAGNOSTICO - AUTO-ROM"
Write-Host ("Porta: " + $port)
Write-Host ("Referencias encontradas: " + $roms.Count)
Write-Host "Mostra deteccao da ROM, polls, snapshots e READ-REPAIR."
Write-Host ""

py -3 .\ra_usb2snes_bridge.py --port $port --rom-dir $romDir --poll-gap-ms 8 --max-snapshot-age-ms 50 --trace-polls --trace-wramsel --trace-ra --trace-snapshots

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$port = "COM7"
$roms = @(Get-ChildItem -File | Where-Object { $_.Extension -ieq ".sfc" -or $_.Extension -ieq ".smc" })

if ($roms.Count -eq 0) {
    Write-Host "ERRO: coloque a ROM suportada pelo RetroAchievements nesta pasta." -ForegroundColor Red
    Read-Host "Enter para sair"
    exit 1
}

if ($roms.Count -gt 1) {
    Write-Host "ERRO: deixe somente UMA ROM .sfc/.smc nesta pasta." -ForegroundColor Red
    $roms | ForEach-Object { Write-Host ("  " + $_.Name) }
    Read-Host "Enter para sair"
    exit 1
}

Write-Host "SNES RP2350B UNIVERSAL v1.6O3 - SKIP GP5 + GP26"
Write-Host ("Porta: " + $port)
Write-Host ("ROM de identificacao RA: " + $roms[0].Name)
Write-Host "Bridge: 1:1, sem traducao Tom & Jerry."
Write-Host ""

py -3 .\ra_usb2snes_bridge.py --port $port --rom $roms[0].FullName

param(
    [string]$Port = "COM7",
    [string]$Rom = ""
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if ([string]::IsNullOrWhiteSpace($Rom)) {
    $localRom = Join-Path $PSScriptRoot "Tom and Jerry (USA).sfc"
    if (Test-Path $localRom) {
        $Rom = $localRom
    } else {
        Write-Host ""
        Write-Host "A ROM FINAL USA e usada apenas para o RA2Snes identificar o jogo."
        $Rom = Read-Host "Cole o caminho de Tom and Jerry (USA).sfc"
        $Rom = $Rom.Trim('"')
    }
}

if (-not (Test-Path $Rom)) {
    throw "ROM nao encontrada: $Rom"
}

Write-Host ""
Write-Host "Iniciando bridge Tom & Jerry BETA v1.6 exact-map..."
Write-Host "Porta: $Port"
Write-Host "ROM de identificacao: $Rom"
Write-Host "Feche PuTTY, QUsb2Snes e qualquer programa usando $Port ou a porta TCP 23074."
Write-Host ""

python .\ra_usb2snes_bridge_beta.py `
    --port $Port `
    --rom $Rom `
    --beta-translate `
    --trace-ra `
    --trace-snapshots

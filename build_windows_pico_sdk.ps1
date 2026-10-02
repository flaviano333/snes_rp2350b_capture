$ErrorActionPreference = "Stop"
if (-not $env:PICO_SDK_PATH) {
    throw "Set PICO_SDK_PATH to your pico-sdk folder first."
}
cmake -S . -B build -G Ninja
cmake --build build
Write-Host "UF2 files:"
Get-ChildItem build\*.uf2

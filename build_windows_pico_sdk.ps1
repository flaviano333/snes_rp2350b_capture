$ErrorActionPreference = "Stop"
if (-not $env:PICO_SDK_PATH) {
    Write-Host "PICO_SDK_PATH nao esta definido. Abra este projeto pelo ambiente oficial do Raspberry Pi Pico SDK/VS Code ou defina a variavel primeiro."
    exit 1
}
cmake -S . -B build -G Ninja
cmake --build build
Write-Host ""
Write-Host "UF2 gerado em: build\snes_rp2350b_capture.uf2"

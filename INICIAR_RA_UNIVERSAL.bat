@echo off
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0INICIAR_RA_UNIVERSAL.ps1"
pause

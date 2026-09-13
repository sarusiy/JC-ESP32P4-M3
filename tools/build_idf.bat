@echo off
set IDF_TOOLS_PATH=C:\Espressif
call C:\esp\v6.1-beta1\esp-idf\export.bat
cd /d C:\projects\JC-ESP32P4-M3
idf.py build
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0publish-firmware-to-drive.ps1"

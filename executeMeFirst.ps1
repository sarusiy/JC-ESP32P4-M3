$ErrorActionPreference = "Stop"

$proj = "C:\projects\JC-ESP32P4-M3"
Set-Location $proj

Write-Host "Stopping stale build processes..."
Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match 'python|cmake|ninja|gmake'
} | ForEach-Object {
    try {
        Stop-Process -Id $_.ProcessId -Force -ErrorAction Stop
    }
    catch {
        Write-Host "Could not stop PID $($_.ProcessId): $($_.Exception.Message)"
    }
}

Write-Host "Removing generated PlatformIO/ESP-IDF state..."
Remove-Item .pio -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item managed_components -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "Starting clean PlatformIO build..."
python -m platformio run -d . -e jc_esp32p4_m3

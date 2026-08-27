$ErrorActionPreference = "Stop"

$proj = "C:\projects\JC-ESP32P4-M3"
Set-Location $proj

Write-Host "Stopping stale build processes..."
Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match 'python|cmake|ninja|gmake|idf.py'
} | ForEach-Object {
    try {
        Stop-Process -Id $_.ProcessId -Force -ErrorAction Stop
    }
    catch {
        Write-Host "Could not stop PID $($_.ProcessId): $($_.Exception.Message)"
    }
}

Write-Host "Removing generated ESP-IDF state..."
Remove-Item build -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item managed_components -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item dependencies.lock -Force -ErrorAction SilentlyContinue

Write-Host "Starting clean ESP-IDF build..."
if (Get-Command idf.py -ErrorAction SilentlyContinue) {
    idf.py fullclean
    idf.py set-target esp32p4
    idf.py build
} else {
    Write-Error "idf.py was not found in PATH. Use the Espressif IDE environment or activate your ESP-IDF shell first."
}

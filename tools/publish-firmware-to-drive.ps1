<#
Uploads the built firmware image (plus a small companion .version.txt) to
the shared "CarTheftGuard" Drive folder, overwriting the previous build in
place (same file ID/link every time). The phone app's About tab reads this
same folder automatically (see CarTheftGuard's DriveUpdates.java) -- this
script is what makes a build show up there without copying the .bin to the
phone by hand. The companion version file lets the app show what version
you're ABOUT to push, next to the "Firmware version" row showing what's
currently running, before you commit to the push (added 2026-09-15 after a
debugging session where it wasn't obvious what a pending update actually
changed).
Requires the "gdrive" rclone remote to already be configured (rclone config).
#>
$rclone = "C:\Users\yossi\AppData\Local\Microsoft\WinGet\Packages\Rclone.Rclone_Microsoft.Winget.Source_8wekyb3d8bbwe\rclone-v1.75.0-windows-amd64\rclone.exe"
$bin = "C:\projects\JC-ESP32P4-M3\build\JC-ESP32P4-M3.bin"

if (-not (Test-Path $bin)) {
    Write-Error "Firmware image not found at $bin - build it first (idf.py build)"
    exit 1
}

& $rclone copyto $bin "gdrive:CarTheftGuard/JC-ESP32P4-M3.bin"
if ($LASTEXITCODE -ne 0) {
    Write-Error "Upload failed"
    exit 1
}

# Same string ESP-IDF embeds in the image itself (esp_app_desc_t.version),
# derived the same way (git describe) -- see main.c's health_http_handler.
$version = git -C "C:\projects\JC-ESP32P4-M3" describe --always --dirty
$versionFile = Join-Path $env:TEMP "JC-ESP32P4-M3.version.txt"
Set-Content -Path $versionFile -Value $version -NoNewline
& $rclone copyto $versionFile "gdrive:CarTheftGuard/JC-ESP32P4-M3.version.txt"
Remove-Item $versionFile

Write-Output "Uploaded ($version). Shareable link:"
& $rclone link "gdrive:CarTheftGuard/JC-ESP32P4-M3.bin"

<#
Uploads the built firmware image to the shared "CarTheftGuard" Drive folder,
overwriting the previous build in place (same file ID/link every time). The
phone app's About tab reads this same folder automatically (see
CarTheftGuard's DriveUpdates.java) -- this script is what makes a build show
up there without copying the .bin to the phone by hand.
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

Write-Output "Uploaded. Shareable link:"
& $rclone link "gdrive:CarTheftGuard/JC-ESP32P4-M3.bin"

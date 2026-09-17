<#
Live serial monitor for the JC-ESP32S3-CAN bring-up board -- just
idf.py monitor, wrapped so it's a double-clickable tool instead of typing
the export.ps1 + cd + idf.py dance every time.

Auto-detects the COM port if not given: prefers "USB Serial Device"
(this board's *native* USB-Serial/JTAG port, GPIO19/20 -- the one that
needs manual BOOT+RST before flashing/monitoring, see
HARDWARE_MIGRATION.md) since that's the port proven to work end-to-end
tonight; "CH343" (the board's *other* USB-C port, through its onboard
USB-UART bridge chip) auto-resets cleanly for flashing but has been
unreliable for monitoring so far -- pass -Port COM9 (or whatever it
enumerates as) explicitly if you want to try it anyway.

If nothing auto-detects, or monitoring shows nothing: this board needs
BOTH USB-C ports powered at once to boot reliably (see
HARDWARE_MIGRATION.md's brownout section) -- plug a second cable from
any 5V source (wall charger, power bank, another PC port) into the
board's other USB-C port before monitoring.
#>
param(
    [string]$Port
)

$ErrorActionPreference = 'Stop'

if (-not $Port) {
    # @() forces array-wrapping -- without it, a single WMI match's .Count
    # can come back $null instead of 1 (WMI/ManagementObject instances
    # don't reliably get PowerShell's synthetic Count property the way
    # plain objects do), silently falling through to the "not found"
    # branch even when exactly one real match exists.
    $candidates = @(Get-WmiObject Win32_SerialPort | Where-Object { $_.Description -match 'USB Serial Device|CH343' })
    if ($candidates.Count -eq 1) {
        $Port = $candidates[0].DeviceID
        Write-Host "Auto-detected port: $Port ($($candidates[0].Description))" -ForegroundColor DarkGray
    } elseif ($candidates.Count -gt 1) {
        Write-Host "Multiple candidate ports found -- pass -Port explicitly:" -ForegroundColor Yellow
        $candidates | ForEach-Object { Write-Host "  $($_.DeviceID)  $($_.Description)" }
        exit 1
    } else {
        Write-Host "No board serial port found. Is it plugged in? (Also check both USB-C ports are" -ForegroundColor Yellow
        Write-Host "powered -- see this script's top comment about the dual-power brownout issue.)" -ForegroundColor Yellow
        exit 1
    }
}

Write-Host "If nothing shows up, or the board says 'waiting for download': tap RST once" -ForegroundColor DarkGray
Write-Host "(the native USB-Serial/JTAG port re-enters download mode just from being" -ForegroundColor DarkGray
Write-Host "opened -- see HARDWARE_MIGRATION.md)." -ForegroundColor DarkGray
Write-Host ""

& C:\esp\v6.1-beta1\esp-idf\export.ps1
Set-Location C:\projects\JC-ESP32S3-CAN
idf.py -p $Port monitor

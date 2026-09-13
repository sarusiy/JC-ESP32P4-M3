<#
Live monitor for the P4's GPS bridge (NEO-6M/8M module on UART1,
GPIO34=RX/GPIO35=TX -- see main.c GPS_UART_*). Polls GET /api/gps
once a second and prints the parsed fix.

The board is AP-only (no home Wi-Fi/STA support) -- join this PC's Wi-Fi
to CarTheftGuard-P4 (password theftguard2026) before running this.
#>
param(
    [string]$BoardIp = '192.168.4.1',
    [int]$PollSeconds = 1
)

$ErrorActionPreference = 'Stop'

Write-Host "Board is AP-only -- join this PC's Wi-Fi to CarTheftGuard-P4 (password theftguard2026) first." -ForegroundColor DarkGray

while ($true) {
    try {
        $gps = Invoke-RestMethod -Uri "http://$BoardIp/api/gps" -Method Get -TimeoutSec 5
        $ts = Get-Date -Format 'HH:mm:ss'
        if ($gps.fix_valid) {
            Write-Host ("[{0}] FIX lat={1:N6} lon={2:N6} speed={3:N1}km/h heading={4:N1}deg sats={5} utc={6} date={7}" -f `
                $ts, $gps.lat, $gps.lon, $gps.speed_kmh, $gps.heading_deg, $gps.satellites, $gps.utc_time, $gps.utc_date)
        } else {
            Write-Host ("[{0}] Waiting for fix... sats={1}" -f $ts, $gps.satellites)
        }
    } catch {
        Write-Host "Waiting for board at http://$BoardIp -- is this PC joined to CarTheftGuard-P4?"
        Write-Host $_.Exception.Message
    }
    Start-Sleep -Seconds $PollSeconds
}

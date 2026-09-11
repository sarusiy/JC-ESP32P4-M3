<#
Live monitor for decoded OBD-II values (RPM, speed, coolant, throttle) --
the same data the app's Monitor tab shows. This is CAN data too, just
already parsed from the raw frames (see can-monitor.ps1 for raw frames).
Note: these only update while CAN mode is ACTIVE (Active mode is what
actually sends the Mode 01 queries -- Passive only listens). Switch mode
first via the Control tab or POST /api/can/mode.
#>
param(
    [string]$BoardIp = '192.168.1.180',
    [int]$PollSeconds = 1
)

$ErrorActionPreference = 'Stop'

while ($true) {
    try {
        $obd = Invoke-RestMethod -Uri "http://$BoardIp/api/obd" -Method Get -TimeoutSec 5
        $ts = Get-Date -Format 'HH:mm:ss'
        Write-Host ("[{0}] RPM={1} Speed={2}km/h Coolant={3}C Throttle={4}% SupportedPIDs={5}" -f `
            $ts, $obd.rpm, $obd.speed_kmh, $obd.coolant_c, $obd.throttle_pct, $obd.supported_pids)
    } catch {
        Write-Host "Waiting for board at http://$BoardIp ..."
        Write-Host $_.Exception.Message
    }
    Start-Sleep -Seconds $PollSeconds
}

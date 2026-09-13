<#
One-shot snapshot of the P4's GET /api/health endpoint -- uptime, restart
reason, heap, CPU idle. Useful first check after any power/USB hiccup:
restart_reason "POWERON" means the board actually lost power (not just a
soft reset), which is a strong hint to check the physical wiring/power
rail before suspecting firmware or the app.

The board is AP-only (no home Wi-Fi/STA support) -- join this PC's Wi-Fi
to CarTheftGuard-P4 (password theftguard2026) before running this.
#>
param(
    [string]$BoardIp = '192.168.4.1'
)

$ErrorActionPreference = 'Stop'

Write-Host "Board is AP-only -- join this PC's Wi-Fi to CarTheftGuard-P4 (password theftguard2026) first." -ForegroundColor DarkGray

try {
    $health = Invoke-RestMethod -Uri "http://$BoardIp/api/health" -TimeoutSec 5
    $health | ConvertTo-Json -Depth 5
} catch {
    Write-Host "Could not reach board at http://$BoardIp -- is this PC joined to CarTheftGuard-P4?"
    Write-Host $_.Exception.Message
}

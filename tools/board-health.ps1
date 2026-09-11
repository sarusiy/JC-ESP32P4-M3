<#
One-shot snapshot of the P4's GET /api/health endpoint -- uptime, restart
reason, heap, CPU idle. Useful first check after any power/USB hiccup:
restart_reason "POWERON" means the board actually lost power (not just a
soft reset), which is a strong hint to check the physical wiring/power
rail before suspecting firmware or the app.
#>
param(
    [string]$BoardIp = '192.168.1.180'
)

$ErrorActionPreference = 'Stop'

try {
    $health = Invoke-RestMethod -Uri "http://$BoardIp/api/health" -TimeoutSec 5
    $health | ConvertTo-Json -Depth 5
} catch {
    Write-Host "Could not reach board at http://$BoardIp -- is it on Wi-Fi?"
    Write-Host $_.Exception.Message
}

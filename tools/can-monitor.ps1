param(
    [string]$BoardIp = '192.168.1.180',
    [ValidateSet('summary','live')]
    [string]$Mode = 'summary',
    [int]$PollSeconds = 1
)

$ErrorActionPreference = 'Stop'

function Get-CanStatus {
    param([string]$Board)
    $uri = "http://$Board/api/can?after=$after"
    $resp = Invoke-RestMethod -Uri $uri -Method Get -TimeoutSec 5
    return $resp
}

$after = 0
while ($true) {
    try {
        $resp = Get-CanStatus -Board $BoardIp
        if ($null -ne $resp.latest) {
            $after = [long]$resp.latest
        }

        if ($Mode -eq 'summary') {
            $batch = if ($null -ne $resp.frames) { $resp.frames.Count } else { 0 }
            $modeName = if ($resp.passive) { 'PASSIVE' } else { 'ACTIVE' }
            $latest = if ($null -ne $resp.latest) { $resp.latest } else { 0 }
            $dropped = if ($null -ne $resp.dropped) { $resp.dropped } else { 0 }
            $hwOverflow = if ($null -ne $resp.hardware_overflow) { $resp.hardware_overflow } else { 0 }
            $isr = if ($null -ne $resp.isr_count) { $resp.isr_count } else { 0 }
            $timeout = if ($null -ne $resp.timeout_count) { $resp.timeout_count } else { 0 }
            Write-Host ("[{0}] Latest:{1} Batch:{2} Dropped:{3} HWOverflow:{4} ISR:{5} Timeout:{6} Mode:{7}" -f (Get-Date -Format 'HH:mm:ss'), $latest, $batch, $dropped, $hwOverflow, $isr, $timeout, $modeName)
        }
        else {
            if ($null -ne $resp.frames) {
                foreach ($f in $resp.frames) {
                    $timeMs = [math]::Round(($f.time_us / 1000.0), 2)
                    Write-Host ("[{0}] seq={1} time_ms={2} id=0x{3:X} dlc={4} data={5}" -f (Get-Date -Format 'HH:mm:ss'), $f.seq, $timeMs, $f.id, $f.dlc, $f.data)
                }
            }
        }
    }
    catch {
        Write-Host "Waiting for board at http://$BoardIp ..."
        Write-Host $_.Exception.Message
    }

    Start-Sleep -Seconds $PollSeconds
}

param(
    [string]$Port = 'COM3',
    [int]$TimeoutSec = 120,
    [Parameter(Mandatory = $true)][string]$Out,
    [string]$Until = '\d+/\d+ PASS\s+\d+ FAIL|END_OF_REPORT'
)
# Opens the APEX USB CDC port with DTR set (the firmware waits for it), keeps
# reading until the end-of-report pattern shows up or the timeout expires.
# Survives the port vanishing while the board re-enumerates after a flash.
# Prints "### PORT BUSY" and stops if the port keeps refusing access (held by
# a terminal) for 15 s.
$deadline = (Get-Date).AddSeconds($TimeoutSec)
$sb = New-Object System.Text.StringBuilder
$sp = $null
$done = $false
$busySince = $null
while (((Get-Date) -lt $deadline) -and -not $done) {
    if ($null -eq $sp) {
        if ([System.IO.Ports.SerialPort]::GetPortNames() -contains $Port) {
            try {
                $sp = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
                $sp.DtrEnable = $true
                $sp.RtsEnable = $true
                $sp.ReadTimeout = 200
                $sp.Open()
                $busySince = $null
            } catch [System.UnauthorizedAccessException] {
                $sp = $null
                if ($null -eq $busySince) { $busySince = Get-Date }
                if (((Get-Date) - $busySince).TotalSeconds -gt 15) {
                    Write-Output "### PORT BUSY ($Port held by another program) ###"
                    exit 0
                }
                Start-Sleep -Milliseconds 300
                continue
            } catch {
                $sp = $null
                Start-Sleep -Milliseconds 300
                continue
            }
        } else {
            Start-Sleep -Milliseconds 300
            continue
        }
    }
    try {
        $s = $sp.ReadExisting()
        if ($s) { [void]$sb.Append($s) }
    } catch {
        try { $sp.Close() } catch {}
        $sp = $null
        continue
    }
    if (($sb.ToString() -replace "$([char]27)\[[0-9;]*[A-Za-z]", '') -match $Until) {
        Start-Sleep -Milliseconds 500
        try { [void]$sb.Append($sp.ReadExisting()) } catch {}
        $done = $true
    }
    Start-Sleep -Milliseconds 100
}
if ($null -ne $sp) { try { $sp.Close() } catch {} }
$esc = [char]27
$text = $sb.ToString() -replace "$esc\[[0-9;]*[A-Za-z]", ''
[System.IO.File]::WriteAllText($Out, $text, (New-Object System.Text.UTF8Encoding($false)))
if (-not $done) { Write-Output "### TIMEOUT after $TimeoutSec s (no end-of-report marker) ###" }
Write-Output $text

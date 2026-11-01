# Copyright (c) 2026 AAI06 Pico-Car team
# SPDX-License-Identifier: Apache-2.0
#
# mqtt_view.ps1 - human-readable view of a log written by mqtt_log.ps1
#
# Usage (no arg = newest log in evidence\buddy1\):
#   powershell -ExecutionPolicy Bypass -File tools\mqtt_view.ps1
#   powershell -ExecutionPolicy Bypass -File tools\mqtt_view.ps1 evidence\buddy1\<file>.log
#
# Collapses the raw per-message lines into:
#   1) a TIMELINE of the things that matter (state changes, faults, commands,
#      runs, barcodes, obstacles, connection), one tidy line each;
#   2) a SUMMARY (line-position mix, faults, wheel stalls, obstacle/false stops).
# Does not touch the .log file - it is read only.

param([string]$Path)

if (-not $Path) {
    $latest = Get-ChildItem (Join-Path $PSScriptRoot "..\evidence\buddy1") -Filter *.log |
              Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if (-not $latest) { Write-Error "No logs in evidence\buddy1\"; exit 1 }
    $Path = $latest.FullName
}
if (-not (Test-Path $Path)) { Write-Error "Not found: $Path"; exit 1 }

function Short-Time([string]$iso) {
    # 2026-10-29T10:26:50.098000000 -> 10:26:50.098
    if ($iso -match 'T(\d\d:\d\d:\d\d)\.(\d{3})') { return "$($matches[1]).$($matches[2])" }
    if ($iso -match 'T(\d\d:\d\d:\d\d)') { return $matches[1] }
    return $iso
}
function Fault-Name([int]$f) {
    if ($f -eq 0) { return "" }
    $bits = @()
    if ($f -band 1)  { $bits += "STALL" }
    if ($f -band 2)  { $bits += "ENCODER" }
    if ($f -band 4)  { $bits += "MOTION-CMD" }
    if ($f -band 32) { $bits += "ULTRASONIC" }
    if ($bits.Count -eq 0) { $bits += "f$f" }
    return ($bits -join "+")
}

$timeline = New-Object System.Collections.Generic.List[string]
$pos = @{}; $faultCounts = @{}
$offline = 0; $obstacleStops = 0; $obstacleFalse = 0
$rStuck = 0; $lStuck = 0
$build = ""; $firstT = $null; $lastT = $null
$prevState = ""; $prevFault = -1

foreach ($raw in [System.IO.File]::ReadLines((Resolve-Path $Path))) {
    $parts = $raw -split ' ', 3
    if ($parts.Count -lt 2) { continue }
    $t = $parts[0]; $topic = $parts[1]
    $payload = ""
    if ($parts.Count -ge 3) { $payload = $parts[2] }
    if (-not $firstT) { $firstT = $t }
    $lastT = $t
    $st = Short-Time $t

    $j = $null
    if ($payload.StartsWith("{")) { try { $j = $payload | ConvertFrom-Json } catch { $j = $null } }

    switch -Wildcard ($topic) {
        "robot/status" {
            $timeline.Add(("{0}  CONN  {1}" -f $st, $payload))
            if ($payload -match "offline") { $offline++ }
        }
        "robot/info" { $build = $payload }
        "robot/state" {
            if ($j) {
                $s = [string]$j.state; $f = [int]$j.fault
                if ($s -ne $prevState -or $f -ne $prevFault) {
                    $fn = Fault-Name $f
                    $mark = ""; if ($f -ne 0) { $mark = "   <- FAULT $fn" }
                    $timeline.Add(("{0}  STATE {1}{2}" -f $st, $s, $mark))
                    $prevState = $s; $prevFault = $f
                }
            }
        }
        "robot/telemetry/line" {
            if ($j) { $p = [string]$j.pos; if ($p) { if ($pos.ContainsKey($p)) { $pos[$p]++ } else { $pos[$p] = 1 } } }
        }
        "robot/telemetry/motion" {
            if ($j) {
                $l = [int]$j.spd_l_mm_s; $r = [int]$j.spd_r_mm_s
                if ($r -eq 0 -and $l -gt 40) { $rStuck++ }
                if ($l -eq 0 -and $r -gt 40) { $lStuck++ }
            }
        }
        "robot/event/command"  { if ($j) { $timeline.Add(("{0}  CMD   {1} (ok={2})" -f $st, $j.cmd, $j.ok)) } }
        "robot/event/run"      { if ($j) { $timeline.Add(("{0}  RUN   {1}" -f $st, $j.run)) } }
        "robot/event/barcode"  { if ($j) { $timeline.Add(("{0}  BCODE {1} -> {2}" -f $st, $j.code, $j.command)) } }
        "robot/event/obstacle" {
            if ($j) {
                $d = [int]$j.dist_mm
                $timeline.Add(("{0}  OBST  dist={1}mm action={2}" -f $st, $d, $j.action))
                $obstacleStops++; if ($d -eq 0) { $obstacleFalse++ }
            }
        }
    }
    if ($j -and $j.fault -ne $null) {
        $fk = [int]$j.fault
        if ($fk -ne 0) { $fn = Fault-Name $fk; if ($faultCounts.ContainsKey($fn)) { $faultCounts[$fn]++ } else { $faultCounts[$fn] = 1 } }
    }
}

# ---- output ----
$W = "`n"
Write-Host ("=" * 60)
Write-Host ("Log:   " + (Split-Path $Path -Leaf))
if ($build) { Write-Host ("Build: " + $build) }
Write-Host ("Span:  " + (Short-Time $firstT) + "  ->  " + (Short-Time $lastT))
Write-Host ("=" * 60)

Write-Host "$W TIMELINE (what changed)"
foreach ($line in $timeline) { Write-Host ("  " + $line) }

Write-Host "$W SUMMARY"
$posTotal = 0; foreach ($v in $pos.Values) { $posTotal += $v }
if ($posTotal -gt 0) {
    $order = "CENTRE","LEFT","RIGHT","LOST","JUNCTION"
    $segs = @()
    foreach ($k in $order) { if ($pos.ContainsKey($k)) { $segs += ("{0} {1}%" -f $k, [int](100 * $pos[$k] / $posTotal)) } }
    Write-Host ("  Line position: " + ($segs -join "   "))
}
if ($faultCounts.Count -gt 0) {
    $fsegs = @(); foreach ($k in $faultCounts.Keys) { $fsegs += ("{0} ({1} msgs)" -f $k, $faultCounts[$k]) }
    Write-Host ("  Faults seen:   " + ($fsegs -join "   "))
} else { Write-Host "  Faults seen:   none" }
Write-Host ("  Wheel stalls:  right {0} / left {1}  (samples with one wheel 0, other >40)" -f $rStuck, $lStuck)
Write-Host ("  Obstacle stops:{0}  (of those, dist=0 false: {1})" -f $obstacleStops, $obstacleFalse)
Write-Host ("  Connection:    {0} offline drop(s)" -f $offline)
Write-Host ""

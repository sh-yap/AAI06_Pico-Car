# Copyright (c) 2026 AAI06 Pico-Car team
# SPDX-License-Identifier: Apache-2.0
#
# mqtt_check.ps1 - analyse a log written by mqtt_log.ps1
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File tools\mqtt_check.ps1 evidence\buddy1\<file>.log
#   (no argument: the newest log in evidence\buddy1\)
#
# Reports message rates per topic (for the resource-efficiency figures),
# heartbeat continuity (gaps = outages or lost messages), dropped events,
# connection transitions, command acknowledgements and run summaries.
# Writes the report next to the log as <file>.report.txt.

param([string]$Path)

if (-not $Path) {
    $latest = Get-ChildItem (Join-Path $PSScriptRoot "..\evidence\buddy1") -Filter *.log |
              Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if (-not $latest) { Write-Error "No logs in evidence\buddy1\"; exit 1 }
    $Path = $latest.FullName
}

$report = New-Object System.Collections.Generic.List[string]
function Out-Line([string]$text) { $report.Add($text); Write-Host $text }

$topics      = @{}
$first       = $null
$last        = $null
$totalBytes  = 0
$hbPrevSeq   = $null
$hbPrevTime  = $null
$hbCount     = 0
$hbSeqGaps   = 0
$hbMissing   = 0
$hbReboots   = 0
$hbMaxGap    = 0.0
$hbLongGaps  = New-Object System.Collections.Generic.List[string]
$maxEvDrop   = 0
$hbPrevT     = 0
$hbMaxSendGap = 0.0
$cmdSentAt   = $null
$ackTimes    = New-Object System.Collections.Generic.List[double]
$events      = New-Object System.Collections.Generic.List[string]

# Share read/write so a log can be analysed while mqtt_log.ps1 is still
# recording into it.
$stream = New-Object System.IO.FileStream((Resolve-Path $Path), [System.IO.FileMode]::Open,
                                          [System.IO.FileAccess]::Read, [System.IO.FileShare]::ReadWrite)
$reader = New-Object System.IO.StreamReader($stream)
$lines  = New-Object System.Collections.Generic.List[string]
while ($null -ne ($l = $reader.ReadLine())) { $lines.Add($l) }
$reader.Close()

# Sizes: mqtt_log.ps1 also writes <log>.hex with the raw bytes, one line per
# message in the same order.  With it, sizes are the bytes actually sent
# (binary or JSON); without it (older logs), the logged text is counted.
$hexSizes = $null
if (Test-Path "$Path.hex") {
    $hexSizes = New-Object System.Collections.Generic.List[int]
    foreach ($h in [System.IO.File]::ReadAllLines("$Path.hex")) {
        $hp = $h.Split(' ', 3)
        if ($hp.Count -ge 3) { $hexSizes.Add([int]($hp[2].Trim().Length / 2)) } else { $hexSizes.Add(0) }
    }
}
$lineIndex = -1

foreach ($line in $lines) {
    $lineIndex++
    $parts = $line.Split(' ', 3)
    if (($parts.Count -lt 2) -or ($parts[0].Length -lt 23)) { continue }

    $iso     = $parts[0].Substring(11, 12)        # HH:mm:ss.fff for the timeline
    $stamp   = [datetime]::ParseExact($parts[0].Substring(0, 23), "yyyy-MM-ddTHH:mm:ss.fff",
                                      [System.Globalization.CultureInfo]::InvariantCulture)
    $t       = ($stamp - [datetime]'2000-01-01').TotalSeconds
    $topic   = $parts[1]
    $payload = ""
    if ($parts.Count -ge 3) { $payload = $parts[2] }

    if ($null -eq $first) { $first = $t }
    $last = $t
    $size = $payload.Length
    if (($null -ne $hexSizes) -and ($lineIndex -lt $hexSizes.Count)) { $size = $hexSizes[$lineIndex] }
    $totalBytes += $size

    if (-not $topics.ContainsKey($topic)) { $topics[$topic] = @{ Count = 0; Bytes = 0 } }
    $topics[$topic]["Count"] += 1
    $topics[$topic]["Bytes"] += $size

    switch -Wildcard ($topic) {
        "robot/heartbeat" {
            $hb = $payload | ConvertFrom-Json
            $hbCount += 1
            if ($hb.ev_drop -gt $maxEvDrop) { $maxEvDrop = $hb.ev_drop }
            if ($null -ne $hbPrevSeq) {
                if ($hb.seq -lt $hbPrevSeq) {
                    $hbReboots += 1
                    $events.Add("$iso  heartbeat seq restarted ($hbPrevSeq -> $($hb.seq)): Pico rebooted")
                } elseif ($hb.seq -gt $hbPrevSeq + 1) {
                    $hbSeqGaps += 1
                    $hbMissing += $hb.seq - $hbPrevSeq - 1
                }
                $gap = $t - $hbPrevTime
                if ($gap -gt $hbMaxGap) { $hbMaxGap = $gap }
                # The Pico stamps each message with its own uptime (t, ms).
                # A long arrival gap with a normal t gap means the network
                # held the messages back (delivered late, not sent late).
                if ($hb.seq -gt $hbPrevSeq) {
                    $sendGap = ($hb.t - $hbPrevT) / 1000.0 / ($hb.seq - $hbPrevSeq)
                    if ($sendGap -gt $hbMaxSendGap) { $hbMaxSendGap = $sendGap }
                }
                if ($gap -gt 2.5) {
                    $hbLongGaps.Add(("{0}  no heartbeat arrived for {1:N1} s" -f $iso, $gap))
                }
            }
            $hbPrevSeq  = $hb.seq
            $hbPrevTime = $t
            $hbPrevT    = $hb.t
        }
        "robot/status"        { $events.Add("$iso  status: $payload") }
        "robot/info"          { $events.Add("$iso  firmware: $payload") }
        "robot/event/command" {
            if ($null -ne $cmdSentAt) {
                $ackMs = [math]::Round(($t - $cmdSentAt) * 1000)
                $ackTimes.Add($ackMs)
                $cmdSentAt = $null
                $events.Add("$iso  command ack ($ackMs ms): $payload")
            } else {
                $events.Add("$iso  command ack: $payload")
            }
        }
        "robot/command"       { $cmdSentAt = $t; $events.Add("$iso  command sent: $payload") }
        "robot/summary"       { $events.Add("$iso  SUMMARY: $payload") }
        "robot/event/run"     { $events.Add("$iso  run: $payload") }
    }
}

if ($null -eq $first) { Write-Error "Log is empty: $Path"; exit 1 }
$duration = [math]::Max($last - $first, 0.001)
$totalMsgs = 0
foreach ($v in $topics.Values) { $totalMsgs += $v['Count'] }

Out-Line "MQTT log report: $(Split-Path $Path -Leaf)"
if ($null -ne $hexSizes) { Out-Line "Sizes: bytes on the wire (from $(Split-Path "$Path.hex" -Leaf))" }
else { Out-Line "Sizes: logged text length (no .hex file)" }
Out-Line ("Duration: {0:N1} s   messages: {1}   avg rate: {2:N1} msg/s   payload: {3:N0} B/s" -f `
          $duration, $totalMsgs, ($totalMsgs / $duration), ($totalBytes / $duration))
Out-Line ""
Out-Line ("{0,-26} {1,7} {2,9} {3,10}" -f "topic", "count", "msg/s", "avg bytes")
foreach ($k in ($topics.Keys | Sort-Object)) {
    $c = $topics[$k]["Count"]
    Out-Line ("{0,-26} {1,7} {2,9:N2} {3,10:N0}" -f $k, $c, ($c / $duration), ($topics[$k]["Bytes"] / $c))
}
Out-Line ""
Out-Line "Heartbeat: $hbCount received, $hbMissing missing in $hbSeqGaps gap(s), $hbReboots reboot(s), longest silence $([math]::Round($hbMaxGap, 1)) s"
Out-Line ("Heartbeat interval by the Pico's own clock: max {0:N2} s (1.00 = sent on time)" -f $hbMaxSendGap)
foreach ($g in $hbLongGaps) { Out-Line "  $g" }
if ($ackTimes.Count -gt 0) {
    $ackStats = $ackTimes | Measure-Object -Average -Maximum
    Out-Line ("Command acknowledgement: {0} commands, avg {1:N0} ms, max {2:N0} ms (laptop -> Pico -> laptop)" -f `
              $ackTimes.Count, $ackStats.Average, $ackStats.Maximum)
}
Out-Line "Events dropped on the robot (max ev_drop): $maxEvDrop"
Out-Line ""
Out-Line "Timeline:"
foreach ($e in $events) { Out-Line "  $e" }

$outFile = [System.IO.Path]::ChangeExtension($Path, ".report.txt")
[System.IO.File]::WriteAllLines($outFile, $report)
Write-Host ""
Write-Host "Report saved: $outFile"

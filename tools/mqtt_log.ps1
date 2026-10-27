# Copyright (c) 2026 AAI06 Pico-Car team
# SPDX-License-Identifier: Apache-2.0
#
# mqtt_log.ps1 - record all robot/# MQTT traffic to evidence/buddy1/<time>_<name>.log
#
# Usage (from the project folder):
#   powershell -ExecutionPolicy Bypass -File tools\mqtt_log.ps1 -Name T4_outage
#
# Each line is:  <local time, ms resolution> <topic> <payload>
# Also echoed to the screen.  Stop with Ctrl+C; the file is kept.
# Analyse afterwards with tools\mqtt_check.ps1.
#
# Works with both firmware formats (COMMS_FORMAT=json|binary): payloads are
# received as hex and binary ones are decoded to the same JSON line the JSON
# firmware sends (tools\robot_decode.ps1).  The raw bytes are also kept in
# <file>.hex (<time> <topic> <hex>) to measure message sizes for CR1.

param(
    [string]$Name   = "run",
    [string]$Broker = "localhost",     # broker runs on this laptop
    [string]$Topic  = "robot/#",
    [int]$Seconds   = 0                  # stop after this long; 0 = until Ctrl+C
)

$exe = "C:\Program Files\mosquitto\mosquitto_sub.exe"
if (-not (Test-Path $exe)) {
    Write-Error "mosquitto_sub not found at $exe"
    exit 1
}

$dir = Join-Path $PSScriptRoot "..\evidence\buddy1"
New-Item -ItemType Directory -Force -Path $dir | Out-Null
$file = Join-Path (Resolve-Path $dir) ("{0}_{1}.log" -f (Get-Date -Format "yyyyMMdd_HHmmss"), $Name)

Write-Host "Recording $Topic from $Broker"
Write-Host "  -> $file"
Write-Host "Press Ctrl+C to stop."

. (Join-Path $PSScriptRoot "robot_decode.ps1")

# UTF-8 without BOM, flushed per line so nothing is lost on Ctrl+C.
$utf8 = New-Object System.Text.UTF8Encoding($false)
$writer = New-Object System.IO.StreamWriter($file, $false, $utf8)
$writer.AutoFlush = $true
$hexWriter = New-Object System.IO.StreamWriter("$file.hex", $false, $utf8)
$hexWriter.AutoFlush = $true
$subArgs = @("-h", $Broker, "-t", $Topic, "-F", "@Y-@m-@dT@H:@M:@S.@N %t %x")
if ($Seconds -gt 0) {
    $subArgs += @("-W", "$Seconds")      # mosquitto_sub exits after this many seconds
    Write-Host "Stopping automatically after $Seconds s."
}
try {
    & $exe @subArgs | ForEach-Object {
        # <time> <topic> <hex>; the hex is empty when a retained message is cleared.
        $parts = $_ -split ' ', 3
        $hex = ''
        if ($parts.Count -ge 3) { $hex = $parts[2].Trim() }
        try {
            $text = ConvertTo-RobotJson -Topic $parts[1] -Hex $hex
        }
        catch {
            $text = "UNDECODED $hex ($($_.Exception.Message))"
        }
        $line = "{0} {1} {2}" -f $parts[0], $parts[1], $text
        $hexWriter.WriteLine($_)
        $writer.WriteLine($line)
        $line
    }
}
finally {
    $writer.Close()
    $hexWriter.Close()
    Write-Host "Saved $file (+ .hex)"
}

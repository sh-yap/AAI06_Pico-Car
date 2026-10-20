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

# UTF-8 without BOM, flushed per line so nothing is lost on Ctrl+C.
$writer = New-Object System.IO.StreamWriter($file, $false, (New-Object System.Text.UTF8Encoding($false)))
$writer.AutoFlush = $true
$subArgs = @("-h", $Broker, "-t", $Topic, "-F", "@Y-@m-@dT@H:@M:@S.@N %t %p")
if ($Seconds -gt 0) {
    $subArgs += @("-W", "$Seconds")      # mosquitto_sub exits after this many seconds
    Write-Host "Stopping automatically after $Seconds s."
}
try {
    & $exe @subArgs | ForEach-Object {
        $writer.WriteLine($_)
        $_
    }
}
finally {
    $writer.Close()
    Write-Host "Saved $file"
}

# Copyright (c) 2026 AAI06 Pico-Car team
# SPDX-License-Identifier: Apache-2.0
#
# soak.ps1 - test T8: record, start a fake run, wait, stop, analyse.
#
# Usage (from the project folder, Pico running the COMMS_DEMO firmware):
#   powershell -ExecutionPolicy Bypass -File tools\soak.ps1 -Minutes 30

param(
    [int]$Minutes   = 30,
    [string]$Broker = "localhost"
)

$pub      = "C:\Program Files\mosquitto\mosquitto_pub.exe"
$runSecs  = $Minutes * 60
$recSecs  = $runSecs + 30            # keep recording past the stop

$recorder = Start-Job -ArgumentList $PSScriptRoot, $recSecs, $Broker -ScriptBlock {
    param($dir, $secs, $broker)
    & powershell -ExecutionPolicy Bypass -File (Join-Path $dir "mqtt_log.ps1") `
        -Name "T8_soak" -Seconds $secs -Broker $broker | Out-Null
}

Start-Sleep -Seconds 5
Write-Host ("{0}  start (soak {1} min)" -f (Get-Date -Format HH:mm:ss), $Minutes)
& $pub -h $Broker -t robot/command -m start

Start-Sleep -Seconds $runSecs
Write-Host ("{0}  stop" -f (Get-Date -Format HH:mm:ss))
& $pub -h $Broker -t robot/command -m stop

Wait-Job $recorder | Out-Null
Remove-Job $recorder
& powershell -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "mqtt_check.ps1")

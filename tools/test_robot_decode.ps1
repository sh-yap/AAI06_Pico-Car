# Copyright (c) 2026 AAI06 Pico-Car team
# SPDX-License-Identifier: Apache-2.0
#
# test_robot_decode.ps1 - checks robot_decode.ps1 against the firmware
# encoder.  The hex strings are the golden messages printed (and locked) by
# tests/comms/test_codec.c; if either side changes the format, this fails.
#
#   powershell -ExecutionPolicy Bypass -File tools\test_robot_decode.ps1

. (Join-Path $PSScriptRoot 'robot_decode.ps1')

$script:checks = 0
$script:failures = 0
function Check([bool]$Ok, [string]$What) {
    $script:checks++
    if (-not $Ok) { $script:failures++; Write-Host "  FAIL: $What" }
}

Write-Host "golden motion message"
$m = Read-RobotPayload 'robot/telemetry/motion' (ConvertFrom-RobotHex '2aa30c002493330a00b0ff4f01a00fe2c870ee8bb703')
Check ($m.v -eq 2)            "v = 2"
Check ($m.t -eq 207050)       "t = 207050"
Check ($m.spd_l_mm_s -eq 201) "spd_l = 201"
Check ($m.spd_r_mm_s -eq -199) "spd_r = -199 (signed)"
Check ($m.enc_l -eq 40)       "enc_l = 40"
Check ($m.enc_r -eq -40)      "enc_r = -40 (signed)"
Check ($m.dist_mm -eq 20)     "dist = 20"
Check ($m.tgt_l_mm_s -eq 250)  "tgt_l = 250"
Check ($m.tgt_r_mm_s -eq -120) "tgt_r = -120 (signed)"
Check ($m.slew_l_mm_s -eq 200) "slew_l = 200"
Check ($m.slew_r_mm_s -eq -100) "slew_r = -100 (signed)"
Check ($m.pwm_l -eq 190)         "pwm_l = 190"
Check ($m.pwm_r -eq -145)        "pwm_r = -145 (signed, 11 bits)"

Write-Host "golden line message"
$l = Read-RobotPayload 'robot/telemetry/line' (ConvertFrom-RobotHex '2aa30c00643f0e4638')
Check ($l.pos -eq 'CENTRE')    "pos = CENTRE"
Check ($l.err -eq -5)          "err = -5 (signed)"
Check ($l.sensors -eq 1)       "sensors = 1"
Check ($l.centre_adc -eq 3100) "centre_adc = 3100"
Check ($l.right_adc -eq 1800)  "right_adc = 1800"

Write-Host "golden calibration message (sensor gap)"
$c = Read-RobotPayload 'robot/event/calibration' (ConvertFrom-RobotHex '027d0000103238d8da90953e70d7da580290ad6dfd0000001e')
Check ($c.result -eq 'SENSOR_GAP') "result = SENSOR_GAP"
Check ($c.centre_thr -eq 1750)     "centre_thr = 1750"
Check ($c.first_right -eq 1)       "first_right = 1"
Check ($c.overlap -eq -6)          "overlap = -6 (a 6 sample gap, signed)"
Check ($c.sweep_deg -eq 30)        "sweep_deg = 30"

Write-Host "golden heartbeat message"
$json = ConvertTo-RobotJson -Topic 'robot/heartbeat' -Hex '3a5900001c030481ff'
Check ($json -eq '{"v":2,"t":5710,"seq":199,"state":"READY","fault":516,"ev_drop":255}') "heartbeat JSON line: $json"
$hb = $json | ConvertFrom-Json
Check ($hb.fault -eq 516) "heartbeat fault is 10 bits (512 = sensor gap, 4 = motion cmd)"
Check ($hb.seq -eq 199) "heartbeat readable by ConvertFrom-Json (mqtt_check.ps1)"

Write-Host "barcode codes: A-D and the course's F, Z"
# Header (v 2, t 0) = 34 bits, then code (3) + command (3).  Bits 34-36 =
# code, 37-39 = command, all in byte 4: code + (command << 3), shifted by 2.
foreach ($c in @(@(1,'A'), @(4,'D'), @(5,'F'), @(6,'Z'), @(7,'INVALID'))) {
    $b4 = (($c[0] + (3 -shl 3)) -shl 2) -band 0xFF          # command 3 = STRAIGHT
    $bc = Read-RobotPayload 'robot/event/barcode' (ConvertFrom-RobotHex ('02000000' + $b4.ToString('x2')))
    Check (($bc.code -eq $c[1]) -and ($bc.command -eq 'STRAIGHT')) "barcode code $($c[0]) -> $($c[1])"
}

Write-Host "text payloads pass through"
$j = '{"v":1,"t":5,"seq":3}'
$hex = -join ([System.Text.Encoding]::UTF8.GetBytes($j) | ForEach-Object { $_.ToString('x2') })
Check ((ConvertTo-RobotJson -Topic 'robot/heartbeat' -Hex $hex) -eq $j) "JSON format unchanged"
Check ((ConvertTo-RobotJson -Topic 'robot/status' -Hex '6f6e6c696e65') -eq 'online') "robot/status text"
Check ((ConvertTo-RobotJson -Topic 'robot/command' -Hex '7374617274') -eq 'start') "robot/command text"
Check ((ConvertTo-RobotJson -Topic 'robot/state' -Hex '') -eq '') "empty payload (retained clear)"

Write-Host "errors are reported, not guessed"
$threw = $false
try { $null = Read-RobotPayload 'robot/telemetry/motion' (ConvertFrom-RobotHex 'a28c32') } catch { $threw = $true }
Check $threw "truncated binary message throws"

Write-Host ""
Write-Host "$script:checks checks, $script:failures failed"
if ($script:failures -ne 0) { exit 1 }

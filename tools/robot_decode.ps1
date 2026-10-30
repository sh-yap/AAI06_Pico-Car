# Copyright (c) 2026 AAI06 Pico-Car team
# SPDX-License-Identifier: Apache-2.0
#
# robot_decode.ps1 - decode robot MQTT payloads (JSON or packed binary).
#
# Dot-source it, then call ConvertTo-RobotJson:
#   . tools\robot_decode.ps1
#   ConvertTo-RobotJson -Topic robot/heartbeat -Hex 3a5900001c030481ff
#     -> {"v":2,"t":5710,"seq":199,"state":"READY","fault":516,"ev_drop":255}
#
# A binary message decodes to the same JSON the firmware would have sent in
# JSON format (only "v" differs: 1 = JSON, 2 = binary), so every tool that
# reads JSON lines keeps working.
#
# The field table below MUST match app_program/comms.c and
# docs/BINARY_FORMAT.md: same order, same widths.  tools\test_robot_decode.ps1
# checks it against the firmware encoder's golden messages.

$script:States  = @('INIT','READY','LINE_FOLLOW','EXECUTE_NAV','OBSTACLE_STOP',
                    'OBSTACLE_SCAN','BYPASS','LINE_RECOVERY','FINISHED','SAFE_STOP')
$script:Nav     = @('NONE','LEFT','RIGHT','STRAIGHT','UTURN')
$script:Line    = @('LEFT','CENTRE','RIGHT','LOST','JUNCTION')
$script:Motion  = @('QUIET','ACCELERATING','CLIMBING_HUMP','DESCENDING_HUMP','IMPACT')
$script:Avoid   = @('STOP','CONTINUE','TURN_LEFT','TURN_RIGHT','REVERSE_RETRY')
$script:Cmd     = @('unknown','start','stop','status')
$script:Run     = @('END','BEGIN')
$script:CalResult = @('OK','CONTRAST','LINE_NOT_FOUND','BUDDY4','SENSOR_GAP')
$script:Barcode = @('','A','B','C','D','F','Z','INVALID')

$script:VersionBits = 2          # binary header: 2-bit version (2), 32-bit time
$script:BinaryVersion = 2

# Field kinds: u = unsigned, s = signed (two's complement), n = name list.
function F([string]$Key, [string]$Kind, [int]$Bits, $Names = $null) {
    [pscustomobject]@{ Key = $Key; Kind = $Kind; Bits = $Bits; Names = $Names }
}

$script:RobotFormat = @{
    'robot/telemetry/motion'  = @((F spd_l_mm_s s 10), (F spd_r_mm_s s 10),
                                  (F enc_l s 19), (F enc_r s 19), (F dist_mm s 16),
                                  (F tgt_l_mm_s s 10), (F tgt_r_mm_s s 10),
                                  (F slew_l_mm_s s 10), (F slew_r_mm_s s 10),
                                  (F pwm_l s 11), (F pwm_r s 11))
    'robot/telemetry/line'    = @((F pos n 3 $script:Line), (F err s 8), (F sensors u 2),
                                  (F centre_adc u 12), (F right_adc u 12))
    'robot/telemetry/terrain' = @((F motion n 3 $script:Motion), (F tilt_mg s 11),
                                  (F hump_dmm u 10), (F max_hump_dmm u 10))
    'robot/heartbeat'         = @((F seq u 16), (F state n 4 $script:States),
                                  (F fault u 10), (F ev_drop u 8))
    'robot/state'             = @((F state n 4 $script:States), (F fault u 10))
    'robot/event/barcode'     = @((F code n 3 $script:Barcode), (F command n 3 $script:Nav))
    'robot/event/hump'        = @((F peak_dmm u 10), (F max_dmm u 10))
    'robot/event/calibration' = @((F result n 3 $script:CalResult),
                                  (F centre_low u 12), (F centre_high u 12),
                                  (F centre_thr u 12), (F centre_full u 12),
                                  (F right_low u 12), (F right_high u 12),
                                  (F right_thr u 12), (F barcode_low u 12),
                                  (F barcode_high u 12), (F barcode_thr u 12),
                                  (F barcode_learned u 1), (F first_right u 1),
                                  (F overlap s 9),
                                  (F b4_floor_mg u 9),
                                  (F b4_samples u 15), (F sweep_deg u 8))
    'robot/event/obstacle'    = @((F dist_mm u 12), (F angle_deg u 8), (F width_mm u 10),
                                  (F clear_l_mm u 12), (F clear_r_mm u 12),
                                  (F action n 3 $script:Avoid))
    'robot/event/motion'      = @((F motion n 3 $script:Motion))
    'robot/event/run'         = @((F run n 1 $script:Run))
    'robot/summary'           = @((F run_ms u 20), (F dist_mm s 16), (F max_hump_dmm u 10),
                                  (F humps u 8), (F barcodes u 8), (F obstacles u 8))
    'robot/event/command'     = @((F cmd n 2 $script:Cmd), (F ok u 1))
}

function ConvertFrom-RobotHex([string]$Hex) {
    $h = $Hex.Trim()
    $bytes = New-Object byte[] ([int]($h.Length / 2))
    for ($i = 0; $i -lt $bytes.Length; $i++) {
        $bytes[$i] = [Convert]::ToByte($h.Substring($i * 2, 2), 16)
    }
    ,$bytes
}

# Bit reader: least significant bit first, as the firmware writes.
function Read-RobotBits([byte[]]$Bytes, [ref]$Pos, [int]$Bits) {
    [uint64]$value = 0
    for ($i = 0; $i -lt $Bits; $i++) {
        $p = $Pos.Value
        if (($p -shr 3) -ge $Bytes.Length) { throw "message too short" }
        if ((($Bytes[$p -shr 3] -shr ($p -band 7)) -band 1) -eq 1) {
            $value = $value -bor ([uint64]1 -shl $i)
        }
        $Pos.Value = $p + 1
    }
    $value
}

# Payload bytes -> ordered fields.  Text payloads (JSON firmware, robot/info,
# robot/status, robot/command, unknown topics) come back as a string.
# A binary message never starts with '{' (0x7B): its low 2 bits are 2.
function Read-RobotPayload([string]$Topic, [byte[]]$Bytes) {
    $isBinary = ($Bytes.Length -gt 0) -and ($Bytes[0] -ne 0x7B) -and
                $script:RobotFormat.ContainsKey($Topic) -and
                (($Bytes[0] -band 0x03) -eq $script:BinaryVersion)
    if (-not $isBinary) {
        return [System.Text.Encoding]::UTF8.GetString($Bytes)
    }

    $pos = 0
    $out = [ordered]@{}
    $out['v'] = [int](Read-RobotBits $Bytes ([ref]$pos) $script:VersionBits)
    $out['t'] = [uint32](Read-RobotBits $Bytes ([ref]$pos) 32)
    foreach ($f in $script:RobotFormat[$Topic]) {
        $raw = Read-RobotBits $Bytes ([ref]$pos) $f.Bits
        switch ($f.Kind) {
            's' {
                $v = [int64]$raw
                if ($raw -band ([uint64]1 -shl ($f.Bits - 1))) { $v -= [int64]1 -shl $f.Bits }
                $out[$f.Key] = $v
            }
            'n' {
                if ($raw -lt $f.Names.Count) { $out[$f.Key] = $f.Names[[int]$raw] }
                else { $out[$f.Key] = 'UNKNOWN' }
            }
            default { $out[$f.Key] = [int64]$raw }
        }
    }
    $out
}

# Hex payload -> one JSON line (or the text itself for text payloads).
function ConvertTo-RobotJson([string]$Topic, [string]$Hex) {
    $decoded = Read-RobotPayload $Topic (ConvertFrom-RobotHex $Hex)
    if ($decoded -is [string]) { return $decoded }
    ($decoded | ConvertTo-Json -Compress)
}

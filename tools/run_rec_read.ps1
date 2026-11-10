# Copyright (c) 2026 AAI06 Pico-Car team
# SPDX-License-Identifier: Apache-2.0
#
# run_rec_read.ps1 - read the test-only run recorder (VC_RECORDER=1 build)
# out of the car with the debug probe, and write a CSV, one row per 20 ms VC
# cycle, oldest first.  Layout: app_program\run_rec.h.
#
# Read out (car powered, after STOP; the car is NOT halted or reset):
#   powershell -ExecutionPolicy Bypass -File tools\run_rec_read.ps1 -Name home1
#   powershell -ExecutionPolicy Bypass -File tools\run_rec_read.ps1 -Elf <rec.elf> -Name home1
# Decode a dump already on disk:
#   powershell -ExecutionPolicy Bypass -File tools\run_rec_read.ps1 -File <dump.bin>
#
# -Elf defaults to the newest build_make\*_rec*_vc_car*.elf (plain _rec or
# _rec_b4rec).  It must be the
# ELF of the image on the car: the address of g_run_rec comes from it.
# Output: evidence\recorder\<time>_<name>.bin (raw) and .csv.  Not committed.
#
# CSV: missed = 1 when ms advanced by more than 30 ms (20 ms + one 10 ms
# tick) since the previous row.  dt_ms is that step.  ramp = terrain ramp
# active.  steer and speed_mm_s are the VC outputs; target_* are Motion's
# wheel targets; left/right_mm_s are measured.  overruns = VC late starts.
# left/right_pwm = signed PWM Motion applied (permille); left/right_i = PID
# integral term ki * integral (permille).
# learn_fwd_gain / learn_turn_gain = Motion's learned forward and turning
# gain (mm/s per permille; stored x1000), learn_fwd_offset = forward offset
# (mm/s; stored x10).  turn_src = 0 default line, 1 pivot seed, 2 driving
# data; fwd_src = 1 when the forward values come from driving data.
# Barcode trace: if the image also has g_bc_rec (app_program\run_rec.h), the
# same command dumps it too and writes <time>_<name>_barcode.csv (and
# _barcode.bin), one row per 10 ms Buddy 3 cycle, oldest first.  Columns:
# dt_ms = ms step from the previous row; barcode_adc / threshold = raw sensor
# and black threshold in use; black = sensor over the threshold; enc /
# enc_delta = average wheel encoder position and its step; capture_done = 1
# on the cycle a capture finished; capture_active = edges stored but not
# finished; edges = edges stored; code = decoded character (empty when none);
# code_id = barcode_t value; centre_adc / right_adc = line sensors.
# Extended 36-byte barcode records use the VC monotonic clock for ms, and
# append left_enc/right_enc, encoder_us (hardware low 32 bits), sample_end_ms
# and acquisition_ms. The ADC channels are sequential, bracketed by ms/end.
# Legacy 20-byte records retain their historical system clock and blank
# appended fields. With -File, pass the barcode dump as -BcFile.
# Buddy 4 trace: if the image also has g_b4_rec (B4_RECORDER=1 build), the
# same command dumps it to <time>_<name>_b4.bin and writes <time>_<name>_b4.csv,
# one row per 10 ms Buddy 4 cycle, oldest first.  With -File, pass the dump as
# -B4File.  Columns: dt_ms = ms step from the previous row; accel_*_mg =
# calibrated accelerometer sample; incline_mg / baseline_mg / delta_mg =
# filtered incline, flat-ground baseline and the offset the detector used;
# noise_mg = learned noise median; active_height_cmm / final_height_cmm =
# hump in progress and last completed hump in 0.01 mm, also as
# active_height_mm / final_height_mm; mag_x/y/z = latest magnetometer counts
# (read every 3rd sample, so values repeat; mag_new marks a fresh one; all 0
# in a B4_REC_MAG=0 build, image name ..._b4rec_nomag);
# left_enc / right_enc = encoder counts, low 16 bits, wrap; drive_speed_mm_s
# = mean absolute wheel speed; dt_us = hardware us since the previous row
# (65535 = that or more); hump_state = b4_types_hump_state_t; motion / turn =
# b4_types_motion_t / b4_types_turn_t (from motion_turn); tilt =
# b4_types_tilt_t; flags = raw byte, then 0/1 columns valid, mag_new,
# height_valid, cue, started, completed, stale, jolt (cue is Buddy 4's
# latched lift cue; jolt is the jolt guard window; the raw target step is
# not recorded).
# Event log: if the image also has g_ev_rec (whole-run log of line losses,
# ramp edges and hump results, 62 events, earliest kept), the same command
# dumps it to <time>_<name>_events.bin and writes <time>_<name>_events.csv:
# ms, event (LINE_LOST value = steer before the loss; LINE_BACK value = ms
# since LINE_LOST; RAMP_START / RAMP_END value 0; HUMP_DONE value = newest
# height in 0.1 mm; RUN_MAX value = the word's highest height in 0.1 mm,
# logged when it changes; LIFT value = the confirmed both-black spell in VC
# cycles), value, count (hump count for HUMP_DONE, 0 for LIFT), valid
# (HUMP_DONE only: the word's height was valid; empty otherwise).  With
# -File, pass the dump as -EvFile.
# A warning is printed if the ring was not frozen (run still going, or the
# image was reset before it ended).

param(
    [string]$Name    = "run",
    [string]$Elf     = "",
    [string]$File    = "",
    [string]$BcFile  = "",
    [string]$B4File  = "",
    [string]$EvFile  = "",
    [string]$OpenOcd = "C:\tools\openocd-rpi-2.3.1",
    [string]$Nm      = "C:\tools\xpack-arm-none-eabi-gcc-15.2.1-1.1\bin\arm-none-eabi-nm.exe",
    [int]$Speed      = 5000,
    [int]$MissedOverMs = 30
)

$ErrorActionPreference = "Stop"
$root   = Join-Path $PSScriptRoot ".."
$outDir = Join-Path $root "evidence\recorder"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$outDir = (Resolve-Path $outDir).Path
$stamp  = Get-Date -Format "yyyyMMdd_HHmmss"

$headerBytes = 20
$recordBytes = 44
$bcRecBytes  = @(20, 36)
$bcMagic     = 0x31524342
$b4Magic     = 0x31523442
$b4RecBytes  = 40
$evMagic     = 0x31564545
$evRecBytes  = 8
$evHeader    = 16
$evNames     = @("?0", "LINE_LOST", "LINE_BACK", "RAMP_START", "RAMP_END",
                 "HUMP_DONE", "RUN_MAX", "REMOVED_IMPACT", "LIFT")
$magic       = 0x31434552
$lineNames   = @("LEFT", "CENTRE", "RIGHT", "LOST", "JUNCTION")
$stateNames  = @("INITIALISE", "READY", "CALIBRATE", "FOLLOW_LINE",
                 "HANDLE_JUNCTION", "TURNING", "OBSTACLE_SCAN", "BYPASS",
                 "REACQUIRE_LINE", "EMERGENCY_STOP", "FINISHED")

function Get-Name($names, $i) {
    if ($i -lt $names.Count) { return $names[$i] } else { return "?$i" }
}

if ($File -ne "") {
    $bin = (Resolve-Path $File).Path
    $csv = [System.IO.Path]::ChangeExtension($bin, ".csv")
    $bcBin = ""
    if ($BcFile -ne "") { $bcBin = (Resolve-Path $BcFile).Path }
    $bcCsv = (Join-Path (Split-Path $bin) ([System.IO.Path]::GetFileNameWithoutExtension($bin))) + "_barcode.csv"
    $b4Bin = ""
    if ($B4File -ne "") { $b4Bin = (Resolve-Path $B4File).Path }
    $b4Csv = (Join-Path (Split-Path $bin) ([System.IO.Path]::GetFileNameWithoutExtension($bin))) + "_b4.csv"
    $evBin = ""
    if ($EvFile -ne "") { $evBin = (Resolve-Path $EvFile).Path }
    $evCsv = (Join-Path (Split-Path $bin) ([System.IO.Path]::GetFileNameWithoutExtension($bin))) + "_events.csv"
}
else {
    if ($Elf -eq "") {
        $latest = Get-ChildItem (Join-Path $root "build_make") -Filter "*_rec*_vc_car*.elf" |
                  Sort-Object LastWriteTime -Descending | Select-Object -First 1
        if ($null -eq $latest) { Write-Error "No recorder ELF found; pass -Elf"; exit 1 }
        $Elf = $latest.FullName
    }
    Write-Host "ELF: $Elf"

    # 1. Address and size of g_run_rec from the ELF symbol table.
    $sym = (& $Nm -S --defined-only $Elf) | Where-Object { $_ -match "\sg_run_rec$" }
    if (-not $sym) { Write-Error "g_run_rec not in $Elf (not a VC_RECORDER=1 build?)"; exit 1 }
    $f    = ($sym | Select-Object -First 1).Trim() -split "\s+"
    $addr = [Convert]::ToUInt32($f[0], 16)
    $size = [Convert]::ToUInt32($f[1], 16)
    Write-Host ("g_run_rec at 0x{0:x8}, {1} bytes" -f $addr, $size)

    # 2. Dump it over SWD.  dump_image reads through the debug access port,
    #    so the cores keep running; nothing here halts or resets the target.
    $bin = Join-Path $outDir ("{0}_{1}.bin" -f $stamp, $Name)
    $csv = Join-Path $outDir ("{0}_{1}.csv" -f $stamp, $Name)
    $binTcl = $bin -replace "\\", "/"

    # Optional barcode ring, dumped in the same OpenOCD session.
    $bcBin = ""
    $bcCsv = Join-Path $outDir ("{0}_{1}_barcode.csv" -f $stamp, $Name)
    $bcCmd = @()
    $bcSym = (& $Nm -S --defined-only $Elf) | Where-Object { $_ -match "\sg_bc_rec$" }
    if ($bcSym) {
        $g      = ($bcSym | Select-Object -First 1).Trim() -split "\s+"
        $bcAddr = [Convert]::ToUInt32($g[0], 16)
        $bcSize = [Convert]::ToUInt32($g[1], 16)
        Write-Host ("g_bc_rec at 0x{0:x8}, {1} bytes" -f $bcAddr, $bcSize)
        $bcBin    = Join-Path $outDir ("{0}_{1}_barcode.bin" -f $stamp, $Name)
        $bcBinTcl = $bcBin -replace "\\", "/"
        $bcCmd    = @("-c", ("dump_image {{{0}}} 0x{1:x8} 0x{2:x}" -f $bcBinTcl, $bcAddr, $bcSize))
    }
    else {
        Write-Warning "g_bc_rec not in the ELF: no barcode trace."
    }
    # Optional Buddy 4 ring (B4_RECORDER=1 builds), same session.
    $b4Bin = ""
    $b4Csv = Join-Path $outDir ("{0}_{1}_b4.csv" -f $stamp, $Name)
    $b4Cmd = @()
    $b4Sym = (& $Nm -S --defined-only $Elf) | Where-Object { $_ -match "\sg_b4_rec$" }
    if ($b4Sym) {
        $h      = ($b4Sym | Select-Object -First 1).Trim() -split "\s+"
        $b4Addr = [Convert]::ToUInt32($h[0], 16)
        $b4Size = [Convert]::ToUInt32($h[1], 16)
        Write-Host ("g_b4_rec at 0x{0:x8}, {1} bytes" -f $b4Addr, $b4Size)
        $b4Bin    = Join-Path $outDir ("{0}_{1}_b4.bin" -f $stamp, $Name)
        $b4BinTcl = $b4Bin -replace "\\", "/"
        $b4Cmd    = @("-c", ("dump_image {{{0}}} 0x{1:x8} 0x{2:x}" -f $b4BinTcl, $b4Addr, $b4Size))
    }
    # Optional event log, same session.
    $evBin = ""
    $evCsv = Join-Path $outDir ("{0}_{1}_events.csv" -f $stamp, $Name)
    $evCmd = @()
    $evSym = (& $Nm -S --defined-only $Elf) | Where-Object { $_ -match "\sg_ev_rec$" }
    if ($evSym) {
        $k      = ($evSym | Select-Object -First 1).Trim() -split "\s+"
        $evAddr = [Convert]::ToUInt32($k[0], 16)
        $evSize = [Convert]::ToUInt32($k[1], 16)
        Write-Host ("g_ev_rec at 0x{0:x8}, {1} bytes" -f $evAddr, $evSize)
        $evBin    = Join-Path $outDir ("{0}_{1}_events.bin" -f $stamp, $Name)
        $evBinTcl = $evBin -replace "\\", "/"
        $evCmd    = @("-c", ("dump_image {{{0}}} 0x{1:x8} 0x{2:x}" -f $evBinTcl, $evAddr, $evSize))
    }
    & (Join-Path $OpenOcd "openocd.exe") -s (Join-Path $OpenOcd "scripts") `
        -f interface/cmsis-dap.cfg -f target/rp2040.cfg `
        -c "adapter speed $Speed" -c init `
        -c ("dump_image {{{0}}} 0x{1:x8} 0x{2:x}" -f $binTcl, $addr, $size) `
        @bcCmd `
        @b4Cmd `
        @evCmd `
        -c shutdown
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $bin)) {
        Write-Error "OpenOCD dump failed (probe connected? car powered?)"
        exit 1
    }
}

# 3. Decode.
$b = [System.IO.File]::ReadAllBytes($bin)
if ($b.Length -lt $headerBytes) { Write-Error "Dump too short ($($b.Length) bytes)"; exit 1 }
$hMagic = [BitConverter]::ToUInt32($b, 0)
$recSz  = [BitConverter]::ToUInt16($b, 4)
$cap    = [BitConverter]::ToUInt16($b, 6)
$wr     = [BitConverter]::ToUInt32($b, 8)
$total  = [BitConverter]::ToUInt32($b, 12)
$frozen = [BitConverter]::ToUInt32($b, 16)

if ($hMagic -ne $magic) {
    Write-Error ("No recorder data: magic 0x{0:x8} (no run began since boot, or wrong image/ELF)" -f $hMagic)
    exit 1
}
if ($recSz -ne $recordBytes) { Write-Error "Record size $recSz, decoder expects $recordBytes"; exit 1 }
if ($b.Length -lt ($headerBytes + $cap * $recSz)) { Write-Error "Dump shorter than the ring"; exit 1 }
if ($frozen -eq 0) { Write-Warning "Ring not frozen: the run had not ended when it was read." }

$n     = [Math]::Min([uint32]$cap, $total)
$first = ([int]$wr - [int]$n + $cap) % $cap

$sb = New-Object System.Text.StringBuilder
[void]$sb.AppendLine("row,ms,dt_ms,missed,cycle,centre_adc,right_adc,line,line_error,sensors,vc_state,steer,speed_mm_s,target_left_mm_s,target_right_mm_s,left_mm_s,right_mm_s,ramp,overruns,left_pwm,right_pwm,left_i,right_i,learn_fwd_gain,learn_fwd_offset,learn_turn_gain,turn_src,fwd_src")
$prev   = $null
$missed = 0
$maxDt  = 0
for ($i = 0; $i -lt $n; $i++) {
    $o   = $headerBytes + (($first + $i) % $cap) * $recSz
    $ms  = [BitConverter]::ToUInt32($b, $o)
    $dt  = ""
    $mis = 0
    if ($null -ne $prev) {
        $d = [int64]$ms - [int64]$prev
        if ($d -lt 0) { $d += 4294967296 }        # 32-bit ms wrap
        $dt = $d
        if ($d -gt $MissedOverMs) { $mis = 1; $missed++ }
        if ($d -gt $maxDt) { $maxDt = $d }
    }
    $prev  = $ms
    $line  = Get-Name $lineNames $b[$o + 10]
    $state = Get-Name $stateNames $b[$o + 13]
    $sbyte = { param($x) if ($x -gt 127) { $x - 256 } else { $x } }
    [void]$sb.AppendLine(("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},{19},{20},{21},{22},{23:0.000},{24:0.0},{25:0.000},{26},{27}" -f `
        $i, $ms, $dt, $mis,
        [BitConverter]::ToUInt16($b, $o + 4),
        [BitConverter]::ToUInt16($b, $o + 6),
        [BitConverter]::ToUInt16($b, $o + 8),
        $line, (& $sbyte $b[$o + 11]), $b[$o + 12], $state,
        (& $sbyte $b[$o + 14]), [BitConverter]::ToInt16($b, $o + 16),
        [BitConverter]::ToInt16($b, $o + 18), [BitConverter]::ToInt16($b, $o + 20),
        [BitConverter]::ToInt16($b, $o + 22), [BitConverter]::ToInt16($b, $o + 24),
        ($b[$o + 15] -band 1), [BitConverter]::ToUInt16($b, $o + 26),
        [BitConverter]::ToInt16($b, $o + 28), [BitConverter]::ToInt16($b, $o + 30),
        [BitConverter]::ToInt16($b, $o + 32), [BitConverter]::ToInt16($b, $o + 34),
        ([BitConverter]::ToInt16($b, $o + 36) / 1000.0),
        ([BitConverter]::ToInt16($b, $o + 38) / 10.0),
        ([BitConverter]::ToInt16($b, $o + 40) / 1000.0),
        ([BitConverter]::ToUInt16($b, $o + 42) -band 3),
        (([BitConverter]::ToUInt16($b, $o + 42) -shr 2) -band 1)))
}
[System.IO.File]::WriteAllText($csv, $sb.ToString(), [System.Text.Encoding]::ASCII)
Write-Host ("{0} rows (run total {1}, frozen {2}), missed cycles {3}, longest step {4} ms" -f $n, $total, $frozen, $missed, $maxDt)
Write-Host "CSV: $csv"

# 4. Barcode trace, if it was dumped.
if ($bcBin -ne "") {
    $c = [System.IO.File]::ReadAllBytes($bcBin)
    if ($c.Length -lt $headerBytes) { Write-Error "Barcode dump too short ($($c.Length) bytes)"; exit 1 }
    $cMagic  = [BitConverter]::ToUInt32($c, 0)
    $cRecSz  = [BitConverter]::ToUInt16($c, 4)
    $cCap    = [BitConverter]::ToUInt16($c, 6)
    $cWr     = [BitConverter]::ToUInt32($c, 8)
    $cTotal  = [BitConverter]::ToUInt32($c, 12)
    $cFrozen = [BitConverter]::ToUInt32($c, 16)
    if ($cMagic -ne $bcMagic) {
        Write-Warning ("No barcode data: magic 0x{0:x8}" -f $cMagic)
    }
    elseif ($cRecSz -notin $bcRecBytes) {
        Write-Error "Barcode record size $cRecSz, decoder expects $bcRecBytes"; exit 1
    }
    elseif ($c.Length -lt ($headerBytes + $cCap * $cRecSz)) {
        Write-Error "Barcode dump shorter than the ring"; exit 1
    }
    else {
        if ($cFrozen -eq 0) { Write-Warning "Barcode ring not frozen." }
        $cn     = [Math]::Min([uint32]$cCap, $cTotal)
        $cFirst = ([int]$cWr - [int]$cn + $cCap) % $cCap
        $cb = New-Object System.Text.StringBuilder
        [void]$cb.AppendLine("row,ms,dt_ms,barcode_adc,threshold,black,enc,enc_delta,capture_done,capture_active,edges,code,code_id,centre_adc,right_adc,left_enc,right_enc,encoder_us,sample_end_ms,acquisition_ms")
        $cPrev    = $null
        $cPrevEnc = $null
        $cMaxDt   = 0
        $cDone    = 0
        for ($i = 0; $i -lt $cn; $i++) {
            $o     = $headerBytes + (($cFirst + $i) % $cCap) * $cRecSz
            $ms    = [BitConverter]::ToUInt32($c, $o)
            $enc   = [BitConverter]::ToInt32($c, $o + 4)
            $flags = $c[$o + 16]
            $dt    = ""
            $de    = ""
            if ($null -ne $cPrev) {
                $d = [int64]$ms - [int64]$cPrev
                if ($d -lt 0) { $d += 4294967296 }
                $dt = $d
                if ($d -gt $cMaxDt) { $cMaxDt = $d }
                $de = $enc - $cPrevEnc
            }
            $cPrev    = $ms
            $cPrevEnc = $enc
            $ch = ""
            if ($c[$o + 19] -ge 32 -and $c[$o + 19] -le 126) { $ch = [string][char]$c[$o + 19] }
            if ($flags -band 2) { $cDone++ }
            $extra = ",,,,,"
            if ($cRecSz -eq 36) {
                $end = [BitConverter]::ToUInt32($c, $o + 32)
                $duration = [int64]$end - [int64]$ms
                if ($duration -lt 0) { $duration += 4294967296 }
                $extra = ",{0},{1},{2},{3},{4}" -f `
                    [BitConverter]::ToInt32($c, $o + 20), [BitConverter]::ToInt32($c, $o + 24), `
                    [BitConverter]::ToUInt32($c, $o + 28), $end, $duration
            }
            [void]$cb.AppendLine((("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}" -f `
                $i, $ms, $dt,
                [BitConverter]::ToUInt16($c, $o + 8), [BitConverter]::ToUInt16($c, $o + 10),
                ($flags -band 1), $enc, $de,
                (($flags -shr 1) -band 1), (($flags -shr 2) -band 1),
                $c[$o + 17], $ch, $c[$o + 18],
                [BitConverter]::ToUInt16($c, $o + 12), [BitConverter]::ToUInt16($c, $o + 14)) + $extra))
        }
        [System.IO.File]::WriteAllText($bcCsv, $cb.ToString(), [System.Text.Encoding]::ASCII)
        Write-Host ("{0} barcode rows (total {1}, frozen {2}), captures finished {3}, longest step {4} ms" -f $cn, $cTotal, $cFrozen, $cDone, $cMaxDt)
        Write-Host "Barcode CSV: $bcCsv"
    }
}

# 5. Buddy 4 trace, if it was dumped.
if ($b4Bin -ne "") {
    $d = [System.IO.File]::ReadAllBytes($b4Bin)
    if ($d.Length -lt $headerBytes) { Write-Error "Buddy 4 dump too short ($($d.Length) bytes)"; exit 1 }
    $dMagic  = [BitConverter]::ToUInt32($d, 0)
    $dRecSz  = [BitConverter]::ToUInt16($d, 4)
    $dCap    = [BitConverter]::ToUInt16($d, 6)
    $dWr     = [BitConverter]::ToUInt32($d, 8)
    $dTotal  = [BitConverter]::ToUInt32($d, 12)
    $dFrozen = [BitConverter]::ToUInt32($d, 16)
    if ($dMagic -ne $b4Magic) {
        Write-Warning ("No Buddy 4 data: magic 0x{0:x8}" -f $dMagic)
    }
    elseif ($dRecSz -ne $b4RecBytes) {
        Write-Error "Buddy 4 record size $dRecSz, decoder expects $b4RecBytes"; exit 1
    }
    elseif ($d.Length -lt ($headerBytes + $dCap * $dRecSz)) {
        Write-Error "Buddy 4 dump shorter than the ring"; exit 1
    }
    else {
        if ($dFrozen -eq 0) { Write-Warning "Buddy 4 ring not frozen." }
        $dn     = [Math]::Min([uint32]$dCap, $dTotal)
        $dFirst = ([int]$dWr - [int]$dn + $dCap) % $dCap
        $db = New-Object System.Text.StringBuilder
        [void]$db.AppendLine("row,ms,dt_ms,accel_x_mg,accel_y_mg,accel_z_mg,incline_mg,baseline_mg,delta_mg,noise_mg,active_height_cmm,active_height_mm,final_height_cmm,final_height_mm,mag_x,mag_y,mag_z,left_enc,right_enc,drive_speed_mm_s,dt_us,hump_state,motion,turn,tilt,flags,valid,mag_new,height_valid,cue,started,completed,stale,jolt")
        $dPrev  = $null
        $dMaxUs = 0
        for ($i = 0; $i -lt $dn; $i++) {
            $o     = $headerBytes + (($dFirst + $i) % $dCap) * $dRecSz
            $ms    = [BitConverter]::ToUInt32($d, $o)
            $dt    = ""
            if ($null -ne $dPrev) {
                $e = [int64]$ms - [int64]$dPrev
                if ($e -lt 0) { $e += 4294967296 }
                $dt = $e
            }
            $dPrev = $ms
            $us    = [BitConverter]::ToUInt16($d, $o + 34)
            if ($us -gt $dMaxUs) { $dMaxUs = $us }
            $fl    = $d[$o + 37]
            $mt    = $d[$o + 38]
            $act   = [BitConverter]::ToUInt16($d, $o + 18)
            $fin   = [BitConverter]::ToUInt16($d, $o + 20)
            [void]$db.AppendLine(("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11:0.00},{12},{13:0.00},{14},{15},{16},{17},{18},{19},{20},{21},{22},{23},{24},{25},{26},{27},{28},{29},{30},{31},{32},{33}" -f `
                $i, $ms, $dt,
                [BitConverter]::ToInt16($d, $o + 4), [BitConverter]::ToInt16($d, $o + 6),
                [BitConverter]::ToInt16($d, $o + 8), [BitConverter]::ToInt16($d, $o + 10),
                [BitConverter]::ToInt16($d, $o + 12), [BitConverter]::ToInt16($d, $o + 14),
                [BitConverter]::ToUInt16($d, $o + 16),
                $act, ($act / 100.0), $fin, ($fin / 100.0),
                [BitConverter]::ToInt16($d, $o + 22), [BitConverter]::ToInt16($d, $o + 24),
                [BitConverter]::ToInt16($d, $o + 26),
                [BitConverter]::ToUInt16($d, $o + 28), [BitConverter]::ToUInt16($d, $o + 30),
                [BitConverter]::ToInt16($d, $o + 32), $us,
                $d[$o + 36], ($mt -band 15), ($mt -shr 4), $d[$o + 39], $fl,
                ($fl -band 1), (($fl -shr 1) -band 1), (($fl -shr 2) -band 1),
                (($fl -shr 3) -band 1), (($fl -shr 4) -band 1),
                (($fl -shr 5) -band 1), (($fl -shr 6) -band 1),
                (($fl -shr 7) -band 1)))
        }
        [System.IO.File]::WriteAllText($b4Csv, $db.ToString(), [System.Text.Encoding]::ASCII)
        Write-Host ("{0} Buddy 4 rows (total {1}, frozen {2}), longest dt_us {3}" -f $dn, $dTotal, $dFrozen, $dMaxUs)
        Write-Host "Buddy 4 CSV: $b4Csv"
    }
}

# 6. Event log, if it was dumped.
if ($evBin -ne "") {
    $e = [System.IO.File]::ReadAllBytes($evBin)
    if ($e.Length -lt $evHeader) { Write-Error "Event dump too short ($($e.Length) bytes)"; exit 1 }
    $eMagic   = [BitConverter]::ToUInt32($e, 0)
    $eRecSz   = [BitConverter]::ToUInt16($e, 4)
    $eCap     = [BitConverter]::ToUInt16($e, 6)
    $eCount   = [BitConverter]::ToUInt16($e, 8)
    $eDropped = [BitConverter]::ToUInt16($e, 10)
    $eFrozen  = [BitConverter]::ToUInt16($e, 12)
    if ($eMagic -ne $evMagic) {
        Write-Warning ("No event data: magic 0x{0:x8}" -f $eMagic)
    }
    elseif ($eRecSz -ne $evRecBytes) {
        Write-Error "Event record size $eRecSz, decoder expects $evRecBytes"; exit 1
    }
    elseif ($e.Length -lt ($evHeader + $eCap * $eRecSz)) {
        Write-Error "Event dump shorter than the log"; exit 1
    }
    else {
        if ($eFrozen -eq 0) { Write-Warning "Event log not frozen." }
        $en = [Math]::Min([int]$eCap, [int]$eCount)
        $eb = New-Object System.Text.StringBuilder
        [void]$eb.AppendLine("row,ms,event,value,count,valid")
        for ($i = 0; $i -lt $en; $i++) {
            $o = $evHeader + $i * $eRecSz
            $eType = $e[$o + 6]
            $eCnt  = $e[$o + 7] -band 0x7F
            $eVal  = ""
            if ($eType -eq 5) {
                $eVal = ($e[$o + 7] -shr 7) -band 1
            }
            [void]$eb.AppendLine(("{0},{1},{2},{3},{4},{5}" -f `
                $i, [BitConverter]::ToUInt32($e, $o), (Get-Name $evNames $eType),
                [BitConverter]::ToInt16($e, $o + 4), $eCnt, $eVal))
        }
        [System.IO.File]::WriteAllText($evCsv, $eb.ToString(), [System.Text.Encoding]::ASCII)
        Write-Host ("{0} events (dropped {1}, frozen {2})" -f $en, $eDropped, $eFrozen)
        Write-Host "Events CSV: $evCsv"
    }
}

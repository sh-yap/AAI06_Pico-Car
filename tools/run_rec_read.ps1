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
# -Elf defaults to the newest build_make\*_rec_vc_car_*.elf.  It must be the
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
# With -File, pass the barcode dump as -BcFile.
# A warning is printed if the ring was not frozen (run still going, or the
# image was reset before it ended).

param(
    [string]$Name    = "run",
    [string]$Elf     = "",
    [string]$File    = "",
    [string]$BcFile  = "",
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
$bcRecBytes  = 20
$bcMagic     = 0x31524342
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
}
else {
    if ($Elf -eq "") {
        $latest = Get-ChildItem (Join-Path $root "build_make") -Filter "*_rec_vc_car*.elf" |
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
    & (Join-Path $OpenOcd "openocd.exe") -s (Join-Path $OpenOcd "scripts") `
        -f interface/cmsis-dap.cfg -f target/rp2040.cfg `
        -c "adapter speed $Speed" -c init `
        -c ("dump_image {{{0}}} 0x{1:x8} 0x{2:x}" -f $binTcl, $addr, $size) `
        @bcCmd `
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
    elseif ($cRecSz -ne $bcRecBytes) {
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
        [void]$cb.AppendLine("row,ms,dt_ms,barcode_adc,threshold,black,enc,enc_delta,capture_done,capture_active,edges,code,code_id,centre_adc,right_adc")
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
            [void]$cb.AppendLine(("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}" -f `
                $i, $ms, $dt,
                [BitConverter]::ToUInt16($c, $o + 8), [BitConverter]::ToUInt16($c, $o + 10),
                ($flags -band 1), $enc, $de,
                (($flags -shr 1) -band 1), (($flags -shr 2) -band 1),
                $c[$o + 17], $ch, $c[$o + 18],
                [BitConverter]::ToUInt16($c, $o + 12), [BitConverter]::ToUInt16($c, $o + 14)))
        }
        [System.IO.File]::WriteAllText($bcCsv, $cb.ToString(), [System.Text.Encoding]::ASCII)
        Write-Host ("{0} barcode rows (total {1}, frozen {2}), captures finished {3}, longest step {4} ms" -f $cn, $cTotal, $cFrozen, $cDone, $cMaxDt)
        Write-Host "Barcode CSV: $bcCsv"
    }
}

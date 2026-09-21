$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$bin = Join-Path $root "STM32_F103_RC_Car\Build\STM32_F103_RC_Car.bin"
$buildScript = Join-Path $PSScriptRoot "build_f103_project.ps1"

# Always rebuild before flashing. This prevents a previous successful .bin
# from being programmed after a later source compilation failed.
& powershell -NoProfile -ExecutionPolicy Bypass -File $buildScript
if ($LASTEXITCODE -ne 0) {
    throw "Firmware build failed; refusing to flash a stale binary."
}

if (!(Test-Path -LiteralPath $bin)) {
    throw ".bin not found. Build first: powershell -ExecutionPolicy Bypass -File Tools\build_f103_project.ps1"
}

# Copy to an ASCII-only local path: some STM32_Programmer_CLI versions cannot
# open files under paths with non-ANSI characters ('바탕 화면'), and the copy
# also forces OneDrive to hydrate a Files-On-Demand placeholder.
$tmpBin = Join-Path $env:TEMP "STM32_F103_RC_Car.bin"
Copy-Item -LiteralPath $bin -Destination $tmpBin -Force

# --- Attempt 1: ST-LINK SWD via STM32_Programmer_CLI --------------------------
$programmer = "C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin\STM32_Programmer_CLI.exe"
if (!(Test-Path $programmer)) {
    $found = Get-ChildItem "C:\ST\*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe" -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($null -eq $found) {
        $found = Get-ChildItem "C:\Program Files*\STMicroelectronics\*\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -ErrorAction SilentlyContinue |
            Select-Object -First 1
    }
    if ($null -ne $found) { $programmer = $found.FullName } else { $programmer = $null }
}

if ($null -ne $programmer) {
    Write-Host "Flashing over SWD with $programmer"
    & $programmer -c port=SWD -w $tmpBin 0x08000000 -v -rst
    if ($LASTEXITCODE -eq 0) {
        Write-Host "SWD flash OK."
        exit 0
    }
    Write-Warning "STM32_Programmer_CLI failed (exit $LASTEXITCODE). Trying the NUCLEO USB drive instead."
} else {
    Write-Warning "STM32_Programmer_CLI not found. Trying the NUCLEO USB drive instead."
}

# --- Attempt 2: drag-and-drop mass-storage flash (NODE_F103RB drive) ----------
$msd = Get-CimInstance Win32_LogicalDisk |
    Where-Object { $_.VolumeName -match "NODE|NOD_|NUCLEO|MBED|DIS_" } |
    Select-Object -First 1
if ($null -eq $msd) {
    throw @"
No flashing path available.
- STM32_Programmer_CLI was not found or failed, and no NUCLEO USB drive (NODE_F103RB) is mounted.
- Connect the board over USB and either install STM32CubeProgrammer or copy this file onto the NODE_F103RB drive manually:
  $bin
"@
}

$drive = $msd.DeviceID
Write-Host "Copying .bin onto NUCLEO drive $drive ($($msd.VolumeName))..."
Copy-Item -LiteralPath $tmpBin -Destination "$drive\" -Force
Write-Host "Copy done. The ST-LINK LED blinks red/green while programming (a few seconds)."
Start-Sleep -Seconds 6

if (Test-Path "$drive\FAIL.TXT") {
    $why = Get-Content "$drive\FAIL.TXT" -ErrorAction SilentlyContinue
    throw "Mass-storage flash FAILED: $why"
}
Write-Host "Mass-storage flash OK (no FAIL.TXT). Press the black RESET button if the app did not restart."

param(
    [ValidateSet(-1, 0, 1)]
    [int]$EncoderFeedback = -1
)

$ErrorActionPreference = "Stop"

function Assert-NativeSuccess([string]$Step) {
    if ($LASTEXITCODE -ne 0) {
        throw "$Step failed with exit code $LASTEXITCODE"
    }
}

function Publish-WithRetry([string]$Source, [string]$Destination) {
    for ($attempt = 1; $attempt -le 10; $attempt++) {
        try {
            Copy-Item -LiteralPath $Source -Destination $Destination -Force
            return
        }
        catch {
            if ($attempt -eq 10) {
                throw
            }
            Start-Sleep -Milliseconds 200
        }
    }
}

$root = Split-Path -Parent $PSScriptRoot
$projectName = "STM32_F103_RC_Car"
$project = Join-Path $root $projectName
$build = Join-Path $project "Build"
$toolBin = "C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
if (!(Test-Path (Join-Path $toolBin "arm-none-eabi-gcc.exe"))) {
    # Plugin folder names change between CubeIDE releases; probe for any install.
    $found = Get-ChildItem "C:\ST\*\STM32CubeIDE\plugins\*gnu-tools-for-stm32*\tools\bin\arm-none-eabi-gcc.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -eq $found) {
        throw "ARM GCC not found under C:\ST. Install STM32CubeIDE or edit `$toolBin in this script."
    }
    $toolBin = $found.DirectoryName
}
$gcc = Join-Path $toolBin "arm-none-eabi-gcc.exe"
$size = Join-Path $toolBin "arm-none-eabi-size.exe"
$objcopy = Join-Path $toolBin "arm-none-eabi-objcopy.exe"

New-Item -ItemType Directory -Force -Path $build | Out-Null
$stage = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("PowerOn_F103_" + [Guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Path $stage | Out-Null

$include = Join-Path $project "Inc"
$startup = Join-Path $project "Startup\startup_stm32f103rb.s"
$main = Join-Path $project "Src\main.c"
$syscalls = Join-Path $project "Src\syscalls.c"
$ld = Join-Path $project "STM32F103RB_FLASH.ld"

$startupObj = Join-Path $stage "startup_stm32f103rb.o"
$mainObj = Join-Path $stage "main.o"
$syscallsObj = Join-Path $stage "syscalls.o"
$elfStage = Join-Path $stage "$projectName.elf"
$binStage = Join-Path $stage "$projectName.bin"
$hexStage = Join-Path $stage "$projectName.hex"
$elf = Join-Path $build "$projectName.elf"
$bin = Join-Path $build "$projectName.bin"
$hex = Join-Path $build "$projectName.hex"
$uploadBin = Join-Path $root "UPLOAD_NUCLEO_F103RB.bin"

# -Os: at -O0 the soft-float control tick blocks UART polling long enough to
# matter; optimized code also keeps the 10 ms loop comfortably on schedule.
$common = @(
    "-mcpu=cortex-m3",
    "-mthumb",
    "-g3",
    "-Os",
    "-ffunction-sections",
    "-fdata-sections",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-DSTM32F103xB",
    "-I$include"
)
if ($EncoderFeedback -ge 0) {
    $common += "-DRC_ENCODER_FEEDBACK_ENABLED=$EncoderFeedback"
}

& $gcc @common -x assembler-with-cpp -c $startup -o $startupObj
Assert-NativeSuccess "startup assembly"
& $gcc @common -std=gnu11 -c $main -o $mainObj
Assert-NativeSuccess "main.c compilation"
& $gcc @common -std=gnu11 -c $syscalls -o $syscallsObj
Assert-NativeSuccess "syscalls.c compilation"

& $gcc "-mcpu=cortex-m3" "-mthumb" `
    "-T$ld" `
    "-Wl,--gc-sections" `
    "-static" `
    "--specs=nano.specs" `
    "--specs=nosys.specs" `
    $startupObj $mainObj $syscallsObj `
    "-Wl,--start-group" "-lc" "-lm" "-lnosys" "-Wl,--end-group" `
    -o $elfStage
Assert-NativeSuccess "firmware link"

& $size $elfStage
Assert-NativeSuccess "firmware size inspection"

# .bin for NUCLEO drag-and-drop (NODE_F103RB drive) and ST-LINK CLI; .hex as a
# second flashable format. The drag-and-drop flasher does NOT accept .elf.
& $objcopy -O binary $elfStage $binStage
Assert-NativeSuccess "binary image generation"
& $objcopy -O ihex $elfStage $hexStage
Assert-NativeSuccess "Intel HEX generation"

$binSize = (Get-Item -LiteralPath $binStage).Length
if ($binSize -gt 500KB) {
    Remove-Item -LiteralPath $stage -Recurse -Force
    throw ".bin is $binSize bytes, over the 500 KB board upload limit"
}

# Compile and convert away from the OneDrive workspace, then publish only a
# complete, verified set. Retry transient OneDrive/antivirus file locks.
Publish-WithRetry $elfStage $elf
Publish-WithRetry $binStage $bin
Publish-WithRetry $hexStage $hex
Publish-WithRetry $binStage $uploadBin
Remove-Item -LiteralPath $stage -Recurse -Force

Write-Host "F103 build complete:"
Write-Host "  ELF: $elf"
Write-Host ("  BIN: {0} ({1:N0} bytes, board upload limit 500 KB)" -f $bin, $binSize)
Write-Host "  HEX: $hex"
Write-Host "  Upload copy: $uploadBin"
Write-Host "Flash: powershell -ExecutionPolicy Bypass -File tools\flash_f103_project.ps1"
Write-Host "   or: copy the .bin onto the NODE_F103RB USB drive"

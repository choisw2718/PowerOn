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

$project = $PSScriptRoot
$root = Split-Path -Parent $project
$sharedProject = Join-Path $root "STM32_F103_RC_Car"
$build = Join-Path $project "Build"
$projectName = "STM32_F103_Right_Rear_Test"

$toolBin = "C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
if (!(Test-Path (Join-Path $toolBin "arm-none-eabi-gcc.exe"))) {
    $found = Get-ChildItem "C:\ST\*\STM32CubeIDE\plugins\*gnu-tools-for-stm32*\tools\bin\arm-none-eabi-gcc.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -eq $found) {
        throw "ARM GCC not found under C:\ST. Install STM32CubeIDE or edit `$toolBin in this script."
    }
    $toolBin = $found.DirectoryName
}

$gcc = Join-Path $toolBin "arm-none-eabi-gcc.exe"
$size = Join-Path $toolBin "arm-none-eabi-size.exe"
$objcopy = Join-Path $toolBin "arm-none-eabi-objcopy.exe"
$include = Join-Path $sharedProject "Inc"
$startup = Join-Path $sharedProject "Startup\startup_stm32f103rb.s"
$main = Join-Path $project "main.c"
$syscalls = Join-Path $sharedProject "Src\syscalls.c"
$linker = Join-Path $sharedProject "STM32F103RB_FLASH.ld"

New-Item -ItemType Directory -Force -Path $build | Out-Null
$stage = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("PowerOn_Right_Rear_Test_" + [Guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Path $stage | Out-Null

try {
    $startupObject = Join-Path $stage "startup_stm32f103rb.o"
    $mainObject = Join-Path $stage "main.o"
    $syscallsObject = Join-Path $stage "syscalls.o"
    $elfStage = Join-Path $stage "$projectName.elf"
    $binStage = Join-Path $stage "$projectName.bin"
    $hexStage = Join-Path $stage "$projectName.hex"
    $elf = Join-Path $build "$projectName.elf"
    $bin = Join-Path $build "$projectName.bin"
    $hex = Join-Path $build "$projectName.hex"

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
        "-I$include",
        "-I$project"
    )

    & $gcc @common -x assembler-with-cpp -c $startup -o $startupObject
    Assert-NativeSuccess "startup assembly"
    & $gcc @common -std=gnu11 -c $main -o $mainObject
    Assert-NativeSuccess "main.c compilation"
    & $gcc @common -std=gnu11 -c $syscalls -o $syscallsObject
    Assert-NativeSuccess "syscalls.c compilation"

    & $gcc "-mcpu=cortex-m3" "-mthumb" `
        "-T$linker" `
        "-Wl,--gc-sections" `
        "-static" `
        "--specs=nano.specs" `
        "--specs=nosys.specs" `
        $startupObject $mainObject $syscallsObject `
        "-Wl,--start-group" "-lc" "-lnosys" "-Wl,--end-group" `
        -o $elfStage
    Assert-NativeSuccess "firmware link"

    & $size $elfStage
    Assert-NativeSuccess "firmware size inspection"
    & $objcopy -O binary $elfStage $binStage
    Assert-NativeSuccess "binary image generation"
    & $objcopy -O ihex $elfStage $hexStage
    Assert-NativeSuccess "Intel HEX generation"

    $binSize = (Get-Item -LiteralPath $binStage).Length
    if ($binSize -gt 500KB) {
        throw ".bin is $binSize bytes, over the 500 KB board upload limit"
    }

    Publish-WithRetry $elfStage $elf
    Publish-WithRetry $binStage $bin
    Publish-WithRetry $hexStage $hex
}
finally {
    if (Test-Path -LiteralPath $stage) {
        Remove-Item -LiteralPath $stage -Recurse -Force
    }
}

Write-Host "Right-rear wheel test build complete:"
Write-Host "  BIN: $bin"
Write-Host "  HEX: $hex"
Write-Host "  ELF: $elf"
Write-Host "Flash: powershell -ExecutionPolicy Bypass -File STM32_F103_Right_Rear_Test\flash.ps1"

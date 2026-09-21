$ErrorActionPreference = "Stop"

$project = $PSScriptRoot
$verifyScript = Join-Path $project "verify.ps1"
$bin = Join-Path $project "Build\STM32_F103_Right_Rear_Test.bin"

# Always rebuild and verify safety markers before flashing.
& powershell -NoProfile -ExecutionPolicy Bypass -File $verifyScript
if ($LASTEXITCODE -ne 0) {
    throw "Right-rear test verification failed; refusing to flash a stale binary."
}

$temporaryBin = Join-Path $env:TEMP "STM32_F103_Right_Rear_Test.bin"
Copy-Item -LiteralPath $bin -Destination $temporaryBin -Force

$programmer = "C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin\STM32_Programmer_CLI.exe"
if (!(Test-Path $programmer)) {
    $found = Get-ChildItem "C:\ST\*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -eq $found) {
        $found = Get-ChildItem "C:\Program Files*\STMicroelectronics\*\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
    }
    if ($null -ne $found) {
        $programmer = $found.FullName
    }
    else {
        $programmer = $null
    }
}

if ($null -ne $programmer) {
    Write-Host "Flashing right-rear wheel test over ST-LINK SWD..."
    & $programmer -c port=SWD -w $temporaryBin 0x08000000 -v -rst
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Right-rear wheel test flash OK."
        exit 0
    }
    Write-Warning "STM32_Programmer_CLI failed. Trying the NUCLEO USB drive."
}

$massStorage = Get-CimInstance Win32_LogicalDisk |
    Where-Object { $_.VolumeName -match "NODE|NOD_|NUCLEO|MBED|DIS_" } |
    Select-Object -First 1
if ($null -eq $massStorage) {
    throw "No NUCLEO flashing path found. Copy this file to NODE_F103RB manually: $bin"
}

Write-Host "Copying right-rear wheel test to $($massStorage.DeviceID) ($($massStorage.VolumeName))..."
Copy-Item -LiteralPath $temporaryBin -Destination "$($massStorage.DeviceID)\" -Force
Write-Host "Copy complete. Press RESET if the test does not start automatically."

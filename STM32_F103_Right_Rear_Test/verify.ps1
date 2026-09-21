$ErrorActionPreference = "Stop"

$project = $PSScriptRoot
$buildScript = Join-Path $project "build.ps1"
$bin = Join-Path $project "Build\STM32_F103_Right_Rear_Test.bin"

& powershell -NoProfile -ExecutionPolicy Bypass -File $buildScript
if ($LASTEXITCODE -ne 0) {
    throw "Right-rear test build failed."
}

if (!(Test-Path -LiteralPath $bin)) {
    throw "Expected test binary was not generated: $bin"
}

$firmwareText = [System.Text.Encoding]::ASCII.GetString(
    [System.IO.File]::ReadAllBytes($bin))
$requiredMarkers = @(
    "RIGHT_REAR_WHEEL_TEST",
    "right-rear-rb35gm-test-v2-20260915",
    "LEFT_DRIVE=LOCKED_OUT",
    "SERVO=NOT_INITIALIZED",
    "AUTO_STOP_MS=1000",
    "MAX_DUTY=10%",
    "MOTOR=RB35GM_09TYPE_24V_26P",
    "BOOT_STATE=STOP"
)

foreach ($marker in $requiredMarkers) {
    if (!$firmwareText.Contains($marker)) {
        throw "Safety marker missing from firmware binary: $marker"
    }
}

$hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bin).Hash
$size = (Get-Item -LiteralPath $bin).Length
Write-Host "Right-rear test firmware verification passed."
Write-Host "  Size: $size bytes"
Write-Host "  SHA-256: $hash"
Write-Host "  Safety markers: left locked out, servo off, 1 s auto-stop, 10% cap"

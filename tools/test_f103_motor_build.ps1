$ErrorActionPreference = "Stop"

$buildScript = Join-Path $PSScriptRoot "build_f103_project.ps1"
$root = Split-Path -Parent $PSScriptRoot
$bin = Join-Path $root "STM32_F103_RC_Car\Build\STM32_F103_RC_Car.bin"
$uploadBin = Join-Path $root "UPLOAD_NUCLEO_F103RB.bin"

Write-Host "Build test 1/2: encoder-independent fallback"
& powershell -NoProfile -ExecutionPolicy Bypass -File $buildScript -EncoderFeedback 0
if ($LASTEXITCODE -ne 0) {
    throw "Feedback-disabled build failed."
}

Write-Host "Build test 2/2: dual rear encoder feedback (shipping image)"
& powershell -NoProfile -ExecutionPolicy Bypass -File $buildScript -EncoderFeedback 1
if ($LASTEXITCODE -ne 0) {
    throw "Feedback-enabled build failed."
}

$firmwareText = [System.Text.Encoding]::ASCII.GetString(
    [System.IO.File]::ReadAllBytes($bin))
if (!$firmwareText.Contains("motor-v7-rb35gm-dual-20260915") -or
    !$firmwareText.Contains("RB35GM_09TYPE_26P") -or
    !$firmwareText.Contains("RB35GM CALIBRATION REQUIRED") -or
    !$firmwareText.Contains("Dual rear encoder feedback: ON") -or
    !$firmwareText.Contains("Rear drive: two RB-35GM")) {
    throw "Shipping binary does not contain the expected dual-RB35GM safety markers."
}

$buildHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bin).Hash
$uploadHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $uploadBin).Hash
if ($buildHash -ne $uploadHash) {
    throw "Build binary and root upload binary do not match."
}

Write-Host "Both dual-RB35GM motor-control build configurations passed."
Write-Host "Shipping BIN SHA-256: $buildHash"

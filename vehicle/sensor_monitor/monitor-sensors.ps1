param(
    [string]$NanoHost = '',
    [double]$Interval = 0.5,
    [double]$Duration = 0
)

$ErrorActionPreference = 'Stop'
$projectRoot = Join-Path $PSScriptRoot '../..'
$monitorArgs = @('-3', '-u', '-m', 'vehicle', '--sensors',
    '--sensor-interval', $Interval.ToString([Globalization.CultureInfo]::InvariantCulture),
    '--sensor-duration', $Duration.ToString([Globalization.CultureInfo]::InvariantCulture))
if ($NanoHost) { $monitorArgs += @('--host', $NanoHost) }
Push-Location -LiteralPath $projectRoot
try {
    & py @monitorArgs
} finally {
    Pop-Location
}
if ($LASTEXITCODE -ne 0) {
    Write-Host "Sensor monitor stopped with error code $LASTEXITCODE. See the message above."
}

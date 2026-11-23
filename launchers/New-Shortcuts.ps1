$ErrorActionPreference = 'Stop'

$projectRoot = (Get-Item -LiteralPath (Join-Path $PSScriptRoot '..')).FullName
$powerShell = Join-Path ([Environment]::SystemDirectory) 'WindowsPowerShell\v1.0\powershell.exe'
$shell = New-Object -ComObject WScript.Shell

$commands = [ordered]@{
    'PowerOn PowerShell'   = ''
    'Manual Control'      = 'py -3 -m vehicle'
    'IR Sensor Monitor'   = 'py -3 -u -m vehicle --sensors'
    'IR Preview'          = 'py -3 -m vehicle --preview-csv vehicle/ir_wall_following/examples/ir_wall_follow_preview.csv'
    'LiDAR Radar'         = 'python -m lidar.radar --fallback-ssh user@andrea.local'
    'LiDAR SSH Preview'   = 'python -m lidar.autopilot --ssh user@andrea.local --seconds 10 --scan-timing'
    'Track Simulator'     = 'python -m simulation.track'
    'Simulation Benchmark' = 'python -m simulation.benchmark'
    'Run Tests'           = 'py -3 tests/run_all.py'
}

$capture = 'build/lidar_captures/20261111_154213/scan.csv'
if (Test-Path -LiteralPath (Join-Path $projectRoot $capture) -PathType Leaf) {
    $commands['LiDAR CSV Replay'] = "python -m lidar.autopilot --csv $capture --scan-timing"
}

foreach ($entry in $commands.GetEnumerator()) {
    $link = $shell.CreateShortcut((Join-Path $PSScriptRoot "$($entry.Key).lnk"))
    $link.TargetPath = $powerShell
    $link.WorkingDirectory = $projectRoot
    $link.Description = "PowerOn - $($entry.Key)"
    $link.Arguments = '-NoLogo -NoProfile -NoExit'
    if ($entry.Value) {
        $link.Arguments += " -Command $($entry.Value)"
    }
    $link.Save()
}

Write-Output "Created $($commands.Count) PowerShell shortcuts in $PSScriptRoot"

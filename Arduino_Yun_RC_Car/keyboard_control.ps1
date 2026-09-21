param(
  [Parameter(Mandatory = $true)]
  [ValidatePattern('^COM[1-9][0-9]*$')]
  [string]$Port,

  [ValidateRange(1200, 2000000)]
  [int]$BaudRate = 115200
)

$ErrorActionPreference = 'Stop'
$serialPort = [System.IO.Ports.SerialPort]::new(
  $Port,
  $BaudRate,
  [System.IO.Ports.Parity]::None,
  8,
  [System.IO.Ports.StopBits]::One
)
$serialPort.DtrEnable = $true
$serialPort.RtsEnable = $false
$serialPort.ReadTimeout = 50
$serialPort.WriteTimeout = 500

function Send-ControlKey {
  param([char]$Command)

  $serialPort.Write([string]$Command)
}

try {
  $serialPort.Open()
  Start-Sleep -Milliseconds 1200
  $null = $serialPort.ReadExisting()

  Write-Host "Connected to $Port at $BaudRate baud"
  Write-Host 'Arrow keys or WASD: drive/steer | C: center | X/Space: stop'
  Write-Host 'I: stop and center | H: help | Q/Esc: quit safely'

  $running = $true
  while ($running)
  {
    if ($serialPort.BytesToRead -gt 0)
    {
      Write-Host -NoNewline $serialPort.ReadExisting()
    }

    if (![Console]::KeyAvailable)
    {
      Start-Sleep -Milliseconds 10
      continue
    }

    $key = [Console]::ReadKey($true)
    $command = $null

    switch ($key.Key)
    {
      'UpArrow'    { $command = 'w' }
      'DownArrow'  { $command = 's' }
      'LeftArrow'  { $command = 'a' }
      'RightArrow' { $command = 'd' }
      'Spacebar'   { $command = ' ' }
      'Escape'     { $running = $false }
      'Q'          { $running = $false }
      default
      {
        $typed = [char]::ToLowerInvariant($key.KeyChar)
        if ('wasdczxih?'.Contains($typed))
        {
          $command = $typed
        }
      }
    }

    if ($null -ne $command)
    {
      Send-ControlKey ([char]$command)
    }
  }
}
finally
{
  if ($serialPort.IsOpen)
  {
    try
    {
      Send-ControlKey 'i'
      Start-Sleep -Milliseconds 50
    }
    finally
    {
      $serialPort.Close()
    }
  }
}

Write-Host 'Disconnected after sending stop-and-center.'

param([string]$LogDirectory = '')

$ErrorActionPreference = 'Stop'
$rootCandidate = [IO.DirectoryInfo]$PSScriptRoot
while ($null -ne $rootCandidate -and -not (Test-Path -LiteralPath (Join-Path $rootCandidate.FullName 'MDK-ARM\CtrlBoard-H7_IMU.uvprojx'))) {
    $rootCandidate = $rootCandidate.Parent
}
if ($null -eq $rootCandidate) { throw 'Cannot locate gimbal project root' }
$root = $rootCandidate.FullName
$config = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'projects.json')) | ConvertFrom-Json
$item = $config.projects.gimbal
$projectRoot = [IO.Path]::GetFullPath((Join-Path $root $item.root))
$projectPath = Join-Path $projectRoot $item.project
$keilExecutable = $config.keil_executable
if (-not (Test-Path -LiteralPath $keilExecutable -PathType Leaf)) { throw "Keil not found: $keilExecutable" }
if (-not (Test-Path -LiteralPath $projectPath -PathType Leaf)) { throw "Project not found: $projectPath" }

if ([string]::IsNullOrWhiteSpace($LogDirectory)) {
    $LogDirectory = Join-Path $root ('AI_Workspace\artifacts\logs\build-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
} elseif (-not [IO.Path]::IsPathRooted($LogDirectory)) {
    $LogDirectory = Join-Path $root $LogDirectory
}
if (Test-Path -LiteralPath (Join-Path $LogDirectory 'gimbal.log')) { throw "Refusing to overwrite build log: $LogDirectory" }
$null = New-Item -ItemType Directory -Path $LogDirectory -Force
$logRoot = (Resolve-Path -LiteralPath $LogDirectory).Path
$logPath = Join-Path $logRoot 'gimbal.log'

[xml]$projectXml = [IO.File]::ReadAllText($projectPath)
$targetXml = @($projectXml.Project.Targets.Target | Where-Object { $_.TargetName -eq $item.target })[0]
if ($null -eq $targetXml) { throw "Keil Target not found: $($item.target)" }
$outputOption = $targetXml.TargetOption.TargetCommonOption
$imagePath = Join-Path (Join-Path (Split-Path -Parent $projectPath) $outputOption.OutputDirectory) ($outputOption.OutputName + '.axf')
$arguments = '-r "{0}" -t "{1}" -o "{2}" -j0' -f $projectPath, $item.target, $logPath
$started = Get-Date
$process = Start-Process -FilePath $keilExecutable -ArgumentList $arguments -WorkingDirectory (Split-Path -Parent $projectPath) -WindowStyle Hidden -PassThru -Wait
$summary = @()
if (Test-Path -LiteralPath $logPath) {
    $summary = @(Get-Content -LiteralPath $logPath | Where-Object { $_ -match 'Error\(s\)|Warning\(s\)|Program Size|Target not created|Build Time|Error:|error:' })
}
$hasZeroErrors = @($summary | Where-Object { $_ -match ' - 0 Error\(s\),' }).Count -gt 0
$freshImage = (Test-Path -LiteralPath $imagePath) -and ((Get-Item -LiteralPath $imagePath).LastWriteTime -ge $started.AddSeconds(-2))
$success = ($process.ExitCode -in @(0, 1)) -and $hasZeroErrors -and $freshImage
$result = [pscustomobject]@{
    Board = 'gimbal'
    Target = $item.target
    Project = $projectPath
    ExitCode = $process.ExitCode
    Started = $started.ToString('o')
    Seconds = [math]::Round(((Get-Date) - $started).TotalSeconds, 2)
    Log = $logPath
    Summary = $summary
    Image = $imagePath
    ImageSHA256 = if ($success) { (Get-FileHash -LiteralPath $imagePath -Algorithm SHA256).Hash } else { $null }
    Success = $success
}
$result | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $logRoot 'results.json') -Encoding UTF8
$result | ConvertTo-Json -Depth 5
# uVision returns 1 for a successful build with warnings.
if (-not $success) { exit 1 }

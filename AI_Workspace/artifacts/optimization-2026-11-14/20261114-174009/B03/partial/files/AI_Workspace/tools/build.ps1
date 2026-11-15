<#
  Full rebuild of the gimbal Keil target.

  Preflight validates the project root, project file, Target name and every enabled
  Keil source entry before the evidence directory is created, so a stale path cannot
  leave a half-built stage package behind.
#>
param(
    [string]$LogDirectory = '',
    [string]$RunId = '',
    [string]$Batch = ''
)

$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------------------
# Step 1: locate this repository regardless of the shell working directory
# ---------------------------------------------------------------------------
$rootCandidate = [IO.DirectoryInfo]$PSScriptRoot
while ($null -ne $rootCandidate -and -not (Test-Path -LiteralPath (Join-Path $rootCandidate.FullName 'MDK-ARM\CtrlBoard-H7_IMU.uvprojx'))) {
    $rootCandidate = $rootCandidate.Parent
}
if ($null -eq $rootCandidate) { throw 'Cannot locate gimbal project root from ' + $PSScriptRoot }
$root = $rootCandidate.FullName
$config = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'projects.json')) | ConvertFrom-Json
$item = $config.projects.gimbal
$keilExecutable = $config.keil_executable

# ---------------------------------------------------------------------------
# Step 2: preflight the project before touching any output directory
# ---------------------------------------------------------------------------
$problems = @()
if (-not (Test-Path -LiteralPath $keilExecutable -PathType Leaf)) {
    $problems += "Keil executable missing: $keilExecutable"
}
$projectRoot = [IO.Path]::GetFullPath((Join-Path $root $item.root))
$projectPath = Join-Path $projectRoot $item.project
$missingEntries = @()
$disabledEntries = @()
$entryCount = 0
$outputInput = $null
if (-not (Test-Path -LiteralPath $projectRoot -PathType Container)) {
    $problems += "project root missing: $projectRoot"
} elseif (-not (Test-Path -LiteralPath $projectPath -PathType Leaf)) {
    $problems += "project file missing: $projectPath"
} else {
    try {
        [xml]$projectXml = [System.IO.File]::ReadAllText($projectPath)
    } catch {
        $problems += "project file unreadable: $projectPath"
    }
    if ($null -ne $projectXml) {
        $targetXml = @($projectXml.Project.Targets.Target | Where-Object { $_.TargetName -eq $item.target })[0]
        if ($null -eq $targetXml) {
            $declared = @($projectXml.Project.Targets.Target | ForEach-Object { $_.TargetName }) -join ', '
            $problems += "Target '$($item.target)' not found (declared: $declared)"
        } else {
            $projectDir = Split-Path -Parent $projectPath
            foreach ($group in $targetXml.Groups.Group) {
                $groupEnabled = $group.GroupOption.CommonProperty.IncludeInBuild
                foreach ($file in $group.Files.File) {
                    $entryCount++
                    $raw = [string]$file.FilePath
                    $include = $file.FileOption.CommonProperty.IncludeInBuild
                    $enabled = if ($include -eq '2' -or $null -eq $include) { $groupEnabled -ne '0' } else { $include -ne '0' }
                    if (-not $enabled) { $disabledEntries += $raw; continue }
                    $resolved = [IO.Path]::GetFullPath((Join-Path $projectDir ($raw -replace '\\', '/')))
                    if (-not (Test-Path -LiteralPath $resolved -PathType Leaf)) { $missingEntries += $raw }
                }
            }
            if ($missingEntries.Count -gt 0) {
                $problems += "$($missingEntries.Count) Keil source entries missing, first: $($missingEntries[0])"
            }
            $common = $targetXml.TargetOption.TargetCommonOption
            $outputInput = [pscustomobject]@{
                OutputDirectory = [string]$common.OutputDirectory
                OutputName      = [string]$common.OutputName
                CreateHexFile   = [string]$common.CreateHexFile
                Device          = [string]$common.Device
                Pack            = [string]$common.PackID
                Image           = [IO.Path]::GetFullPath((Join-Path (Join-Path $projectDir ([string]$common.OutputDirectory)) ([string]$common.OutputName + '.axf')))
            }
        }
    }
}
if ($problems.Count -gt 0) {
    throw ("Preflight failed, no build directory created:`n  " + ($problems -join "`n  "))
}

# ---------------------------------------------------------------------------
# Step 3: resolve the evidence directory (batch packages follow the report约定)
# ---------------------------------------------------------------------------
if ([string]::IsNullOrWhiteSpace($LogDirectory)) {
    if (-not [string]::IsNullOrWhiteSpace($RunId)) {
        $batchName = if ([string]::IsNullOrWhiteSpace($Batch)) { 'adhoc' } else { $Batch }
        $LogDirectory = Join-Path $root ("AI_Workspace\artifacts\optimization-2026-11-14\" + $RunId + "\" + $batchName + "\build")
    } else {
        $LogDirectory = Join-Path $root ('AI_Workspace\artifacts\logs\build-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
    }
} elseif (-not [IO.Path]::IsPathRooted($LogDirectory)) {
    $LogDirectory = Join-Path $root $LogDirectory
}
$null = New-Item -ItemType Directory -Path $LogDirectory -Force
$logRoot = (Resolve-Path -LiteralPath $LogDirectory).Path
$logPath = Join-Path $logRoot 'gimbal.log'
if (Test-Path -LiteralPath $logPath) { throw "Refusing to overwrite build evidence: $logPath" }

# ---------------------------------------------------------------------------
# Step 4: build the target and record the full evidence set
# ---------------------------------------------------------------------------
$arguments = '-r "{0}" -t "{1}" -o "{2}" -j0' -f $projectPath, $item.target, $logPath
$started = Get-Date
$process = Start-Process -FilePath $keilExecutable -ArgumentList $arguments -WorkingDirectory (Split-Path -Parent $projectPath) -WindowStyle Hidden -PassThru -Wait
$summary = @()
if (Test-Path -LiteralPath $logPath) {
    $summary = @(Get-Content -LiteralPath $logPath | Where-Object { $_ -match 'Error\(s\)|Warning\(s\)|Program Size|Target not created|Build Time|Error:|error:' })
}
$hasZeroErrors = @($summary | Where-Object { $_ -match ' - 0 Error\(s\),' }).Count -gt 0
$freshImage = (Test-Path -LiteralPath $outputInput.Image) -and ((Get-Item -LiteralPath $outputInput.Image).LastWriteTime -ge $started.AddSeconds(-2))
$success = ($process.ExitCode -in @(0, 1)) -and $hasZeroErrors -and $freshImage
$result = [pscustomobject]@{
    Board           = 'gimbal'
    Target          = $item.target
    Project         = $projectPath
    OutputDirectory = $outputInput.OutputDirectory
    OutputName      = $outputInput.OutputName
    Device          = $outputInput.Device
    Pack            = $outputInput.Pack
    KeilEntryCount  = $entryCount
    MissingEntries  = $missingEntries
    DisabledEntries = $disabledEntries
    ExitCode        = $process.ExitCode
    Started         = $started.ToString('o')
    Seconds         = [math]::Round(((Get-Date) - $started).TotalSeconds, 2)
    Log             = $logPath
    Summary         = $summary
    Image           = $outputInput.Image
    ImageSHA256     = if ($success) { (Get-FileHash -LiteralPath $outputInput.Image -Algorithm SHA256).Hash } else { $null }
    Success         = $success
}
$result | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $logRoot 'results.json') -Encoding UTF8
$result | ConvertTo-Json -Depth 5
# uVision returns 1 for a successful build with warnings.
if (-not $success) { exit 1 }

param(
    [Parameter(Mandatory=$true)][string]$ProjectRoot,
    [Parameter(Mandatory=$true)][string]$IocFile,
    [Parameter(Mandatory=$true)][string]$LogDirectory,
    [string]$FirmwarePath = ''
)

$ErrorActionPreference = 'Stop'
$rootCandidate = [IO.DirectoryInfo]$PSScriptRoot
while ($null -ne $rootCandidate -and -not (Test-Path -LiteralPath (Join-Path $rootCandidate.FullName 'MDK-ARM\CtrlBoard-H7_IMU.uvprojx'))) {
    $rootCandidate = $rootCandidate.Parent
}
if ($null -eq $rootCandidate) { throw 'Cannot locate gimbal project root' }
$workspaceRoot = $rootCandidate.FullName
$allowedCopyRoot = [IO.Path]::GetFullPath((Join-Path $workspaceRoot 'AI_Workspace\cache\cubemx'))
$config = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'projects.json')) | ConvertFrom-Json
$mxExecutable = $config.cubemx_executable
$mxInstall = Split-Path -Parent $mxExecutable
$javaExecutable = Join-Path $mxInstall 'jre\bin\java.exe'
if (-not [IO.Path]::IsPathRooted($ProjectRoot)) { $ProjectRoot = Join-Path $workspaceRoot $ProjectRoot }
$projectRootPath = (Resolve-Path -LiteralPath $ProjectRoot).Path
# This entry point is limited to disposable copies under this project.
if (-not $projectRootPath.StartsWith($allowedCopyRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'CubeMX validation must run in this project AI_Workspace/cache/cubemx copy'
}
if ([IO.Path]::IsPathRooted($IocFile) -or [IO.Path]::GetFileName($IocFile) -ne $IocFile) {
    throw 'IocFile must be a filename within the copy'
}
$iocPath = Join-Path $projectRootPath $IocFile
if (-not (Test-Path -LiteralPath $iocPath -PathType Leaf)) { throw "IOC missing: $iocPath" }
if (-not [IO.Path]::IsPathRooted($LogDirectory)) { $LogDirectory = Join-Path $workspaceRoot $LogDirectory }
$logPathCandidate = [IO.Path]::GetFullPath($LogDirectory)
if (-not $logPathCandidate.StartsWith((Join-Path $workspaceRoot 'AI_Workspace') + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'LogDirectory must stay inside this project AI_Workspace'
}
$LogDirectory = $logPathCandidate
$null = New-Item -ItemType Directory -Path $LogDirectory -Force
$logRoot = (Resolve-Path -LiteralPath $LogDirectory).Path
$scriptPath = Join-Path $logRoot 'commands.txt'
$outputPath = Join-Path $logRoot 'stdout.log'
if (Test-Path -LiteralPath $outputPath) { throw "Refusing to overwrite CubeMX log: $outputPath" }
$commands = @(
    'popupwrapper set',
    'popupwrapper record_answer "New STM32Cube firmware version available" BUTTON_1',
    'popupwrapper record_answer "Warning: Code Generation" YES',
    ('config load "{0}"' -f $iocPath),
    ('project path "{0}"' -f $projectRootPath)
)
if (-not [string]::IsNullOrWhiteSpace($FirmwarePath)) {
    $commands += ('project setCustomFWPath "{0}"' -f (Resolve-Path -LiteralPath $FirmwarePath).Path)
}
$commands += @('project generate','exit')
[IO.File]::WriteAllText($scriptPath,($commands -join "`n")+"`n",[Text.UTF8Encoding]::new($false))
$arguments = '-Xmx1024m --add-opens java.desktop/java.awt=ALL-UNNAMED --add-exports java.desktop/sun.awt=ALL-UNNAMED -Djavax.net.ssl.trustStoreType=WINDOWS-ROOT -jar "{0}" -q "{1}"' -f $mxExecutable,$scriptPath
$started = Get-Date
$process = Start-Process -FilePath $javaExecutable -ArgumentList $arguments -WorkingDirectory $mxInstall -WindowStyle Hidden -RedirectStandardOutput $outputPath -RedirectStandardError (Join-Path $logRoot 'stderr.log') -PassThru -Wait
$outputLines = @(Get-Content -LiteralPath $outputPath)
$generatorErrors = @($outputLines | Where-Object { $_ -match '\[ERROR\]' })
$hasCompletion = @($outputLines | Where-Object { $_ -eq 'OK' }).Count -gt 0
$success = $process.ExitCode -eq 0 -and $hasCompletion -and $generatorErrors.Count -eq 0
$result = [pscustomobject]@{
    Project = $projectRootPath
    Started = $started.ToString('o')
    ExitCode = $process.ExitCode
    Seconds = [math]::Round(((Get-Date)-$started).TotalSeconds,2)
    Log = $outputPath
    GeneratorErrorCount = $generatorErrors.Count
    HasCompletionMarker = $hasCompletion
    Success = $success
}
$result | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $logRoot 'result.json') -Encoding utf8
$result | ConvertTo-Json
if (-not $success) { exit 1 }
# A successful CLI run is only the first gate. Review the generated files,
# compile this project, then compare retained functions and regenerate.

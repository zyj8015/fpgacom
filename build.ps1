param(
    [string]$TdExe = 'D:\Anlogic\TD5.6.2\bin\td_commands_prompt.exe',
    [switch]$NoBit
)
$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $TdExe)) { throw "TD executable not found: $TdExe" }
$projectDirectory = Join-Path $PSScriptRoot 'src\td_project'
$outputDirectory = Join-Path $PSScriptRoot $(if ($NoBit) { 'build\check_td562' } else { 'build\td562' })
New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
$bitPath = Join-Path $outputDirectory 'func1.bit'
if (-not $NoBit -and (Test-Path -LiteralPath $bitPath)) { Remove-Item -LiteralPath $bitPath -Force }
$previousBuildOutput = $env:FUNC1_TD_BUILD_OUTPUT
$previousNoBit = $env:FUNC1_TD_NO_BIT
$previousTdTemp = $env:TEMP
$previousTdTmp = $env:TMP
$tdTempDirectory = Join-Path $PSScriptRoot 'build\td_temp'
New-Item -ItemType Directory -Path $tdTempDirectory -Force | Out-Null
Push-Location -LiteralPath $projectDirectory
try {
    $env:TEMP = $tdTempDirectory
    $env:TMP = $tdTempDirectory
    $env:FUNC1_TD_BUILD_OUTPUT = $outputDirectory
    $env:FUNC1_TD_NO_BIT = if ($NoBit) { '1' } else { '0' }
    Write-Host 'Using one TD process for synthesis, placement and routing (no import_db).'
    if ($NoBit) { Write-Host 'Compile check only: no bit file will be generated.' }
    $ErrorActionPreference = 'Continue'
    & $TdExe 'build.tcl' 2>&1 | Tee-Object -FilePath (Join-Path $outputDirectory 'build.log')
    $buildExitCode = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($buildExitCode -ne 0) { throw "TD build failed with exit code $buildExitCode" }
    if (Select-String -LiteralPath (Join-Path $outputDirectory 'build.log') -Pattern '\bERROR:' -Quiet) {
        throw 'TD reported an error; inspect build.log'
    }
    if ($NoBit) {
        if (-not (Select-String -LiteralPath (Join-Path $outputDirectory 'build.log') -Pattern 'PHOTO_CHECK_COMPLETE' -Quiet)) {
            throw 'TD did not complete the compile check; inspect build.log'
        }
        Write-Host ('Compile check completed; inspect timing report in ' + $outputDirectory)
    } else {
        if (-not (Test-Path -LiteralPath $bitPath)) { throw 'TD did not produce a bit; inspect build.log' }
        Get-FileHash -LiteralPath $bitPath -Algorithm SHA256 | Format-List
    }
} finally {
    $env:TEMP = $previousTdTemp
    $env:TMP = $previousTdTmp
    $env:FUNC1_TD_BUILD_OUTPUT = $previousBuildOutput
    $env:FUNC1_TD_NO_BIT = $previousNoBit
    Pop-Location
}

param(
    [string]$Team = "local",
    [string]$Scenario = "public_1",
    [int]$Attempt = 1,
    [string]$Gui = "false",
    [string]$Mode = "realtime"
)

$ErrorActionPreference = "Continue"

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host "  Virtual Courier: Official Jury Evaluation Run" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

# 1. Check Docker daemon
$serverVer = docker version --format '{{.Server.Version}}' 2>$null
if (-not $serverVer -or $LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] Docker Desktop is not running or not responding." -ForegroundColor Red
    Write-Host "Please start Docker Desktop on Windows and wait until it is ready." -ForegroundColor Yellow
    exit 1
}

# 2. Check Scenario file
$scenarioFile = Join-Path $PSScriptRoot "scene\scenarios\$Scenario.yaml"
if (-not (Test-Path $scenarioFile)) {
    if ($Scenario -like "[0-9]*") {
        $Scenario = "random_$Scenario"
        $scenarioFile = Join-Path $PSScriptRoot "scene\scenarios\$Scenario.yaml"
    }
    if (-not (Test-Path $scenarioFile)) {
        Write-Host "[ERROR] Scenario file not found: $scenarioFile" -ForegroundColor Red
        exit 1
    }
}

$useGui = if ($Gui -eq "true" -or $Gui -eq "1" -or $Gui -eq "$true") { "true" } else { "false" }

Write-Host ">>> Team:       $Team" -ForegroundColor Green
Write-Host ">>> Scenario:   $Scenario" -ForegroundColor Green
Write-Host ">>> Attempt:    $Attempt" -ForegroundColor Green
Write-Host ">>> Webots GUI: $useGui" -ForegroundColor Green
Write-Host ">>> Mode:       $Mode" -ForegroundColor Green

# 3. Clean temporary build artifacts in src/solution
$solutionPath = (Resolve-Path "$PSScriptRoot\src\solution").Path
Remove-Item -Recurse -Force "$solutionPath\build" -ErrorAction SilentlyContinue
Remove-Item -Recurse -Force "$solutionPath\install" -ErrorAction SilentlyContinue
Remove-Item -Recurse -Force "$solutionPath\log" -ErrorAction SilentlyContinue

# Ensure results directory exists
$resultsDir = Join-Path $PSScriptRoot "results"
if (-not (Test-Path $resultsDir)) {
    New-Item -ItemType Directory -Path $resultsDir | Out-Null
}

# 4. Set Environment Variables
$env:VC_TEAM = $Team
$env:SCENARIO = $Scenario
$env:VC_ATTEMPT = $Attempt
$env:WEBOTS_GUI = $useGui
$env:WEBOTS_MODE = $Mode
$env:COMPOSE_PROJECT_NAME = "vc_eval"
$env:SOLUTION_DIR = $solutionPath
$env:SCENARIO_DIR = (Resolve-Path "$PSScriptRoot\scene\scenarios").Path

# 5. Stop existing containers
Write-Host "`n>>> Stopping any existing vc_eval containers..." -ForegroundColor Yellow
docker compose -p vc_eval -f "$PSScriptRoot\docker-compose.yaml" -f "$PSScriptRoot\docker-compose.eval.yaml" down --remove-orphans 2>$null
docker rm -f vc_solution vc_scene vc_controllers 2>$null

# 6. Run official jury simulation
Write-Host ">>> Starting Jury Evaluation..." -ForegroundColor Cyan
$exitCode = 0
try {
    docker compose -p vc_eval -f "$PSScriptRoot\docker-compose.yaml" -f "$PSScriptRoot\docker-compose.eval.yaml" up --abort-on-container-exit --exit-code-from scene
    $exitCode = $LASTEXITCODE
} finally {
    Write-Host "`n>>> Cleaning up containers..." -ForegroundColor Yellow
    docker compose -p vc_eval -f "$PSScriptRoot\docker-compose.yaml" -f "$PSScriptRoot\docker-compose.eval.yaml" down --remove-orphans 2>$null
    docker rm -f vc_solution vc_scene vc_controllers 2>$null
}

# 7. Find and parse result JSON
$foundResults = Get-ChildItem -Path $resultsDir -Filter "${Team}_${Scenario}_${Attempt}_*.json" | Sort-Object LastWriteTime -Descending
if ($foundResults -and $foundResults.Count -gt 0) {
    $latestPath = $foundResults[0].FullName
    $json = Get-Content $latestPath -Raw -Encoding UTF8 | ConvertFrom-Json
    Write-Host "`n==========================================================" -ForegroundColor Green
    Write-Host "  JURY EVALUATION RESULT" -ForegroundColor Green
    Write-Host "==========================================================" -ForegroundColor Green
    Write-Host "Team:          $($json.team)"
    Write-Host "Scenario:      $($json.scenario)"
    Write-Host "Attempt:       $($json.attempt)"
    Write-Host "SCORE:         $($json.score)" -ForegroundColor Yellow
    Write-Host "Elapsed:       $($json.elapsed) s"
    Write-Host "Collisions:    $($json.collisions)"
    Write-Host "Finish reason: $($json.finish_reason)"
    Write-Host "----------------------------------------------------------"
    Write-Host "Timeline:"
    foreach ($entry in $json.log) {
        $pts = if ($entry.points -ge 0) { "+$($entry.points)" } else { "$($entry.points)" }
        Write-Host ("{0,7:F1} s  {1,4}  {2}" -f $entry.t, $pts, $entry.text)
    }
    Write-Host "==========================================================" -ForegroundColor Green
    Write-Host "Report file: $latestPath"
} else {
    Write-Warning "Result JSON file was not found (Exit code: $exitCode)."
}

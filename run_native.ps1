param(
    [string]$Scenario = "public_1",
    [string]$Mode = "realtime",
    [string]$WebotsPath = "$env:LOCALAPPDATA\Programs\Webots\msys64\mingw64\bin\webots.exe"
)

$ErrorActionPreference = "Continue"

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host "  Virtual Courier: Native Windows GPU Execution" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

# 1. Check Webots Path
if (-not (Test-Path $WebotsPath)) {
    $altPath = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
    if (Test-Path $altPath) {
        $WebotsPath = $altPath
    } else {
        Write-Error "Webots executable not found at: $WebotsPath"
        exit 1
    }
}
Write-Host "[1/4] Webots: $WebotsPath" -ForegroundColor Green

# 2. Get Windows Host IP for WSL/Docker containers
$wslIp = (Get-NetIPAddress -InterfaceAlias "*WSL*" -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object -ExpandProperty IPAddress -First 1)
if (-not $wslIp) {
    $wslIp = "172.21.224.1"
}
Write-Host "[2/4] Windows Host IP for Docker: $wslIp" -ForegroundColor Green
$env:WEBOTS_HOST = $wslIp

# 3. Generate world with <extern> controllers
$scenarioPath = Join-Path $PSScriptRoot "scene\scenarios\$Scenario.yaml"
if (-not (Test-Path $scenarioPath)) {
    Write-Error "Scenario file not found: $scenarioPath"
    exit 1
}

Write-Host "[3/4] Generating Webots world from $Scenario.yaml..." -ForegroundColor Green
python "$PSScriptRoot\scene\generator\generate_world.py" "$scenarioPath" --out "$PSScriptRoot\scene\webots\worlds" --name current --extern
if ($LASTEXITCODE -ne 0) {
    Write-Error "Failed to generate Webots world."
    exit 1
}

$worldPath = Join-Path $PSScriptRoot "scene\webots\worlds\current.wbt"

# 4. Stop any stale containers / processes
Write-Host "[4/4] Starting Native Webots (RTX GPU) and Docker controllers..." -ForegroundColor Green
docker compose -f "$PSScriptRoot\docker-compose.native.yaml" down --remove-orphans 2>$null
docker compose -f "$PSScriptRoot\docker-compose.yaml" down --remove-orphans 2>$null
docker rm -f vc_solution vc_scene vc_controllers 2>$null
Get-Process -Name "*webots*" -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue

# Launch Native Webots on Windows (Hardware DirectX / OpenGL on GPU)
$webotsProc = Start-Process -FilePath $WebotsPath -ArgumentList @("--batch", "--port=1234", "--mode=$Mode", $worldPath) -PassThru

try {
    # Give Webots a moment to initialize port 1234
    Start-Sleep -Seconds 3

    # Launch Docker services: vc_controllers + vc_solution
    docker compose -f "$PSScriptRoot\docker-compose.native.yaml" up
}
finally {
    Write-Host "`nStopping simulation..." -ForegroundColor Yellow
    docker compose -f "$PSScriptRoot\docker-compose.native.yaml" down --remove-orphans 2>$null
    docker rm -f vc_solution vc_scene vc_controllers 2>$null
    Get-Process -Name "*webots*" -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
    Write-Host "Simulation stopped successfully." -ForegroundColor Green
}

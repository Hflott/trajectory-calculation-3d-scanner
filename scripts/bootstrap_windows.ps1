Param(
    [switch]$InstallApps
)

$ErrorActionPreference = "Stop"

function Write-Step($msg) {
    Write-Host "`n==> $msg" -ForegroundColor Cyan
}

Write-Step "Checking Windows prerequisites"

if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
    throw "WSL is not available on this machine."
}

$wslVersion = & wsl --status 2>$null
Write-Host $wslVersion

if (-not (Get-Command winget.exe -ErrorAction SilentlyContinue)) {
    Write-Warning "winget is not installed. Install apps manually if needed."
} elseif ($InstallApps) {
    Write-Step "Installing host apps with winget"
    winget install --id Git.Git -e --accept-package-agreements --accept-source-agreements
    winget install --id Docker.DockerDesktop -e --accept-package-agreements --accept-source-agreements
    winget install --id Microsoft.VisualStudioCode -e --accept-package-agreements --accept-source-agreements
}

Write-Step "Ensuring WSL2 Ubuntu is installed"
try {
    & wsl -d Ubuntu -- echo "Ubuntu already installed" | Out-Null
} catch {
    Write-Host "Installing Ubuntu in WSL (you may need to restart and run this script again)..."
    wsl --install -d Ubuntu
}

Write-Step "Next steps"
Write-Host "1. Start Docker Desktop and ensure WSL integration is enabled for Ubuntu."
Write-Host "2. Open this repo in VS Code."
Write-Host "3. Reopen in Dev Container."
Write-Host "4. In container terminal run:"
Write-Host "   ./scripts/devcontainer_setup.sh"
Write-Host "   ./scripts/run_mock_gui.sh"

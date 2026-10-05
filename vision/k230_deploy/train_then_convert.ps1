[CmdletBinding()]
param(
    # Existing training script. Relative paths inside it are resolved from ProjectRoot.
    [string]$TrainScript = "C:\yolo_all\train.py",
    [string]$ProjectRoot = "C:\yolo_all",

    # Use a full python.exe path here if "python" is not the training environment.
    [string]$PythonExe = "python",
    [string[]]$TrainArguments = @(),

    [string]$RunsRoot = "C:\yolo_all\runs\detect",
    [string]$Dataset = "C:\yolo_all\dataset",
    [string]$OutputDir = "C:\yolo_all\k230_export",
    [ValidatePattern('^\d+(,\d+)*$')]
    [string]$Sizes = "320",
    [ValidateRange(1, 10000)]
    [int]$CalibrationSamples = 100,
    [ValidateRange(1, 10)]
    [int]$BuildAttempts = 5,
    [switch]$RebuildDockerImage,
    [switch]$OpenOutputFolder,

    # Optional: use this exact checkpoint instead of auto-detecting the newest best.pt.
    [string]$BestPtOverride = ""
)

$ErrorActionPreference = "Stop"
$convertScript = Join-Path $PSScriptRoot "convert_best_to_kmodel.ps1"

Write-Host "============================================================" -ForegroundColor DarkCyan
Write-Host " YOLO training -> automatic K230 KModel conversion" -ForegroundColor Green
Write-Host "============================================================" -ForegroundColor DarkCyan
Write-Host "Training script: $TrainScript"
Write-Host "Working folder : $ProjectRoot"
Write-Host "Python         : $PythonExe"
Write-Host "Runs folder    : $RunsRoot"

if (-not (Test-Path -LiteralPath $TrainScript -PathType Leaf)) {
    throw "Training script not found: $TrainScript"
}
if (-not (Test-Path -LiteralPath $ProjectRoot -PathType Container)) {
    throw "Project directory not found: $ProjectRoot"
}
if (-not (Test-Path -LiteralPath $convertScript -PathType Leaf)) {
    throw "Conversion script not found: $convertScript"
}

$trainingStarted = Get-Date
Push-Location $ProjectRoot
try {
    Write-Host "`n[1/3] Training started. KModel conversion will run only after training exits successfully." -ForegroundColor Cyan
    & $PythonExe $TrainScript @TrainArguments
    $trainingExitCode = $LASTEXITCODE
}
finally {
    Pop-Location
}

if ($trainingExitCode -ne 0) {
    throw "Training failed with exit code $trainingExitCode. KModel conversion was not started."
}
Write-Host "`n[2/3] Training completed successfully." -ForegroundColor Green

if (-not [string]::IsNullOrWhiteSpace($BestPtOverride)) {
    $bestPt = (Resolve-Path -LiteralPath $BestPtOverride).Path
}
else {
    if (-not (Test-Path -LiteralPath $RunsRoot -PathType Container)) {
        throw "Runs directory not found: $RunsRoot"
    }
    $allBest = Get-ChildItem -LiteralPath $RunsRoot -Recurse -File -Filter best.pt |
        Sort-Object LastWriteTime -Descending
    if (-not $allBest) {
        throw "No best.pt was found under: $RunsRoot"
    }

    # Prefer a checkpoint touched by this training run. If the final epochs did
    # not improve fitness, Ultralytics may leave best.pt older than the start
    # time, so fall back to the newest checkpoint with a visible warning.
    $currentRunBest = $allBest | Where-Object {
        $_.LastWriteTime -ge $trainingStarted.AddMinutes(-1)
    } | Select-Object -First 1
    if ($currentRunBest) {
        $bestPt = $currentRunBest.FullName
    }
    else {
        $bestPt = $allBest[0].FullName
        Write-Warning "No best.pt timestamp matched this run; using newest checkpoint: $bestPt"
    }
}

Write-Host "Selected checkpoint: $bestPt" -ForegroundColor Green
Write-Host "`n[3/3] Starting automatic KModel conversion." -ForegroundColor Cyan

$convertArguments = @{
    BestPt             = $bestPt
    Dataset            = $Dataset
    OutputDir          = $OutputDir
    Sizes              = $Sizes
    CalibrationSamples = $CalibrationSamples
    BuildAttempts      = $BuildAttempts
}
if ($RebuildDockerImage) {
    $convertArguments.RebuildDockerImage = $true
}
if ($OpenOutputFolder) {
    $convertArguments.OpenOutputFolder = $true
}

& $convertScript @convertArguments
if ($LASTEXITCODE -ne 0) {
    throw "Automatic KModel conversion failed with exit code $LASTEXITCODE"
}

Write-Host "`nTraining and K230 conversion are both complete." -ForegroundColor Green
Write-Host "Output directory: $OutputDir" -ForegroundColor Green

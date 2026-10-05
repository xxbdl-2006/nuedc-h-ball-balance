[CmdletBinding()]
param(
    # Existing or newly trained Ultralytics checkpoint.
    [string]$BestPt = "C:\yolo_all\runs\detect\train3\weights\best.pt",

    # Dataset root. INT8 calibration images are read from train\images.
    [string]$Dataset = "C:\yolo_all\dataset",

    # All ONNX/KModel/JSON files are written here by default.
    [string]$OutputDir = "C:\yolo_all\k230_export",

    # One size ("320") or multiple comma-separated sizes ("320,640").
    [ValidatePattern('^\d+(,\d+)*$')]
    [string]$Sizes = "320",

    # More real images can improve INT8 calibration, but conversion is slower.
    [ValidateRange(1, 10000)]
    [int]$CalibrationSamples = 100,

    [ValidateRange(1, 10)]
    [int]$BuildAttempts = 5,

    # Use this only on a new PC or after converter/Dockerfile changes.
    [switch]$RebuildDockerImage,

    # Open the output directory after a successful conversion.
    [switch]$OpenOutputFolder,

    # Normally the bundled converter is used automatically.
    [string]$ConverterScript = ""
)

$ErrorActionPreference = "Stop"

function Write-Step([string]$Message) {
    Write-Host "`n[+] $Message" -ForegroundColor Cyan
}

if ([string]::IsNullOrWhiteSpace($ConverterScript)) {
    $ConverterScript = Join-Path $PSScriptRoot "converter\run.ps1"
}

Write-Host "============================================================" -ForegroundColor DarkCyan
Write-Host " YOLOv8 best.pt -> K230 INT8 KModel (nncase 2.11.0)" -ForegroundColor Green
Write-Host "============================================================" -ForegroundColor DarkCyan
Write-Host "Checkpoint : $BestPt"
Write-Host "Dataset    : $Dataset"
Write-Host "Output     : $OutputDir"
Write-Host "Sizes      : $Sizes"
Write-Host "Calibration: $CalibrationSamples images"
Write-Host "Docker     : $(if ($RebuildDockerImage) { 'build/update image' } else { 'reuse existing image' })"

if (-not (Test-Path -LiteralPath $BestPt -PathType Leaf)) {
    throw "best.pt not found: $BestPt"
}
if (-not (Test-Path -LiteralPath (Join-Path $Dataset "train\images") -PathType Container)) {
    throw "Calibration directory not found: $(Join-Path $Dataset 'train\images')"
}
if (-not (Test-Path -LiteralPath $ConverterScript -PathType Leaf)) {
    throw "Bundled converter not found: $ConverterScript"
}

New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null

$arguments = @{
    BestPt            = $BestPt
    Dataset           = $Dataset
    OutputDir         = $OutputDir
    Sizes             = $Sizes
    CalibrationSamples = $CalibrationSamples
    BuildAttempts     = $BuildAttempts
}
if (-not $RebuildDockerImage) {
    $arguments.SkipBuild = $true
}

Write-Step "Starting Docker conversion"
try {
    & $ConverterScript @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Converter returned exit code $LASTEXITCODE"
    }
}
catch {
    Write-Host "`n[FAILED] KModel conversion did not finish." -ForegroundColor Red
    if (-not $RebuildDockerImage) {
        Write-Host "If Docker says the image does not exist, rerun with:" -ForegroundColor Yellow
        Write-Host "  .\convert_best_to_kmodel.ps1 -RebuildDockerImage" -ForegroundColor Yellow
    }
    throw
}

Write-Step "Verifying generated files"
$sizeList = $Sizes -split ','
foreach ($size in $sizeList) {
    $onnx = Join-Path $OutputDir "yolov8n_${size}.onnx"
    $kmodel = Join-Path $OutputDir "yolov8n_${size}_int8.kmodel"
    $metadata = Join-Path $OutputDir "yolov8n_${size}.json"
    foreach ($path in @($onnx, $kmodel, $metadata)) {
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
            throw "Expected output was not generated: $path"
        }
    }
    $kmodelInfo = Get-Item -LiteralPath $kmodel
    $hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $kmodel).Hash
    Write-Host "[OK] $($kmodelInfo.FullName)" -ForegroundColor Green
    Write-Host "     Bytes : $($kmodelInfo.Length)"
    Write-Host "     SHA256: $hash"
}

Write-Host "`nConversion completed successfully." -ForegroundColor Green
Write-Host "Copy the generated *_int8.kmodel to: CanMV\sdcard\kmodel" -ForegroundColor Green
Write-Host "The current 320 model is loaded on K230 as:" -ForegroundColor Green
Write-Host "  /sdcard/kmodel/yolov8n_320_int8.kmodel" -ForegroundColor Green

if ($OpenOutputFolder) {
    Start-Process explorer.exe -ArgumentList $OutputDir
}

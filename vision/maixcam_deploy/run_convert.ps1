param(
    [ValidateRange(1, 10000)]
    [int]$CalibrationCount = 100
)

$ErrorActionPreference = "Stop"

$workspace = (Resolve-Path -LiteralPath $PSScriptRoot).Path
$image = "sophgo/tpuc_dev:latest"

if (-not (Test-Path -LiteralPath (Join-Path $workspace "export.onnx"))) {
    throw "export.onnx was not found in: $workspace"
}

if (-not (Test-Path -LiteralPath (Join-Path $workspace "test.jpg"))) {
    throw "test.jpg was not found in: $workspace"
}

docker run --privileged --rm `
    -e "CALIBRATION_COUNT=$CalibrationCount" `
    -v "${workspace}:/workspace" `
    -w /workspace `
    $image `
    bash /workspace/convert.sh

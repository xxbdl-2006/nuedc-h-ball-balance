#!/usr/bin/env bash
set -euo pipefail

# MaixCAM-Pro (CV181x) YOLO11n INT8 conversion settings.
# Environment variables may override these defaults when needed.
MODEL_NAME="${MODEL_NAME:-steel_ball_yolo11n}"
ONNX_MODEL="${ONNX_MODEL:-./export.onnx}"
TEST_IMAGE="${TEST_IMAGE:-./test.jpg}"
CALIBRATION_DIR="${CALIBRATION_DIR:-./images}"
CALIBRATION_COUNT="${CALIBRATION_COUNT:-100}"
INPUT_SIZE="${INPUT_SIZE:-640}"
LABELS="${LABELS:-steel ball}"

# These output names match the Ultralytics YOLO11n ONNX detection head.
OUTPUT_BOX="${OUTPUT_BOX:-/model.23/dfl/conv/Conv_output_0}"
OUTPUT_CLS="${OUTPUT_CLS:-/model.23/Sigmoid_output_0}"

if [[ ! -f "${ONNX_MODEL}" ]]; then
  echo "ERROR: ONNX model not found: ${ONNX_MODEL}" >&2
  exit 1
fi

if [[ ! -f "${TEST_IMAGE}" ]]; then
  echo "ERROR: test image not found: ${TEST_IMAGE}" >&2
  exit 1
fi

if [[ ! -d "${CALIBRATION_DIR}" ]]; then
  echo "ERROR: calibration directory not found: ${CALIBRATION_DIR}" >&2
  exit 1
fi

if [[ ! "${CALIBRATION_COUNT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: CALIBRATION_COUNT must be a positive integer." >&2
  exit 1
fi

AVAILABLE_CALIBRATION_IMAGES=$(find "${CALIBRATION_DIR}" -maxdepth 1 -type f \
  \( -iname '*.jpg' -o -iname '*.jpeg' -o -iname '*.png' -o -iname '*.bmp' \) \
  | wc -l | tr -d '[:space:]')

if (( AVAILABLE_CALIBRATION_IMAGES == 0 )); then
  echo "ERROR: no calibration images found in ${CALIBRATION_DIR}" >&2
  exit 1
fi

EFFECTIVE_CALIBRATION_COUNT="${CALIBRATION_COUNT}"
if (( AVAILABLE_CALIBRATION_IMAGES < CALIBRATION_COUNT )); then
  EFFECTIVE_CALIBRATION_COUNT="${AVAILABLE_CALIBRATION_IMAGES}"
  echo "WARNING: requested ${CALIBRATION_COUNT} calibration images, but only ${AVAILABLE_CALIBRATION_IMAGES} are available." >&2
  echo "WARNING: using all available images; 100-200 MaixCAM-Pro photos are recommended." >&2
fi

echo "Model: ${ONNX_MODEL}"
echo "Input: ${INPUT_SIZE}x${INPUT_SIZE} RGB"
echo "Calibration images: ${EFFECTIVE_CALIBRATION_COUNT}/${AVAILABLE_CALIBRATION_IMAGES}"

python3 -m pip install -U tpu_mlir

model_transform.py \
  --model_name "${MODEL_NAME}" \
  --model_def "${ONNX_MODEL}" \
  --input_shapes "[[1,3,${INPUT_SIZE},${INPUT_SIZE}]]" \
  --mean '0,0,0' \
  --scale '0.00392156862745098,0.00392156862745098,0.00392156862745098' \
  --keep_aspect_ratio \
  --pixel_format rgb \
  --channel_format nchw \
  --output_names "${OUTPUT_BOX},${OUTPUT_CLS}" \
  --test_input "${TEST_IMAGE}" \
  --test_result "${MODEL_NAME}_top_outputs.npz" \
  --tolerance 0.99,0.99 \
  --mlir "${MODEL_NAME}.mlir"

run_calibration.py "${MODEL_NAME}.mlir" \
  --dataset "${CALIBRATION_DIR}" \
  --input_num "${EFFECTIVE_CALIBRATION_COUNT}" \
  -o "${MODEL_NAME}_cali_table"

model_deploy.py \
  --mlir "${MODEL_NAME}.mlir" \
  --quantize INT8 \
  --quant_input \
  --calibration_table "${MODEL_NAME}_cali_table" \
  --processor cv181x \
  --test_input "${MODEL_NAME}_in_f32.npz" \
  --test_reference "${MODEL_NAME}_top_outputs.npz" \
  --tolerance 0.9,0.6 \
  --model "${MODEL_NAME}_int8.cvimodel"

cat > "${MODEL_NAME}.mud" <<EOF
[basic]
type = cvimodel
model = ${MODEL_NAME}_int8.cvimodel

[extra]
model_type = yolo11
input_type = rgb
mean = 0, 0, 0
scale = 0.00392156862745098, 0.00392156862745098, 0.00392156862745098
labels = ${LABELS}
EOF

echo "Conversion complete:"
echo "  ${MODEL_NAME}_int8.cvimodel"
echo "  ${MODEL_NAME}.mud"

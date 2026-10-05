"""Train YOLO11n on the merged steel-ball dataset for MaixCAM-Pro."""

from pathlib import Path

from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parent

# Model and dataset
MODEL = PROJECT_ROOT / "yolo11n.pt"
DATA = PROJECT_ROOT / "steel_ball_yolo_dataset" / "data.yaml"

# Core training settings
EPOCHS = 300
BATCH = 16
IMGSZ = 640
DEVICE = 0
WORKERS = 4
PATIENCE = 80

# Optimizer and learning rate
OPTIMIZER = "AdamW"
LR0 = 0.001
LRF = 0.01
WEIGHT_DECAY = 0.0005

# Data augmentation for lighting and distance changes
HSV_H = 0.015
HSV_S = 0.4
HSV_V = 0.5
MOSAIC = 0.3
CLOSE_MOSAIC = 20
FLIPLR = 0.5
DEGREES = 0.0
TRANSLATE = 0.1
SCALE = 0.5


def main() -> None:
    model = YOLO(str(MODEL))
    model.train(
        data=str(DATA),
        epochs=EPOCHS,
        batch=BATCH,
        imgsz=IMGSZ,
        device=DEVICE,
        workers=WORKERS,
        patience=PATIENCE,
        optimizer=OPTIMIZER,
        lr0=LR0,
        lrf=LRF,
        weight_decay=WEIGHT_DECAY,
        cos_lr=True,
        hsv_h=HSV_H,
        hsv_s=HSV_S,
        hsv_v=HSV_V,
        mosaic=MOSAIC,
        close_mosaic=CLOSE_MOSAIC,
        fliplr=FLIPLR,
        flipud=0.0,
        degrees=DEGREES,
        translate=TRANSLATE,
        scale=SCALE,
        mixup=0.0,
        amp=True,
        seed=42,
        deterministic=True,
        pretrained=True,
        val=True,
        plots=True,
        project=str(PROJECT_ROOT / "runs" / "detect"),
        name="steel_ball_ti_merged",
        exist_ok=False,
    )


if __name__ == "__main__":
    main()

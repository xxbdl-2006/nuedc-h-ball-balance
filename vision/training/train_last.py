"""使用已有 last.pt 在更新后的钢球数据集上继续微调。"""

from pathlib import Path

from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parent

# 从已有训练权重开始，但创建新的训练任务，以便读取更新后的数据集。
MODEL = PROJECT_ROOT / "runs" / "detect" / "train4" / "weights" / "last.pt"
DATA = PROJECT_ROOT / "steel_ball_yolo_dataset" / "data.yaml"

EPOCHS = 200
BATCH = 16
IMGSZ = 640
DEVICE = 0
WORKERS = 4
PATIENCE = 30


def main() -> None:
    if not MODEL.is_file():
        raise FileNotFoundError(f"找不到初始权重：{MODEL}")
    if not DATA.is_file():
        raise FileNotFoundError(f"找不到数据集配置：{DATA}")

    model = YOLO(str(MODEL))
    model.train(
        data=str(DATA),
        resume=False,
        epochs=EPOCHS,
        batch=BATCH,
        imgsz=IMGSZ,
        device=DEVICE,
        workers=WORKERS,
        patience=PATIENCE,
        optimizer="AdamW",
        lr0=0.001,
        lrf=0.01,
        weight_decay=0.0005,
        cos_lr=True,
        mosaic=0.3,
        close_mosaic=20,
        fliplr=0.5,
        flipud=0.0,
        mixup=0.0,
        amp=True,
        seed=42,
        deterministic=True,
        pretrained=True,
        val=True,
        plots=True,
        project=str(PROJECT_ROOT / "runs" / "detect"),
        name="steel_ball_updated_finetune",
        exist_ok=False,
    )


if __name__ == "__main__":
    main()

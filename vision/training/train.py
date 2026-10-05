"""YOLO 训练入口。

负样本没有单独的训练开关：只要将不含目标的图片放在
dataset/train/images 中，并在 dataset/train/labels 中创建同名的空 .txt
文件，YOLO 就会把它作为负样本参与训练。
"""

from ultralytics import YOLO


# ===== 常用训练参数：按需要在这里修改 =====
MODEL = "yolo11n.pt"  # 从官方预训练模型开始；微调已有模型时换成 best.pt 的路径
DATA = "data.yaml"
EPOCHS = 200           # 训练轮数
BATCH = 16             # 批大小；显存不足依次改为 8、4
IMGSZ = 640            # 输入图片尺寸
DEVICE = 0             # GPU 编号；没有 NVIDIA GPU 时使用 "cpu"
WORKERS = 4            # Windows 建议 0～4；若数据加载报错，改为 0
PATIENCE = 30          # 验证指标连续多少轮没有提升时早停
OPTIMIZER = "AdamW"    # "AdamW"、"SGD" 或 "auto"
LR0 = 0.001            # 初始学习率
LRF = 0.01             # 最终学习率相对 LR0 的比例
WEIGHT_DECAY = 0.0005  # 权重衰减，过拟合时可适当增大

# ===== 数据增强参数 =====
MOSAIC = 1.0           # 马赛克增强概率；最后 CLOSE_MOSAIC 轮自动关闭
CLOSE_MOSAIC = 10
FLIPLR = 0.5           # 水平翻转概率
DEGREES = 0.0          # 随机旋转角度
TRANSLATE = 0.1        # 随机平移比例
SCALE = 0.5            # 随机缩放幅度


if __name__ == "__main__":
    model = YOLO(MODEL)
    model.train(
        data=DATA,
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
        mosaic=MOSAIC,
        close_mosaic=CLOSE_MOSAIC,
        fliplr=FLIPLR,
        degrees=DEGREES,
        translate=TRANSLATE,
        scale=SCALE,
        project="runs/detect",
        name="steel_ball_tune",
        exist_ok=False,
    )

# 2026 全国大学生电子设计竞赛 H 题 —— 车载平衡滚球运动控制系统

> 小车沿黑线循迹行驶的同时，控制钢球在摆杆上稳定于指定位置。
> 系统由 **MSPM0G3507 主控** + **MaixCAM Pro / K230 边缘视觉** 两级组成，
> 视觉在边缘端跑 YOLO 检测钢球位置，主控完成循迹、滚球平衡与任务状态机调度。

本仓库收录该题目的**下位机固件（TI MSPM0G3507）**与**视觉侧完整工程**
（数据集、YOLO 训练脚本、K230 / MaixCAM 量化部署产物）。

---

## 系统架构

```
┌──────────────────────┐        UART 自定义帧协议        ┌────────────────────────┐
│  MaixCAM Pro / K230  │ ──────────────────────────────► │   TI MSPM0G3507        │
│  边缘视觉             │   (钢球像素坐标 / 置信度)      │   主控                  │
│  ├─ YOLO11n INT8      │                                │   ├─ 8 路红外循迹       │
│  ├─ 钢球检测          │                                │   ├─ 差速速度环 PID     │
│  └─ 三点标定 → 坐标   │                                │   ├─ 滚球串级 PID       │
└──────────────────────┘                                │   ├─ BNO08X 前馈        │
                                                        │   └─ 任务状态机         │
                                                        └───────┬────────────────┘
                                                    TB6612 电机 / X42S 闭环步进
```

---

## 一、下位机固件（MSPM0G3507）

TI MSPM0G3507，CCS 工程，目录结构与外设覆盖见 [`BOARD_TEST_README.md`](BOARD_TEST_README.md)。

### BSP 驱动层（`Drivers/BSP/`）

| 模块 | 文件 | 职责 |
| --- | --- | --- |
| 视觉通信 | `k230.c/h` | 接收 MaixCAM/K230 的钢球坐标帧，解析为目标位置 |
| PID 控制 | `pid.c` `pid_control.c` | 通用 PID + 三闭环控制结构 |
| 循迹 | `redlight.c` | 8 路红外光电阵列，加权质心法提取黑线中心 |
| 电机 | `motor.c` `encoder.c` | TB6612 驱动 + 正交编码器测速 |
| 姿态 | `bsp_hc05.c` `wit.h` | BNO08X（I²C/SHTP）与 JY61P/WIT（UART）姿态源 |
| 舵机 | `bsp_sg90.c` | 摆杆角度执行机构 |
| 人机交互 | `oled_display.c` `key.c` `buzzer.c` | 5 页 OLED 调试界面 + 按键状态机 |

### 外设与引脚

- OLED：PA1 SCL / PA0 SDA
- **UART1 K230 / MaixCAM：PA8 TX / PA9 RX**
- UART2 WIT / JY61P：PB15 TX / PB16 RX，115200
- BNO08X / BNO085 I²C：PB2 SCL / PB3 SDA
- 灰度传感器：PB19、PB17、PA16、PA14、PB20、PB25、PA25、PA27
- 编码器：左 PB26/PA13，右 PB5/PB4
- 按键：PB23（KEY1）/ PB27（KEY2），蜂鸣器 PA29

### 调试能力

- 蓝牙（JDY31，UART0）支持 `!` 结尾的 ASCII 调参命令，可在线整定左右轮
  速度环的 Kp/Ki/Kd 与目标值（单位：编码器计数 / 50 ms），并支持 5 Hz 状态流。
- 可向 FireWater 输出 20 Hz 波形，用于三 PID 环路的在线观测。

---

## 二、视觉侧（`vision/`）

### 数据集（`dataset/`）

自己采集并标注的钢球检测数据集，YOLO 格式（class `0 = steelball`）：

| 划分 | 图片 | 标注 |
| --- | --- | --- |
| train | 268 | 268 |
| valid | 42 | 42 |
| test | 35 | 35 |
| **合计** | **345** | **345** |

### 训练（`vision/training/`）

- `Ti_train.py` —— 主训练脚本，YOLO11n / 640×640 / 300 epochs / AdamW /
  余弦退火，针对赛场光照与距离变化做了 HSV 增强（h=0.015, s=0.4, v=0.5）、
  mosaic=0.3、scale=0.5。
- `train_last.py` —— 从已有权重在更新后的数据集上继续微调。
- `data_split.py` / `convert_ndjson_to_yolo.py` —— 标注格式转换与数据集划分。
- `export_maixcam_320.py` —— 导出 320×320 的 MaixCAM 部署模型。

### 部署

- **K230（`vision/k230_deploy/`）**：`nncase` 量化为 INT8 `.kmodel`，
  配套 `canmv_yolov8_lcd.py` 板载 LCD 实时检测程序，使用说明见同目录
  `使用说明_K230_YOLOv8.md`。
- **MaixCAM Pro（`vision/maixcam_deploy/`）**：量化为 `.cvimodel` + `.mud` 配置，
  即最终上场方案。

### 精度与性能

- 相对 YOLOv5 基线：识别精度提升的同时降低计算开销，**运行帧率提升约 30%**。
- 通过**三点标定**完成像素坐标到实际物理位置的换算。
- 主控侧成绩：定点停车平均偏差 **1.20 cm**（最大 1.63 cm）；
  动态平衡任务球位最大误差 **0.89 cm**。

---

## 复现步骤

```bash
# 1. 环境
pip install ultralytics

# 2. 训练（需先放置 yolo11n.pt 权重）
cd vision/training
python Ti_train.py

# 3. 导出 MaixCAM 部署模型
python export_maixcam_320.py

# 4. 下位机：用 Code Composer Studio 打开工程根目录的 CCS 工程并烧录
```

---

## 技术栈

`TI MSPM0G3507` · `MaixCAM Pro` · `K230` · `YOLO11n` · `INT8 量化 (nncase / cvimodel)` ·
`TB6612` · `X42S 闭环步进` · `BNO08X` · `串级 PID` · `加权质心法` · `UART/SPI/I²C` ·
`多任务状态机` · `Code Composer Studio`

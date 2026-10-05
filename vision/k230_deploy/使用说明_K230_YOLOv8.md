# 亚博 K230 YOLOv8 使用说明

本目录对应以下环境：

- 开发板：亚博智能 K230，使用板载 LCD。
- 固件：`CanMV_K230_YAHBOOM_micropython_v1.8-0-gc2d1f5c_nncase_v2.11.0.img.gz`。
- 模型：YOLOv8n，320×320，10 类检测。
- 编译器：`nncase 2.11.0`、`nncase-kpu 2.11.0`。
- KModel 输入：`uint8 RGB NCHW [1,3,320,320]`，模型内部执行 `/255`。
- KModel 输出：`[1,14,2100]`，其中 4 个框坐标加 10 个类别概率。

## 一、目录里的文件

```text
yolov8n_320_int8.kmodel       K230 最终模型
yolov8n_320.onnx              中间 ONNX 模型
yolov8n_320.json              模型输入输出和量化信息
canmv_yolov8_lcd.py           亚博板载 LCD 实时检测程序
convert_best_to_kmodel.ps1    单独转换已有 best.pt
train_then_convert.ps1        训练成功后自动转换最新 best.pt
converter\                    完整 Docker 转换器
```

## 二、把模型放到 K230

你现在放置的位置是正确的：

```text
Windows 中看到：此电脑\CanMV\sdcard\kmodel\yolov8n_320_int8.kmodel
CanMV 程序中看到：/sdcard/kmodel/yolov8n_320_int8.kmodel
```

文件名必须和程序顶部的 `MODEL_PATH` 一致。不要只复制 `.py` 而漏掉 `.kmodel`。

## 三、第一次运行板载 LCD 程序

1. 用 USB 连接 K230，打开 CanMV IDE。
2. 确认固件是带 `nncase_v2.11.0` 的亚博版本。
3. 在 CanMV IDE 打开 `canmv_yolov8_lcd.py`。
4. 连接开发板后点击“运行”。
5. 正常情况下，板载 ST7701 LCD 会显示 640×480 摄像头画面、检测框、英文类别和置信度。
6. IDE 串口区每 30 帧打印一次 FPS 和检测结果。

建议先在 IDE 中连续测试，再决定是否开机自动运行。测试稳定后，将脚本复制成：

```text
/sdcard/main.py
```

K230 上电后就会自动执行。调试阶段不要急着覆盖现有 `main.py`，避免脚本异常后反复自动启动。

## 四、可以修改的识别参数

编辑 `canmv_yolov8_lcd.py` 顶部“User parameters”区域：

```python
CONFIDENCE_THRESHOLD = 0.35
NMS_THRESHOLD = 0.45
MAX_NMS_CANDIDATES = 200
MAX_DETECTIONS = 30
PRINT_EVERY_N_FRAMES = 30
DEBUG_TIMING = 0
```

- 漏检较多：把置信度从 `0.35` 降到 `0.25`，但误检会增多。
- 重叠框较多：把 NMS 阈值从 `0.45` 降到 `0.35`。
- 后处理太慢：降低 `MAX_NMS_CANDIDATES` 或提高置信度。
- 需要详细耗时：把 `DEBUG_TIMING` 改成 `1`。
- 串口输出影响速度：把 `PRINT_EVERY_N_FRAMES` 改成 `0`。

以下值与当前 KModel 编译参数绑定，不应随意更改：

```python
MODEL_INPUT_SIZE = [320, 320]
LETTERBOX_VALUE = 114
CLASSES = [
    "cat", "chicken", "cow", "dog", "fox",
    "goat", "horse", "person", "racoon", "skunk"
]
```

类别顺序必须和训练数据、`yolov8n_320.json` 完全一致。

## 五、单独转换已有 best.pt

先启动 Docker Desktop，等到 Docker Engine 正常运行。然后打开 PowerShell：

```powershell
cd C:\yolo_all\k230_export
Set-ExecutionPolicy -Scope Process Bypass
.\convert_best_to_kmodel.ps1
```

默认参数是：

```text
best.pt     C:\yolo_all\runs\detect\train3\weights\best.pt
数据集      C:\yolo_all\dataset
输出目录    C:\yolo_all\k230_export
输入尺寸    320
校准图片数  100
```

临时修改参数不需要编辑文件，例如：

```powershell
.\convert_best_to_kmodel.ps1 `
  -BestPt "C:\yolo_all\runs\detect\train4\weights\best.pt" `
  -Dataset "C:\yolo_all\dataset" `
  -OutputDir "C:\yolo_all\k230_export" `
  -Sizes 320 `
  -CalibrationSamples 150 `
  -OpenOutputFolder
```

当前电脑已经有构建成功的 Docker 镜像，所以默认直接复用，速度最快。只有以下情况才加 `-RebuildDockerImage`：

- 换了一台电脑。
- 手动删除了 Docker 镜像。
- 修改了 `converter\Dockerfile` 或转换器 Python 文件。

```powershell
.\convert_best_to_kmodel.ps1 -RebuildDockerImage
```

不要为了换一个 `best.pt` 重建 Docker 镜像；权重是运行时挂载进去的。

## 六、训练完成后自动转换

当前训练脚本是：

```text
C:\yolo_all\train.py
```

正确的自动化方式是让训练进程正常结束后再转换。不要在 `best.pt` 第一次出现时马上转换，因为 Ultralytics 会在训练过程中多次更新它。

运行：

```powershell
cd C:\yolo_all\k230_export
Set-ExecutionPolicy -Scope Process Bypass
.\train_then_convert.ps1
```

该脚本会：

```text
在 C:\yolo_all 中运行 train.py
→ 只有训练退出码为 0 才继续
→ 在 C:\yolo_all\runs\detect 中寻找最新 best.pt
→ 调用 Docker 转换器
→ 把 ONNX、JSON、KModel 写入 C:\yolo_all\k230_export
→ 检查 KModel 文件并打印 SHA256
```

修改训练 Python 环境、尺寸或校准数量：

```powershell
.\train_then_convert.ps1 `
  -PythonExe "C:\path\to\python.exe" `
  -TrainScript "C:\yolo_all\train.py" `
  -Sizes 320 `
  -CalibrationSamples 150 `
  -OpenOutputFolder
```

如果训练脚本把权重保存到自定义位置，可以明确指定：

```powershell
.\train_then_convert.ps1 `
  -BestPtOverride "C:\my_runs\weights\best.pt"
```

注意：`BestPtOverride` 仍然是在训练成功退出后才使用，不会绕过训练失败检查。

## 七、转换成功时应该看到

```text
KMODEL OK: /output/yolov8n_320_int8.kmodel (... bytes)
All requested models were generated under /output
Conversion finished: C:\yolo_all\k230_export
```

随后目录里应有：

```text
yolov8n_320.onnx
yolov8n_320.json
yolov8n_320_int8.kmodel
```

## 八、常见问题

### 1. `kmodel not found`

确认 Windows 资源管理器中存在：

```text
此电脑\CanMV\sdcard\kmodel\yolov8n_320_int8.kmodel
```

并确认扩展名不是隐藏后形成的 `.kmodel.kmodel`。

### 2. KModel 加载时报版本错误

确认开发板固件文件名包含：

```text
nncase_v2.11.0
```

不同 nncase 主/次版本生成的 KModel 不建议混用。

### 3. 有画面但完全没有框

先把：

```python
CONFIDENCE_THRESHOLD = 0.35
```

改成：

```python
CONFIDENCE_THRESHOLD = 0.20
```

同时观察 IDE 是否打印输出形状错误。不要在 CanMV 程序中额外除以 255；归一化已经编译进 KModel。

### 4. 框的位置上下偏移

不要删除 letterbox。640×480 输入会等比例缩到 320×240，并在 320×320 模型画面上下各填充 40 像素。程序会在后处理中撤销这 40 像素。

### 5. 画面正常但类别不对

检查 `CLASSES` 顺序，必须与 JSON 中的顺序一致。不要把 `racoon` 自行改成另一位置；可以改显示拼写，但不能改变列表顺序。

### 6. FPS 较低

先保持 320 模型，关闭 `DEBUG_TIMING` 和逐帧打印，提高置信度并降低 `MAX_NMS_CANDIDATES`。640 模型通常更适合追求小目标精度，而不适合优先追求实时帧率。

## 九、当前 KModel 校验值

```text
文件：yolov8n_320_int8.kmodel
字节：3358648
SHA256：4FF711C008D837489B77554EA104ACE97EFC742168FA1F32EBD2B07C733A0CED
```

复制到 SD 卡后若怀疑文件损坏，可以重新从电脑端复制该文件。

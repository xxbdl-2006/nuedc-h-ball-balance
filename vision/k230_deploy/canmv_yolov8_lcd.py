"""
Yahboom K230 + CanMV + built-in ST7701 LCD
YOLOv8 detection demo for yolov8n_320_int8.kmodel.

Model input : uint8 RGB NCHW [1, 3, 320, 320]
Model output: float [1, 14, 2100] = 4 boxes + 10 class scores

Copy the model to:
    /sdcard/kmodel/yolov8n_320_int8.kmodel

Open this file in CanMV IDE and run it. After it works, rename/copy it to
/sdcard/main.py only if you want it to start automatically on power-up.
"""

from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import os
import time
import gc


# ========================= User parameters =========================
MODEL_PATH = "/sdcard/kmodel/yolov8n_320_int8.kmodel"

MODEL_INPUT_SIZE = [320, 320]       # [width, height]; must match the kmodel
CAMERA_SIZE = [640, 480]            # RGB888P input supplied by Yahboom PipeLine
DISPLAY_SIZE = [640, 480]           # Yahboom built-in LCD
DISPLAY_MODE = "lcd"                # Do not change to hdmi for the built-in LCD

CONFIDENCE_THRESHOLD = 0.35         # Lower it to 0.25 if detections are missed
NMS_THRESHOLD = 0.45
CLASS_AWARE_NMS = True
MAX_NMS_CANDIDATES = 200             # Limits MicroPython NMS workload
MAX_DETECTIONS = 30

LETTERBOX_VALUE = 114                # Must match the Docker conversion metadata
LABEL_FONT_SIZE = 20
PRINT_EVERY_N_FRAMES = 30            # 0 disables periodic console output
DEBUG_TIMING = 0                     # 1 prints AIBase/Ai2d timing details

CLASSES = [
    "cat",
    "chicken",
    "cow",
    "dog",
    "fox",
    "goat",
    "horse",
    "person",
    "racoon",
    "skunk",
]

# RGBA colors used by the OSD layer.
COLORS = [
    (255, 255, 64, 64),
    (255, 64, 255, 64),
    (255, 64, 128, 255),
    (255, 255, 192, 64),
    (255, 192, 64, 255),
    (255, 64, 255, 255),
    (255, 255, 128, 192),
    (255, 128, 255, 192),
    (255, 192, 192, 255),
    (255, 255, 255, 255),
]
# ==================================================================


def file_exists(path):
    try:
        os.stat(path)
        return True
    except Exception:
        return False


def clip(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def box_iou(a, b):
    """IoU for [x1, y1, x2, y2, score, class_id]."""
    inter_x1 = max(a[0], b[0])
    inter_y1 = max(a[1], b[1])
    inter_x2 = min(a[2], b[2])
    inter_y2 = min(a[3], b[3])
    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - inter_area
    if union <= 0.0:
        return 0.0
    return inter_area / union


def nms(candidates):
    """Confidence-sorted, optionally class-aware NMS."""
    candidates.sort(key=lambda det: det[4], reverse=True)
    if len(candidates) > MAX_NMS_CANDIDATES:
        candidates = candidates[:MAX_NMS_CANDIDATES]

    kept = []
    for det in candidates:
        suppressed = False
        for selected in kept:
            if CLASS_AWARE_NMS and det[5] != selected[5]:
                continue
            if box_iou(det, selected) > NMS_THRESHOLD:
                suppressed = True
                break
        if not suppressed:
            kept.append(det)
            if len(kept) >= MAX_DETECTIONS:
                break
    return kept


class YOLOv8DetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, labels,
                 rgb888p_size, display_size, debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.labels = labels
        self.class_count = len(labels)
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

        self.scale = 1.0
        self.pad_left = 0
        self.pad_top = 0

    def get_letterbox_param(self):
        src_w, src_h = self.rgb888p_size
        dst_w, dst_h = self.model_input_size
        scale = min(dst_w / src_w, dst_h / src_h)
        new_w = int(round(src_w * scale))
        new_h = int(round(src_h * scale))
        pad_w = dst_w - new_w
        pad_h = dst_h - new_h
        left = pad_w // 2
        right = pad_w - left
        top = pad_h // 2
        bottom = pad_h - top
        return top, bottom, left, right, scale

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, scale = self.get_letterbox_param()
            self.scale = scale
            self.pad_left = left
            self.pad_top = top

            # Keep this value equal to the training/export letterbox value.
            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right],
                0,
                [LETTERBOX_VALUE, LETTERBOX_VALUE, LETTERBOX_VALUE],
            )
            self.ai2d.resize(nn.interp_method.tf_bilinear,
                             nn.interp_mode.half_pixel)
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )

    def _prediction_view(self, output):
        """Return (array, channel_first, candidate_count)."""
        pred = output
        if len(pred.shape) == 3:
            if pred.shape[0] != 1:
                raise RuntimeError("Unexpected YOLOv8 batch: {}".format(pred.shape))
            pred = pred[0]

        expected_channels = 4 + self.class_count
        if len(pred.shape) != 2:
            raise RuntimeError("Unexpected YOLOv8 output shape: {}".format(pred.shape))
        if pred.shape[0] == expected_channels:
            return pred, True, pred.shape[1]
        if pred.shape[1] == expected_channels:
            return pred, False, pred.shape[0]
        raise RuntimeError(
            "Output shape {} does not contain {} channels".format(
                pred.shape, expected_channels
            )
        )

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            pred, channel_first, candidate_count = self._prediction_view(results[0])
            src_w, src_h = self.rgb888p_size
            candidates = []

            for i in range(candidate_count):
                if channel_first:
                    cx = float(pred[0][i])
                    cy = float(pred[1][i])
                    bw = float(pred[2][i])
                    bh = float(pred[3][i])
                else:
                    cx = float(pred[i][0])
                    cy = float(pred[i][1])
                    bw = float(pred[i][2])
                    bh = float(pred[i][3])

                best_class = 0
                best_score = -1.0
                for class_id in range(self.class_count):
                    if channel_first:
                        score = float(pred[4 + class_id][i])
                    else:
                        score = float(pred[i][4 + class_id])
                    if score > best_score:
                        best_score = score
                        best_class = class_id

                if best_score < CONFIDENCE_THRESHOLD:
                    continue

                # The ONNX output uses xywh coordinates in the 320x320
                # letterboxed model image. Undo padding and scaling.
                x1 = (cx - bw * 0.5 - self.pad_left) / self.scale
                y1 = (cy - bh * 0.5 - self.pad_top) / self.scale
                x2 = (cx + bw * 0.5 - self.pad_left) / self.scale
                y2 = (cy + bh * 0.5 - self.pad_top) / self.scale

                x1 = clip(x1, 0.0, src_w - 1.0)
                y1 = clip(y1, 0.0, src_h - 1.0)
                x2 = clip(x2, 0.0, src_w - 1.0)
                y2 = clip(y2, 0.0, src_h - 1.0)
                if x2 - x1 < 2.0 or y2 - y1 < 2.0:
                    continue
                candidates.append([x1, y1, x2, y2, best_score, best_class])

            return nms(candidates)

    def draw_result(self, pipeline, detections):
        pipeline.osd_img.clear()
        src_w, src_h = self.rgb888p_size
        disp_w, disp_h = self.display_size

        for det in detections:
            x1 = int(det[0] * disp_w / src_w)
            y1 = int(det[1] * disp_h / src_h)
            x2 = int(det[2] * disp_w / src_w)
            y2 = int(det[3] * disp_h / src_h)
            score = det[4]
            class_id = int(det[5])
            color = COLORS[class_id % len(COLORS)]

            width = max(1, x2 - x1)
            height = max(1, y2 - y1)
            pipeline.osd_img.draw_rectangle(
                x1, y1, width, height, color=color, thickness=2
            )
            text_y = max(0, y1 - LABEL_FONT_SIZE - 2)
            label = "{} {:.2f}".format(self.labels[class_id], score)
            pipeline.osd_img.draw_string_advanced(
                x1, text_y, LABEL_FONT_SIZE, label, color=color
            )


def main():
    if not file_exists(MODEL_PATH):
        print("ERROR: kmodel not found:", MODEL_PATH)
        print("Copy yolov8n_320_int8.kmodel to CanMV/sdcard/kmodel first.")
        raise OSError(MODEL_PATH)

    print("=== Yahboom K230 YOLOv8 ===")
    print("Model:", MODEL_PATH)
    print("Camera: {}x{}  LCD: {}x{}".format(
        CAMERA_SIZE[0], CAMERA_SIZE[1], DISPLAY_SIZE[0], DISPLAY_SIZE[1]
    ))
    print("Confidence:", CONFIDENCE_THRESHOLD, "NMS:", NMS_THRESHOLD)

    pipeline = None
    detector = None
    frame_count = 0
    clock = time.clock()

    try:
        pipeline = PipeLine(
            rgb888p_size=CAMERA_SIZE,
            display_size=DISPLAY_SIZE,
            display_mode=DISPLAY_MODE,
        )
        pipeline.create()

        detector = YOLOv8DetectionApp(
            MODEL_PATH,
            model_input_size=MODEL_INPUT_SIZE,
            labels=CLASSES,
            rgb888p_size=CAMERA_SIZE,
            display_size=DISPLAY_SIZE,
            debug_mode=DEBUG_TIMING,
        )
        detector.config_preprocess()
        print("Model loaded. Detection started; press Stop in CanMV IDE to exit.")

        while True:
            os.exitpoint()
            clock.tick()
            frame = pipeline.get_frame()
            detections = detector.run(frame)
            detector.draw_result(pipeline, detections)
            pipeline.show_image()

            frame_count += 1
            if PRINT_EVERY_N_FRAMES > 0 and frame_count % PRINT_EVERY_N_FRAMES == 0:
                print("FPS: {:.1f}, detections: {}".format(
                    clock.fps(), len(detections)
                ))
                for det in detections:
                    print("  {} {:.3f} [{},{},{},{}]".format(
                        CLASSES[int(det[5])], det[4],
                        int(det[0]), int(det[1]), int(det[2]), int(det[3])
                    ))
            gc.collect()

    except KeyboardInterrupt:
        print("Stopped by user.")
    except Exception as error:
        print("YOLOv8 runtime error:", error)
        raise
    finally:
        if detector is not None:
            detector.deinit()
        if pipeline is not None:
            pipeline.destroy()
        gc.collect()
        print("Resources released.")


if __name__ == "__main__":
    main()

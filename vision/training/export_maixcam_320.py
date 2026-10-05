"""Export the latest trained steel-ball YOLO11n as a fixed 320x320 ONNX model.

Run this in the same Python environment used for Ultralytics training.  The
existing 640x640 weights and ONNX files are not overwritten.
"""

from pathlib import Path
import shutil

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent
SOURCE_WEIGHTS = (
    ROOT / "runs" / "detect" / "steel_ball_ti_merged3" / "weights" / "best.pt"
)
OUTPUT_DIR = ROOT / "maixcam_convert_320"
EXPORT_WEIGHTS = OUTPUT_DIR / "steel_ball_ti_merged3_yolo11n_320.pt"
EXPORT_ONNX = EXPORT_WEIGHTS.with_suffix(".onnx")


def main() -> None:
    if not SOURCE_WEIGHTS.exists():
        raise FileNotFoundError(SOURCE_WEIGHTS)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    if EXPORT_ONNX.exists():
        raise FileExistsError(
            f"Refusing to overwrite existing export: {EXPORT_ONNX}"
        )

    shutil.copy2(SOURCE_WEIGHTS, EXPORT_WEIGHTS)
    model = YOLO(str(EXPORT_WEIGHTS))
    exported = Path(
        model.export(
            format="onnx",
            imgsz=320,
            dynamic=False,
            simplify=True,
            half=False,
            int8=False,
        )
    )

    print("Exported:", exported.resolve())
    print("Next: convert this ONNX to a CV181X INT8 .cvimodel/.mud in MaixHub.")


if __name__ == "__main__":
    main()

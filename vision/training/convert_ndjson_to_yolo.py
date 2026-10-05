"""Convert an Ultralytics NDJSON detection dataset to YOLO folder format."""

from __future__ import annotations

import argparse
import json
import os
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import requests
import yaml
from PIL import Image


SPLIT_ALIASES = {
    "train": "train",
    "val": "val",
    "valid": "val",
    "validation": "val",
    "test": "test",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path, help="Ultralytics .ndjson file")
    parser.add_argument("output", type=Path, help="Output YOLO dataset directory")
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--retries", type=int, default=4)
    return parser.parse_args()


def load_records(source: Path) -> tuple[dict, list[dict]]:
    metadata: dict = {}
    images: list[dict] = []
    with source.open("r", encoding="utf-8-sig") as file:
        for line_number, line in enumerate(file, start=1):
            if not line.strip():
                continue
            record = json.loads(line)
            record_type = record.get("type")
            if record_type == "dataset":
                metadata = record
            elif record_type == "image":
                split = SPLIT_ALIASES.get(str(record.get("split", "")).lower())
                if split is None:
                    raise ValueError(
                        f"Unsupported split on line {line_number}: {record.get('split')!r}"
                    )
                record["_split"] = split
                images.append(record)
    if not images:
        raise ValueError("No image records found in the NDJSON file")
    return metadata, images


def image_is_valid(path: Path, width: int, height: int) -> bool:
    if not path.is_file() or path.stat().st_size == 0:
        return False
    try:
        with Image.open(path) as image:
            image.verify()
        with Image.open(path) as image:
            return image.size == (width, height)
    except Exception:
        return False


def download_image(
    record: dict, output: Path, retries: int
) -> tuple[str, bool, str | None]:
    name = Path(record["file"]).name
    split = record["_split"]
    destination = output / "images" / split / name
    width = int(record["width"])
    height = int(record["height"])

    if image_is_valid(destination, width, height):
        return name, True, None

    temporary = destination.with_suffix(destination.suffix + ".part")
    for attempt in range(1, retries + 1):
        try:
            temporary.unlink(missing_ok=True)
            with requests.get(record["url"], stream=True, timeout=(15, 180)) as response:
                response.raise_for_status()
                with temporary.open("wb") as file:
                    for chunk in response.iter_content(chunk_size=1024 * 1024):
                        if chunk:
                            file.write(chunk)
            if not image_is_valid(temporary, width, height):
                raise ValueError("downloaded file is not a valid image or has wrong dimensions")
            os.replace(temporary, destination)
            return name, False, None
        except Exception as error:
            temporary.unlink(missing_ok=True)
            if attempt == retries:
                return name, False, str(error)
            time.sleep(min(2**attempt, 10))
    return name, False, "unknown download error"


def write_labels(records: list[dict], output: Path) -> int:
    box_count = 0
    for record in records:
        name = Path(record["file"]).name
        label_path = output / "labels" / record["_split"] / f"{Path(name).stem}.txt"
        lines = []
        for box in record.get("annotations", {}).get("boxes", []):
            if len(box) != 5:
                raise ValueError(f"Invalid box in {name}: {box!r}")
            class_id, center_x, center_y, width, height = box
            if not (
                0 <= float(center_x) <= 1
                and 0 <= float(center_y) <= 1
                and 0 < float(width) <= 1
                and 0 < float(height) <= 1
            ):
                raise ValueError(f"Out-of-range box in {name}: {box!r}")
            lines.append(
                f"{int(class_id)} {float(center_x):.10g} {float(center_y):.10g} "
                f"{float(width):.10g} {float(height):.10g}"
            )
            box_count += 1
        label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
    return box_count


def write_data_yaml(metadata: dict, output: Path) -> None:
    raw_names = metadata.get("class_names", {})
    if isinstance(raw_names, dict):
        names = {int(key): value for key, value in raw_names.items()}
    elif isinstance(raw_names, list):
        names = {index: value for index, value in enumerate(raw_names)}
    else:
        raise ValueError("Dataset metadata does not contain valid class_names")

    config = {
        "path": output.resolve().as_posix(),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "names": names,
    }
    (output / "data.yaml").write_text(
        yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )


def main() -> None:
    args = parse_args()
    source = args.source.resolve()
    output = args.output.resolve()
    if not source.is_file():
        raise FileNotFoundError(source)

    metadata, records = load_records(source)
    basenames = [Path(record["file"]).name for record in records]
    if len(basenames) != len(set(basenames)):
        raise ValueError("Duplicate image basenames found in the NDJSON file")

    for split in ("train", "val", "test"):
        (output / "images" / split).mkdir(parents=True, exist_ok=True)
        (output / "labels" / split).mkdir(parents=True, exist_ok=True)

    print(f"Dataset: {metadata.get('name', source.stem)}")
    print(f"Images: {len(records)}")
    errors: list[tuple[str, str]] = []
    skipped = 0
    completed = 0
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as executor:
        futures = [
            executor.submit(download_image, record, output, args.retries)
            for record in records
        ]
        for future in as_completed(futures):
            name, was_skipped, error = future.result()
            completed += 1
            skipped += int(was_skipped)
            if error is not None:
                errors.append((name, error))
            if completed % 50 == 0 or completed == len(records):
                print(
                    f"Downloaded/checked {completed}/{len(records)} "
                    f"(existing {skipped}, errors {len(errors)})",
                    flush=True,
                )

    if errors:
        error_file = output / "download_errors.txt"
        error_file.write_text(
            "\n".join(f"{name}\t{error}" for name, error in errors) + "\n",
            encoding="utf-8",
        )
        raise RuntimeError(f"{len(errors)} images failed; see {error_file}")

    (output / "download_errors.txt").unlink(missing_ok=True)
    box_count = write_labels(records, output)
    write_data_yaml(metadata, output)
    print(f"Labels written: {len(records)} files, {box_count} boxes")
    print(f"Completed: {output}")


if __name__ == "__main__":
    main()

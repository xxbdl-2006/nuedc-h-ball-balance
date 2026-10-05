"""Create a leakage-resistant YOLO train/valid/test split.

The image names show five independent capture sequences.  Frames from the
same sequence are kept in one split so that near-identical neighbouring video
frames cannot inflate validation or test metrics.
"""

from __future__ import annotations

import csv
import re
import shutil
from collections import Counter
from pathlib import Path


SPLIT_BY_SOURCE = {
    "frame1_": "train",
    "frame3_": "train",
    "sb_": "train",
    "ball_": "valid",
    "frame2_": "test",
}
SPLIT_NAMES = ("train", "valid", "test")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def source_name(stem: str) -> str:
    """Return the non-numeric prefix used as the capture-sequence ID."""
    match = re.match(r"^(.*?)(\d+)$", stem)
    if not match:
        raise ValueError(f"Image name has no trailing frame number: {stem}")
    return match.group(1)


def validate_label(path: Path, class_count: int) -> int:
    """Validate a YOLO label and return its number of boxes."""
    box_count = 0
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_number}: expected 5 fields")
        try:
            class_id = int(fields[0])
            coordinates = [float(value) for value in fields[1:]]
        except ValueError as exc:
            raise ValueError(f"{path}:{line_number}: invalid number") from exc
        if not 0 <= class_id < class_count:
            raise ValueError(f"{path}:{line_number}: invalid class ID {class_id}")
        if any(value < 0.0 or value > 1.0 for value in coordinates):
            raise ValueError(f"{path}:{line_number}: coordinate outside [0, 1]")
        box_count += 1
    return box_count


def main() -> None:
    project_dir = Path(__file__).resolve().parent
    source_dir = project_dir / "dataset_org"
    image_dir = source_dir / "images"
    label_dir = source_dir / "labels"
    output_dir = project_dir / "dataset"

    class_file = source_dir / "classes.txt"
    classes = [
        line.strip()
        for line in class_file.read_text(encoding="utf-8-sig").splitlines()
        if line.strip()
    ]
    if not classes:
        raise ValueError(f"No class names found in {class_file}")

    images = sorted(
        path
        for path in image_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )
    labels = {path.stem: path for path in label_dir.glob("*.txt")}
    image_stems = {path.stem for path in images}
    missing_labels = sorted(image_stems - labels.keys())
    orphan_labels = sorted(labels.keys() - image_stems)
    if missing_labels or orphan_labels:
        raise ValueError(
            f"Image/label mismatch; missing labels={missing_labels}, "
            f"orphan labels={orphan_labels}"
        )

    records: list[tuple[str, Path, Path, str, int]] = []
    for image_path in images:
        source = source_name(image_path.stem)
        if source not in SPLIT_BY_SOURCE:
            raise ValueError(
                f"Unknown capture sequence {source!r}; add it to SPLIT_BY_SOURCE"
            )
        split = SPLIT_BY_SOURCE[source]
        label_path = labels[image_path.stem]
        boxes = validate_label(label_path, len(classes))
        records.append((split, image_path, label_path, source, boxes))

    # Rebuild only the generated output directory; dataset_org is never changed.
    if output_dir.exists():
        shutil.rmtree(output_dir)
    for split in SPLIT_NAMES:
        (output_dir / split / "images").mkdir(parents=True)
        (output_dir / split / "labels").mkdir(parents=True)

    for split, image_path, label_path, _source, _boxes in records:
        shutil.copy2(image_path, output_dir / split / "images" / image_path.name)
        shutil.copy2(label_path, output_dir / split / "labels" / label_path.name)

    shutil.copy2(class_file, output_dir / "classes.txt")
    yaml_lines = [
        f"path: {output_dir.as_posix()}",
        "train: train/images",
        "val: valid/images",
        "test: test/images",
        "",
        f"nc: {len(classes)}",
        "names:",
        *[f"  {index}: {name}" for index, name in enumerate(classes)],
        "",
    ]
    (output_dir / "data.yaml").write_text("\n".join(yaml_lines), encoding="utf-8")

    with (output_dir / "split_manifest.csv").open(
        "w", encoding="utf-8-sig", newline=""
    ) as manifest:
        writer = csv.writer(manifest)
        writer.writerow(("split", "source", "image", "label", "boxes"))
        for split, image_path, label_path, source, boxes in records:
            writer.writerow((split, source, image_path.name, label_path.name, boxes))

    image_counts = Counter(record[0] for record in records)
    box_counts = Counter()
    negative_counts = Counter()
    for split, _image, _label, _source, boxes in records:
        box_counts[split] += boxes
        negative_counts[split] += boxes == 0

    print(f"Source images: {len(records)}")
    for split in SPLIT_NAMES:
        ratio = image_counts[split] / len(records)
        print(
            f"{split}: {image_counts[split]} images ({ratio:.1%}), "
            f"{box_counts[split]} boxes, {negative_counts[split]} negatives"
        )
    print(f"Completed: {output_dir}")


if __name__ == "__main__":
    main()

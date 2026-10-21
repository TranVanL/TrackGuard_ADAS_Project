"""Print the first KITTI tracking-label rows for inspection."""

from argparse import ArgumentParser
from collections import Counter
from pathlib import Path
import sys
from typing import Optional


PROJECT_ROOT = Path(__file__).resolve().parents[1]
LABEL_DIR = PROJECT_ROOT / "data" / "kitti_subset" / "label_02"
KITTI_TRACKING_FIELD_COUNT = 17


def find_label_file(value: Optional[str]) -> Path:
    """Resolve a label path independently of the current working directory."""
    if value:
        requested = Path(value).expanduser()
        candidates = (
            (requested,) if requested.is_absolute() else
            (Path.cwd() / requested, PROJECT_ROOT / requested, LABEL_DIR / requested)
        )
        for candidate in candidates:
            if candidate.is_file():
                return candidate
        return candidates[0]

    # KITTI tracking sequence files use four digits (for example, 0000.txt).
    # Keep 00000.txt as a fallback for locally renamed files.
    for name in ("0000.txt", "00000.txt"):
        candidate = LABEL_DIR / name
        if candidate.is_file():
            return candidate

    available = sorted(LABEL_DIR.glob("*.txt"))
    if len(available) == 1:
        return available[0]
    return LABEL_DIR / "0000.txt"


def main() -> int:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "label_file",
        nargs="?",
        help="label file path or sequence filename (default: 0000.txt)",
    )
    args = parser.parse_args()
    label_path = find_label_file(args.label_file)

    if not label_path.is_file():
        print(
            f"Không tìm thấy file KITTI label: {label_path}\n"
            f"Hãy đặt file vào {LABEL_DIR} hoặc truyền đường dẫn, ví dụ:\n"
            "  python3 inspect_kitti.py 0000.txt",
            file=sys.stderr,
        )
        return 1

    rows = []
    invalid_rows = []
    with label_path.open("r", encoding="utf-8") as file:
        for line_no, line in enumerate(file, start=1):
            text = line.strip()
            if not text:
                continue

            fields = text.split()
            if len(fields) < KITTI_TRACKING_FIELD_COUNT:
                invalid_rows.append((line_no, len(fields)))
                continue

            try:
                frame = int(fields[0])
                track_id = int(fields[1])
                truncated = float(fields[3])
                occluded = int(fields[4])
                alpha = float(fields[5])
                bbox = tuple(float(value) for value in fields[6:10])
                dimensions = tuple(float(value) for value in fields[10:13])
                location = tuple(float(value) for value in fields[13:16])
                rotation_y = float(fields[16])
                score = float(fields[17]) if len(fields) > 17 else None
            except ValueError:
                invalid_rows.append((line_no, len(fields)))
                continue

            rows.append(
                {
                    "line_no": line_no,
                    "text": text,
                    "field_count": len(fields),
                    "frame": frame,
                    "track_id": track_id,
                    "class_name": fields[2],
                    "truncated": truncated,
                    "occluded": occluded,
                    "alpha": alpha,
                    "bbox": bbox,
                    "dimensions": dimensions,
                    "location": location,
                    "rotation_y": rotation_y,
                    "score": score,
                }
            )

    print(f"Label file: {label_path}")
    print("\n=== First 20 rows ===")
    for row in rows[:20]:
        left, top, right, bottom = row["bbox"]
        height, width, length = row["dimensions"]
        location_x, location_y, location_z = row["location"]
        print(
            f"\nLine {row['line_no']}: {row['field_count']} fields\n"
            f"  frame      : {row['frame']}\n"
            f"  track_id   : {row['track_id']}\n"
            f"  type       : {row['class_name']}\n"
            f"  truncated  : {row['truncated']:g}\n"
            f"  occluded   : {row['occluded']}\n"
            f"  alpha      : {row['alpha']:g}\n"
            "  bbox       : "
            f"left={left:g}, top={top:g}, right={right:g}, bottom={bottom:g}\n"
            "  dimensions : "
            f"height={height:g}, width={width:g}, length={length:g}\n"
            "  location   : "
            f"x={location_x:g}, y={location_y:g}, z={location_z:g}\n"
            f"  rotation_y : {row['rotation_y']:g}"
        )
        if row["score"] is not None:
            print(f"\n  score      : {row['score']:g}")
        print("\n  raw        : " + row["text"])
        print("-" * 70)

    if not rows:
        print("Không có dòng label hợp lệ để thống kê.", file=sys.stderr)
        return 1

    frames = [row["frame"] for row in rows]
    class_counts = Counter(row["class_name"] for row in rows)
    track_ids = list(dict.fromkeys(row["track_id"] for row in rows))
    occlusion_counts = Counter(row["occluded"] for row in rows)
    truncation_counts = Counter(row["truncated"] for row in rows)

    print("\n=== Summary ===")
    print(f"Valid rows: {len(rows)}")
    print(f"Frame range: {min(frames)} - {max(frames)}")

    print("Class count:")
    for class_name, count in class_counts.most_common():
        print(f"  {class_name}: {count}")

    print("Track IDs (first 10 unique): " + ", ".join(map(str, track_ids[:10])))
 
    print("Occlusion count:")
    for value, count in sorted(occlusion_counts.items()):
        print(f"  {value}: {count}")
    print("Occlusion samples (first 10): " + ", ".join(
        str(row["occluded"]) for row in rows[:10]
    ))

    print("Truncation count:")
    for value, count in sorted(truncation_counts.items()):
        print(f"  {value:g}: {count}")
    print("Truncation samples (first 10): " + ", ".join(
        f"{row['truncated']:g}" for row in rows[:10]
    ))

    if invalid_rows:
        print(
            f"\nWarning: bỏ qua {len(invalid_rows)} dòng không hợp lệ "
            f"(ví dụ line {invalid_rows[0][0]} có {invalid_rows[0][1]} fields).",
            file=sys.stderr,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
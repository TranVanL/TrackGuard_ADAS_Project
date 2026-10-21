"""Inspect KITTI OXTS rows and summarize ego-motion fields."""

from argparse import ArgumentParser
from collections import Counter
from math import isfinite, isnan
from pathlib import Path
import sys
from typing import Optional


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OXTS_DIR = PROJECT_ROOT / "data" / "kitti_subset" / "oxts"
KITTI_OXTS_FIELD_COUNT = 30


def find_oxts_file(value: Optional[str]) -> Path:
    """Resolve an OXTS path independently of the current working directory."""
    if value:
        requested = Path(value).expanduser()
        candidates = (
            (requested,)
            if requested.is_absolute()
            else (
                Path.cwd() / requested,
                PROJECT_ROOT / requested,
                OXTS_DIR / requested,
            )
        )
        for candidate in candidates:
            if candidate.is_file():
                return candidate
        return candidates[0]

    for name in ("0000.txt", "00000.txt"):
        candidate = OXTS_DIR / name
        if candidate.is_file():
            return candidate

    available = sorted(OXTS_DIR.glob("*.txt"))
    if len(available) == 1:
        return available[0]
    return OXTS_DIR / "0000.txt"


def format_value(value: float) -> str:
    """Format finite values compactly while keeping NaN visible."""
    if isnan(value):
        return "nan"
    if not isfinite(value):
        return str(value)
    return f"{value:g}"


def main() -> int:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "oxts_file",
        nargs="?",
        help="OXTS file path or sequence filename (default: 0000.txt)",
    )
    args = parser.parse_args()
    oxts_path = find_oxts_file(args.oxts_file)

    if not oxts_path.is_file():
        print(
            f"Không tìm thấy file KITTI OXTS: {oxts_path}\n"
            f"Hãy đặt file vào {OXTS_DIR} hoặc truyền đường dẫn, ví dụ:\n"
            "  python3 inspect_oxts.py 0000.txt",
            file=sys.stderr,
        )
        return 1

    rows = []
    invalid_rows = []
    logical_frame_index = 0
    with oxts_path.open("r", encoding="utf-8") as file:
        for line_no, line in enumerate(file, start=1):
            text = line.strip()
            if not text:
                continue

            frame_index = logical_frame_index
            logical_frame_index += 1
            fields = text.split()
            if len(fields) < KITTI_OXTS_FIELD_COUNT:
                invalid_rows.append((line_no, len(fields), "too few fields"))
                continue

            try:
                values = tuple(float(value) for value in fields)
            except ValueError:
                invalid_rows.append((line_no, len(fields), "non-numeric field"))
                continue

            rows.append(
                {
                    "line_no": line_no,
                    "frame_index": frame_index,
                    "text": text,
                    "field_count": len(fields),
                    "values": values,
                }
            )

    if not rows:
        print("Không có dòng OXTS hợp lệ để thống kê.", file=sys.stderr)
        return 1

    print(f"OXTS file: {oxts_path}")
    print(f"Expected fields per row: {KITTI_OXTS_FIELD_COUNT}")
    print("Note: OXTS không chứa timestamp riêng; với dữ liệu synced, line index tương ứng frame index.")

    print("\n=== First 20 rows ===")
    for row in rows[:20]:
        values = row["values"]
        print(
            f"\nLine {row['line_no']} / frame {row['frame_index']}: "
            f"{row['field_count']} fields\n"
            "  position  : "
            f"lat={format_value(values[0])} deg, "
            f"lon={format_value(values[1])} deg, "
            f"alt={format_value(values[2])} m\n"
            "  attitude  : "
            f"roll={format_value(values[3])} rad, "
            f"pitch={format_value(values[4])} rad, "
            f"yaw={format_value(values[5])} rad\n"
            "  velocity  : "
            f"vf={format_value(values[8])} m/s, "
            f"vl={format_value(values[9])} m/s, "
            f"vu={format_value(values[10])} m/s\n"
            "  angular   : "
            f"wz={format_value(values[19])} rad/s\n"
            "  quality   : "
            f"pos_accuracy={format_value(values[23])} m, "
            f"vel_accuracy={format_value(values[24])} m/s, "
            f"navstat={int(values[25])}, numsats={int(values[26])}, "
            f"posmode={int(values[27])}, velmode={int(values[28])}, "
            f"orimode={int(values[29])}\n"
            f"  raw       : {row['text']}\n"
            + "-" * 70
        )

    values = [row["values"] for row in rows]
    frame_indices = [row["frame_index"] for row in rows]
    navstat_counts = Counter(int(row[25]) for row in values)
    numsats_values = [row[26] for row in values]
    yaw_values = [row[5] for row in values]
    vf_values = [row[8] for row in values]
    wz_values = [row[19] for row in values]
    nan_counts = {
        index: sum(isnan(row[index]) for row in values)
        for index in range(KITTI_OXTS_FIELD_COUNT)
    }

    print("\n=== Summary ===")
    print(f"Valid rows: {len(rows)}")
    print(f"Valid frame range: {min(frame_indices)} - {max(frame_indices)}")
    print(
        "Yaw range: "
        f"{format_value(min(yaw_values))} - {format_value(max(yaw_values))} rad"
    )
    print(
        "Forward velocity range: "
        f"{format_value(min(vf_values))} - {format_value(max(vf_values))} m/s"
    )
    print(
        "Yaw-rate range: "
        f"{format_value(min(wz_values))} - {format_value(max(wz_values))} rad/s"
    )
    print(
        "Satellite count range: "
        f"{format_value(min(numsats_values))} - {format_value(max(numsats_values))}"
    )

    print("navstat count:")
    for value, count in sorted(navstat_counts.items()):
        print(f"  {value}: {count}")

    nan_fields = [index for index, count in nan_counts.items() if count]
    if nan_fields:
        print("Fields containing NaN: " + ", ".join(map(str, nan_fields)))
    else:
        print("Fields containing NaN: none")

    if invalid_rows:
        print(
            f"\nWarning: bỏ qua {len(invalid_rows)} dòng không hợp lệ "
            f"(ví dụ line {invalid_rows[0][0]}: {invalid_rows[0][2]}).",
            file=sys.stderr,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

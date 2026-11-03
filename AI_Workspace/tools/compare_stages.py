"""Compare immutable local gimbal stages: source hashes, warnings, capacity, MAP symbols."""
import argparse
import csv
import json
import re
from collections import Counter
from pathlib import Path
from project_paths import project_path


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def warnings(stage):
    with (stage / "gimbal-warnings.csv").open(encoding="utf-8-sig", newline="") as stream:
        return Counter((row["file"].replace("\\", "/"), row["line"], row["code"], row["message"])
                       for row in csv.DictReader(stream))


def map_symbols(stage):
    content = (stage / "gimbal.map").read_text(encoding="gb18030", errors="replace")
    return set(re.findall(r"^\s+([A-Za-z_]\w*)\s+0x[0-9a-fA-F]+\s+Thumb Code\s+\d+", content, re.M))


def compare(before, after):
    a = read_json(before / "gimbal-sha256.json")
    b = read_json(after / "gimbal-sha256.json")
    old = read_json(before / "build-metrics.json")["gimbal"]
    new = read_json(after / "build-metrics.json")["gimbal"]
    old_warnings, new_warnings = warnings(before), warnings(after)
    old_symbols, new_symbols = map_symbols(before), map_symbols(after)
    keys = sorted(a.keys() | b.keys())
    return {
        "before_stage": before.name,
        "after_stage": after.name,
        "changed_source_or_config": [key for key in keys if a.get(key) != b.get(key)],
        "build_success": old["success"] and new["success"],
        "errors": [old["errors"], new["errors"]],
        "warnings": [old["warnings"], new["warnings"]],
        "new_warning_rows": [list(row) for row in sorted((new_warnings - old_warnings).elements())],
        "removed_warning_rows": [list(row) for row in sorted((old_warnings - new_warnings).elements())],
        "size_delta_bytes": {key: new[key] - old[key] for key in
                             ("code_bytes", "ro_data_bytes", "rw_data_bytes", "zi_data_bytes")},
        "map_removed_thumb_symbols": sorted(old_symbols - new_symbols),
        "map_added_thumb_symbols": sorted(new_symbols - old_symbols),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("--result", type=Path, required=True)
    args = parser.parse_args()
    result = compare(project_path(args.before), project_path(args.after))
    destination = project_path(args.result)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))

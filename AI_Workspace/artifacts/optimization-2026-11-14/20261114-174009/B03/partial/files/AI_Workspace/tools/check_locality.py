"""Fail if the active Keil project reads source or include paths outside this root."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from project_paths import ROOT, PROJECTS, project_path
from snapshot import project_inventory


def check():
    root = ROOT.resolve()
    result = {"root": str(root), "projects": {}}
    for name, (project_root, project) in PROJECTS.items():
        project_root = project_root.resolve()
        project_root.relative_to(root)
        targets, entries = project_inventory(project_root, project)
        mdk = (project_root / project).parent
        paths = [Path(row["path"]) for row in entries]
        missing = [str(path) for path in paths if not path.is_file()]
        external = []
        for path in paths:
            try:
                path.relative_to(root)
            except ValueError:
                external.append(str(path))
        tree = ET.parse(project_root / project)
        includes = []
        for target in tree.findall("./Targets/Target"):
            controls = target.find("./TargetOption/TargetArmAds/Cads/VariousControls")
            for raw in controls.findtext("IncludePath", "").split(";"):
                if raw:
                    path = (mdk / raw.replace("\\", "/")).resolve()
                    includes.append(str(path))
                    try:
                        path.relative_to(root)
                    except ValueError:
                        external.append(str(path))
                    if not path.is_dir():
                        missing.append(str(path))
        result["projects"][name] = {"target_count": len(targets), "keil_entries": len(paths),
                                    "include_directories": len(includes), "external_paths": external,
                                    "missing_paths": missing}
    result["passed"] = all(not item["external_paths"] and not item["missing_paths"]
                           for item in result["projects"].values())
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", type=Path, required=True)
    args = parser.parse_args()
    outcome = check()
    destination = project_path(args.result)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(outcome, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(outcome, ensure_ascii=False, indent=2))
    if not outcome["passed"]:
        raise SystemExit(1)

"""Capture immutable raw evidence for one firmware repository.

Used before the project-locating tools are repaired (B00), so it must not depend
on projects.json: the project file is discovered under MDK-ARM/ instead.

Outputs, all under the given --out directory:
  capture.json          status summary (git state, counts, exclusions)
  source-sha256.json    relative path -> sha256 for every captured file
  sources.zip           byte-identical copy of those files
  git-head.txt, git-status.txt, git-diff.patch, git-stash.txt
  firmware/             existing AXF/HEX/MAP/build logs, copied with hashes
  excluded-index.csv    path,size,mtime for preserved historical areas (not zipped)
"""
import argparse
import csv
import hashlib
import json
import shutil
import subprocess
import zipfile
from datetime import datetime, timezone
from pathlib import Path

CAPTURE_EXTENSIONS = {".c", ".h", ".cpp", ".hpp", ".s", ".lib", ".uvprojx",
                      ".uvoptx", ".ioc", ".ld", ".sct", ".bat", ".cmake",
                      ".json", ".py", ".ps1"}
CAPTURE_NAMES = {"AGENTS.md", ".mxproject", ".gitignore"}
# Directories whose contents are historical evidence, build output or caches.
# They are indexed (path/size/mtime) but never rewritten into a new package.
INDEX_ONLY_PARTS = {"snapshots", "cache", "artifacts", "Objects", "Listings",
                    "RTE", "__pycache__", ".git", "history"}
FIRMWARE_EXTENSIONS = {".axf", ".hex", ".map", ".elf", ".bin"}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def locate_project(root):
    candidates = sorted((root / "MDK-ARM").glob("*.uvprojx"))
    if len(candidates) != 1:
        raise SystemExit("expected exactly one MDK-ARM/*.uvprojx in %s, got %d"
                         % (root, len(candidates)))
    return candidates[0]


def output_directories(root, project_file):
    import xml.etree.ElementTree as ET

    directories = set()
    tree = ET.parse(str(project_file))
    for target in tree.findall("./Targets/Target"):
        common = target.find("./TargetOption/TargetCommonOption")
        if common is None:
            continue
        relative = (common.findtext("OutputDirectory") or "").replace("\\", "/")
        if relative:
            directories.add((project_file.parent / relative).resolve())
        listings = (common.findtext("ListingPath") or "").replace("\\", "/")
        if listings:
            directories.add((project_file.parent / listings).resolve())
    return directories


def run_git(root, arguments):
    completed = subprocess.run(["git"] + arguments, cwd=str(root),
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return completed.returncode, completed.stdout.decode("utf-8", "replace")


def capture_git(root, out):
    status, head = run_git(root, ["rev-parse", "HEAD"])
    _, branch_lines = run_git(root, ["rev-parse", "--abbrev-ref", "HEAD"])
    branch = (branch_lines.splitlines() or [""])[0]
    _, short = run_git(root, ["status", "--short", "--untracked-files=all"])
    _, diff = run_git(root, ["diff", "--binary", "HEAD"])
    _, stash = run_git(root, ["stash", "list"])
    (out / "git-head.txt").write_text(
        "returncode=%d\nHEAD=%s\nbranch=%s\n" % (status, head.strip(), branch.strip()),
        encoding="utf-8")
    (out / "git-status.txt").write_text(short, encoding="utf-8")
    (out / "git-diff.patch").write_text(diff, encoding="utf-8")
    (out / "git-stash.txt").write_text(stash, encoding="utf-8")
    return {"head": head.strip(), "branch": branch.strip(),
            "head_reachable": status == 0,
            "status_entries": len([line for line in short.splitlines() if line.strip()]),
            "diff_bytes": len(diff)}


def collect(root, project_file, excluded_outputs):
    included, indexed = {}, {}
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(root)
        if any(part in INDEX_ONLY_PARTS for part in relative.parts):
            indexed[relative.as_posix()] = path.stat()
            continue
        if any(output in path.parents for output in excluded_outputs):
            indexed[relative.as_posix()] = path.stat()
            continue
        if path.suffix.lower() in CAPTURE_EXTENSIONS or path.name in CAPTURE_NAMES:
            included[relative.as_posix()] = path
        else:
            indexed[relative.as_posix()] = path.stat()
    return included, indexed


def write_package(root, out, project_file, included):
    hashes = {relative: sha256(path) for relative, path in included.items()}
    (out / "source-sha256.json").write_text(
        json.dumps(hashes, indent=2, sort_keys=True), encoding="utf-8")
    with zipfile.ZipFile(out / "sources.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for relative, path in included.items():
            archive.write(str(path), relative)
    return hashes


def copy_firmware(root, out, excluded_outputs):
    firmware = out / "firmware"
    firmware.mkdir(parents=True, exist_ok=True)
    rows = []
    search = set(excluded_outputs) | {root / "AI_Workspace" / "artifacts"}
    for directory in sorted(p for p in search if p.is_dir()):
        for path in sorted(directory.rglob("*")):
            if not path.is_file():
                continue
            if path.suffix.lower() not in FIRMWARE_EXTENSIONS and "log" not in path.name.lower():
                continue
            relative = path.relative_to(root).as_posix().replace("/", "__")
            target = firmware / relative
            if target.exists():
                target = firmware / ("dup__" + relative)
            shutil.copy2(path, target)
            rows.append({"source": path.relative_to(root).as_posix(),
                         "copied_as": target.name, "bytes": path.stat().st_size,
                         "mtime": datetime.fromtimestamp(
                             path.stat().st_mtime, timezone.utc).isoformat(),
                         "sha256": sha256(target)})
    return rows


def write_index(out, indexed):
    with (out / "excluded-index.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["path", "bytes", "mtime_utc"])
        for relative in sorted(indexed):
            stat = indexed[relative]
            writer.writerow([relative, stat.st_size,
                             datetime.fromtimestamp(stat.st_mtime, timezone.utc).isoformat()])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--label", required=True)
    parser.add_argument("--phase", default="B00")
    parser.add_argument("--run-id", required=True)
    arguments = parser.parse_args()

    root = arguments.repo.resolve()
    out = arguments.out.resolve()
    if out.exists() and any(out.iterdir()):
        raise SystemExit("refusing to overwrite existing evidence: %s" % out)
    out.mkdir(parents=True, exist_ok=True)

    project_file = locate_project(root)
    excluded_outputs = output_directories(root, project_file)
    included, indexed = collect(root, project_file, excluded_outputs)

    summary = {
        "label": arguments.label,
        "phase": arguments.phase,
        "run_id": arguments.run_id,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "repo_root": str(root),
        "project_file": str(project_file.relative_to(root).as_posix()),
        "excluded_output_directories": [str(p) for p in sorted(excluded_outputs)],
        "git": capture_git(root, out),
        "captured_file_count": len(included),
        "captured_bytes": sum(path.stat().st_size for path in included.values()),
        "indexed_file_count": len(indexed),
    }
    summary["source_sha256"] = write_package(root, out, project_file, included)
    write_index(out, indexed)
    summary["firmware"] = copy_firmware(root, out, excluded_outputs)
    summary.pop("source_sha256")
    summary["source_sha256_entries"] = len(
        json.loads((out / "source-sha256.json").read_text(encoding="utf-8")))
    summary["sources_zip_sha256"] = sha256(out / "sources.zip")
    (out / "capture.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({key: summary[key] for key in
                      ("label", "repo_root", "captured_file_count", "captured_bytes",
                       "indexed_file_count", "git", "sources_zip_sha256")},
                     indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

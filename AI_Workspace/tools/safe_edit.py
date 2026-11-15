"""Apply literal replacements to a source file without touching its encoding.

Firmware sources in these repositories mix ASCII, UTF-8 and GBK. Rewriting a GBK
file as UTF-8 destroys its Chinese comments, so the bytes are decoded with the
detected codec and re-encoded with the same codec, and the CRLF/LF style of the
file is preserved.

Usage:
    python safe_edit.py detect <file> [...]
    python safe_edit.py apply <spec.json> [--check]

spec.json:
    {
      "edits": [
        {"file": "application/foo.c",
         "replacements": [{"old": "literal", "new": "literal", "count": 1}]}
      ]
    }

Relative file paths in the spec resolve against the repository root found from
this script's location. --check verifies every literal without writing.
"""
import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def repository_root():
    markers = ["MDK-ARM/standard_tpye_c.uvprojx", "MDK-ARM/CtrlBoard-H7_IMU.uvprojx"]
    for parent in HERE.parents:
        if any((parent / marker).is_file() for marker in markers):
            return parent
    raise SystemExit("cannot locate project root from " + str(HERE))


BOMS = {b"\xef\xbb\xbf": "utf-8-sig"}


def detect_bytes(data):
    for bom, codec in BOMS.items():
        if data.startswith(bom):
            return codec
    try:
        data.decode("utf-8")
        return "utf-8"
    except UnicodeDecodeError:
        pass
    try:
        data.decode("gbk")
        return "gbk"
    except UnicodeDecodeError:
        return None


def read_text(path):
    data = path.read_bytes()
    codec = detect_bytes(data)
    if codec is None:
        raise SystemExit("%s: cannot detect text encoding, refusing to rewrite" % path)
    return codec, data.decode(codec)


def apply_edits(path, replacements, check):
    codec, text = read_text(path)
    newline = "\r\n" if "\r\n" in text else "\n"
    original_crlf = text.count("\r\n")
    for item in replacements:
        old, new = item["old"], item["new"]
        expected = item.get("count", 1)
        found = text.count(old)
        if found != expected:
            raise SystemExit("%s: expected %d occurrence(s) of %r, found %d"
                             % (path, expected, old[:60], found))
        text = text.replace(old, new)
    if text.count("\r\n") != original_crlf and newline == "\r\n":
        raise SystemExit("%s: replacement changed line endings" % path)
    if check:
        print("OK(no write) %s codec=%s replacements=%d" % (path, codec, len(replacements)))
        return codec, 0
    data = text.encode(codec)
    previous = path.stat().st_size
    path.write_bytes(data)
    print("wrote %s codec=%s bytes %d->%d replacements=%d"
          % (path, codec, previous, len(data), len(replacements)))
    return codec, len(replacements)


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="mode", required=True)
    detect = sub.add_parser("detect")
    detect.add_argument("files", nargs="+", type=Path)
    apply = sub.add_parser("apply")
    apply.add_argument("spec", type=Path)
    apply.add_argument("--check", action="store_true")
    arguments = parser.parse_args()

    root = repository_root()
    if arguments.mode == "detect":
        for item in arguments.files:
            path = item if item.is_absolute() else root / item
            codec, text = read_text(path)
            non_ascii = sum(1 for character in text if ord(character) > 127)
            newline = "CRLF" if "\r\n" in text else "LF"
            print("%-10s %-5s %6d non-ascii  %s" % (codec, newline, non_ascii,
                                                    path.relative_to(root).as_posix()))
        return 0

    spec = json.loads(arguments.spec.read_text(encoding="utf-8-sig"))
    total = 0
    for edit in spec["edits"]:
        path = root / edit["file"]
        codec, count = apply_edits(path, edit["replacements"], arguments.check)
        total += 1
        if count and "expected_sha256" in edit:
            import hashlib
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            status = "match" if actual == edit["expected_sha256"] else "MISMATCH"
            print("sha256 %s %s" % (status, actual))
    print("files=%d check=%s" % (total, arguments.check))
    return 0


if __name__ == "__main__":
    sys.exit(main())

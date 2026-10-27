#!/usr/bin/env python3
"""Fail if a canonical libstd header still mentions the ETLSTD namespace macro."""
import re
import sys
from pathlib import Path

CANONICAL_DIR = Path(__file__).resolve().parent.parent / "libstd" / "include" / "c++" / "v1"


def main():
    pattern = re.compile(r"\bETLSTD\b")
    found = 0
    for path in sorted(p for p in CANONICAL_DIR.rglob("*") if p.is_file()):
        for number, line in enumerate(path.read_text(errors="replace").splitlines(), 1):
            if pattern.search(line):
                print(f"{path.relative_to(CANONICAL_DIR.parent.parent.parent.parent)}:{number}: {line.strip()}", file=sys.stderr)
                found += 1
    return 1 if found else 0


if __name__ == "__main__":
    sys.exit(main())

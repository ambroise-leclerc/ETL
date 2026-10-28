#!/usr/bin/env python3
"""Validate the supported AVR list, emit the CI matrix, and inspect profile artifacts."""
import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PROFILES = ("minimal", "full")


def devices(root=ROOT):
    rows = json.loads((root / "tools/avr_matrix.json").read_text())
    names = [row["mcu"] for row in rows]
    if not names or len(names) != len(set(names)):
        raise ValueError("AVR matrix must contain unique devices")
    for row in rows:
        if not {"mcu", "f_cpu"} <= set(row) or set(row) - {"mcu", "f_cpu", "timer0_vectors"} or not re.fullmatch(r"[1-9][0-9]*UL", row["f_cpu"]):
            raise ValueError(f"Invalid AVR matrix row: {row}")
        if "timer0_vectors" in row and not re.fullmatch(r"[0-9]+ [0-9]+", row["timer0_vectors"]):
            raise ValueError(f"Invalid Timer0 regression vectors: {row}")
    modeled = {name.lower() for name in re.findall(
        r"defined\s*\(\s*__AVR_(AT(?:mega|tiny)\w+)__\s*\)", (root / "include/etl/ioports.h").read_text())}
    if set(names) != modeled:
        raise ValueError(f"AVR matrix differs from ETL devices: missing={modeled - set(names)}, extra={set(names) - modeled}")
    documented = re.findall(r"^\| `(at(?:mega|tiny)\w+)` \| `([0-9]+UL)` \|$",
                            (root / "README.md").read_text(), re.MULTILINE)
    if documented != [(row["mcu"], row["f_cpu"]) for row in rows]:
        raise ValueError("README supported AVR table differs from tools/avr_matrix.json")
    return rows


def check_artifacts(directory, profile):
    name = f"etl_avr_std_{profile}_smoke"
    for suffix in ("elf", "hex", "map"):
        path = directory / f"{name}.{suffix}"
        if not path.is_file() or not path.stat().st_size:
            raise ValueError(f"Missing or empty AVR artifact: {path}")
    path = directory / f"{name}.map"
    contents = path.read_text()
    opposite = "full" if profile == "minimal" else "minimal"
    if re.search(r"libstdc\+\+|GLIBCXX|" + re.escape(f"libetlstd_{opposite}.a"), contents, re.IGNORECASE):
        raise ValueError(f"Contaminated AVR profile link map: {path}")
    if f"libetlstd_{profile}.a" not in contents or f"__etl_profile_{profile}" not in contents:
        raise ValueError(f"Missing ETLStd profile archive/marker in {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    output = parser.add_mutually_exclusive_group()
    output.add_argument("--github", action="store_true", help="emit all device/profile pairs as a GitHub matrix")
    output.add_argument("--devices", action="store_true", help="emit MCU/F_CPU rows for legacy device checks")
    output.add_argument("--github-regression", action="store_true", help="emit the existing interrupt regression targets")
    parser.add_argument("--artifacts", type=Path)
    parser.add_argument("--profile", choices=PROFILES)
    args = parser.parse_args()
    try:
        rows = devices()
        if args.artifacts:
            for profile in (args.profile,) if args.profile else PROFILES:
                check_artifacts(args.artifacts, profile)
        if args.github:
            print(json.dumps({"include": [dict(mcu=row["mcu"], f_cpu=row["f_cpu"], profile=profile)
                                          for row in rows for profile in PROFILES]}, separators=(",", ":")))
        elif args.devices:
            print("\n".join(f"{row['mcu']} {row['f_cpu']}" for row in rows))
        elif args.github_regression:
            print(json.dumps([row for row in rows if "timer0_vectors" in row], separators=(",", ":")))
        else:
            print(f"AVR matrix: {len(rows)} devices, {len(rows) * len(PROFILES)} device/profile combinations OK")
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()

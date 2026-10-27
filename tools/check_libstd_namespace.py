#!/usr/bin/env python3
"""Source policy: canonical code uses std directly, never ETLSTD or repository-qualified standard includes.

Rules (every violation is reported as `path:line: [rule] message`, paths relative to the repository root):
  etlstd-token          ETLSTD / etlstd token; only allowlisted 0.2 compatibility files may keep it.
  libstd-include-path   #include <libstd/include/...> or <../libstd/include/...> (angle or quote form).
  etlstd-namespace-decl `namespace etlstd` declaration; never allowlistable.

Scanned: libstd/include/c++, include, examples, test. The 0.2 wrappers directly under libstd/include are not scanned.
Allowlist (tools/libstd_source_policy_allowlist.txt): `<rule> <path>` per line; stale entries fail the check.

Usage: check_libstd_namespace.py [--root DIR] [--allowlist FILE] [--self-test]
"""
import argparse
import re
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
DEFAULT_ROOT = TOOLS_DIR.parent
DEFAULT_ALLOWLIST = TOOLS_DIR / "libstd_source_policy_allowlist.txt"
FIXTURES_DIR = TOOLS_DIR / "check_libstd_namespace_fixtures"

SCANNED_DIRS = ("libstd/include/c++", "include", "examples", "test")

RULES = {
    "etlstd-token": (re.compile(r"\b(?:ETLSTD|etlstd)\b"), "ETLSTD/etlstd is forbidden, use std directly"),
    "libstd-include-path": (
        re.compile(r"^\s*#\s*include\s*[<\"](?:\.\./)*libstd/include/"),
        "repository-qualified standard include, include the standard name (e.g. <memory>)",
    ),
    "etlstd-namespace-decl": (re.compile(r"\bnamespace\s+etlstd\b"), "namespace etlstd must not be declared"),
}
NON_ALLOWLISTABLE = {"etlstd-namespace-decl"}


def read_text(path):
    data = path.read_bytes()
    if b"\0" in data:
        return None
    return data.decode("utf-8", errors="replace")


def parse_allowlist(path, root):
    """Return ({(rule, relpath): line_number}, [error diagnostics])."""
    entries, errors = {}, []
    if not path.is_file():
        return entries, errors
    label = path.name
    for number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 2 or fields[0] not in RULES:
            errors.append(f"{label}:{number}: [bad-allowlist-entry] expected '<rule> <path>' with rule in {sorted(RULES)}")
        elif fields[0] in NON_ALLOWLISTABLE:
            errors.append(f"{label}:{number}: [bad-allowlist-entry] rule {fields[0]} cannot be allowlisted")
        elif (fields[0], fields[1]) in entries:
            errors.append(f"{label}:{number}: [bad-allowlist-entry] duplicate entry {fields[0]} {fields[1]}")
        else:
            entries[(fields[0], fields[1])] = number
    return entries, errors


def scan(root, allowlist_path):
    """Return the sorted list of diagnostics for the tree at root."""
    allowed, diagnostics = parse_allowlist(allowlist_path, root)
    used = set()
    violations = []
    for directory in SCANNED_DIRS:
        base = root / directory
        if not base.is_dir():
            continue
        for path in sorted(p for p in base.rglob("*") if p.is_file()):
            text = read_text(path)
            if text is None:
                continue
            rel = path.relative_to(root).as_posix()
            for number, line in enumerate(text.splitlines(), 1):
                for rule, (pattern, message) in RULES.items():
                    if pattern.search(line):
                        if (rule, rel) in allowed:
                            used.add((rule, rel))
                        else:
                            violations.append((rel, number, rule, f"{rel}:{number}: [{rule}] {message}"))
    for (rule, rel), number in sorted(allowed.items(), key=lambda item: item[1]):
        if (rule, rel) not in used:
            diagnostics.append(f"{allowlist_path.name}:{number}: [stale-allowlist] {rule} {rel} no longer has any violation, remove it")
    return diagnostics + [entry[3] for entry in sorted(violations)]


def self_test():
    """Each fixture directory is a mini repository root with `expected.txt` and an optional `allowlist.txt`."""
    failures = 0
    fixtures = sorted(p for p in FIXTURES_DIR.iterdir() if p.is_dir())
    if not fixtures:
        print(f"no fixtures found in {FIXTURES_DIR}", file=sys.stderr)
        return 1
    for fixture in fixtures:
        expected = (fixture / "expected.txt").read_text(encoding="utf-8").splitlines()
        actual = scan(fixture, fixture / "allowlist.txt")
        if actual == expected:
            print(f"ok   {fixture.name}")
        else:
            failures += 1
            print(f"FAIL {fixture.name}\n  expected: {expected}\n  actual:   {actual}", file=sys.stderr)
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--allowlist", type=Path, default=DEFAULT_ALLOWLIST)
    parser.add_argument("--self-test", action="store_true", help="run the checker against its fixtures")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    diagnostics = scan(args.root.resolve(), args.allowlist)
    for diagnostic in diagnostics:
        print(diagnostic, file=sys.stderr)
    return 1 if diagnostics else 0


if __name__ == "__main__":
    sys.exit(main())

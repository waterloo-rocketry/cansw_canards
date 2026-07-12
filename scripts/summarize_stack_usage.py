#!/usr/bin/env python3
"""Summarize GCC .su stack-usage reports.

Default input root: .pio/build/debug/src

Each .su line is expected to look like:
  path/to/file.c:LINE:COLUMN:FUNCTION<TAB>BYTES<TAB>QUALIFIER
"""

from __future__ import annotations

import argparse
import collections
import pathlib
import sys
from dataclasses import dataclass


@dataclass(frozen=True)
class Entry:
    su_file: pathlib.Path
    location: str
    function: str
    bytes_used: int
    qualifier: str


def parse_su_file(path: pathlib.Path) -> list[Entry]:
    entries: list[Entry] = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if not line:
            continue
        parts = line.split("\t")
        if len(parts) != 3:
            continue
        left, bytes_str, qualifier = parts
        left_parts = left.rsplit(":", 3)
        if len(left_parts) != 4:
            continue
        src_file, src_line, src_col, function = left_parts
        location = f"{src_file}:{src_line}:{src_col}"
        try:
            bytes_used = int(bytes_str)
        except ValueError:
            continue
        entries.append(
            Entry(
                su_file=path,
                location=location,
                function=function,
                bytes_used=bytes_used,
                qualifier=qualifier,
            )
        )
    return entries


def human(num: int) -> str:
    return f"{num:,}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize stack usage from GCC .su files")
    parser.add_argument(
        "root",
        nargs="?",
        default=".pio/build/debug/src",
        help="Root directory to scan for .su files (default: .pio/build/debug/src)",
    )
    parser.add_argument(
        "--top",
        type=int,
        default=20,
        help="How many largest functions/files to show (default: 20)",
    )
    args = parser.parse_args()

    root = pathlib.Path(args.root)
    if not root.exists():
        print(f"ERROR: root does not exist: {root}", file=sys.stderr)
        return 2

    su_files = sorted(root.rglob("*.su"))
    if not su_files:
        print(f"No .su files found under: {root}")
        return 1

    entries: list[Entry] = []
    for su in su_files:
        entries.extend(parse_su_file(su))

    if not entries:
        print(f"No parseable .su entries found under: {root}")
        return 1

    total_bytes = sum(e.bytes_used for e in entries)

    by_qualifier: dict[str, int] = collections.defaultdict(int)
    by_file: dict[str, int] = collections.defaultdict(int)
    for e in entries:
        by_qualifier[e.qualifier] += e.bytes_used
        by_file[e.location.rsplit(":", 2)[0]] += e.bytes_used

    largest_functions = sorted(entries, key=lambda e: e.bytes_used, reverse=True)[: args.top]
    largest_files = sorted(by_file.items(), key=lambda kv: kv[1], reverse=True)[: args.top]

    print("=== Stack Usage Summary (.su) ===")
    print(f"Scan root: {root}")
    print(f".su files: {len(su_files)}")
    print(f"Functions parsed: {len(entries)}")
    print(f"Total bytes (sum of all reported functions): {human(total_bytes)}")
    print()

    print("By qualifier:")
    for qualifier, total in sorted(by_qualifier.items(), key=lambda kv: kv[1], reverse=True):
        print(f"  {qualifier:<20} {human(total):>10} B")
    print()

    print(f"Top {args.top} functions by reported bytes:")
    for e in largest_functions:
        print(
            f"  {human(e.bytes_used):>10} B  {e.function:<35} "
            f"[{e.qualifier}]  {e.location}"
        )
    print()

    print(f"Top {args.top} source files by summed bytes:")
    for src, total in largest_files:
        print(f"  {human(total):>10} B  {src}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

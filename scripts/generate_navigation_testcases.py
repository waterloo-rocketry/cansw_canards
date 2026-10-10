#!/usr/bin/env python3
"""Convert MATLAB navigation logs into parallel, codegen-typed C arrays.

Uses only the Python standard library. Emits both the original logged inputs
and a separate variant with generate_testcases_mat.m's sensor-status rules.
The latter uses Python's seeded RNG, not MATLAB's exact random sequence.
"""

import argparse
import copy
import math
from pathlib import Path
import random
import re


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG = ROOT / "src/third_party/closedrocket-dev/embedded-coder/scripts/navigator_log.txt"
SENSORS = {
    "board_accel": 3, "board_gyro": 3, "mti_accel": 3, "mti_gyro": 3,
    "ad_accel": 3, "ad_gyro": 3, "board_baro": 1, "board_mag": 3,
    "mti_baro": 1, "mti_mag": 3,
}
BIAS = {
    "board_gyro": "bias_board_gyro", "mti_gyro": "bias_mti_gyro",
    "ad_gyro": "bias_ad_gyro", "board_mag_earth": "bias_board_mag",
    "mti_mag_earth": "bias_mti_mag", "board_baro": "bias_board_baro",
    "mti_baro": "bias_mti_baro",
}
FIELD = re.compile(r"^([A-Za-z][A-Za-z0-9_]*)\s*=\s*$")
NUMBER = r"[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?"
SCALE = re.compile(rf"^({NUMBER})\s*\*\s*$")
COLUMNS = re.compile(r"^Columns?\s+\d+(?:\s+through\s+\d+)?\s*$")


def numeric_tokens(line):
    tokens = line.split()
    if all(re.fullmatch(NUMBER, token) for token in tokens):
        return tokens
    # The supplied log omits separators between some %.4f matrix entries.
    # Restrict recovery to that observed format; a greedy generic float regex
    # would incorrectly read 0.12040.0000 as 0.12040 followed by .0000.
    tokens = []
    position = 0
    while position < len(line):
        if line[position].isspace():
            position += 1
            continue
        match = re.match(r"[+-]?\d+\.\d{4}", line[position:])
        if not match:
            raise ValueError(f"invalid numeric row: {line!r}")
        tokens.append(match[0])
        position += len(match[0])
    return tokens


def parse_numeric(lines):
    """Reassemble MATLAB display blocks as rows, applying the display scale."""
    blocks = []
    block = []
    scale = 1.0
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if COLUMNS.fullmatch(line):
            if block:
                blocks.append(block)
                block = []
            continue
        match = SCALE.fullmatch(line)
        if match:
            scale = float(match[1])
            continue
        tokens = numeric_tokens(line)
        row = [float(token) for token in tokens]
        if block and len(row) != len(block[0]):
            raise ValueError("inconsistent numeric row width")
        block.append(row)
    if block:
        blocks.append(block)
    if not blocks or len({len(block) for block in blocks}) != 1:
        raise ValueError("missing numeric data or inconsistent matrix block heights")
    rows = [
        [value * scale for block in blocks for value in block[i]]
        for i in range(len(blocks[0]))
    ]
    if not all(math.isfinite(value) for row in rows for value in row):
        raise ValueError("non-finite numeric value")
    return rows


def parse_log(path):
    records = []
    record = {}
    field = None
    lines = []

    def finish_field():
        if field is not None:
            try:
                record[field] = parse_numeric(lines)
            except ValueError as exc:
                raise ValueError(f"record {len(records) + 1}, field {field}: {exc}") from exc

    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        match = FIELD.fullmatch(line.strip())
        if match:
            finish_field()
            field = match[1]
            if field == "dt" and record:
                records.append(record)
                record = {}
            if not record and field != "dt":
                raise ValueError(f"line {line_number}: record must begin with dt")
            if field in record:
                raise ValueError(f"line {line_number}: duplicate field {field}")
            lines = []
        elif field is not None:
            lines.append(line)
        elif line.strip():
            raise ValueError(f"line {line_number}: content before first field")
    finish_field()
    if record:
        records.append(record)
    if not records:
        raise ValueError("no navigation records found")
    validate_records(records)
    return records


def validate_records(records):
    shapes = {"dt": 1, "flight_phase": 1, "x": 11, "P": 121}
    shapes.update({source: SENSORS[name.replace("_earth", "")]
                   for name, source in BIAS.items()})
    for sensor, size in SENSORS.items():
        shapes.update({f"{sensor}_meas": size, f"{sensor}_filt": size,
                       f"{sensor}_status": 1})
    for index, record in enumerate(records, 1):
        if record.keys() != shapes.keys():
            raise ValueError(f"record {index}: missing fields {sorted(shapes.keys() - record.keys())}; "
                             f"unexpected fields {sorted(record.keys() - shapes.keys())}")
        for name, size in shapes.items():
            rows = record[name]
            height, width = len(rows), len(rows[0])
            valid = (height, width) == (11, 11) if name == "P" else (
                (height, width) in {(size, 1), (1, size)}
            )
            if not valid:
                raise ValueError(f"record {index}, {name}: invalid shape {height}x{width}")
            if (name == "flight_phase" or name.endswith("_status")) and rows[0][0] not in (0, 1):
                raise ValueError(f"record {index}, {name}: expected boolean 0 or 1")


def edited_records(records, seed):
    """Apply the MATLAB generator's rules in the same sensor iteration order."""
    records = copy.deepcopy(records)
    rng = random.Random(seed)
    for index, record in enumerate(records, 1):
        if index % 2 == 0:
            for sensor in ("board_baro", "board_mag", "mti_baro", "mti_mag"):
                record[f"{sensor}_status"] = [[1.0]]
        for sensor in SENSORS:
            name = f"{sensor}_status"
            if record[name][0][0] and rng.random() < 0.02:
                record[name] = [[0.0]]
    return records


def numeric_initializer(rows):
    # MATLAB Coder uses column-major storage: P[row + 11 * column].
    values = [format(rows[row][column], ".17g")
              for column in range(len(rows[0])) for row in range(len(rows))]
    return values[0] if len(values) == 1 else "{ " + ", ".join(values) + " }"


def struct_initializer(record, fields):
    return "{ " + ", ".join(f".{name} = {numeric_initializer(record[source])}"
                              for name, source in fields.items()) + " }"


def render_header(records, prefix, description):
    guard = prefix.upper() + "_H"
    count = prefix.upper() + "_COUNT"
    lines = ["/* Generated by generate_navigation_testcases.py.",
             f" * {description}",
             " * Matching indices across all arrays form one navigation testcase.",
             " * P is flattened in MATLAB column-major order.",
             " * Copy x, P, bias, and sens_filt before calling navigation_codegen_entry:",
             " * the entry modifies these inputs. SD must be initialized by the caller.",
             " */", f"#ifndef {guard}", f"#define {guard}", "",
             '#include "GNC_codegen.h"', "", f"#define {count} {len(records)}", ""]
    arrays = [
        ("double", "dt", "", lambda r: numeric_initializer(r["dt"])),
        ("bool", "flight_phase", "", lambda r: "true" if r["flight_phase"][0][0] else "false"),
        ("double", "x", "[11]", lambda r: numeric_initializer(r["x"])),
        ("double", "P", "[121]", lambda r: numeric_initializer(r["P"])),
        ("struct1_T", "bias", "", lambda r: struct_initializer(r, BIAS)),
        ("struct2_T", "sens_filt", "", lambda r: struct_initializer(
            r, {sensor: f"{sensor}_filt" for sensor in SENSORS})),
        ("struct3_T", "sens_in", "", lambda r: "{ " + ", ".join(
            f".{sensor} = {{ .meas = {numeric_initializer(r[f'{sensor}_meas'])}, "
            f".status = {'true' if r[f'{sensor}_status'][0][0] else 'false'} }}"
            for sensor in SENSORS) + " }"),
    ]
    for c_type, name, dimensions, initializer in arrays:
        lines.append(f"static const {c_type} {prefix}_{name}[{count}]{dimensions} = {{")
        lines.extend(f"    {initializer(record)}," for record in records)
        lines.extend(["};", ""])
    lines.extend([f"#endif /* {guard} */", ""])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--output", type=Path, default=ROOT / "scripts/navigation_testcases.h",
                        help="header containing exact logged inputs")
    parser.add_argument("--edited-output", type=Path,
                        default=ROOT / "scripts/navigation_testcases_matlab_edited.h",
                        help="separate header containing MATLAB-style status modifications")
    parser.add_argument("--seed", type=int, default=20260716)
    parser.add_argument("--force", action="store_true", help="replace existing output headers")
    args = parser.parse_args()
    paths = [args.output.resolve(), args.edited_output.resolve()]
    if len(set(paths)) != 2 or args.log.resolve() in paths:
        parser.error("input and both output paths must be distinct")
    for path in paths:
        if path.exists() and not args.force:
            parser.error(f"output already exists: {path}; use --force to replace it")
        if not path.parent.is_dir():
            parser.error(f"output directory does not exist: {path.parent}")
    try:
        records = parse_log(args.log)
        headers = [render_header(records, "nav_testcases", "Sensor statuses preserved exactly as logged."),
                   render_header(edited_records(records, args.seed), "nav_testcases_matlab_edited",
                                 f"MATLAB-style status rules; Python random.Random seed {args.seed}, "
                                 "2% dropout. Not MATLAB's exact RNG sequence.")]
        for path, header in zip(paths, headers):
            with path.open("w" if args.force else "x") as stream:
                stream.write(header)
            print(f"Wrote {len(records)} navigation testcases to {path}")
    except (OSError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()

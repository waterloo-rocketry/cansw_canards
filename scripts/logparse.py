#!/usr/bin/env python3
import argparse
import sys
import struct
from collections import namedtuple
import csv

"""
Special values from src/application/logger/log.h
This script won't work properly if these values are inconsistent with the logger header file!
"""

# Size of each message region in data buffers (bytes)
MAX_MSG_DATA_LENGTH = 32
# Magic number encoded into message type values
LOG_DATA_MAGIC = 0x4c44
# Macro used to encode magic value into message type values
def M(v): return ((v & 0xffff) << 16) | LOG_DATA_MAGIC

# Definition of a message type's name, struct.unpack format string, and corresponding list of struct field names
Spec = namedtuple("Spec", ["name", "format", "fields"])

"""
Definition of message types and their structs.
See https://docs.python.org/3/library/struct.html#format-strings for format string syntax.

format quick reference:
- Start string with '<' for little-endian byte order
- 'L' for uint32_t (unsigned long), 'l' for int32_t (long)
- 'f' for float, 'd' for double
"""
FORMATS = {
    0x44414548: Spec("header", "<LL", ["version", "index"]),
    M(0x01): Spec("test", "<f", ["test_val"]),  
    
    M(0x02): Spec("navigator_pt1", "<fffffL",
    [
        "q_w", "q_x", "q_y", "q_z", "altitude", "norm",
    ]),
    M(0x03): Spec("navigator_pt2", "<ffffff",
    [
        "vel_x", "vel_y", "vel_z",
        "rate_x", "rate_y", "rate_z",
    ]),
    M(0x04): Spec("controller", "<fff",
    [
        "command", "roll_target", "canard_coeff",
    ]),

    M(0x05): Spec("board_imu", "<ffffff",
    [
        "accel_x", "accel_y", "accel_z",
        "gyro_x", "gyro_y", "gyro_z",
    ]),
    M(0x06): Spec("board_baro", "<Ll", ["baro", "thermometer"]),

    M(0x07): Spec("board_mag", "<ffffff",
    [
        "accel_x", "accel_y", "accel_z",
        "mag_x", "mag_y", "mag_z",
    ]),
    M(0x08): Spec("movella_pt1", "<ffffff",
    [
        "accel_x", "accel_y", "accel_z",
        "gyro_x", "gyro_y", "gyro_z",
    ]),
    M(0x09): Spec("movella_pt2", "<fffL",
    [
        "mag_x", "mag_y", "mag_z",
        "baro",
    ]),
    M(0x0A): Spec("movella_pt3", "<ffff",
    [
        "q_w", "q_x", "q_y", "q_z",
    ]),
    M(0x0B): Spec("movella_pt4", "<ffffff",
    [
        "rate_x", "rate_y", "rate_z",
        "vel_x", "vel_y", "vel_z",
    ]),
    M(0x0C): Spec("ad_accel", "<fff",
    [
        "accel_x", "accel_y", "accel_z",
    ]),
    M(0x0D): Spec("ad_gyro", "<l", ["gyro"]),

    M(0x0E): Spec("servo_motor", "<lll",
    [
        "angle", "curr", "temp",
    ]),

    # Insert new types above this line in the format:
    # M(unique_small_integer): Spec(name, format, [field, ...]),
}

def parse_argv(argv):
    p = argparse.ArgumentParser()
    p.add_argument("infile")
    return p.parse_args(argv[1:])

def main(argv=None):
    args = parse_argv(argv or sys.argv)
    with open(args.infile, "rb") as f:
        data = f.read()

    txt_filename = args.infile + "-parsed.txt"
    csv_filename = args.infile + "-parsed.csv"

    # Collect all CSV columns
    csv_columns = ["timestamp"]
    for spec in FORMATS.values():
        if spec.name == "header":
            continue
        for field in spec.fields:
            name = f"{spec.name}.{field}"
            if name not in csv_columns:
                csv_columns.append(name)

    csv_rows = []

    with open(txt_filename, "w") as txt_file:
        # Loop through message regions
        for pos in range(0, len(data), MAX_MSG_DATA_LENGTH):
            type_int, timestamp = struct.unpack_from("<LL", data, pos)

            row = {col: "" for col in csv_columns}
            # convert tenth_ms to ms
            row["timestamp"] = timestamp / 10.0

            print(f"[{timestamp}] ", end="", file=txt_file)

            try:
                spec = FORMATS[type_int]

                print(f"{spec.name} (type ", end="", file=txt_file)
                if spec.name != "header":
                    print(type_int >> 16, end="", file=txt_file)
                else:
                    print("\"HEAD\"", end="", file=txt_file)
                print(") with", file=txt_file)

                values = struct.unpack_from(spec.format, data, pos + 8)

                for field, value in zip(spec.fields, values):
                    print(f"    {field}: {value}", file=txt_file)

                    if spec.name != "header":
                        row[f"{spec.name}.{field}"] = value

            except KeyError:
                print(
                    f"unknown type 0x{type_int:08x} with\n    unknown data: {data[pos + 8:pos + MAX_MSG_DATA_LENGTH]}",
                    file=txt_file,
                )

            csv_rows.append(row)

    with open(csv_filename, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=csv_columns)
        writer.writeheader()
        writer.writerows(csv_rows)

if __name__ == "__main__":
    main()

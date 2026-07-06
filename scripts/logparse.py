#!/usr/bin/env python3
import argparse
import sys
import struct
from collections import namedtuple

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
    M(0x02): Spec("navigator_pt1", "<fffffff",
    [
        "orient_w", "orient_x", "orient_y", "orient_z",
        "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
    ]),
    M(0x03): Spec("navigator_pt2", "<ffffL",
    [
        "velocity_x", "velocity_y", "velocity_z",
        "altitude",
        "variance_norm",
    ]),

    M(0x04): Spec("controller", "<fff",
    [
        "command", "roll_target", "canard_coeff",
    ]),

    M(0x05): Spec("st_imu", "<ffffff",
    [
        "accelerometer_x", "accelerometer_y", "accelerometer_z",
        "gyroscope_x", "gyroscope_y", "gyroscope_z",
    ]),

    M(0x06): Spec("barometer", "<Ll", ["barometer", "thermometer"]),

    M(0x07): Spec("compass", "<ffffff",
    [
        "accelerometer_x", "accelerometer_y", "accelerometer_z",
        "magnetometer_x", "magnetometer_y", "magnetometer_z",
    ]),

    M(0x08): Spec("movella_pt1", "<ffffff",
    [
        "accelerometer_x", "accelerometer_y", "accelerometer_z",
        "gyroscope_x", "gyroscope_y", "gyroscope_z",
    ]),
    M(0x09): Spec("movella_pt2", "<fffL",
    [
        "magnetometer_x", "magnetometer_y", "magnetometer_z",
        "barometer",
    ]),
    M(0x0A): Spec("movella_pt3", "<ffff",
    [
        "orient_w", "orient_x", "orient_y", "orient_z",
    ]),
    M(0x0B): Spec("movella_pt4", "<ffffff",
    [
        "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
        "velocity_x", "velocity_y", "velocity_z",
    ]),

    M(0x0C): Spec("ad_accel", "<fff",
    [
        "accelerometer_x", "accelerometer_y", "accelerometer_z",
    ]),
    M(0x0D): Spec("ad_gyro", "<l", ["gyroscope"]),

    M(0x0E): Spec("encoder", "<lll",
    [
        "motor_angle", "motor_current", "motor_temperature",
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
    # Loop through message regions
    for pos in range(0, len(data), MAX_MSG_DATA_LENGTH):
        type_int, timestamp = struct.unpack_from("<LL", data, pos)
        print(f"[{timestamp}] ", end="")
        try:
            spec = FORMATS[type_int]
            print(f"{spec.name} (type ", end="")
            if spec.name != "header":
                print(type_int >> 16, end="")
            else:
                print("\"HEAD\"", end="")
            print(") with")
            values = struct.unpack_from(spec.format, data, pos + 8)
            for field, value in zip(spec.fields, values):
                print(f"    {field}: {value}")
        except KeyError:
            print(f"unknown type 0x{type_int:08x} with\n    unknown data:", data[pos + 8:pos + MAX_MSG_DATA_LENGTH])

if __name__ == "__main__":
    main()

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
    M(0x02): Spec("canard_cmd", "<ff", ["cmd_angle", "ref_signal"]),
    M(0x03): Spec("controller_input", "<fffff", ["roll_angle", "roll_rate", "canard_angle", "p_dyn", "canard_coeff"]),
    M(0x06): Spec("encoder", "<f?", ["angle", "is_dead"]),
    M(0x10): Spec("movella_pt1", "<fff",
    [
        "movella_acc_x", "movella_acc_y", "movella_acc_z",
    ]),
    M(0x11): Spec("movella_pt2", "<fff",
    [
        "movella_gyr_x", "movella_gyr_y", "movella_gyr_z",
    ]),
    M(0x12): Spec("movella_pt3", "<ffffL?",
    [
        "movella_mag_x", "movella_mag_y", "movella_mag_z",
        "movella_bar",
        "movella_time",
        "movella_is_dead",
    ]),
    M(0x13): Spec("ekf_ctx_pt1", "<fffff",
    [
        "attitude_w", "attitude_x", "attitude_y", "attitude_z",
        "altitude",
    ]),
    M(0x14): Spec("ekf_ctx_pt2", "<fffff",
    [
        "rates_x", "rates_y", "rates_z",
        "CL", "delta", 
    ]),
    M(0x15): Spec("ekf_ctx_pt3", "<ffff",
    [
        "velocity_x", "velocity_y", "velocity_z",
         "t"
    ]),
    
    M(0x16): Spec("pololu_pt1", "<fff",
    [
        "pololu_acc_x", "pololu_acc_y", "pololu_acc_z",
    ]),
    M(0x17): Spec("pololu_pt2", "<fff",
    [
        "pololu_gyr_x", "pololu_gyr_y", "pololu_gyr_z",
    ]),
    M(0x18): Spec("pololu_pt3", "<ffffL?",
    [
        "pololu_mag_x", "pololu_mag_y", "pololu_mag_z",
        "pololu_bar",
        "pololu_time",
        "pololu_is_dead",
    ]),
    M(0x19): Spec("raw_pololu_pt1", "<hhhhhh",
    [
        "acc_x", "acc_y", "acc_z",
        "gyro_x", "gyro_y", "gyro_z",
    ]),
    M(0x1A): Spec("raw_pololu_pt2", "<hhhih",
    [
        "mag_x", "mag_y", "mag_z",
        "baro_pres", "baro_temp"
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
        type_int, timestamp = struct.unpack_from("<Lf", data, pos)
        print(f"[{timestamp:.1f}] ", end="")
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

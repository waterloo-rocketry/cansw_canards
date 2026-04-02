#!/usr/bin/env python3
import argparse
import sys
import struct
import csv
import os
from collections import namedtuple, defaultdict

# Constants
MAX_MSG_DATA_LENGTH = 32
LOG_DATA_MAGIC = 0x4c44
def M(v): return ((v & 0xffff) << 16) | LOG_DATA_MAGIC

# Spec definition
Spec = namedtuple("Spec", ["name", "format", "fields"])

FORMATS = {
    0x44414548: Spec("header", "<LL", ["version", "index"]),
    M(0x01): Spec("test", "<f", ["test_val"]),  
    M(0x02): Spec("canard_cmd", "<ff", ["cmd_angle", "ref_signal"]),
    M(0x03): Spec("controller_input", "<fffff", ["roll_angle", "roll_rate", "canard_angle", "p_dyn", "canard_coeff"]),
    M(0x06): Spec("encoder", "<f", ["encoder_value"]),
    M(0x10): Spec("movella_pt1", "<fff", ["movella_acc_x", "movella_acc_y", "movella_acc_z"]),
    M(0x11): Spec("movella_pt2", "<fff", ["movella_gyr_x", "movella_gyr_y", "movella_gyr_z"]),
    M(0x12): Spec("movella_pt3", "<ffffL?", ["movella_mag_x", "movella_mag_y", "movella_mag_z", "movella_bar", "movella_time", "movella_is_dead"]),
    M(0x13): Spec("ekf_ctx_pt1", "<fffff", ["attitude_w", "attitude_x", "attitude_y", "attitude_z", "altitude"]),
    M(0x14): Spec("ekf_ctx_pt2", "<fffff", ["rates_x", "rates_y", "rates_z", "CL", "delta"]),
    M(0x15): Spec("ekf_ctx_pt3", "<fffL", ["velocity_x", "velocity_y", "velocity_z", "t_tenth_ms"]),
    M(0x16): Spec("pololu_pt1", "<fff", ["pololu_acc_x", "pololu_acc_y", "pololu_acc_z"]),
    M(0x17): Spec("pololu_pt2", "<fff", ["pololu_gyr_x", "pololu_gyr_y", "pololu_gyr_z"]),
    M(0x18): Spec("pololu_pt3", "<ffffL?", ["pololu_mag_x", "pololu_mag_y", "pololu_mag_z", "pololu_bar", "pololu_time", "pololu_is_dead"]),
    M(0x19): Spec("raw_pololu_pt1", "<hhhhhh", ["acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]),
    M(0x1A): Spec("raw_pololu_pt2", "<hhhih", ["mag_x", "mag_y", "mag_z", "baro_pres", "baro_temp"]),
}

def parse_argv(argv):
    p = argparse.ArgumentParser()
    p.add_argument("infile")
    p.add_argument("-o", "--outdir", default="log_csv_output", help="Directory for output CSV files")
    return p.parse_args(argv[1:])

def main(argv=None):
    args = parse_argv(argv or sys.argv)

    os.makedirs(args.outdir, exist_ok=True)

    csv_files = {}
    csv_writers = {}

    with open(args.infile, "rb") as f:
        data = f.read()

    for pos in range(0, len(data), MAX_MSG_DATA_LENGTH):
        try:
            type_int, timestamp = struct.unpack_from("<LL", data, pos)
            spec = FORMATS[type_int]
            values = struct.unpack_from(spec.format, data, pos + 8)
            row = {"timestamp": timestamp}
            row.update(zip(spec.fields, values))

            # Set up file and writer if not already done
            if spec.name not in csv_writers:
                filename = os.path.join(args.outdir, f"{spec.name}.csv")
                csv_file = open(filename, "w", newline="")
                csv_files[spec.name] = csv_file
                writer = csv.DictWriter(csv_file, fieldnames=["timestamp"] + spec.fields)
                writer.writeheader()
                csv_writers[spec.name] = writer

            csv_writers[spec.name].writerow(row)

        except KeyError:
            # Unknown log type
            continue
        except struct.error:
            # Partial/truncated data
            continue

    # Close all open files
    for f in csv_files.values():
        f.close()

if __name__ == "__main__":
    main()

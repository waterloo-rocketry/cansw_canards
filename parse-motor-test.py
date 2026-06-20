import re
import csv
import argparse
import os

cmd_re = re.compile(
    r"\[(\d+)\]\s+t;([-+]?\d+(?:\.\d+)?)deg(?:\s+fail:\s*\d+)?"
)

fb_re = re.compile(
    r"\[(\d+)\]\s+t;"
    r"\s*([-+]?\d+(?:\.\d+)?)deg,"
    r"\s*([-+]?\d+(?:\.\d+)?)erpm,"
    r"\s*([-+]?\d+(?:\.\d+)?)A,"
    r"\s*err\s*(-?\d+)"
)

def parse_log(input_file, output_file):
    current_cmd = 0.0
    rows = []

    with open(input_file, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:

            if "fail" in line:
                print(f"[FAIL] {line.strip()}")

            m = cmd_re.search(line)
            if m:
                current_cmd = float(m.group(2))
                continue

            m = fb_re.search(line)
            if m:
                rows.append({
                    "timestamp_ms": int(m.group(1)),
                    "cmd_deg": current_cmd,
                    "position_deg": float(m.group(2)),
                    "speed_erpm": float(m.group(3)),
                    "current_a": float(m.group(4)),
                    "fault_code": int(m.group(5)),
                })

    os.makedirs(os.path.dirname(output_file) or ".", exist_ok=True)

    with open(output_file, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "timestamp_ms",
                "cmd_deg",
                "position_deg",
                "speed_erpm",
                "current_a",
                "fault_code",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {len(rows)} rows -> {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="input log file")
    parser.add_argument("output", help="output csv file")
    args = parser.parse_args()

    parse_log(args.input, args.output)

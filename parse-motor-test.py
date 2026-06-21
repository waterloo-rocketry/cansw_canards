import re
import csv
import argparse

line_re = re.compile(
    r"\[(\d+)\]\s+t;"
    r"([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),(\d+)"
)

def main(input_file, output_file):
    rows = []

    with open(input_file, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:

            m = line_re.search(line)

            # Ignore any line that doesn't match the expected format
            if m is None:
                continue

            rows.append([
                int(m.group(1)),      # log_timestamp_ms
                float(m.group(2)),    # cmd_deg
                float(m.group(3)),    # position_deg
                float(m.group(4)),    # speed_erpm
                float(m.group(5)),    # current_a
                int(m.group(6)),      # fault_code
                int(m.group(7)),      # fb_timestamp_ms
            ])

    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "log_timestamp_ms",
            "cmd_deg",
            "position_deg",
            "speed_erpm",
            "current_a",
            "fault_code",
            "fb_timestamp_ms",
        ])

        writer.writerows(rows)

    print(f"Wrote {len(rows)} rows to {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("output")
    args = parser.parse_args()

    main(args.input, args.output)
#!/usr/bin/env python3
"""Plot the logged-data rate encoded by timestamps in a binary logger file.

Each binary log record is 32 bytes and contains a little-endian message type
followed by a uint32 timestamp in milliseconds.  The resulting rate describes
how quickly records were produced.  It does not measure the execution time of
the SD-card f_write() call.
"""

import argparse
import csv
import html
import struct
import sys
from pathlib import Path


RECORD_SIZE_BYTES = 32
HEADER_TYPE = 0x44414548
LOG_DATA_MAGIC = 0x4C44
UINT32_MODULUS = 1 << 32


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Graph binary-log data rate using its embedded timestamps."
    )
    parser.add_argument("input", type=Path, help="Binary logger file")
    parser.add_argument(
        "--window",
        type=float,
        default=1.0,
        help="Rate-bucket width in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output SVG path (default: <input>-write-speed.svg)",
    )
    parser.add_argument(
        "--csv",
        dest="csv_output",
        type=Path,
        help="Optionally save the plotted rate values as CSV",
    )
    return parser.parse_args()


def is_valid_type(message_type: int) -> bool:
    return message_type == HEADER_TYPE or (message_type & 0xFFFF) == LOG_DATA_MAGIC


def read_timestamps(path: Path) -> tuple[list[int], int, int]:
    file_size = path.stat().st_size
    complete_bytes = file_size - (file_size % RECORD_SIZE_BYTES)
    trailing_bytes = file_size - complete_bytes
    timestamps: list[int] = []
    invalid_records = 0
    pending_headers = 0

    with path.open("rb") as binary_file:
        while binary_file.tell() < complete_bytes:
            record = binary_file.read(RECORD_SIZE_BYTES)
            message_type, timestamp_ms = struct.unpack_from("<II", record)
            if not is_valid_type(message_type):
                invalid_records += 1
                continue
            if message_type == HEADER_TYPE:
                # A buffer header is timestamped when the buffer is initialized,
                # potentially long before that buffer is filled and written. Assign
                # its 32 bytes to the first data record in the same buffer so stale
                # header timestamps do not create false backward jumps or spikes.
                pending_headers += 1
                continue
            if pending_headers:
                timestamps.extend([timestamp_ms] * pending_headers)
                pending_headers = 0
            timestamps.append(timestamp_ms)

    return timestamps, invalid_records, trailing_bytes


def unwrap_timestamps(timestamps: list[int]) -> list[int]:
    if not timestamps:
        return []

    unwrapped = [timestamps[0]]
    wrap_offset = 0
    previous_raw = timestamps[0]

    for timestamp in timestamps[1:]:
        if timestamp < previous_raw:
            backward_jump = previous_raw - timestamp
            if backward_jump > UINT32_MODULUS // 2:
                wrap_offset += UINT32_MODULUS
            else:
                raise ValueError(
                    "timestamps move backward from "
                    f"{previous_raw} ms to {timestamp} ms; the file is not in "
                    "chronological order"
                )
        unwrapped.append(timestamp + wrap_offset)
        previous_raw = timestamp

    return unwrapped


def calculate_rates(
    timestamps_ms: list[int], window_seconds: float
) -> tuple[list[float], list[float], list[float]]:
    window_ms = window_seconds * 1000.0
    start_ms = timestamps_ms[0]
    bucket_counts: dict[int, int] = {}

    for timestamp_ms in timestamps_ms:
        bucket = int((timestamp_ms - start_ms) // window_ms)
        bucket_counts[bucket] = bucket_counts.get(bucket, 0) + 1

    last_bucket = max(bucket_counts)
    time_seconds: list[float] = []
    records_per_second: list[float] = []
    kib_per_second: list[float] = []

    for bucket in range(last_bucket + 1):
        count = bucket_counts.get(bucket, 0)
        time_seconds.append((bucket + 0.5) * window_seconds)
        records_per_second.append(count / window_seconds)
        kib_per_second.append(
            count * RECORD_SIZE_BYTES / 1024.0 / window_seconds
        )

    return time_seconds, records_per_second, kib_per_second


def write_csv(
    path: Path,
    time_seconds: list[float],
    records_per_second: list[float],
    kib_per_second: list[float],
) -> None:
    with path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["time_s", "records_per_s", "KiB_per_s"])
        writer.writerows(zip(time_seconds, records_per_second, kib_per_second))


def write_svg(
    path: Path,
    input_name: str,
    window_seconds: float,
    time_seconds: list[float],
    kib_per_second: list[float],
) -> None:
    width = 1200
    height = 600
    left = 90
    right = 30
    top = 60
    bottom = 70
    plot_width = width - left - right
    plot_height = height - top - bottom
    max_time = max(time_seconds) if time_seconds else 1.0
    max_rate = max(kib_per_second) if kib_per_second else 1.0
    max_time = max(max_time, 1.0)
    max_rate = max(max_rate, 1.0)

    points = " ".join(
        f"{left + time / max_time * plot_width:.2f},"
        f"{top + plot_height - rate / max_rate * plot_height:.2f}"
        for time, rate in zip(time_seconds, kib_per_second)
    )
    title = html.escape(
        f"{input_name} logged-data rate ({window_seconds:g} s buckets)"
    )
    svg: list[str] = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<g font-family="sans-serif" fill="#222">',
        f'<text x="{width / 2}" y="30" text-anchor="middle" '
        f'font-size="20">{title}</text>',
    ]

    tick_count = 5
    for tick in range(tick_count + 1):
        fraction = tick / tick_count
        x = left + fraction * plot_width
        time_label = fraction * max_time
        y = top + plot_height - fraction * plot_height
        rate_label = fraction * max_rate
        svg.extend(
            [
                f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" '
                f'y2="{top + plot_height}" stroke="#ddd"/>',
                f'<text x="{x:.2f}" y="{top + plot_height + 25}" '
                f'text-anchor="middle" font-size="12">{time_label:.1f}</text>',
                f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" '
                f'y2="{y:.2f}" stroke="#ddd"/>',
                f'<text x="{left - 10}" y="{y + 4:.2f}" text-anchor="end" '
                f'font-size="12">{rate_label:.1f}</text>',
            ]
        )

    svg.extend(
        [
            f'<line x1="{left}" y1="{top}" x2="{left}" '
            f'y2="{top + plot_height}" stroke="#222"/>',
            f'<line x1="{left}" y1="{top + plot_height}" '
            f'x2="{left + plot_width}" y2="{top + plot_height}" stroke="#222"/>',
            f'<polyline points="{points}" fill="none" stroke="#1976d2" '
            'stroke-width="1.5"/>',
            f'<text x="{left + plot_width / 2}" y="{height - 18}" '
            'text-anchor="middle" font-size="14">Time since first valid record (s)</text>',
            f'<text x="22" y="{top + plot_height / 2}" text-anchor="middle" '
            'font-size="14" transform="rotate(-90 22 '
            f'{top + plot_height / 2})">Logged data rate (KiB/s)</text>',
            '</g>',
            '</svg>',
        ]
    )
    path.write_text("\n".join(svg) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.window <= 0:
        print("error: --window must be greater than zero", file=sys.stderr)
        return 2
    if not args.input.is_file():
        print(f"error: input file does not exist: {args.input}", file=sys.stderr)
        return 2

    timestamps, invalid_records, trailing_bytes = read_timestamps(args.input)
    if not timestamps:
        print("error: no valid logger records found", file=sys.stderr)
        return 1

    try:
        timestamps = unwrap_timestamps(timestamps)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    time_seconds, records_per_second, kib_per_second = calculate_rates(
        timestamps, args.window
    )

    if args.csv_output:
        write_csv(
            args.csv_output, time_seconds, records_per_second, kib_per_second
        )

    output_path = args.output or args.input.with_name(
        f"{args.input.stem}-write-speed.svg"
    )
    write_svg(
        output_path,
        args.input.name,
        args.window,
        time_seconds,
        kib_per_second,
    )

    elapsed_seconds = (timestamps[-1] - timestamps[0]) / 1000.0
    valid_bytes = len(timestamps) * RECORD_SIZE_BYTES
    average_kib_per_second = (
        valid_bytes / 1024.0 / elapsed_seconds if elapsed_seconds > 0 else 0.0
    )
    print(f"Valid records: {len(timestamps):,}")
    print(f"Invalid records skipped: {invalid_records:,}")
    print(f"Trailing bytes skipped: {trailing_bytes}")
    print(f"Timestamp span: {elapsed_seconds:.3f} s")
    print(f"Average logged-data rate: {average_kib_per_second:.3f} KiB/s")
    print(f"Graph saved to: {output_path}")
    if args.csv_output:
        print(f"CSV saved to: {args.csv_output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

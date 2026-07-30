#!/usr/bin/env python3
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def to_numeric(series: pd.Series) -> pd.Series:
    return pd.to_numeric(series, errors="coerce")


def load_from_wide_csv(csv_path: Path) -> dict[str, pd.DataFrame]:
    df = pd.read_csv(csv_path)
    if "timestamp" not in df.columns:
        raise ValueError("CSV must contain a 'timestamp' column")

    type_map: dict[str, pd.DataFrame] = {}

    for col in df.columns:
        if col == "timestamp":
            continue

        if "." in col:
            type_name, field_name = col.split(".", 1)
        else:
            type_name, field_name = "misc", col

        sub = df[["timestamp", col]].copy()
        sub = sub.rename(columns={col: field_name})
        sub[field_name] = to_numeric(sub[field_name])

        # Keep rows where this specific type field is present.
        sub = sub.dropna(subset=[field_name])
        if sub.empty:
            continue

        if type_name not in type_map:
            type_map[type_name] = sub
        else:
            type_map[type_name] = pd.merge(
                type_map[type_name], sub, on="timestamp", how="outer", sort=True
            )

    for key in list(type_map.keys()):
        type_map[key] = type_map[key].sort_values("timestamp").reset_index(drop=True)

    return type_map


def load_from_csv_dir(csv_dir: Path) -> dict[str, pd.DataFrame]:
    type_map: dict[str, pd.DataFrame] = {}

    for csv_file in sorted(csv_dir.glob("*.csv")):
        df = pd.read_csv(csv_file)
        if "timestamp" not in df.columns:
            continue

        for col in df.columns:
            if col != "timestamp":
                df[col] = to_numeric(df[col])

        type_map[csv_file.stem] = df.sort_values("timestamp").reset_index(drop=True)

    return type_map


def get_event_times_seconds(df: pd.DataFrame, timestamp_scale: float) -> np.ndarray:
    if "timestamp" not in df.columns:
        return np.array([], dtype=float)

    ts = to_numeric(df["timestamp"]).to_numpy(dtype=float)
    ts = ts[np.isfinite(ts)]
    if ts.size == 0:
        return np.array([], dtype=float)

    # Unique timestamps define message events for this type.
    ts = np.unique(ts)
    return ts * timestamp_scale


def average_rate_in_last_window(times_s: np.ndarray, window_s: float) -> tuple[float, int, float, float]:
    if times_s.size == 0:
        return 0.0, 0, float("nan"), float("nan")

    end_t = float(times_s[-1])
    start_t = end_t - window_s
    mask = times_s >= start_t
    window_times = times_s[mask]

    event_count = int(window_times.size)
    avg_rate_hz = event_count / window_s

    return avg_rate_hz, event_count, start_t, end_t


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compute average logging rate over the trailing window (default 100 s) "
            "for each message type."
        )
    )
    parser.add_argument("input_path", help="Path to wide parsed CSV or folder of per-type CSV files")
    parser.add_argument(
        "--window-seconds",
        type=float,
        default=100.0,
        help="Trailing window length in seconds (default: 100)",
    )
    parser.add_argument(
        "--timestamp-scale",
        type=float,
        default=1e-3,
        help="Scale factor converting raw timestamps to seconds (default: 0.001 for ms->s)",
    )
    parser.add_argument(
        "--out-csv",
        default="",
        help="Optional output CSV path for the summary table",
    )
    parser.add_argument(
        "--plot-file",
        default="",
        help="Optional output PNG path for rolling average logging frequency over time",
    )
    return parser.parse_args()


def rolling_rate_over_time(times_s: np.ndarray, window_s: float) -> tuple[np.ndarray, np.ndarray]:
    if times_s.size == 0:
        return np.array([], dtype=float), np.array([], dtype=float)

    t = np.asarray(times_s, dtype=float)
    t = t[np.isfinite(t)]
    if t.size == 0:
        return np.array([], dtype=float), np.array([], dtype=float)

    t.sort()
    rates = np.zeros(t.size, dtype=float)
    left = 0

    for right in range(t.size):
        while t[right] - t[left] > window_s:
            left += 1
        count = right - left + 1
        rates[right] = count / window_s

    return t, rates


def plot_rate_over_time(
    type_times_map: dict[str, np.ndarray], window_seconds: float, out_png: Path
) -> None:
    if not type_times_map:
        raise ValueError("No message types available to plot")

    fig, ax = plt.subplots(figsize=(14, 8))

    plotted = 0
    global_t_min = None

    for type_name, times_s in sorted(type_times_map.items()):
        t, rates = rolling_rate_over_time(times_s, window_seconds)
        if t.size == 0:
            continue

        if global_t_min is None:
            global_t_min = float(np.min(t))
        else:
            global_t_min = min(global_t_min, float(np.min(t)))

        plotted += 1
        ax.plot(t, rates, linewidth=1.2, label=type_name)

    if plotted == 0:
        raise ValueError("No valid event timestamps to plot")

    if global_t_min is not None:
        ax.cla()
        plotted = 0
        for type_name, times_s in sorted(type_times_map.items()):
            t, rates = rolling_rate_over_time(times_s, window_seconds)
            if t.size == 0:
                continue
            plotted += 1
            ax.plot(t - global_t_min, rates, linewidth=1.2, label=type_name)

        if plotted == 0:
            raise ValueError("No valid event timestamps to plot")

    ax.set_xlabel("time since first event [s]")
    ax.set_ylabel("Average logging frequency [Hz]")
    ax.set_title(f"Rolling Average Logging Frequency (Trailing {window_seconds:g} s Window)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, ncol=2)

    fig.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    args = parse_args()
    input_path = Path(args.input_path)

    if input_path.is_dir():
        type_map = load_from_csv_dir(input_path)
    else:
        type_map = load_from_wide_csv(input_path)

    if not type_map:
        raise ValueError("No usable message types found in input")

    rows = []
    type_times_map: dict[str, np.ndarray] = {}
    for type_name, df in sorted(type_map.items()):
        times_s = get_event_times_seconds(df, args.timestamp_scale)
        type_times_map[type_name] = times_s
        avg_hz, count, win_start, win_end = average_rate_in_last_window(
            times_s, args.window_seconds
        )
        rows.append(
            {
                "type": type_name,
                "avg_rate_hz_last_window": avg_hz,
                "events_in_window": count,
                "window_start_s": win_start,
                "window_end_s": win_end,
            }
        )

    result_df = pd.DataFrame(rows)
    result_df = result_df.sort_values("avg_rate_hz_last_window", ascending=False)

    print("Average logging rate summary")
    print(f"Window: last {args.window_seconds:.3f} s")
    print(result_df.to_string(index=False, float_format=lambda x: f"{x:0.6f}"))

    if args.out_csv:
        out_path = Path(args.out_csv)
        result_df.to_csv(out_path, index=False)
        print(f"\nSaved summary CSV: {out_path}")

    if args.plot_file:
        plot_path = Path(args.plot_file)
        plot_rate_over_time(type_times_map, args.window_seconds, plot_path)
        print(f"Saved plot: {plot_path}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse
from pathlib import Path
from typing import NamedTuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def _to_numeric(series: pd.Series) -> pd.Series:
    return pd.to_numeric(series, errors="coerce")


def load_from_wide_csv(csv_path: Path) -> dict[str, pd.DataFrame]:
    df = pd.read_csv(csv_path)
    if "timestamp" not in df.columns:
        raise ValueError("CSV must contain a 'timestamp' column.")

    type_map: dict[str, pd.DataFrame] = {}

    for col in df.columns:
        if col == "timestamp":
            continue

        if "." in col:
            type_name, field = col.split(".", 1)
        else:
            type_name, field = "misc", col

        sub = df[["timestamp", col]].copy()
        sub = sub.rename(columns={col: field})
        sub[field] = _to_numeric(sub[field])
        sub = sub.dropna(subset=[field])

        if sub.empty:
            continue

        if type_name not in type_map:
            type_map[type_name] = sub
        else:
            type_map[type_name] = pd.merge(
                type_map[type_name], sub, on="timestamp", how="outer", sort=True
            )

    if not type_map:
        raise ValueError("No numeric value columns found in the input CSV.")

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
                df[col] = _to_numeric(df[col])

        type_name = csv_file.stem
        type_map[type_name] = df.sort_values("timestamp").reset_index(drop=True)

    if not type_map:
        raise ValueError("No valid CSV files with a 'timestamp' column found in folder.")

    return type_map


def add_time_seconds(type_map: dict[str, pd.DataFrame], timestamp_scale: float) -> None:
    global_min_ts = None
    for df in type_map.values():
        ts = _to_numeric(df["timestamp"]).to_numpy(dtype=float)
        finite = ts[np.isfinite(ts)]
        if finite.size == 0:
            continue
        local_min = float(np.min(finite))
        if (global_min_ts is None) or (local_min < global_min_ts):
            global_min_ts = local_min

    if global_min_ts is None:
        global_min_ts = 0.0

    for type_name, df in type_map.items():
        ts = _to_numeric(df["timestamp"]).to_numpy(dtype=float)
        ts = (ts - global_min_ts) * timestamp_scale
        df["time_s"] = ts
        type_map[type_name] = df


def _global_time_bounds(type_map: dict[str, pd.DataFrame]) -> tuple[float, float] | tuple[None, None]:
    tmins = []
    tmaxs = []
    for df in type_map.values():
        t = _to_numeric(df["time_s"]).to_numpy(dtype=float)
        t = t[np.isfinite(t)]
        if t.size == 0:
            continue
        tmins.append(float(np.min(t)))
        tmaxs.append(float(np.max(t)))

    if not tmins:
        return None, None

    return min(tmins), max(tmaxs)


class ChannelSpec(NamedTuple):
    type_name: str
    field_candidates: tuple[str, ...]
    label: str


class GroupSpec(NamedTuple):
    name: str
    channels: tuple[ChannelSpec, ...]


def _get_channel_series(
    type_map: dict[str, pd.DataFrame], channel: ChannelSpec
) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
    if channel.type_name not in type_map:
        return None, None

    df = type_map[channel.type_name]
    col = None
    for candidate in channel.field_candidates:
        if candidate in df.columns:
            col = candidate
            break
    if col is None:
        return None, None

    x = _to_numeric(df["time_s"]).to_numpy(dtype=float)
    y = _to_numeric(df[col]).to_numpy(dtype=float)
    mask = np.isfinite(x) & np.isfinite(y)
    if not np.any(mask):
        return None, None

    return x[mask], y[mask]


def _rolling_last_second_frequency(event_times_s: np.ndarray) -> np.ndarray:
    t = np.asarray(event_times_s, dtype=float)
    t = t[np.isfinite(t)]
    if t.size == 0:
        return np.array([], dtype=float)

    t.sort()
    hz = np.zeros_like(t)
    left = 0

    for right in range(t.size):
        while t[right] - t[left] > 1.0:
            left += 1
        hz[right] = float(right - left + 1)

    return hz


def _group_specs() -> list[GroupSpec]:
    # Groups aligned with sensor telemetry/logging paths in sensor_handler telemetry callbacks.
    return [
        GroupSpec(
            "LSM6 Acceleration",
            (
                ChannelSpec("board_imu", ("accel_x", "board_imu_accel_x"), "accel_x"),
                ChannelSpec("board_imu", ("accel_y", "board_imu_accel_y"), "accel_y"),
                ChannelSpec("board_imu", ("accel_z", "board_imu_accel_z"), "accel_z"),
            ),
        ),
        GroupSpec(
            "LSM6 Gyroscope",
            (
                ChannelSpec("board_imu", ("gyro_x", "board_imu_gyro_x"), "gyro_x"),
                ChannelSpec("board_imu", ("gyro_y", "board_imu_gyro_y"), "gyro_y"),
                ChannelSpec("board_imu", ("gyro_z", "board_imu_gyro_z"), "gyro_z"),
            ),
        ),
        GroupSpec(
            "ADXL380 Acceleration",
            (
                ChannelSpec("ad_breakout", ("accel_x", "ad_accel_x"), "accel_x"),
                ChannelSpec("ad_breakout", ("accel_y", "ad_accel_y"), "accel_y"),
                ChannelSpec("ad_breakout", ("accel_z", "ad_accel_z"), "accel_z"),
            ),
        ),
        GroupSpec(
            "ADXRS649 Gyroscope",
            (ChannelSpec("ad_breakout", ("gyro", "ad_gyro"), "gyro"),),
        ),
        GroupSpec(
            "Board Magnetometer",
            (
                ChannelSpec("board_mag_baro", ("mag_x",), "mag_x"),
                ChannelSpec("board_mag_baro", ("mag_y",), "mag_y"),
                ChannelSpec("board_mag_baro", ("mag_z",), "mag_z"),
            ),
        ),
        GroupSpec(
            "Board Barometer",
            (
                ChannelSpec("board_mag_baro", ("baro",), "baro"),
                ChannelSpec("board_mag_baro", ("temp",), "temp"),
            ),
        ),
        GroupSpec(
            "MTI Acceleration",
            (
                ChannelSpec("movella_pt1", ("accel_x", "movella_acc_x"), "accel_x"),
                ChannelSpec("movella_pt1", ("accel_y", "movella_acc_y"), "accel_y"),
                ChannelSpec("movella_pt1", ("accel_z", "movella_acc_z"), "accel_z"),
            ),
        ),
        GroupSpec(
            "MTI Gyroscope",
            (
                ChannelSpec("movella_pt1", ("gyro_x", "movella_gyr_x"), "gyro_x"),
                ChannelSpec("movella_pt1", ("gyro_y", "movella_gyr_y"), "gyro_y"),
                ChannelSpec("movella_pt1", ("gyro_z", "movella_gyr_z"), "gyro_z"),
            ),
        ),
        GroupSpec(
            "MTI Magnetometer",
            (
                ChannelSpec("movella_pt2", ("mag_x", "movella_mag_x"), "mag_x"),
                ChannelSpec("movella_pt2", ("mag_y", "movella_mag_y"), "mag_y"),
                ChannelSpec("movella_pt2", ("mag_z", "movella_mag_z"), "mag_z"),
            ),
        ),
        GroupSpec(
            "MTI Barometer",
            (ChannelSpec("movella_pt2", ("baro", "movella_bar"), "baro"),),
        ),
        GroupSpec(
            "MTI Quaternion",
            (
                ChannelSpec("movella_pt3", ("q_w", "orient_w"), "q_w"),
                ChannelSpec("movella_pt3", ("q_x", "orient_x"), "q_x"),
                ChannelSpec("movella_pt3", ("q_y", "orient_y"), "q_y"),
                ChannelSpec("movella_pt3", ("q_z", "orient_z"), "q_z"),
            ),
        ),
        GroupSpec(
            "Navigator Orientation Quaternion",
            (
                ChannelSpec("navigator_pt1", ("q_w",), "q_w"),
                ChannelSpec("navigator_pt1", ("q_x",), "q_x"),
                ChannelSpec("navigator_pt1", ("q_y",), "q_y"),
                ChannelSpec("navigator_pt1", ("q_z",), "q_z"),
            ),
        ),
        GroupSpec(
            "Navigator Altitude",
            (ChannelSpec("navigator_pt1", ("altitude",), "altitude"),),
        ),
        GroupSpec(
            "Navigator Variance Norm",
            (ChannelSpec("navigator_pt1", ("norm",), "norm"),),
        ),
        GroupSpec(
            "Navigator Velocity",
            (
                ChannelSpec("navigator_pt2", ("vel_x",), "vel_x"),
                ChannelSpec("navigator_pt2", ("vel_y",), "vel_y"),
                ChannelSpec("navigator_pt2", ("vel_z",), "vel_z"),
            ),
        ),
        GroupSpec(
            "Navigator Angular Velocity",
            (
                ChannelSpec("navigator_pt2", ("rate_x",), "rate_x"),
                ChannelSpec("navigator_pt2", ("rate_y",), "rate_y"),
                ChannelSpec("navigator_pt2", ("rate_z",), "rate_z"),
            ),
        ),
        GroupSpec(
            "Controller",
            (
                ChannelSpec("controller", ("command",), "command"),
                ChannelSpec("controller", ("canard_coeff",), "canard_coeff"),
                ChannelSpec("controller", ("body_coeff",), "body_coeff"),
                ChannelSpec("controller", ("roll_angle_target",), "roll_angle_target"),
                ChannelSpec("controller", ("roll_rate_target",), "roll_rate_target"),
            ),
        ),
        GroupSpec(
            "Servo Motor",
            (
                ChannelSpec("servo_motor", ("angle",), "angle"),
                ChannelSpec("servo_motor", ("curr",), "curr"),
                ChannelSpec("servo_motor", ("temp",), "temp"),
            ),
        ),
    ]


def _available_groups(type_map: dict[str, pd.DataFrame]) -> list[GroupSpec]:
    available: list[GroupSpec] = []
    for group in _group_specs():
        has_data = False
        for channel in group.channels:
            x, y = _get_channel_series(type_map, channel)
            if x is not None and y is not None:
                has_data = True
                break
        if has_data:
            available.append(group)
    return available


def plot_values(type_map: dict[str, pd.DataFrame]) -> plt.Figure:
    groups = _available_groups(type_map)
    if not groups:
        raise ValueError("No telemetry groups with plottable data were found.")

    fig, axes = plt.subplots(
        len(groups), 1, figsize=(15, max(3.5 * len(groups), 6)), sharex=True
    )
    if len(groups) == 1:
        axes = [axes]

    x_min, x_max = _global_time_bounds(type_map)

    for ax, group in zip(axes, groups):
        plotted = 0
        for channel in group.channels:
            x, y = _get_channel_series(type_map, channel)
            if x is None or y is None:
                continue
            ax.plot(x, y, linewidth=1.0, label=channel.label)
            plotted += 1

        ax.set_title(group.name)
        ax.set_xlabel("global time [s]")
        ax.set_ylabel("value")
        ax.grid(True, alpha=0.3)
        if (x_min is not None) and (x_max is not None):
            ax.set_xlim(x_min, x_max)
        if plotted > 0:
            ax.legend(loc="upper right", fontsize=8, ncol=2)

    fig.suptitle("Telemetry Values by Sensor Group", fontsize=14)
    fig.tight_layout()
    return fig


def plot_frequency_over_time(type_map: dict[str, pd.DataFrame]) -> plt.Figure:
    groups = _available_groups(type_map)
    if not groups:
        raise ValueError("No telemetry groups with plottable data were found.")

    fig, axes = plt.subplots(
        len(groups), 1, figsize=(15, max(3.0 * len(groups), 6)), sharex=True
    )
    if len(groups) == 1:
        axes = [axes]

    x_min, x_max = _global_time_bounds(type_map)

    for ax, group in zip(axes, groups):
        event_time_arrays = []
        for channel in group.channels:
            x, y = _get_channel_series(type_map, channel)
            if x is None or y is None:
                continue
            if x.size > 0:
                event_time_arrays.append(x)

        if event_time_arrays:
            merged_t = np.unique(np.concatenate(event_time_arrays))
            hz = _rolling_last_second_frequency(merged_t)
            ax.plot(merged_t, hz, linewidth=1.2)
        else:
            ax.text(0.5, 0.5, "No valid frequency data", ha="center", va="center")

        ax.set_title(f"{group.name} Frequency (last 1s window)")
        ax.set_xlabel("global time [s]")
        ax.set_ylabel("Hz")
        ax.grid(True, alpha=0.3)
        if (x_min is not None) and (x_max is not None):
            ax.set_xlim(x_min, x_max)

    fig.suptitle("Telemetry Frequency Over Time", fontsize=14)
    fig.tight_layout()
    return fig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot logged values and create a separate frequency section per value type. "
            "Input may be a wide CSV from logparse.py or a folder of per-type CSV files from csvparse.py."
        )
    )
    parser.add_argument("input_path", help="CSV file path or directory of CSV files")
    parser.add_argument(
        "--timestamp-scale",
        type=float,
        default=1e-3,
        help="Scale factor to convert timestamp units to seconds (default: 0.001 for ms->s)",
    )
    parser.add_argument(
        "--save-prefix",
        default="sensor_plots",
        help="Prefix for output PNG files",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Disable interactive window display",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = Path(args.input_path)

    if input_path.is_dir():
        type_map = load_from_csv_dir(input_path)
    else:
        type_map = load_from_wide_csv(input_path)

    add_time_seconds(type_map, args.timestamp_scale)

    fig_values = plot_values(type_map)
    fig_freq = plot_frequency_over_time(type_map)

    values_path = f"{args.save_prefix}_values.png"
    freq_path = f"{args.save_prefix}_frequency.png"
    fig_values.savefig(values_path, dpi=150, bbox_inches="tight")
    fig_freq.savefig(freq_path, dpi=150, bbox_inches="tight")

    print(f"Saved {values_path}")
    print(f"Saved {freq_path}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()

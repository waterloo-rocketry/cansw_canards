#!/usr/bin/env python3
"""
Plot value and frequency data for every CANARD message type found in one or
more omnibus .txt log files (the raw `['CAN/Parsley', unix_ts, {...}]` line
format used by the rocket logging tooling).

For every distinct message "type" -- identified by (msg_type, msg_metadata)
-- this produces one subplot in each of two output images:

  <prefix>_values.png     -- decoded field value(s) vs. time
  <prefix>_frequency.png  -- message rate (rolling last-1s window) vs. time

Usage:
    python plot_canard_messages.py all_messages_omnibuslogs.txt
    python plot_canard_messages.py logs/*.txt --save-prefix out/canard
    python plot_canard_messages.py log_dir/            # picks up all *.txt

Only messages with board_type_id == 'CANARD' are included.
"""
from __future__ import annotations

import argparse
import ast
import sys
from pathlib import Path
from typing import Any, NamedTuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# --------------------------------------------------------------------------
# Parsing
# --------------------------------------------------------------------------

class Message(NamedTuple):
    type_key: str       # e.g. "SENSOR_3D_ANALOG16::DEM_3D_SENSOR_CANARD_LSM6DSV32X_ACCEL"
    time: float          # seconds (from data['time'], falls back to log timestamp)
    fields: dict[str, Any]  # data dict minus 'time'


def _type_key(msg_type: Any, msg_metadata: Any) -> str:
    if msg_metadata in (None, 0, "", "0"):
        return str(msg_type)
    return f"{msg_type}::{msg_metadata}"


def _iter_log_lines(paths: list[Path]):
    for path in paths:
        with open(path, "r", errors="replace") as f:
            for line in f:
                line = line.strip()
                if line:
                    yield line


def collect_txt_files(input_paths: list[str]) -> list[Path]:
    files: list[Path] = []
    for raw in input_paths:
        p = Path(raw)
        if p.is_dir():
            files.extend(sorted(p.glob("*.txt")))
        elif p.is_file():
            files.append(p)
        else:
            raise FileNotFoundError(f"Input path does not exist: {p}")
    if not files:
        raise ValueError("No .txt log files found in the given input path(s).")
    return files


def parse_canard_messages(paths: list[Path]) -> list[Message]:
    messages: list[Message] = []
    n_lines = 0
    n_bad = 0
    n_non_canard = 0

    for line in _iter_log_lines(paths):
        n_lines += 1
        try:
            record = ast.literal_eval(line)
        except (ValueError, SyntaxError):
            n_bad += 1
            continue

        if not isinstance(record, (list, tuple)) or len(record) < 3:
            n_bad += 1
            continue

        payload = record[2]
        if not isinstance(payload, dict):
            n_bad += 1
            continue

        if payload.get("board_type_id") != "CANARD":
            n_non_canard += 1
            continue

        data = payload.get("data") or {}
        time_val = data.get("time", record[1])
        try:
            time_val = float(time_val)
        except (TypeError, ValueError):
            continue

        fields = {k: v for k, v in data.items() if k != "time"}
        type_key = _type_key(payload.get("msg_type"), payload.get("msg_metadata"))
        messages.append(Message(type_key=type_key, time=time_val, fields=fields))

    print(
        f"Parsed {n_lines} lines: {len(messages)} CANARD messages, "
        f"{n_non_canard} non-CANARD, {n_bad} unparsable/skipped."
    )
    return messages


def build_type_frames(messages: list[Message]) -> dict[str, pd.DataFrame]:
    by_type: dict[str, list[dict[str, Any]]] = {}
    for msg in messages:
        row = {"time": msg.time, **msg.fields}
        by_type.setdefault(msg.type_key, []).append(row)

    type_map: dict[str, pd.DataFrame] = {}
    for type_key, rows in by_type.items():
        df = pd.DataFrame(rows).sort_values("time").reset_index(drop=True)
        type_map[type_key] = df
    return type_map


def normalize_time(type_map: dict[str, pd.DataFrame]) -> None:
    global_min = min(df["time"].min() for df in type_map.values() if not df.empty)
    for df in type_map.values():
        df["time"] = df["time"] - global_min


# --------------------------------------------------------------------------
# Field classification (numeric vs. categorical/string payloads, e.g.
# error bitfields and severities which are strings, not numbers)
# --------------------------------------------------------------------------

def _numeric_and_categorical_fields(df: pd.DataFrame) -> tuple[list[str], list[str]]:
    numeric_cols: list[str] = []
    categorical_cols: list[str] = []
    for col in df.columns:
        if col == "time":
            continue
        try:
            pd.to_numeric(df[col])
            numeric_cols.append(col)
        except (ValueError, TypeError):
            categorical_cols.append(col)
    return numeric_cols, categorical_cols


# --------------------------------------------------------------------------
# Frequency
# --------------------------------------------------------------------------

def _rolling_last_second_frequency(event_times_s: np.ndarray) -> np.ndarray:
    t = np.asarray(event_times_s, dtype=float)
    t = t[np.isfinite(t)]
    if t.size == 0:
        return np.array([], dtype=float)

    t = np.sort(t)
    hz = np.zeros_like(t)
    left = 0
    for right in range(t.size):
        while t[right] - t[left] > 1.0:
            left += 1
        hz[right] = float(right - left + 1)
    return hz


def _global_time_bounds(type_map: dict[str, pd.DataFrame]) -> tuple[float, float] | tuple[None, None]:
    mins, maxs = [], []
    for df in type_map.values():
        t = df["time"].to_numpy(dtype=float)
        t = t[np.isfinite(t)]
        if t.size == 0:
            continue
        mins.append(float(t.min()))
        maxs.append(float(t.max()))
    if not mins:
        return None, None
    return min(mins), max(maxs)


# --------------------------------------------------------------------------
# Plotting
# --------------------------------------------------------------------------

def plot_values(type_map: dict[str, pd.DataFrame]) -> plt.Figure:
    type_keys = sorted(type_map.keys())
    fig, axes = plt.subplots(
        len(type_keys), 1, figsize=(15, max(2.6 * len(type_keys), 6)), sharex=False
    )
    if len(type_keys) == 1:
        axes = [axes]

    x_min, x_max = _global_time_bounds(type_map)

    for ax, type_key in zip(axes, type_keys):
        df = type_map[type_key]
        t = df["time"].to_numpy(dtype=float)
        numeric_cols, categorical_cols = _numeric_and_categorical_fields(df)

        for col in numeric_cols:
            y = pd.to_numeric(df[col]).to_numpy(dtype=float)
            mask = np.isfinite(t) & np.isfinite(y)
            ax.plot(t[mask], y[mask], linewidth=1.0, marker=".", markersize=2, label=col)

        legend_notes = []
        if categorical_cols:
            ax2 = ax.twinx()
            for i, col in enumerate(categorical_cols):
                codes, uniques = pd.factorize(df[col].astype(str))
                ax2.step(t, codes, where="post", linewidth=1.0, linestyle="--",
                          label=col, alpha=0.8)
                mapping = ", ".join(f"{code}={val}" for code, val in enumerate(uniques))
                legend_notes.append(f"{col}: {mapping}")
            ax2.set_ylabel("category code")
            ax2.legend(loc="upper right", fontsize=7)

        ax.set_title(type_key, fontsize=10)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("value")
        ax.grid(True, alpha=0.3)
        if (x_min is not None) and (x_max is not None):
            ax.set_xlim(x_min, x_max)
        if numeric_cols:
            ax.legend(loc="upper left", fontsize=7, ncol=3)
        if legend_notes:
            ax.text(
                0.01, -0.35, "\n".join(legend_notes),
                transform=ax.transAxes, fontsize=6.5, va="top", ha="left",
                wrap=True,
            )

    fig.suptitle("CANARD Message Values by Type", fontsize=14)
    fig.tight_layout()
    return fig


def plot_frequency(type_map: dict[str, pd.DataFrame]) -> plt.Figure:
    type_keys = sorted(type_map.keys())
    fig, axes = plt.subplots(
        len(type_keys), 1, figsize=(15, max(2.2 * len(type_keys), 6)), sharex=True
    )
    if len(type_keys) == 1:
        axes = [axes]

    x_min, x_max = _global_time_bounds(type_map)

    for ax, type_key in zip(axes, type_keys):
        t = type_map[type_key]["time"].to_numpy(dtype=float)
        hz = _rolling_last_second_frequency(t)
        if hz.size:
            ax.plot(np.sort(t), hz, linewidth=1.2)
            avg_hz = len(t) / max(t.max() - t.min(), 1e-9)
            ax.axhline(avg_hz, color="gray", linewidth=0.8, linestyle=":", alpha=0.7)
            ax.text(
                0.99, 0.9, f"avg {avg_hz:.1f} Hz  (n={len(t)})",
                transform=ax.transAxes, fontsize=7, ha="right", va="top",
            )
        else:
            ax.text(0.5, 0.5, "No data", ha="center", va="center")

        ax.set_title(type_key, fontsize=10)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("Hz")
        ax.grid(True, alpha=0.3)
        if (x_min is not None) and (x_max is not None):
            ax.set_xlim(x_min, x_max)

    fig.suptitle("CANARD Message Frequency Over Time (last 1s window)", fontsize=14)
    fig.tight_layout()
    return fig


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot per-message-type values and frequency from CANARD omnibus .txt logs."
    )
    parser.add_argument(
        "input_paths", nargs="+",
        help="One or more .txt log files and/or directories containing .txt logs",
    )
    parser.add_argument(
        "--save-prefix", default="canard",
        help="Prefix for output PNG files (default: 'canard')",
    )
    parser.add_argument(
        "--no-normalize", action="store_true",
        help="Do not shift time so the earliest message starts at t=0",
    )
    parser.add_argument(
        "--no-show", action="store_true",
        help="Disable interactive window display",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    files = collect_txt_files(args.input_paths)
    print(f"Reading {len(files)} log file(s): {', '.join(str(f) for f in files)}")

    messages = parse_canard_messages(files)
    if not messages:
        raise ValueError("No CANARD messages found in the given input file(s).")

    type_map = build_type_frames(messages)
    print(f"Found {len(type_map)} distinct CANARD message types.")

    if not args.no_normalize:
        normalize_time(type_map)

    fig_values = plot_values(type_map)
    fig_freq = plot_frequency(type_map)

    save_prefix = Path(args.save_prefix)
    values_path = Path(f"{save_prefix}_values.png")
    freq_path = Path(f"{save_prefix}_frequency.png")
    values_path.parent.mkdir(parents=True, exist_ok=True)

    fig_values.savefig(values_path, dpi=150, bbox_inches="tight")
    fig_freq.savefig(freq_path, dpi=150, bbox_inches="tight")

    print(f"Saved {values_path}")
    print(f"Saved {freq_path}")

    if not args.no_show:
        plt.show()
    else:
        plt.close(fig_values)
        plt.close(fig_freq)


if __name__ == "__main__":
    main()

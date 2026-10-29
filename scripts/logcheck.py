#!/usr/bin/env python3
"""Verify a bin file against the data-collection requirements.

Check three things:

  1. Each sensor's logging rate matches what firmware actually schedules for the
     flight phase it was recorded in.
  2. No dropped log buffers and no backward-moving timestamps anywhere in the file.
  3. No stalls/spikes in any channel's sample timing
"""
import argparse
import re
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
from logparse import FORMATS, MAX_MSG_DATA_LENGTH 
import plot_logs

HEADER_TYPE = 0x44414548

# fsm_state_t, src/common/gnc/gnc_types.h
FSM_STATES = {
    0: "IDLE",
    1: "PAD_FILTER",
    2: "PAD_NAV",
    3: "BOOST",
    4: "ACT_ALLOWED",
    5: "RECOVERY",
    6: "SLEEPY",
    7: "ERROR",
}

HIGH_RATE_HZ = 200

EXPECTED_RATE_HZ = {
    # sensor_high_rate_sd_log(): LSM6 accel/gyro, AD accel/gyro, MTI accel/gyro
    "board_imu": {
        "IDLE": 1, "PAD_FILTER": 20, "PAD_NAV": HIGH_RATE_HZ, "BOOST": HIGH_RATE_HZ,
        "ACT_ALLOWED": HIGH_RATE_HZ, "RECOVERY": 20, "SLEEPY": 1,
    },
    "ad_breakout": {
        "IDLE": 1, "PAD_FILTER": 20, "PAD_NAV": HIGH_RATE_HZ, "BOOST": HIGH_RATE_HZ,
        "ACT_ALLOWED": HIGH_RATE_HZ, "RECOVERY": 20, "SLEEPY": 1,
    },
    "movella_pt1": {
        "IDLE": 1, "PAD_FILTER": 20, "PAD_NAV": HIGH_RATE_HZ, "BOOST": HIGH_RATE_HZ,
        "ACT_ALLOWED": HIGH_RATE_HZ, "RECOVERY": 20, "SLEEPY": 1,
    },
    # sensor_low_rate_sd_log(): board mag/baro/temp, MTI mag/baro
    "board_mag_baro": {
        "IDLE": 1, "PAD_FILTER": 20, "PAD_NAV": 50, "BOOST": 50,
        "ACT_ALLOWED": 50, "RECOVERY": 20, "SLEEPY": 1,
    },
    "movella_pt2": {
        "IDLE": 1, "PAD_FILTER": 20, "PAD_NAV": 50, "BOOST": 50,
        "ACT_ALLOWED": 50, "RECOVERY": 20, "SLEEPY": 1,
    },
    # movella_state_sd_log(): MTI orientation quaternion
    "movella_pt3": {
        "IDLE": 1, "PAD_FILTER": 20, "PAD_NAV": 20, "BOOST": 20,
        "ACT_ALLOWED": 20, "RECOVERY": 20, "SLEEPY": 1,
    },
    # nav_sd_telemetry(): not registered for STATE_IDLE at all
    "navigator_pt1": {
        "IDLE": 0, "PAD_FILTER": 20, "PAD_NAV": HIGH_RATE_HZ, "BOOST": HIGH_RATE_HZ,
        "ACT_ALLOWED": HIGH_RATE_HZ, "RECOVERY": 20, "SLEEPY": 1,
    },
    "navigator_pt2": {
        "IDLE": 0, "PAD_FILTER": 20, "PAD_NAV": HIGH_RATE_HZ, "BOOST": HIGH_RATE_HZ,
        "ACT_ALLOWED": HIGH_RATE_HZ, "RECOVERY": 20, "SLEEPY": 1,
    },
    # ctrl_sd_telemetry(): not registered for STATE_IDLE or STATE_PAD_FILTER
    "controller": {
        "IDLE": 0, "PAD_FILTER": 0, "PAD_NAV": HIGH_RATE_HZ, "BOOST": HIGH_RATE_HZ,
        "ACT_ALLOWED": HIGH_RATE_HZ, "RECOVERY": 20, "SLEEPY": 1,
    },
    # ak45_sd_telemetry(): angle/current/temp all logged together in one record
    "servo_motor": {
        "IDLE": 1, "PAD_FILTER": 10, "PAD_NAV": HIGH_RATE_HZ, "BOOST": HIGH_RATE_HZ,
        "ACT_ALLOWED": HIGH_RATE_HZ, "RECOVERY": 10, "SLEEPY": 1,
    },
    # "test" and "header" are not sensor channels; no rate expectation.
}

# Tolerance for throttled tiers before flagging FAIL.
RATE_FAIL_FRACTION = 0.9  # below 90% of expected -> FAIL

# A rolling 1-second rate above this multiple of the expected/typical rate (or
# expected+SPIKE_RATE_FLOOR_HZ, whichever is higher) is a timing spike: samples
# arriving in a burst rather than at a steady cadence.
SPIKE_RATE_MULTIPLIER = 2.5
SPIKE_RATE_FLOOR_HZ = 5.0

TRANSITION_RE = re.compile(
    r"\[(\d+)\]\s*INFO;\s*FlightPhase;\s*State transition:\s*(\d+)\s*->\s*(\d+)"
)


def parse_bin(path: Path) -> dict[str, pd.DataFrame]:
    """Directly decode a .BIN log into one DataFrame per message type (incl. header)."""
    data = path.read_bytes()
    rows: dict[str, list[dict]] = {}
    unknown_records = 0
    trailing_bytes = len(data) % MAX_MSG_DATA_LENGTH

    for pos in range(0, len(data) - trailing_bytes, MAX_MSG_DATA_LENGTH):
        type_int, timestamp = struct.unpack_from("<LL", data, pos)
        spec = FORMATS.get(type_int)
        if spec is None:
            unknown_records += 1
            continue

        row = {"timestamp": timestamp}
        values = struct.unpack_from(spec.format, data, pos + 8)
        row.update(zip(spec.fields, values))
        rows.setdefault(spec.name, []).append(row)

    if unknown_records:
        print(f"warning: skipped {unknown_records} record(s) with an unrecognized type", file=sys.stderr)
    if trailing_bytes:
        print(f"warning: {trailing_bytes} trailing byte(s) did not form a complete record", file=sys.stderr)

    type_map = {
        name: pd.DataFrame(type_rows).sort_values("timestamp").reset_index(drop=True)
        for name, type_rows in rows.items()
    }
    return type_map


def find_companion_textlog(bin_path: Path) -> Path | None:
    candidate = bin_path.with_suffix(".TXT")
    if candidate.is_file():
        return candidate
    candidate = bin_path.with_suffix(".txt")
    if candidate.is_file():
        return candidate
    return None


def parse_phase_intervals(
    textlog_path: Path, ts_min: int, ts_max: int
) -> list[tuple[int, int, str]]:
    """Return [(start_ts, end_ts, state_name), ...] spanning [ts_min, ts_max]."""
    transitions: list[tuple[int, int]] = []
    with textlog_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = TRANSITION_RE.search(line)
            if not m:
                continue
            ts, _curr, new_state = int(m.group(1)), int(m.group(2)), int(m.group(3))
            transitions.append((ts, new_state))

    transitions.sort(key=lambda t: t[0])

    intervals: list[tuple[int, int, str]] = []
    state_id = 0  # STATE_IDLE at boot
    boundary = ts_min
    for ts, new_state in transitions:
        if ts <= ts_min:
            state_id = new_state
            continue
        if ts >= ts_max:
            break
        intervals.append((boundary, ts, FSM_STATES.get(state_id, f"UNKNOWN({state_id})")))
        boundary = ts
        state_id = new_state

    intervals.append((boundary, ts_max, FSM_STATES.get(state_id, f"UNKNOWN({state_id})")))
    return intervals


@dataclass
class Finding:
    level: str  # "FAIL", "WARN", or "INFO"
    message: str


def check_dropped_buffers(type_map: dict[str, pd.DataFrame]) -> list[Finding]:
    findings: list[Finding] = []
    header = type_map.get("header")
    if header is None or header.empty:
        findings.append(Finding("WARN", "No header records found; cannot check for dropped buffers."))
        return findings

    idx = header.sort_values("timestamp")["index"].to_numpy(dtype=np.int64)
    gaps = np.where(np.diff(idx) != 1)[0]
    if gaps.size == 0:
        findings.append(Finding("INFO", f"Buffer index is gapless across {len(idx)} buffers (no dropped buffers)."))
    else:
        for gap_pos in gaps[:10]:
            before, after = idx[gap_pos], idx[gap_pos + 1]
            findings.append(
                Finding("FAIL", f"Dropped buffer(s): index jumped from {before} to {after} (missing {after - before - 1}).")
            )
        if gaps.size > 10:
            findings.append(Finding("FAIL", f"...and {gaps.size - 10} more buffer index gap(s)."))
    return findings


def check_backward_timestamps(type_map: dict[str, pd.DataFrame]) -> list[Finding]:
    findings: list[Finding] = []
    for type_name, df in type_map.items():
        ts = df["timestamp"].to_numpy(dtype=np.int64)
        backward = np.where(np.diff(ts) < 0)[0]
        if backward.size:
            first = backward[0]
            findings.append(
                Finding(
                    "FAIL",
                    f"{type_name}: timestamp moves backward from {ts[first]} to {ts[first + 1]} "
                    f"({backward.size} such jump(s) total).",
                )
            )
    return findings


def check_timing_gaps(type_map: dict[str, pd.DataFrame], phases: list[tuple[int, int, str]]) -> list[Finding]:
    """Flag any inter-sample gap much larger than the expected period for its phase, or a
    fallback of the channel's own median period if no phase info is available."""
    findings: list[Finding] = []

    for type_name, df in type_map.items():
        if type_name in ("header", "test") or len(df) < 3:
            continue

        ts = df["timestamp"].to_numpy(dtype=np.int64)
        deltas = np.diff(ts).astype(np.float64)
        rates = EXPECTED_RATE_HZ.get(type_name)

        # Fallback threshold
        fallback_ms = max(5 * float(np.median(deltas)), 50.0)
        thresholds = np.full(deltas.shape, fallback_ms)

        if rates and phases:
            sample_ts = ts[:-1]  # timestamp each delta is measured *from*
            for start, end, phase_name in phases:
                expected = rates.get(phase_name)
                if not (isinstance(expected, (int, float)) and expected > 0):
                    continue
                mask = (sample_ts >= start) & (sample_ts < end)
                thresholds[mask] = max(3000.0 / expected, 50.0)

        bad = np.where(deltas > thresholds)[0]
        if bad.size:
            worst_idx = bad[np.argmax(deltas[bad])]
            findings.append(
                Finding(
                    "FAIL" if bad.size > 3 else "WARN",
                    f"{type_name}: {bad.size} timing gap(s) found; worst is {deltas[worst_idx]:.0f} ms "
                    f"at timestamp {ts[worst_idx]} (threshold {thresholds[worst_idx]:.0f} ms).",
                )
            )

    return findings


def check_timing_spikes(type_map: dict[str, pd.DataFrame], phases: list[tuple[int, int, str]]) -> list[Finding]:
    """Flag bursts: windows where a channel's rolling 1-second rate jumps well above
    what's expected for its phase, i.e. samples arriving in a clump and then a lull,
    rather than at a steady cadence. The mirror image of check_timing_gaps().

    The rolling rate is computed independently per phase segment (not across the
    whole channel) so a downward rate step at a phase boundary -- e.g. BOOST's 200 Hz
    dropping to RECOVERY's 20 Hz -- doesn't get misread as a spike: a global rolling
    window would still be full of the *previous* phase's faster samples for up to a
    second after the boundary, which is a windowing artifact, not a real burst.
    """
    findings: list[Finding] = []

    for type_name, df in type_map.items():
        if type_name in ("header", "test") or len(df) < 3:
            continue

        ts = df["timestamp"].to_numpy(dtype=np.int64)
        rates = EXPECTED_RATE_HZ.get(type_name)

        segments: list[tuple[np.ndarray, float | None]] = []
        if phases:
            for start, end, phase_name in phases:
                seg_ts = ts[(ts >= start) & (ts < end)]
                if seg_ts.size < 2:
                    continue
                expected = rates.get(phase_name) if rates else None
                segments.append((seg_ts, expected if isinstance(expected, (int, float)) and expected > 0 else None))
        else:
            segments = [(ts, None)]

        bad_ts: list[np.ndarray] = []
        bad_hz: list[np.ndarray] = []
        bad_expected: list[np.ndarray] = []

        for seg_ts, expected in segments:
            if expected is None:
                # No usable phase rate for this segment: fall back to the segment's
                # own typical period.
                median_delta_ms = float(np.median(np.diff(seg_ts).astype(np.float64)))
                expected = 1000.0 / median_delta_ms if median_delta_ms > 0 else 0.0

            rolling_hz = plot_logs._rolling_last_second_frequency(seg_ts.astype(np.float64) / 1000.0)
            threshold = max(expected * SPIKE_RATE_MULTIPLIER, expected + SPIKE_RATE_FLOOR_HZ)

            bad = np.where(rolling_hz > threshold)[0]
            if bad.size:
                bad_ts.append(seg_ts[bad])
                bad_hz.append(rolling_hz[bad])
                bad_expected.append(np.full(bad.size, expected))

        if bad_ts:
            all_ts = np.concatenate(bad_ts)
            all_hz = np.concatenate(bad_hz)
            all_expected = np.concatenate(bad_expected)
            worst = np.argmax(all_hz)
            findings.append(
                Finding(
                    "FAIL" if all_ts.size > 3 else "WARN",
                    f"{type_name}: {all_ts.size} timing spike sample(s) found; worst is "
                    f"{all_hz[worst]:.0f} Hz (rolling 1s window) at timestamp {all_ts[worst]} "
                    f"(expected ~{all_expected[worst]:.0f} Hz).",
                )
            )

    return findings


def check_phase_rates(
    type_map: dict[str, pd.DataFrame], phases: list[tuple[int, int, str]]
) -> list[Finding]:
    findings: list[Finding] = []
    if not phases:
        findings.append(Finding("WARN", "No flight-phase text log available; skipping per-phase rate checks."))
        return findings

    for type_name, expected_by_phase in EXPECTED_RATE_HZ.items():
        df = type_map.get(type_name)
        ts = df["timestamp"].to_numpy(dtype=np.int64) if df is not None else np.array([], dtype=np.int64)

        for start, end, phase_name in phases:
            expected = expected_by_phase.get(phase_name)
            duration_s = (end - start) / 1000.0
            if duration_s <= 0:
                continue
            count = int(np.sum((ts >= start) & (ts < end)))
            observed_hz = count / duration_s

            if expected == 0:
                if count > 0:
                    findings.append(
                        Finding(
                            "WARN",
                            f"{type_name} in {phase_name}: {count} record(s) logged but firmware "
                            f"does not schedule this channel during {phase_name}.",
                        )
                    )
                continue

            if expected is None:
                continue  # channel not expected to exist in this phase table at all

            if observed_hz < expected * RATE_FAIL_FRACTION:
                findings.append(
                    Finding(
                        "FAIL",
                        f"{type_name} in {phase_name}: {observed_hz:.1f} Hz observed, expected ~{expected} Hz.",
                    )
                )
            else:
                findings.append(
                    Finding("INFO", f"{type_name} in {phase_name}: {observed_hz:.1f} Hz observed (expected ~{expected} Hz).")
                )

    return findings


def _shade_phases(fig, phases: list[tuple[int, int, str]], ts_min: int) -> None:
    if not phases:
        return
    colors = ["#00000010", "#00000000"]
    for i, (start, end, name) in enumerate(phases):
        start_s = (start - ts_min) / 1000.0
        end_s = (end - ts_min) / 1000.0
        for ax in fig.axes:
            ax.axvspan(start_s, end_s, color=colors[i % 2], zorder=0)
        if fig.axes:
            fig.axes[0].text(
                (start_s + end_s) / 2, fig.axes[0].get_ylim()[1], name,
                ha="center", va="bottom", fontsize=7, rotation=0, clip_on=False,
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path, help="Binary logger .BIN file")
    parser.add_argument("--textlog", type=Path, help="Companion .TXT log (default: same name next to the .BIN)")
    parser.add_argument("--no-plots", action="store_true", help="Skip generating value/frequency plots")
    parser.add_argument("--save-prefix", help="Prefix for output PNGs (default: next to the input file)")
    parser.add_argument("--no-show", action="store_true", help="Don't open plot windows, just save PNGs")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.input.is_file():
        print(f"error: input file does not exist: {args.input}", file=sys.stderr)
        return 2

    type_map = parse_bin(args.input)
    if not type_map:
        print("error: no recognized records found in file", file=sys.stderr)
        return 2

    all_ts = np.concatenate([df["timestamp"].to_numpy(dtype=np.int64) for df in type_map.values()])
    ts_min, ts_max = int(all_ts.min()), int(all_ts.max())

    textlog_path = args.textlog or find_companion_textlog(args.input)
    phases: list[tuple[int, int, str]] = []
    if textlog_path and textlog_path.is_file():
        phases = parse_phase_intervals(textlog_path, ts_min, ts_max)
        print(f"Using flight phases from {textlog_path}:")
        for start, end, name in phases:
            print(f"  {name:12s} [{start:>10d}, {end:>10d}) ms  ({(end - start) / 1000.0:.1f} s)")
    else:
        print("No companion .TXT log found/given; per-phase rate checks will be skipped.")
    print()

    findings: list[Finding] = []
    findings += check_dropped_buffers(type_map)
    findings += check_backward_timestamps(type_map)
    findings += check_timing_gaps(type_map, phases)
    findings += check_timing_spikes(type_map, phases)
    findings += check_phase_rates(type_map, phases)

    fails = [f for f in findings if f.level == "FAIL"]
    warns = [f for f in findings if f.level == "WARN"]
    infos = [f for f in findings if f.level == "INFO"]

    for f in fails + warns:
        print(f"[{f.level}] {f.message}")
    print()
    print(f"{len(fails)} FAIL, {len(warns)} WARN, {len(infos)} informational")

    if not args.no_plots:
        plot_map = {name: df.copy() for name, df in type_map.items() if name != "header"}
        plot_logs.add_time_seconds(plot_map, 1e-3)
        fig_values = plot_logs.plot_values(plot_map)
        fig_freq = plot_logs.plot_frequency_over_time(plot_map)
        _shade_phases(fig_values, phases, ts_min)
        _shade_phases(fig_freq, phases, ts_min)

        save_prefix = Path(args.save_prefix) if args.save_prefix else args.input.with_suffix("")
        values_path = Path(f"{save_prefix}_values.png")
        freq_path = Path(f"{save_prefix}_frequency.png")
        fig_values.savefig(values_path, dpi=150, bbox_inches="tight")
        fig_freq.savefig(freq_path, dpi=150, bbox_inches="tight")
        print(f"Saved {values_path}")
        print(f"Saved {freq_path}")

        if not args.no_show:
            import matplotlib.pyplot as plt
            plt.show()

    if fails:
        return 2
    if warns:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

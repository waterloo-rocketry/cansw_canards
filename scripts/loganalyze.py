#!/usr/bin/env python3

import argparse
import os
import re
import sys
from dataclasses import dataclass, field

HEARTBEAT_PERIOD_MS = 1000 # health check task runs at 1 hz
GAP_FACTOR = 2.0  # gap of more than heartbeat period x2 is considered an issue

INIT_SOURCES = {"init", "SystemInit"}
INIT_SUCCESS_SOURCE = "SystemInit"
INIT_SUCCESS_SUBSTR = "All tasks created successfully"

MODULE_ID_NAMES = {
    0: "MODULE_ADC",
    1: "MODULE_ADXL380",
    2: "MODULE_ADXRS649",
    3: "MODULE_AK45",
    4: "MODULE_CAN_HANDLER",
    5: "MODULE_CONTROLLER",
    6: "MODULE_FLIGHT_PHASE",
    7: "MODULE_FSM",
    8: "MODULE_GPIO",
    9: "MODULE_I2C",
    10: "MODULE_IIS2MDC",
    11: "MODULE_LOGGER",
    12: "MODULE_LSM6DSV32X",
    13: "MODULE_MOVELLA",
    14: "MODULE_MS5611",
    15: "MODULE_NAVIGATOR",
    16: "MODULE_POWER_HANDLER",
    17: "MODULE_SD_CARD",
    18: "MODULE_SENSOR_HANDLER",
    19: "MODULE_TELEMETRY",
    20: "MODULE_TIMER",
    21: "MODULE_UART",
}
HEALTH_SEVERITY_NAMES = {0: "HEALTH_OK", 1: "HEALTH_ERROR", 2: "HEALTH_FATAL"}

MODULE_ERROR_CODE_NAMES = {
    0: "ERR_BAT1_FAULT",
    1: "ERR_BAT2_FAULT",
    2: "ERR_DEVICE_FAULT",
    3: "ERR_FILE_SYSTEM",
    4: "ERR_HARDWARE_FAIL",
    5: "ERR_LOW_POWER_MODE_WITH_EXT_5V_ON",
    6: "ERR_COMM_FAILURE",
    7: "ERR_CRC_FAILED",
    8: "ERR_NO_DATA",
    9: "ERR_RX_FAILURE",
    10: "ERR_TIMEOUT",
    11: "ERR_TX_FAILURE",
    12: "ERR_ERROR_STATE",
    13: "ERR_FAILED_CALIBRATION",
    14: "ERR_NOT_CALIBRATED",
    15: "ERR_LOOP_TIMING",
    16: "ERR_NOT_INIT",
    17: "ERR_OS",
    18: "ERR_CODEGEN",
    19: "ERR_UNEXPECTED_EVENT",
    20: "ERR_INVALID_PARAM",
    21: "ERR_MATH",
    22: "ERR_OUT_OF_BOUNDS",
    23: "ERR_OVERFLOW",
    24: "ERR_INTERNAL",
}
NOT_INIT_BIT = 16  # ERR_NOT_INIT (CANARDS_MODULE_E_NOT_INIT_OFFSET)

@dataclass
class Section:
    """One report section: a title, body lines, failure reasons (fail the
    run), and notes (informational, don't fail the run).
    """

    title: str
    lines: list = field(default_factory=list)
    failures: list = field(default_factory=list)
    notes: list = field(default_factory=list)

@dataclass
class TxtLine:
    timestamp: int
    level: str
    source: str
    message: str
    truncated: bool

_LINE_RE = re.compile(r"^\[(\d+)\]\s*(\w+);\s*([^;]*);\s*(.*)$")

def parse_txt(text):
    """Parse text-log lines"""
    parsed = []
    for raw in text.splitlines():
        line = raw.rstrip("\x00").rstrip()
        if not line:
            continue
        truncated = line.startswith("!")
        m = _LINE_RE.match(line[1:] if truncated else line)
        if not m:
            # Keep unparseable non-empty lines visible but don't crash.
            parsed.append(TxtLine(-1, "?", "?", line, truncated))
            continue
        parsed.append(
            TxtLine(
                timestamp=int(m.group(1)),
                level=m.group(2),
                source=m.group(3).strip(),
                message=m.group(4).strip(),
                truncated=truncated,
            )
        )
    return parsed

def _gaps(timestamps):
    ts = sorted(timestamps)
    return [ts[i] - ts[i - 1] for i in range(1, len(ts))]

def _decode_module_status(message):
    """Turn 'module=%d: sev=%d, err=%d' into readable names"""
    m = re.search(r"module=(\d+):\s*sev=(\d+),\s*err=(\d+)", message)
    if not m:
        return None
    mod, sev, err = int(m.group(1)), int(m.group(2)), int(m.group(3))
    mod_name = MODULE_ID_NAMES.get(mod, f"module {mod}")
    sev_name = HEALTH_SEVERITY_NAMES.get(sev, f"sev {sev}")
    bits = [
        MODULE_ERROR_CODE_NAMES.get(b, f"bit{b}")
        for b in range(32)
        if err & (1 << b)
    ]
    err_str = "|".join(bits) if bits else "0"
    is_not_init = bool(err & (1 << NOT_INIT_BIT))
    return mod_name, sev_name, err_str, is_not_init

def _effective_level(ln):
    """Text log level, promoted to FATAL if the decoded module status is
    HEALTH_FATAL. health_checks.c's process_module_status() logs even a
    HEALTH_FATAL-severity module status at text level WARN, then calls
    proc_handle_fatal_error() (which halts/resets the board) without logging
    anything further -- so without this promotion, the one line marking a
    genuinely fatal condition would never be classified as FATAL.
    """
    decoded = _decode_module_status(ln.message)
    if decoded and decoded[1] == "HEALTH_FATAL":
        return "FATAL"
    return ln.level

def check_txt(parsed):
    """Scan for FATAL/WARN lines, decode health module-status lines, flag
    init-related failures, and note truncated lines (truncation alone
    doesn't fail the run).
    """
    err_sec = Section("TXT ERRORS")
    fatals, warns, init_fails, truncs = [], [], [], []

    for ln in parsed:
        if ln.truncated:
            truncs.append(ln)

        decoded = _decode_module_status(ln.message)
        detail = ""
        level = ln.level
        if decoded:
            mod_name, sev_name, err_str, is_not_init = decoded
            detail = f"  -> {mod_name} {sev_name} err={err_str}"
            if is_not_init:
                init_fails.append((ln, mod_name))
            if sev_name == "HEALTH_FATAL":
                level = "FATAL"

        if ln.source in INIT_SOURCES and ln.level in ("WARN", "FATAL"):
            init_fails.append((ln, ln.source))

        if level == "FATAL":
            fatals.append((ln, detail))
        elif level == "WARN":
            warns.append((ln, detail))

    if fatals:
        err_sec.lines.append(f"{len(fatals)} FATAL line(s):")
        for ln, detail in fatals:
            err_sec.lines.append(f"    [{ln.timestamp}] {ln.source}; {ln.message}{detail}")
            err_sec.failures.append(f"FATAL @ {ln.timestamp}: {ln.source}; {ln.message}")
    if warns:
        err_sec.lines.append(f"{len(warns)} WARN line(s):")
        for ln, detail in warns:
            err_sec.lines.append(f"    [{ln.timestamp}] {ln.source}; {ln.message}{detail}")
    if init_fails:
        # Collapse repeats of the same source/module into a single summary.
        groups = {}
        for ln, who in init_fails:
            groups.setdefault(who, []).append(ln)

        err_sec.lines.append(
            f"{len(init_fails)} init-failure line(s) across {len(groups)} source(s):"
        )
        for who, lns in groups.items():
            lns.sort(key=lambda l: l.timestamp)
            first, last = lns[0], lns[-1]
            if len(lns) == 1:
                err_sec.lines.append(f"    [{first.timestamp}] {who}: {first.message}")
                err_sec.failures.append(f"init failure @ {first.timestamp}: {who}; {first.message}")
            else:
                err_sec.lines.append(
                    f"    {who}: {len(lns)} occurrence(s), {first.timestamp}..{last.timestamp} ms "
                    f"(latest: {last.message})"
                )
                err_sec.failures.append(
                    f"init failure: {who} not initialized "
                    f"({len(lns)} occurrences, {first.timestamp}..{last.timestamp} ms)"
                )
    if truncs:
        err_sec.lines.append(f"{len(truncs)} truncated line(s) (leading '!')")
        err_sec.notes.append(f"{len(truncs)} truncated text log line(s)")
    if not err_sec.lines:
        err_sec.lines.append("no FATAL/WARN/init-failure/truncated lines")
    return err_sec

def _decorated_line(ln):
    decoded = _decode_module_status(ln.message)
    detail = f"  -> {decoded[0]} {decoded[1]} err={decoded[2]}" if decoded else ""
    return f"[{ln.timestamp}] {ln.source}; {ln.message}{detail}"

def check_fatal_errors(parsed):
    """List FATAL log lines, including module-status lines decoded as
    HEALTH_FATAL even though those are logged at text level WARN.
    """
    sec = Section("FATAL ERRORS")
    fatals = [ln for ln in parsed if _effective_level(ln) == "FATAL"]
    if not fatals:
        sec.lines.append("no FATAL lines")
    sec.lines.extend(_decorated_line(ln) for ln in fatals)
    return sec

def check_warnings(parsed):
    """List WARN log lines (excludes HEALTH_FATAL module-status lines,
    which check_fatal_errors treats as FATAL despite their text level).
    """
    sec = Section("WARNINGS")
    warns = [ln for ln in parsed if _effective_level(ln) == "WARN"]
    if not warns:
        sec.lines.append("no WARN lines")
    sec.lines.extend(_decorated_line(ln) for ln in warns)
    return sec

def check_truncated(parsed):
    """List the truncated log lines"""
    sec = Section("TRUNCATED")
    truncs = [ln for ln in parsed if ln.truncated]
    if not truncs:
        sec.lines.append("no truncated lines")
    sec.lines.extend(_decorated_line(ln) for ln in truncs)
    return sec

def check_init_completion(parsed):
    """system_init_task logs 'All tasks created successfully.' from source 'SystemInit'"""
    sec = Section("INIT COMPLETION")
    done = [
        ln
        for ln in parsed
        if ln.source == INIT_SUCCESS_SOURCE
        and ln.level == "INFO"
        and INIT_SUCCESS_SUBSTR in ln.message
    ]
    if done:
        sec.lines.append(f"system init completed @ {done[0].timestamp} ms")
    else:
        sec.lines.append(f"no '{INIT_SUCCESS_SUBSTR}' line found")
        sec.failures.append("system init never completed (no SystemInit success line in log)")
    return sec

def check_heartbeat(parsed):
    """Check for heartbeat gaps"""
    sec = Section("HEARTBEAT")
    beats = [
        ln.timestamp
        for ln in parsed
        if ln.level == "INFO"
        and ln.source == "logger"
        and ln.message.startswith("init=")
        and ln.timestamp >= 0
    ]
    if not beats:
        sec.lines.append("no logger heartbeat lines found")
        sec.failures.append("no logger heartbeat found in text log")
        return sec

    beats.sort()
    gaps = _gaps(beats)
    limit = HEARTBEAT_PERIOD_MS * GAP_FACTOR
    big = [(beats[i], gaps[i - 1]) for i in range(1, len(beats)) if gaps[i - 1] > limit]
    sec.lines.append(
        f"{len(beats)} heartbeat(s), span {beats[0]}..{beats[-1]} ms, "
        f"max gap {max(gaps) if gaps else 0} ms"
    )
    for ts, g in big:
        sec.failures.append(
            f"heartbeat gap {g} ms > {limit:.0f} ms ending @ {ts} (possible stall)"
        )
    if not big:
        sec.lines.append("Gap within tolerance")
    return sec

def analyze(txt_path):
    """Run all checks against a text log and return (sections, ok)."""
    with open(txt_path, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()
    parsed = parse_txt(text)

    summary = Section("TXT SUMMARY")
    real = [ln for ln in parsed if ln.timestamp >= 0]
    if real:
        summary.lines.append(
            f"{txt_path}: {len(parsed)} line(s), span {real[0].timestamp}..{real[-1].timestamp} ms"
        )
    else:
        summary.lines.append(f"{txt_path}: {len(parsed)} line(s)")

    sections = [
        summary,
        check_fatal_errors(parsed),
        check_warnings(parsed),
        check_truncated(parsed),
        check_txt(parsed),
        check_init_completion(parsed),
        check_heartbeat(parsed),
    ]

    ok = not any(sec.failures for sec in sections)
    return sections, ok

DEFAULT_SHOWN_SECTIONS = {"FATAL ERRORS", "INIT COMPLETION", "HEARTBEAT"}

def format_report(sections, ok, include_warns=False, truncated_only=False):
    shown = set(DEFAULT_SHOWN_SECTIONS)
    if include_warns:
        shown.add("WARNINGS")
    if truncated_only:
        shown.add("TRUNCATED")

    out = []
    for sec in sections:
        if sec.title not in shown:
            continue
        out.append(f"=== {sec.title} ===")
        out.extend(f"  {line}" for line in sec.lines)
        out.append("")
    out.append("=== RESULT ===")
    if ok:
        out.append("  PASS")
    else:
        out.append("  FAIL")
        for sec in sections:
            for reason in sec.failures:
                out.append(f"  - [{sec.title}] {reason}")
    for sec in sections:
        for note in sec.notes:
            out.append(f"  * [{sec.title}] {note}")
    return "\n".join(out)

def parse_argv(argv):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("txt_file", help="the .TXT text log file")
    p.add_argument(
        "--include-warns",
        action="store_true",
        help="also show log level WARN lines",
    )
    p.add_argument(
        "--include-truncated",
        action="store_true",
        help="also show TRUNCATED log lines",
    )
    return p.parse_args(argv[1:])

def main(argv=None):
    args = parse_argv(argv or sys.argv)

    if not os.path.isfile(args.txt_file):
        print(f"error: no such file: {args.txt_file}", file=sys.stderr)
        return 2

    sections, ok = analyze(args.txt_file)
    print(
        format_report(
            sections,
            ok,
            include_warns=args.include_warns,
            truncated_only=args.include_truncated,
        )
    )
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())

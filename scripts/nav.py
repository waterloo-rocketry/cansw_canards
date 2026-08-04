import argparse
import re
import statistics
import matplotlib.pyplot as plt


NAV_PT1_RE = re.compile(r"\[(\d+)\]\s+navigator_pt2\s")

def check_monotonic(ts):
    for i in range(1, len(ts)):
        if ts[i] < ts[i-1]:
            print(
                f"Timestamp went backwards at index {i}: "
                f"{ts[i-1]} -> {ts[i]}"
            )

def load_nav_pt1_timestamps(path):
    timestamps = []

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            match = NAV_PT1_RE.search(line)
            if match:
                timestamps.append(int(match.group(1)))

    return timestamps


def analyze_timestamps(ts):
    if len(ts) < 2:
        print("Not enough navigator_pt1 timestamps found")
        return

    diffs = [
        ts[i] - ts[i - 1]
        for i in range(1, len(ts))
    ]

    duration_ms = ts[-1] - ts[0]

    print(f"navigator_pt1 samples: {len(ts)}")
    print(f"start timestamp: {ts[0]} ms")
    print(f"end timestamp:   {ts[-1]} ms")
    print(f"duration:        {duration_ms / 1000:.3f} s")
    print()

    print("Sample period:")
    print(f"  min:    {min(diffs)} ms")
    print(f"  avg:    {statistics.mean(diffs):.3f} ms")
    print(f"  median: {statistics.median(diffs)} ms")
    print(f"  max:    {max(diffs)} ms")
    print()

    rate = (len(ts) - 1) / (duration_ms / 1000)

    print("Average frequency:")
    print(f"  {rate:.2f} Hz")
    print()

    # Detect missed samples based on unusually large gaps
    expected_period = statistics.median(diffs)
    threshold = expected_period * 1.5

    gaps = [
        diff for diff in diffs
        if diff > threshold
    ]

    print("Timestamp gaps:")
    print(f"  expected period: {expected_period} ms")
    print(f"  threshold:       > {threshold:.1f} ms")
    print(f"  missed periods:  {len(gaps)}")

    if gaps:
        print(f"  largest gap:     {max(gaps)} ms")
        print(f"  total gap time:  {sum(gaps)} ms")


def plot_timestamps(ts):
    if len(ts) < 2:
        return

    diffs = [
        ts[i] - ts[i - 1]
        for i in range(1, len(ts))
    ]

    plt.figure()
    plt.plot(ts)
    plt.xlabel("navigator_pt1 sample index")
    plt.ylabel("timestamp (ms)")
    plt.title("navigator_pt1 timestamp progression")
    plt.grid(True)

    plt.figure()
    plt.plot(diffs)
    plt.xlabel("navigator_pt1 sample index")
    plt.ylabel("Δtimestamp (ms)")
    plt.title("navigator_pt1 sample intervals")
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="log file")
    args = parser.parse_args()

    timestamps = load_nav_pt1_timestamps(args.input)

    analyze_timestamps(timestamps)
    plot_timestamps(timestamps)
    check_monotonic(timestamps)

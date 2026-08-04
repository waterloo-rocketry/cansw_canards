import argparse
import csv
import statistics
import matplotlib.pyplot as plt


def load_timestamps(path):
    timestamps = []

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.DictReader(f)

        for row in reader:
            try:
                timestamps.append(int(row["timestamp"]))
            except (ValueError, KeyError):
                continue

    return timestamps


def summarize_timestamps(ts):
    if len(ts) < 2:
        print("Not enough timestamps found")
        return

    diffs = [ts[i] - ts[i - 1] for i in range(1, len(ts))]

    duration_ms = ts[-1] - ts[0]

    print(f"Samples: {len(ts)}")
    print(f"Duration: {duration_ms / 1000:.3f} s")
    print()

    print("Timestamp delta:")
    print(f"  min: {min(diffs)} ms")
    print(f"  avg: {statistics.mean(diffs):.3f} ms")
    print(f"  median: {statistics.median(diffs)} ms")
    print(f"  max: {max(diffs)} ms")
    print()

    # Detect large gaps
    threshold = max(10, statistics.mean(diffs) * 5)

    jumps = [
        diff for diff in diffs
        if diff > threshold
    ]

    print("Time jumps:")
    print(f"  threshold: > {threshold:.1f} ms")
    print(f"  count: {len(jumps)}")

    if jumps:
        print(f"  largest jump: {max(jumps)} ms")
        print(f"  average jump: {statistics.mean(jumps):.1f} ms")
        print(f"  total jump time: {sum(jumps)} ms")
        print(f"  fraction of runtime: {100 * sum(jumps) / duration_ms:.2f}%")
    else:
        print("  none")

    print()

    rate = (len(ts) - 1) / (duration_ms / 1000)

    print("Average logging rate:")
    print(f"  {rate:.2f} samples/sec")


def plot_timestamps(ts):
    if not ts:
        print("No timestamps found")
        return

    diffs = [ts[i] - ts[i - 1] for i in range(1, len(ts))]

    plt.figure()
    plt.plot(ts)
    plt.xlabel("sample index")
    plt.ylabel("timestamp (ms)")
    plt.title("Log timestamp progression")
    plt.grid(True)

    plt.figure()
    plt.plot(diffs)
    plt.xlabel("sample index")
    plt.ylabel("Δtimestamp (ms)")
    plt.title("Timestamp gaps")
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="CSV log file")
    args = parser.parse_args()

    ts = load_timestamps(args.input)

    summarize_timestamps(ts)
    plot_timestamps(ts)

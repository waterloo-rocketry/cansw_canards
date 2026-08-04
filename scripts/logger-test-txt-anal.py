import argparse
import re
import statistics
import matplotlib.pyplot as plt


LOG_RE = re.compile(r"\[(\d+)\].*LOGGER WRITE TEST")

MSG_SIZE_BYTES = 128


def load_timestamps(path):
    timestamps = []

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            match = LOG_RE.search(line)
            if match:
                timestamps.append(int(match.group(1)))

    return timestamps


def analyze(timestamps):
    if len(timestamps) < 2:
        print("Not enough log messages found")
        return

    diffs = [
        timestamps[i] - timestamps[i - 1]
        for i in range(1, len(timestamps))
    ]

    duration_ms = timestamps[-1] - timestamps[0]

    msgs = len(timestamps)
    msgs_per_sec = msgs / (duration_ms / 1000)
    bytes_per_sec = msgs_per_sec * MSG_SIZE_BYTES

    print(f"Messages: {msgs}")
    print(f"Duration: {duration_ms / 1000:.3f} s")
    print()

    print("Timestamp spacing:")
    print(f"  min:    {min(diffs)} ms")
    print(f"  avg:    {statistics.mean(diffs):.3f} ms")
    print(f"  median: {statistics.median(diffs)} ms")
    print(f"  max:    {max(diffs)} ms")
    print()

    print("Throughput:")
    print(f"  messages/sec: {msgs_per_sec:.2f}")
    print(f"  bytes/sec:    {bytes_per_sec:.1f}")
    print(f"  KiB/sec:      {bytes_per_sec / 1024:.2f}")
    print(f"  MiB/sec:      {bytes_per_sec / 1024 / 1024:.3f}")
    print()

    # Detect timestamp stalls
    expected = statistics.median(diffs)
    threshold = expected * 2

    gaps = [
        diff for diff in diffs
        if diff > threshold
    ]

    print("Large timestamp gaps:")
    print(f"  threshold: > {threshold:.1f} ms")
    print(f"  count: {len(gaps)}")

    if gaps:
        print(f"  largest gap: {max(gaps)} ms")
        print(f"  average gap: {statistics.mean(gaps):.1f} ms")
        print(f"  total gap time: {sum(gaps)} ms")


def plot(timestamps):
    if len(timestamps) < 2:
        return

    diffs = [
        timestamps[i] - timestamps[i - 1]
        for i in range(1, len(timestamps))
    ]

    plt.figure()
    plt.plot(timestamps)
    plt.xlabel("message index")
    plt.ylabel("timestamp (ms)")
    plt.title("Logger timestamp progression")
    plt.grid(True)

    plt.figure()
    plt.plot(diffs)
    plt.xlabel("message index")
    plt.ylabel("Δtimestamp (ms)")
    plt.title("Logger message intervals")
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="text log file")
    args = parser.parse_args()

    ts = load_timestamps(args.input)

    analyze(ts)
    plot(ts)

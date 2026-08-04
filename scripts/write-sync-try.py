import re
import argparse
import matplotlib.pyplot as plt

LINE_RE = re.compile(
    r"\[(\d+)\].*?open_time=(\d+),\s*sync_time=(\d+),\s*try=(\d+)"
)


def load_data(filename):
    timestamps = []
    open_times = []
    sync_times = []
    tries = []

    with open(filename, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = LINE_RE.search(line)
            if not m:
                continue

            timestamps.append(int(m.group(1)))
            open_times.append(int(m.group(2)))
            sync_times.append(int(m.group(3)))
            tries.append(int(m.group(4)))

    return timestamps, open_times, sync_times, tries


def print_stats(name, values):
    if not values:
        return

    vals = sorted(values)
    n = len(vals)

    print(f"{name}")
    print(f"  count: {n}")
    print(f"  avg:   {sum(vals)/n:.2f} ms")
    print(f"  min:   {vals[0]} ms")
    print(f"  p50:   {vals[n//2]} ms")
    print(f"  p95:   {vals[int(0.95*n)]} ms")
    print(f"  max:   {vals[-1]} ms")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Plot SD open/sync timing diagnostics."
    )
    parser.add_argument("logfile")
    args = parser.parse_args()

    ts, open_ms, sync_ms, tries = load_data(args.logfile)

    if not ts:
        print("No matching log lines found.")
        return

    print_stats("Open time", open_ms)
    print_stats("Sync time", sync_ms)

    print("Retries")
    print(f"  count: {len(tries)}")
    print(f"  max try: {max(tries)}")
    print(f"  retries (>1): {sum(t > 1 for t in tries)}")
    print()

    plt.figure(figsize=(12, 4))
    plt.plot(ts, open_ms, ".", markersize=2)
    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Open time (ms)")
    plt.title("SD File Open Time")
    plt.grid(True)

    plt.figure(figsize=(12, 4))
    plt.plot(ts, sync_ms, ".", markersize=2)
    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Sync time (ms)")
    plt.title("SD Sync Time")
    plt.grid(True)

    plt.figure(figsize=(12, 4))
    plt.plot(ts, tries, ".", markersize=3)
    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Try number")
    plt.title("Write Retry Count")
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()

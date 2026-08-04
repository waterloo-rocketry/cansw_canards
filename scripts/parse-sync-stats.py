import argparse
import re
import matplotlib.pyplot as plt

# Example:
# [5390] INFO; hi; open_time=8, sync_time=6

line_re = re.compile(
    r"\[(\d+)\].*?open_time=(\d+),\s*sync_time=(\d+)"
)


def load_data(filename):
    timestamps = []
    open_times = []
    sync_times = []

    with open(filename, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = line_re.search(line)
            if not m:
                continue

            timestamps.append(int(m.group(1)))
            open_times.append(int(m.group(2)))
            sync_times.append(int(m.group(3)))

    return timestamps, open_times, sync_times


def print_stats(name, values):
    if not values:
        print(f"{name}: no samples")
        return

    vals = sorted(values)

    def pct(p):
        return vals[int((len(vals) - 1) * p)]

    print(f"\n{name}")
    print(f"  samples : {len(values)}")
    print(f"  avg     : {sum(values)/len(values):.2f} ms")
    print(f"  min     : {min(values)} ms")
    print(f"  p50     : {pct(0.50)} ms")
    print(f"  p90     : {pct(0.90)} ms")
    print(f"  p99     : {pct(0.99)} ms")
    print(f"  max     : {max(values)} ms")
    print(f"  >10 ms  : {sum(v > 10 for v in values)}")
    print(f"  >20 ms  : {sum(v > 20 for v in values)}")
    print(f"  >50 ms  : {sum(v > 50 for v in values)}")
    print(f"  >100 ms : {sum(v > 100 for v in values)}")


def main():
    parser = argparse.ArgumentParser(
        description="Plot SD open() and sync() timing from logger output."
    )
    parser.add_argument("logfile")
    args = parser.parse_args()

    ts, open_ms, sync_ms = load_data(args.logfile)

    if not ts:
        print("No matching log lines found.")
        return

    print_stats("Open", open_ms)
    print_stats("Sync", sync_ms)

    plt.figure(figsize=(12, 5))
    plt.plot(ts, open_ms, ".", markersize=3, label="open()")
    plt.plot(ts, sync_ms, ".", markersize=3, label="sync()")
    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Time (ms)")
    plt.title("SD Card Open/Sync Times")
    plt.grid(True)
    plt.legend()

    plt.figure(figsize=(12, 5))
    plt.plot(ts, open_ms, linewidth=1)
    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Open Time (ms)")
    plt.title("SD Open Time vs Time")
    plt.grid(True)

    plt.figure(figsize=(12, 5))
    plt.plot(ts, sync_ms, linewidth=1)
    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Sync Time (ms)")
    plt.title("SD Sync Time vs Time")
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()

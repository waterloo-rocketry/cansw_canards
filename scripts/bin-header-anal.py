import argparse
import re
import statistics
import matplotlib.pyplot as plt

HEADER_RE = re.compile(r"\[(\d+)\]\s+header\s+\(type\s+\"HEAD\"\)")
INDEX_RE = re.compile(r"index:\s*(\d+)")

DATA_MSGS_PER_BUFFER = 1024


def load_headers(path):
    timestamps = []
    indices = []

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = iter(f)

        for line in lines:
            m = HEADER_RE.search(line)
            if not m:
                continue

            ts = int(m.group(1))

            # Skip "version" line
            try:
                next(lines)
                index_line = next(lines)
            except StopIteration:
                break

            m2 = INDEX_RE.search(index_line)
            if not m2:
                continue

            timestamps.append(ts)
            indices.append(int(m2.group(1)))

    return timestamps, indices


def summarize(ts, indices):
    if len(ts) < 2:
        print("Not enough header messages found")
        return

    dt = [ts[i] - ts[i - 1] for i in range(1, len(ts))]
    di = [indices[i] - indices[i - 1] for i in range(1, len(indices))]

    duration_ms = ts[-1] - ts[0]
    buffers = indices[-1] - indices[0]

    print(f"Header messages: {len(ts)}")
    print(f"Buffer index range: {indices[0]} -> {indices[-1]}")
    print(f"Runtime: {duration_ms/1000:.3f} s")
    print()

    print("Buffer interval statistics:")
    print(f"  min:    {min(dt)} ms")
    print(f"  avg:    {statistics.mean(dt):.2f} ms")
    print(f"  median: {statistics.median(dt):.2f} ms")
    print(f"  max:    {max(dt)} ms")
    print()

    expected = statistics.median(dt)
    threshold = expected * 1.5

    gaps = [x for x in dt if x > threshold]

    print("Large gaps:")
    print(f"  threshold: > {threshold:.1f} ms")
    print(f"  count: {len(gaps)}")

    if gaps:
        print(f"  largest: {max(gaps)} ms")
        print(f"  average: {statistics.mean(gaps):.1f} ms")
        print(f"  total stalled time: {sum(gaps)} ms")

    print()

    buffers_per_sec = buffers / (duration_ms / 1000)
    msgs_per_sec = buffers_per_sec * DATA_MSGS_PER_BUFFER
    bytes_per_sec = buffers_per_sec * 32768

    print("Throughput:")
    print(f"  buffers/sec : {buffers_per_sec:.2f}")
    print(f"  msgs/sec    : {msgs_per_sec:.1f}")
    print(f"  KiB/sec     : {bytes_per_sec/1024:.1f}")
    print(f"  MiB/sec     : {bytes_per_sec/1024/1024:.3f}")


def plot(ts):
    if len(ts) < 2:
        return

    dt = [ts[i] - ts[i - 1] for i in range(1, len(ts))]

    plt.figure()
    plt.plot(ts)
    plt.title("Header timestamps")
    plt.xlabel("Buffer index")
    plt.ylabel("Timestamp (ms)")
    plt.grid(True)

    plt.figure()
    plt.plot(dt)
    plt.title("Time to fill one data buffer")
    plt.xlabel("Buffer index")
    plt.ylabel("ms")
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="log file")
    args = parser.parse_args()

    timestamps, indices = load_headers(args.input)

    summarize(timestamps, indices)
    plot(timestamps)

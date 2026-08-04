import re
import argparse
import matplotlib.pyplot as plt

ts_re = re.compile(r"\[(\d+)\]")

def load_timestamps(path):
    ts = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = ts_re.search(line)
            if m:
                ts.append(int(m.group(1)))
    return ts

def plot_timestamps(ts):
    if not ts:
        print("No timestamps found")
        return

    diffs = [ts[i] - ts[i - 1] for i in range(1, len(ts))]

    plt.figure()
    plt.plot(ts, label="timestamp")
    plt.xlabel("sample index")
    plt.ylabel("timestamp (ms)")
    plt.title("Log timestamp progression")
    plt.grid(True)

    plt.figure()
    plt.plot(diffs, label="Δtimestamp")
    plt.xlabel("sample index")
    plt.ylabel("delta (ms)")
    plt.title("Timestamp gaps (jumps detection)")
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="log file")
    args = parser.parse_args()

    ts = load_timestamps(args.input)
    plot_timestamps(ts)
import argparse
import re
import statistics
import matplotlib.pyplot as plt


LOGGER_RE = re.compile(
    r"\[(\d+)\].*"
    r"type=(\w+).*"
    r"msgs=(\d+).*"
    r"write=(\d+)ms.*"
    r"reset=(\d+)ms.*"
    r"total=(\d+)ms.*"
    r"size=(\d+)"
)

SD_RE = re.compile(
    r"\[(\d+)\].*"
    r"total_ms=(\d+).*"
    r"open_ms=(\d+).*"
    r"write_ms=(\d+).*"
    r"close_ms=(\d+).*"
    r"bytes=(\d+).*"
    r"max_write=(\d+)"
)


def parse(filename):
    logger = []
    sd = []

    with open(filename, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = LOGGER_RE.search(line)
            if m:
                logger.append({
                    "timestamp": int(m.group(1)),
                    "type": m.group(2),
                    "msgs": int(m.group(3)),
                    "write": int(m.group(4)),
                    "reset": int(m.group(5)),
                    "total": int(m.group(6)),
                    "size": int(m.group(7)),
                })

            m = SD_RE.search(line)
            if m:
                sd.append({
                    "timestamp": int(m.group(1)),
                    "total": int(m.group(2)),
                    "open": int(m.group(3)),
                    "write": int(m.group(4)),
                    "close": int(m.group(5)),
                    "bytes": int(m.group(6)),
                    "max_write": int(m.group(7)),
                })

    return logger, sd


def stats(name, values):
    print(name)
    print(f"  count: {len(values)}")
    print(f"  avg:   {statistics.mean(values):.2f} ms")
    print(f"  p50:   {statistics.median(values)} ms")
    print(f"  max:   {max(values)} ms")
    print()


def print_stats(logger, sd):

    print("LOGGER TASK")
    if logger:
        stats(
            "total",
            [x["total"] for x in logger]
        )
        stats(
            "write portion",
            [x["write"] for x in logger]
        )

        stalls = [x for x in logger if x["total"] > 100]

        print(f"Logger stalls >100ms: {len(stalls)}")

        for x in stalls[:20]:
            print(
                f"  [{x['timestamp']}] "
                f"total={x['total']}ms "
                f"write={x['write']}ms "
                f"reset={x['reset']}ms"
            )

    print()

    print("SD CARD")
    if sd:
        stats(
            "sd total",
            [x["total"] for x in sd]
        )
        stats(
            "sd write",
            [x["write"] for x in sd]
        )
        stats(
            "sd open",
            [x["open"] for x in sd]
        )
        stats(
            "sd close",
            [x["close"] for x in sd]
        )

        stalls = [x for x in sd if x["total"] > 100]

        print(f"SD stalls >100ms: {len(stalls)}")

        for x in stalls[:20]:
            print(
                f"  [{x['timestamp']}] "
                f"total={x['total']}ms "
                f"open={x['open']}ms "
                f"write={x['write']}ms "
                f"close={x['close']}ms"
            )


def plot(logger, sd):

    if logger:
        t = [x["timestamp"] for x in logger]

        plt.figure()
        plt.plot(t, [x["total"] for x in logger])
        plt.xlabel("time (ms)")
        plt.ylabel("logger total (ms)")
        plt.title("Logger task latency")
        plt.grid()

    if sd:
        t = [x["timestamp"] for x in sd]

        plt.figure()
        plt.plot(t, [x["total"] for x in sd], label="total")
        plt.plot(t, [x["open"] for x in sd], label="open")
        plt.plot(t, [x["write"] for x in sd], label="write")
        plt.plot(t, [x["close"] for x in sd], label="close")

        plt.xlabel("time (ms)")
        plt.ylabel("SD time (ms)")
        plt.title("SD card operation timing")
        plt.legend()
        plt.grid()

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="log file")
    args = parser.parse_args()

    logger, sd = parse(args.input)

    print_stats(logger, sd)
    plot(logger, sd)

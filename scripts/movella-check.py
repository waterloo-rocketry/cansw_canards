import argparse
import pandas as pd
import matplotlib.pyplot as plt

FIELDS = [
    "movella_pt1.accel_x",
    "movella_pt1.accel_y",
    "movella_pt1.accel_z",
    "movella_pt1.gyro_x",
    "movella_pt1.gyro_y",
    "movella_pt1.gyro_z",
]


def main():
    parser = argparse.ArgumentParser(
        description="Analyze whether Movella PT1 data changes each sample."
    )
    parser.add_argument("csv")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    if "timestamp" not in df.columns:
        raise RuntimeError("CSV has no timestamp column.")

    for f in FIELDS:
        if f not in df.columns:
            raise RuntimeError(f"Missing column: {f}")

    timestamps = df["timestamp"]

    # True if any Movella field changed from previous row
    changed = [True]

    for i in range(1, len(df)):
        new_data = False

        for field in FIELDS:
            if df.at[i, field] != df.at[i - 1, field]:
                new_data = True
                break

        changed.append(new_data)

    df["new_data"] = changed

    total = len(df)
    updates = sum(changed)
    repeats = total - updates

    print(f"Samples           : {total}")
    print(f"New data          : {updates}")
    print(f"Repeated samples  : {repeats}")
    print(f"Update percentage : {100*updates/total:.2f}%")

    # Longest run of repeated data
    longest_repeat = 0
    current_repeat = 0

    repeat_start = None
    longest_start = None
    longest_end = None

    for i in range(1, len(changed)):
        if not changed[i]:
            if current_repeat == 0:
                repeat_start = timestamps.iloc[i]

            current_repeat += 1

            if current_repeat > longest_repeat:
                longest_repeat = current_repeat
                longest_start = repeat_start
                longest_end = timestamps.iloc[i]
        else:
            current_repeat = 0

    print()
    print(f"Longest repeated run : {longest_repeat} samples")

    if longest_repeat:
        duration = longest_end - longest_start
        print(f"Start timestamp      : {longest_start} ms")
        print(f"End timestamp        : {longest_end} ms")
        print(f"Duration             : {duration} ms")

    # Plot 1
    plt.figure(figsize=(12,4))
    plt.plot(timestamps, changed, ".", markersize=2)
    plt.yticks([0,1], ["Repeated","New"])
    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Movella update")
    plt.title("Movella PT1 New Data Detection")
    plt.grid(True)

    # Plot 2
    plt.figure(figsize=(12,4))
    plt.plot(timestamps, df[FIELDS[0]], label="accel_x")
    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Acceleration X")
    plt.title("Movella PT1 accel_x")
    plt.grid(True)

    # Plot 3
    plt.figure(figsize=(12,4))
    plt.plot(timestamps, df[FIELDS[3]], label="gyro_x")
    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Gyro X")
    plt.title("Movella PT1 gyro_x")
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()

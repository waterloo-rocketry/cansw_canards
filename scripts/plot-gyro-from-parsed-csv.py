#!/usr/bin/env python3
import argparse
import pandas as pd
import matplotlib.pyplot as plt


def parse_argv(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("csvfile")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_argv(argv)

    df = pd.read_csv(args.csvfile)

    # Remove rows where AD gyro was not logged
    gyro_df = df[df["ad_gyro.gyro"].notna()].copy()

    if gyro_df.empty:
        print("No ad_gyro data found")
        return

    time_ms = gyro_df["timestamp"]
    gyro = gyro_df["ad_gyro.gyro"] / (3.14 / 180)

    plt.figure(figsize=(10, 5))
    plt.plot(time_ms, gyro)
    plt.xlabel("Time (ms)")
    plt.ylabel("AD Gyro")
    plt.title("AD Gyro vs Timestamp")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
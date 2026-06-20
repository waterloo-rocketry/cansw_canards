import re
import sys
import numpy as np
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Usage: python plot.py <logfile>")
    sys.exit(1)

LOG_FILE = sys.argv[1]

pattern = re.compile(r"\[\d+\]\s*test;now\s+(\d+)\s*ms", re.IGNORECASE)

times_ms = []

with open(LOG_FILE, "r", errors="ignore") as f:
    for line in f:
        m = pattern.search(line)
        if m:
            times_ms.append(int(m.group(1)))

if len(times_ms) == 0:
    raise RuntimeError("No 'test;now' entries found")

times_ms = np.array(times_ms)

deltas = np.diff(times_ms)

mean = np.mean(deltas)
std = np.std(deltas)
threshold = mean + 3 * std

gaps = np.where(deltas > threshold)[0]

print(f"File: {LOG_FILE}")
print(f"Entries: {len(times_ms)}")
print(f"Mean delta: {mean:.2f} ms")
print(f"Std delta:  {std:.2f} ms")
print(f"Gap threshold: {threshold:.2f} ms")
print(f"Detected gaps: {len(gaps)}")

if len(gaps) > 0:
    print("First gap indices:", gaps[:10])
    print("First gap values:", deltas[gaps[:10]])

t = np.arange(len(times_ms))

plt.figure(figsize=(12, 5))
plt.plot(t, times_ms, linewidth=1)
plt.title("test;now Timestamp Progression")
plt.xlabel("Entry Index")
plt.ylabel("Timestamp (ms)")
plt.grid(True)

if len(gaps) > 0:
    plt.scatter(gaps, times_ms[gaps], color="red")

plt.tight_layout()
plt.show()

plt.figure(figsize=(12, 5))
plt.plot(deltas, linewidth=1)
plt.title("Delta Between test;now Entries")
plt.xlabel("Entry Index")
plt.ylabel("Delta (ms)")
plt.grid(True)

if len(gaps) > 0:
    plt.scatter(gaps, deltas[gaps], color="red")

plt.tight_layout()
plt.show()
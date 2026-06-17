import re
import numpy as np
import matplotlib.pyplot as plt

LOG_FILE = "TESTUWU.TXT"

# Example:
# cnt:1521 now:53670ms write:27ms
pattern = re.compile(
    r"cnt:(\d+)\s+now:(\d+)ms\s+write:(\d+)ms",
    re.IGNORECASE
)

counts = []
timestamps_ms = []
write_times_ms = []

with open(LOG_FILE, "rb") as f:
    data = f.read().decode("utf-8", errors="ignore")

for match in pattern.finditer(data):
    counts.append(int(match.group(1)))
    timestamps_ms.append(int(match.group(2)))
    write_times_ms.append(int(match.group(3)))

if len(counts) == 0:
    raise RuntimeError("No log entries found")

# ---------------------------
# Check for skipped counts
# ---------------------------

missing_counts = []

for prev, curr in zip(counts[:-1], counts[1:]):
    if curr != prev + 1:
        missing_counts.extend(range(prev + 1, curr))

if missing_counts:
    print("SKIPPED COUNTS: YES")
    print(f"Missing {len(missing_counts)} count values")
    print("First few missing:", missing_counts[:20])
else:
    print("SKIPPED COUNTS: NO")

# ---------------------------
# Timestamp period analysis
# ---------------------------

periods_ms = np.diff(timestamps_ms)

print()
print(f"Entries: {len(counts)}")
print(f"Average period: {np.mean(periods_ms):.3f} ms")
print(f"Min period:     {np.min(periods_ms):.3f} ms")
print(f"Max period:     {np.max(periods_ms):.3f} ms")
print(f"Std dev:        {np.std(periods_ms):.3f} ms")

# ---------------------------
# Plot
# ---------------------------
# ---------------------------
# Time axis (seconds)
# ---------------------------

time_s = np.array(timestamps_ms) / 1000.0

# Period between log entries
periods_ms = np.diff(timestamps_ms)

# ---------------------------
# Plot
# ---------------------------

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# SD write duration over time
ax1.plot(time_s, write_times_ms, linewidth=1)
ax1.set_title("SD Write Duration vs Time")
ax1.set_ylabel("Write Duration (ms)")
ax1.grid(True)

# Actual period between successive log entries
ax2.plot(time_s[1:], periods_ms, linewidth=1)
ax2.set_title("Period Between Successive Log Entries")
ax2.set_xlabel("Time Since Start (s)")
ax2.set_ylabel("Period (ms)")
ax2.grid(True)

plt.tight_layout()
plt.show()
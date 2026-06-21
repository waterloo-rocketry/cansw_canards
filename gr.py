import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("sintestmore.csv")

# Convert ms to seconds
log_time_s = df["log_timestamp_ms"] / 1000.0
fb_time_s = df["fb_timestamp_ms"] / 1000.0

# Position vs command
plt.figure()
plt.plot(log_time_s, df["cmd_deg"], label="cmd_deg")
plt.plot(fb_time_s, df["position_deg"], label="position_deg")
plt.xlabel("Time (s)")
plt.ylabel("Degrees")
plt.title("Command vs Position")
plt.grid(True)
plt.legend()

# Speed
plt.figure()
plt.plot(fb_time_s, df["speed_erpm"])
plt.xlabel("Time (s)")
plt.ylabel("eRPM")
plt.title("Speed")
plt.grid(True)

# Current
plt.figure()
plt.plot(fb_time_s, df["current_a"])
plt.xlabel("Time (s)")
plt.ylabel("Current (A)")
plt.title("Current")
plt.grid(True)

plt.show()
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

problem = "msd_w_kalman_filter"

y = pd.read_csv(f"examples/{problem}_solution.csv", sep="\s+")
t = pd.read_csv(f"examples/{problem}_time.csv", sep=" ")

pos = y["Pos"].to_numpy()
vel = y["Vel"].to_numpy()
time = t["time"].to_numpy()

meas = pd.read_csv(f"examples/{problem}_meas_history.csv", sep="\s+")
meas_pos = meas["Pos"].to_numpy()
meas_vel = meas["Vel"].to_numpy()

estimated = pd.read_csv(f"examples/{problem}_est_history.csv", sep="\s+")
est_pos = estimated["Pos"].to_numpy()
est_vel = estimated["Vel"].to_numpy()

# plot pos, vel
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
fig.suptitle(f"{problem}")
axs[0,0].plot(time, pos, label = "true")
axs[0,0].plot(time, meas_pos, label = "measured")
axs[0,0].plot(time, est_pos, '--', label = "estimated")
axs[0,0].set_ylabel("pos (m)")
axs[0,0].legend()
axs[0,0].grid(True)

axs[0,1].plot(time, vel, label = "true")
axs[0,1].plot(time, meas_vel, label = "measured")
axs[0,1].plot(time, est_vel, '--', label = "estimated")
axs[0,1].set_xlabel("time (s)")
axs[0,1].set_ylabel("vel (m/s)")
axs[0,1].legend()
axs[0,1].grid(True)

axs[1,1].plot(pos, vel, label = "true")
axs[1,1].plot(est_pos, est_vel, '--', label = "estimated")
axs[1,1].set_xlabel("pos (m)")
axs[1,1].set_ylabel("vel (m/s)")
axs[1,1].grid(True)

# Adjust layout to prevent overlapping
plt.tight_layout()

plt.show()
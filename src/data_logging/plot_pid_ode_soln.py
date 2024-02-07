import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# problem = "mass_spring_damper"
# problem = "van_der_pol"
problem = "dc_motor"

y = pd.read_csv(f"examples/{problem}_solution.csv", sep="\s+")
t = pd.read_csv(f"examples/{problem}_time.csv", sep=" ")

pos = y["Pos"].to_numpy()
vel = y["Vel"].to_numpy()
time = t["time"].to_numpy()

error = pd.read_csv(f"examples/err_history.csv", sep="\s+")
# control_ip = pd.read_csv(f"control_ip_history.csv", sep="\s")

pos_err = error["Pos_err"].to_numpy()
vel_err = error["Vel_err"].to_numpy()
# U1 = control_ip["U1"].to_numpy()
# U2 = control_ip["U2"].to_numpy()

# plot pos, vel
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
fig.suptitle(f"{problem}")
axs[0,0].plot(time, pos)
axs[0,0].set_ylabel("pos (m)")
axs[0,0].grid(True)

axs[0,1].plot(time, vel)
axs[0,1].set_xlabel("time (s)")
axs[0,1].set_ylabel("vel (m/s)")
axs[0,1].grid(True)

axs[1,1].plot(pos, vel)
axs[1,1].set_xlabel("pos (m)")
axs[1,1].set_ylabel("vel (m/s)")
axs[1,1].grid(True)

# plot control input, error
fig2, axs2 = plt.subplots(2, 2, figsize=(10, 8))
fig2.suptitle(f"{problem}")
axs2[0,0].plot(pos_err)
axs2[0,0].set_ylabel("pos_err (m)")
axs2[0,0].grid(True)

axs2[0,1].plot(vel_err)
axs2[0,1].set_xlabel("time (s)")
axs2[0,1].set_ylabel("vel_err (m/s)")
axs2[0,1].grid(True)

# axs2[1,0].plot(U1)
# axs2[1,0].set_xlabel("time (s)")
# axs2[1,0].set_ylabel("U1 (N)")
# axs2[1,0].grid(True)

# axs2[1,1].plot(U2)
# axs2[1,1].set_xlabel("time (s)")
# axs2[1,1].set_ylabel("U2 (N)")
# axs2[1,1].grid(True)

# Adjust layout to prevent overlapping
plt.tight_layout()

plt.show()
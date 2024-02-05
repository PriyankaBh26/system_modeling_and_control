import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# problem = "mass_spring_damper"
problem = "van_der_pol"

y = pd.read_csv(f"{problem}_solution.csv", sep="\s+")
# print(y.head())

t = pd.read_csv(f"{problem}_time.csv", sep=" ")
# print(t.head())

pos = y["Pos"].to_numpy()
vel = y["Vel"].to_numpy()
time = t["time"].to_numpy()

fig, axs = plt.subplots(2, 2, figsize=(10, 8))
fig.suptitle(f"{problem}")

axs[0,0].plot(time, pos)
# axs[0,0].ylabel("pos (m)")
# axs[0,0].grid(True)

axs[0,1].plot(time, vel)
# axs[0,1].xlabel("time (s)")
# axs[0,1].ylabel("vel (m/s)")
# axs[0,1].grid(True)

axs[1, 1].plot(pos, vel)
# axs[0,1].xlabel("pos (m)")
# axs[0,1].ylabel("vel (m/s)")
# axs[0,1].grid(True)

# Adjust layout to prevent overlapping
plt.tight_layout()

plt.show()
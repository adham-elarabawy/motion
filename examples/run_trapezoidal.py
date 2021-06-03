import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import sys

sys.path.append("..")
from engine.trapezoidal import Trapezoidal


# visualization params
sns.set_theme(style="darkgrid")
sns.color_palette("crest", as_cmap=True)
sampling_res = 1000

# initialize and compute jerk limited trajectory
tp_traj = Trapezoidal(0, 10, 2.5, 0, 5, 10)

# print relevant params
print(f"T_a = {tp_traj.T_a}, T_d = {tp_traj.T_d}, T = {tp_traj.T}")

tp_pos, tp_vel, tp_accel = [], [], []
times = np.linspace(0, tp_traj.T, sampling_res)
for time in times:
    p, v, a = tp_traj.sample_trajectory(time)
    tp_pos.append(p)
    tp_vel.append(v)
    tp_accel.append(a)

tp = pd.DataFrame({"time" : times, "position" : tp_pos, "velocity" : tp_vel, "acceleration" : tp_accel})
tp = tp.melt('time', var_name='measurements', value_name='values')
sns.lineplot(x="time", y="values", hue='measurements', data=tp, palette="magma")
plt.title("Trapezoidal Motion Plan")
plt.legend()
plt.show()

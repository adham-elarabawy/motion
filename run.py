import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from engine.jerk_limited import JerkLimited


# visualization params
sns.set_theme(style="darkgrid")
sns.color_palette("crest", as_cmap=True)
sampling_res = 100

# initialize and compute jerk limited trajectory
jl_traj = JerkLimited(0, 10, 7.5, 0, 10, 10, 30, alpha=0.99)

# print relevant params
print(f"T_a = {jl_traj.T_a}, T_v = {jl_traj.T_v}, T_d = {jl_traj.T_d}, T_j1 = {jl_traj.T_j1}, T_j2 = {jl_traj.T_j2}, v_lim = {jl_traj.v_lim}")

jl_pos, jl_vel, jl_accel, jl_jerk = [], [], [], []
times = np.linspace(0, jl_traj.T, sampling_res)
for time in times:
    p, v, a, j = jl_traj.sample_trajectory(time)
    jl_pos.append(p)
    jl_vel.append(v)
    jl_accel.append(a)
    jl_jerk.append(j)

jl = pd.DataFrame({"time" : times, "position" : jl_pos, "velocity" : jl_vel, "acceleration" : jl_accel, "jerk" : jl_jerk})
jl = jl.melt('time', var_name='measurements', value_name='values')
sns.lineplot(x="time", y="values", hue='measurements', data=jl, palette="magma")

# sns.relplot(times, jl_jerk, label='jerk')
# sns.relplot(times, jl_accel, label='acceleration')
# sns.relplot(times, jl_vel, label='velocity')
# sns.relplot(times, jl_pos, label='position')

plt.title("Jerk-Limited Motion Plan")
#
plt.legend()
# sns.grid()
plt.show()
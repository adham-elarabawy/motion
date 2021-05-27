import numpy as np
import matplotlib.pyplot as plt
from engine.jerk_limited import JerkLimited

# visualization params
sampling_res = 100

# initialize and compute jerk limited trajectory
jl_traj = JerkLimited(0, 10, 0, 0, 10, 20, 30, alpha=0.99)

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

plt.plot(times, jl_pos, label='position')
plt.plot(times, jl_vel, label='velocity')
plt.plot(times, jl_accel, label='acceleration')
plt.plot(times, jl_jerk, label='jerk')
plt.legend()
plt.grid()
plt.show()
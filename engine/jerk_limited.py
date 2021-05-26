import numpy as np


class JerkLimited:
    def __init__(self, _q0, _q1, _v0, _v1, v_max, a_max, j_max):
        # given the initial conditions, compute the sign-transformed init/final conditions
        self.omega = np.sign(_q0 - _q1)
        self.q0 = self.omega * _q0
        self.q1 = self.omega * _q1
        self.v0 = self.omega * _v0
        self.v1 = self.omega * _v1

        # To verify if the max accel is reached:
        max_accel_reached = True
        min_accel_reached = True
        if ((v_max - self.v0) * j_max) < np.pow(a_max, 2):
            max_accel_reached = False
        if ((v_max - self.v1) * j_max) < np.pow(a_max, 2):
            min_accel_reached = False

        is_traj_computed = False

        while not is_traj_computed:
            # If 3.19 holds
            if max_accel_reached:
                self.T_j1 = np.sqrt((v_max - self.v0) / j_max)
                self.T_a = 2 * self.T_j1
            else:
                self.T_j1 = a_max / j_max
                self.T_a = self.T_j1 + (v_max - self.v0) / a_max

            # If 3.20 holds
            if min_accel_reached:
                self.T_j2 = np.sqrt((v_max - self.v1) / j_max)
                self.T_d = 2 * self.T_j2
            else:
                self.T_j2 = a_max / j_max
                self.T_d = self.T_j2 + (v_max - self.v1) / a_max

            # Computing the time duration of the constant velocity segment
            self.T_v = (self.q1 - self.q0) / v_max - (self.T_a / 2) * (1 + self.v0 / v_max) - (self.T_d / 2) * (
                        1 + self.v1 / v_max)

            # Computing the empirical values of the max/min accel & max vel
            self.a_lim_a = j_max * self.T_j1
            self.a_lim_d = -j_max * self.T_j2
            self.v_lim = self.v0 + (self.T_a - self.T_j1) * self.a_lim_a

            if self.T_v > 0:
                # v_max is reached
                # compute trajectory according to (3.30a) - (3.30g) & (3.33)
                is_traj_computed = True
            else:
                # v_max is not reached
                print("iterative approach not implemented yet.")

    def sample_trajectory(self, t):
        # acceleration phase
        if 0 <= t <= self.T_j1:
            pos = self.q0 + self.v0 * t + self.j_max * (t ** 3) / 6
            vel = self.v0 + self.j_max * (t ** 2) / 2
            accel = self.j_max * t
            jerk = self.j_max
        elif self.T_j1 < t <= self.T_a - self.T_j1:
            pos = self.q0 + self.v0 * t + self.a_lim_a / 6 * (3 * t ** 2 - 3 * self.T_j1 * t + self.T_j1 ** 2)
            vel = self.v0 + self.a_lim_a * (t - self.T_j1 / 2)
            accel = self.a_lim_a
            jerk = 0
        elif (self.T_a - self.T_j1) < t <= self.T_a:
            pos = self.q0 + (self.v_lim + self.v0) * self.T_a / 2 - self.v_lim * (self.T_a - t) - self.j_min * (
                        (self.T_a - t) ** 3) / 6
            vel = self.v_lim + self.j_min * ((self.T_a - t) ** 2) / 2
            accel = -self.j_min * (self.T_a - t)
            jerk = self.j_min
        # constant velocity phase
        elif self.T_a < t <= self.T_a + self.T_v:
            pos = self.q0 + (self.v_lim + self.v0) * self.T_a / 2 + self.v_lim * (t - self.T_a)
            vel = self.v_lim
            accel = 0
            jerk = 0
        elif self.T - self.T_d < t <= self.T - self.T_d + self.T_j2:
            pos = self.q1 - (self.v_lim + self.v1) * self.T_d / 2 + self.v_lim(t - self.T + self.T_d) - self.j_max * (
                        (t - self.T + self.T_d) ** 3) / 6
            vel = self.v_lim - self.j_max * ((t - self.T + self.T_d) ** 2) / 2

        return (pos, vel, accel, jerk)

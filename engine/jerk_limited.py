import numpy as np


# @SOURCE http://home.elka.pw.edu.pl/~ptrojane/books/Trajectory_Planning_for_Automatic_Machines_and_Robots.pdf

class JerkLimited:
    def __init__(self, _q0, _q1, _v0, _v1, _v_max, _a_max, _j_max, alpha=0.9):
        # given the initial conditions, compute the sign-transformed init/final conditions
        self.alpha = alpha
        self.omega = np.sign(_q1 - _q0)

        _v_min = -_v_max
        _a_min = -_a_max
        _j_min = -_j_max

        self.q0 = self.omega * _q0
        self.q1 = self.omega * _q1
        self.v0 = self.omega * _v0
        self.v1 = self.omega * _v1
        self.v_max = ((self.omega + 1) / 2) * _v_max + ((self.omega - 1) / 2) * _v_min
        self.v_min = ((self.omega + 1) / 2) * _v_min + ((self.omega - 1) / 2) * _v_max
        self.a_max = ((self.omega + 1) / 2) * _a_max + ((self.omega - 1) / 2) * _a_min
        self.a_min = ((self.omega + 1) / 2) * _a_min + ((self.omega - 1) / 2) * _a_max
        self.j_max = ((self.omega + 1) / 2) * _j_max + ((self.omega - 1) / 2) * _j_min
        self.j_min = ((self.omega + 1) / 2) * _j_min + ((self.omega - 1) / 2) * _j_max
        self.valid_traj = True

        # # TODO: check if trajectory can be computed
        # self.theta = min(np.sqrt(np.abs(self.v1 - self.v0) / self.j_max), self.a_max / self.j_max)
        #
        # if self.theta < (self.a_max / self.j_max):
        #     self.valid = (self.q1 - self.q0) > self.theta * (self.v0 + self.v1)
        # if self.theta == (self.a_max / self.j_max):
        #     self.valid = (self.q1 - self.q0) > 0.5 * (self.v0 + self.v1) * (self.theta + np.abs(self.v1 - self.v0)/self.a_max)

        # To verify if the max accel is reached:
        max_accel_reached = True
        min_accel_reached = True
        if ((self.v_max - self.v0) * self.j_max) < pow(self.a_max, 2):
            max_accel_reached = False

        if ((self.v_max - self.v1) * self.j_max) < pow(self.a_max, 2):
            min_accel_reached = False

        is_traj_computed = False

        while not is_traj_computed:
            # If 3.19 holds
            if not max_accel_reached:
                self.T_j1 = np.sqrt((self.v_max - self.v0) / self.j_max)
                self.T_a = 2 * self.T_j1
            else:
                self.T_j1 = self.a_max / self.j_max
                self.T_a = self.T_j1 + (self.v_max - self.v0) / self.a_max

            # If 3.20 holds
            if not min_accel_reached:
                self.T_j2 = np.sqrt((self.v_max - self.v1) / self.j_max)
                self.T_d = 2 * self.T_j2
            else:
                self.T_j2 = self.a_max / self.j_max
                self.T_d = self.T_j2 + (self.v_max - self.v1) / self.a_max

            # Computing the time duration of the constant velocity segment
            self.T_v = (self.q1 - self.q0) / self.v_max - (self.T_a / 2) * (1 + self.v0 / self.v_max) - (
                        self.T_d / 2) * (
                               1 + self.v1 / self.v_max)

            if self.T_v > 0:
                # v_max is reached
                # compute trajectory according to (3.30a) - (3.30g) & (3.33)
                is_traj_computed = True
            else:
                # v_max is not reached
                self.T_v = 0
                self.T_j1 = self.a_max / self.j_max
                self.T_j2 = self.T_j1
                self.T_j = self.T_j1
                delta = (self.a_max**4) / (self.j_max**2) + 2 * (self.v0**2 + self.v1**2) + self.a_max*(4 * (self.q1 - self.q0) - 2 * self.a_max/self.j_max * (self.v0 + self.v1))
                self.T_a = ((self.a_max**2) / (self.j_max) - 2*self.v0 + np.sqrt(delta)) / (2 * self.a_max)
                self.T_d = ((self.a_max**2) / (self.j_max) - 2*self.v1 + np.sqrt(delta)) / (2 * self.a_max)
                print(f"a_max = {self.a_max}, j_max = {self.j_max}, delta = {delta}, v0 = {self.v0}")
                if self.T_a < 0:
                    # Compute trajectory parameters with 3.28a, 3.28b
                    self.T_a = 0
                    self.T_d = 2 * (self.q1 - self.q0) / (self.v1 + self.v0)
                    self.T_j1 = 0
                    self.T_j2 = (self.j_max * (self.q1 - self.q0) - np.sqrt(self.j_max * (self.j_max * (self.q1 - self.q0)**2 + (self.v1 + self.v0)**2 * (self.v1 - self.v0)))) / (self.j_max * (self.v1 + self.v0))
                    is_traj_computed = True
                elif self.T_d < 0:
                    # Compute trajectory parameters with 3.29a, 3.29b
                    self.T_d = 0
                    self.T_a = 2 * (self.q1 - self.q0) / (self.v1 + self.v0)
                    self.T_j1 = (self.j_max * (self.q1 - self.q0) - np.sqrt(self.j_max * (self.j_max * (self.q1 - self.q0)**2 - (self.v1 + self.v0)**2 * (self.v1 - self.v0)))) / (self.j_max * (self.v1 + self.v0))
                    self.T_j2 = 0
                    is_traj_computed = True
                else:
                    if self.T_a > 2 * self.T_j and self.T_d > 2 * self.T_j:
                        is_traj_computed = True
                    else:
                        self.a_max = self.alpha * self.a_max
                        self.a_min = self.alpha * self.a_min

            self.T = self.T_a + self.T_v + self.T_d

            # Computing the empirical values of the max/min accel & max vel
            self.a_lim_a = self.j_max * self.T_j1
            self.a_lim_d = -self.j_max * self.T_j2
            self.v_lim = self.v0 + (self.T_a - self.T_j1) * self.a_lim_a

    def sample_trajectory(self, t):
        # acceleration phase
        if 0 <= t <= self.T_j1:
            pos = self.q0 + self.v0 * t + self.j_max * (t ** 3) / 6
            vel = self.v0 + self.j_max * (t ** 2) / 2
            accel = self.j_max * t
            jerk = self.j_max
        if self.T_j1 <= t <= self.T_a - self.T_j1:
            pos = self.q0 + self.v0 * t + self.a_lim_a / 6 * (3 * t ** 2 - 3 * self.T_j1 * t + self.T_j1 ** 2)
            vel = self.v0 + self.a_lim_a * (t - self.T_j1 / 2)
            accel = self.a_lim_a
            jerk = 0
        if (self.T_a - self.T_j1) <= t <= self.T_a:
            pos = self.q0 + (self.v_lim + self.v0) * self.T_a / 2 - self.v_lim * (self.T_a - t) - self.j_min * (
                    (self.T_a - t) ** 3) / 6
            vel = self.v_lim + self.j_min * ((self.T_a - t) ** 2) / 2
            accel = -self.j_min * (self.T_a - t)
            jerk = self.j_min
        # constant velocity phase
        if self.T_a <= t <= self.T_a + self.T_v:
            pos = self.q0 + (self.v_lim + self.v0) * self.T_a / 2 + self.v_lim * (t - self.T_a)
            vel = self.v_lim
            accel = 0
            jerk = 0
        if self.T - self.T_d <= t <= self.T - self.T_d + self.T_j2:
            pos = self.q1 - (self.v_lim + self.v1) * self.T_d / 2 + self.v_lim * (t - self.T + self.T_d) - self.j_max * (
                    (t - self.T + self.T_d) ** 3) / 6
            vel = self.v_lim - self.j_max * ((t - self.T + self.T_d) ** 2) / 2
            accel = -self.j_max * (t - self.T + self.T_d)
            jerk = self.j_min
        if self.T - self.T_d + self.T_j2 <= t <= self.T - self.T_j2:
            pos = self.q1 - (self.v_lim + self.v1) * self.T_d / 2 + self.v_lim * (t - self.T + self.T_d) + \
                  (self.a_lim_d / 6) * (3 * (t - self.T + self.T_d) ** 2 - 3 * self.T_j2 * (
                        t - self.T + self.T_d) + self.T_j2 ** 2)
            vel = self.v_lim + self.a_lim_d * (t - self.T + self.T_d - self.T_j2 / 2)
            accel = -self.j_max * self.T_j2
            jerk = 0
        if self.T - self.T_j2 <= t <= self.T:
            pos = self.q1 - self.v1 * (self.T - t) - self.j_max * ((self.T - t) ** 3) / 6
            vel = self.v1 + self.j_max * ((self.T - t) ** 2) / 2
            accel = -self.j_max * (self.T - t)
            jerk = self.j_max

        return self.omega * pos, self.omega * vel, self.omega * accel, self.omega * jerk

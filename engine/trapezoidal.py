import numpy as np


# @SOURCE https://www.pmdcorp.com/resources/type/articles/get/mathematics-of-motion-control-profiles-article

class Trapezoidal:
    def __init__(self, _q0, _q1, _v0, _v1, _v_max, _a_max, t0 = 0):
        self.q0 = _q0
        self.q1 = _q1
        self.v0 = _v0
        self.v1 = _v1
        self.v_max = _v_max
        self.a_max = _a_max
        self.t0 = t0

        self.h = self.q1 - self.q0

        # check if the trajectory is feasible
        assert(self.a_max * self.h < (np.abs(pow(self.v0, 2) - pow(self.v0, 2)) / 2), "Trajectory is not feasible with the given parameters.")

        is_max_vel_reached = self.h * self.a_max > (self.v_max**2 - (self.v0**2 + self.v1**2)/2)

        if is_max_vel_reached:
            # case 1: v_max is actually reached and maintained (trapezoidal)
            self.v_v = self.v_max
            self.T_a = (self.v_v - self.v0) / self.a_max
            self.T_d = (self.v_v - self.v1) / self.a_max
            self.T = (self.h / self.v_max) + (self.v_max / (2 * self.a_max)) * (1 - self.v0 / self.v_max)**2 + (self.v_max / (2 * self.a_max)) * (1 - self.v1 / self.v_max)**2
        else:
            # case 2: v_max is not reached (triangular)
            self.v_v = np.sqrt(self.h * self.a_max + (self.v0**2 + self.v1**2)/2)
            self.T_a = (self.v_v - self.v0) / self.a_max
            self.T_d = (self.v_v - self.v1) / self.a_max
            self.T = self.T_a + self.T_d




    def sample_trajectory(self, t):
        if self.t0 <= t <= self.T_a:
            pos = self.q0 + self.v0 * (t - self.t0) + (self.v_v - self.v0) / (2 * self.T_a) * (t - self.t0)**2
            vel = self.v0 + (self.v_v - self.v0) / self.T_a * (t - self.t0)
            accel = (self.v_v - self.v0) / self.T_a
        elif self.t0 + self.T_a <= t <= self.t0 + self.T - self.T_d:
            pos = self.q0 + self.v0 * self.T_a / 2 + self.v_v * (t - self.t0 - self.T_a/2)
            vel = self.v_v
            accel = 0
        elif self.t0 + self.T - self.T_d <= t <= self.t0 + self.T:
            pos = self.q1 - self.v1 * (self.t0 + self.T - t) - (self.v_v - self.v1) / (2 * self.T_d) * (self.t0 + self.T - t)**2
            vel = self.v1 + (self.v_v - self.v1) / self.T_d * (self.t0 + self.T - t)
            accel = -(self.v_v - self.v1) / self.T_d
        return pos, vel, accel

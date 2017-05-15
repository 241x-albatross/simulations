import numpy as np

class PathController:
    def __init__(self, mission):
        self.mission_ = mission
        self.wpt_i_ = 0
        self.radius_ = 10.
        self.done = False

    def policy(self, x):
        gamma_c = 0
        phi_c = 20./180 * np.pi
        Va_c = 5.

        # increment waypoint index if necessary
        if not self.done:
            if np.linalg.norm(x[0:2] - self.mission_[self.wpt_i_]) < self.radius_:
                self.wpt_i_ += 1
                if self.wpt_i_ >= len(self.mission_):
                    self.done = True

        if not self.done:
            path_start = x[0:2]
            if self.wpt_i_ > 0:
                path_start = self.mission_[self.wpt_i_ - 1]
            path_end = self.mission_[self.wpt_i_]
            q = path_end - path_start
            p = path_end - x[0:2]
            e = np.array(x[0:2]) - path_start

            chi_q = np.arctan2(q[1], q[0])
            chi_inf = 0.3*np.pi
            epy = np.cos(chi_q)*e[1] - np.sin(chi_q)*e[0]
            kpath = 0.05

            chi_c = chi_q - chi_inf * 2 / np.pi * np.arctan(kpath*epy)
            while chi_c < -np.pi:
                chi_c += 2*np.pi

            h_c = 70.

            kp_alt = 0.02
            kp_course = 0.5
            gamma_c = kp_alt*(h_c - x[2])
            phi_c = kp_course*(chi_c - x[3])
            Va_c = 15.

        return np.array([Va_c, phi_c, gamma_c])

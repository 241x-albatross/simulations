import numpy as np

class PathController:
    def __init__(self, mission):
        self.mission_ = mission
        self.wpt_i_ = 0
        self.radius_ = 10.
        self.done = False
        self.stats = {"epy":[], "chi_q":[], "chi_c":[]}

    def policy(self, x):
        gamma_c = 0
        phi_c = 20./180 * np.pi
        Va_c = 10.
        epy = -1
        chi_c = 0
        chi_q = 0

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
            epy = np.cos(chi_q)*e[1] - np.sin(chi_q)*e[0]
            epx = np.sin(chi_q)*e[1] + np.cos(chi_q)*e[0]
            path_length = np.linalg.norm(q)
            if path_length - epx < 0:
                self.wpt_i_ += 1
                if self.wpt_i_ >= len(self.mission_):
                    self.done = True

            chi_inf = 0.2*np.pi
            kpath = 0.1

            chi_c = chi_q - chi_inf * 2 / np.pi * np.arctan(kpath*epy)
            while chi_c < -np.pi:
                chi_c += 2*np.pi

            h_c = 70.

            kp_alt = 0.02
            kp_course = 2.5
            gamma_c = np.clip(kp_alt*(h_c - x[2]), -np.pi/4, np.pi/4)
            course_error = (chi_c - x[3])
            while course_error < -np.pi:
                course_error += 2*np.pi
            while course_error > np.pi:
                course_error -= 2*np.pi
            phi_c = np.clip(kp_course*course_error, -0.785, 0.785)
            Va_c = 15.

        self.stats["epy"].append(epy)
        self.stats["chi_q"].append(chi_q)
        self.stats["chi_c"].append(chi_c)
        return np.array([Va_c, phi_c, gamma_c])

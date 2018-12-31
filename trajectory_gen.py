import numpy as np
from base_struct import TARGET, TIME_HORIZON, TIME_PREDICT
from frenet_utils import frenet_to_cartesian

class Coefficient:
    def __init__(self, coeff_in):
        self.coefficient = coeff_in
        coefficient_tmp = []
        coefficient_tmp.append(coeff_in)
        coefficient_tmp.append([coeff_in[1], 2*coeff_in[2], 3*coeff_in[3], 4*coeff_in[4], 5*coeff_in[5], 0.0])
        coefficient_tmp.append([2*coeff_in[2], 6*coeff_in[3], 12*coeff_in[4], 20*coeff_in[5], 0.0, 0.0])
        self.coefficient_mat = np.array(coefficient_tmp).T

    def eval(self, start, stop, time_step):
        t_mat = np.array([[1, i, i**2, i**3, i**4, i**5] for i in np.arange(start, stop, time_step)])
        return t_mat.dot(self.coefficient_mat)


class TrajectoryGen:
    def __init__(self):
        self.trajectories_sd = []
        self.coefficients = []

    def generate_trajectories(self, start):
        return self.jerk_minimal_generator(start)

    @staticmethod
    def jerk_minimal_solver(start, end, T):
        a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
        c_0 = a_0 + a_1 * T + a_2 * T ** 2
        c_1 = a_1 + 2 * a_2 * T
        c_2 = 2 * a_2

        A = np.array([
            [T ** 3, T ** 4, T ** 5],
            [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
            [6 * T, 12 * T ** 2, 20 * T ** 3],
        ])
        B = np.array([
            end[0] - c_0,
            end[1] - c_1,
            end[2] - c_2
        ])
        a_3_4_5 = np.linalg.solve(A, B)
        alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])
        return Coefficient(alphas)

    def jerk_minimal_generator(self, start):
        res_trajectories = []
        res_coefficients = []
        for target in TARGET:
            coefficient_s = self.jerk_minimal_solver(start[0], target[0], TIME_PREDICT)
            coefficient_d = self.jerk_minimal_solver(start[1], target[1], TIME_PREDICT)
            res_coefficients.append((coefficient_s, coefficient_d))
            res_trajectories.append(np.hstack((coefficient_s.eval(TIME_HORIZON, TIME_PREDICT, TIME_HORIZON),
                                               coefficient_d.eval(TIME_HORIZON, TIME_PREDICT, TIME_HORIZON))))
        self.coefficients = res_coefficients
        self.trajectories_sd = res_trajectories
        return res_trajectories, res_coefficients

    def get_trajectory_xy(self, reference_line, index):
        return reference_line.calculate_cartesian_points(self.trajectories_sd[index])


# debug usage
if __name__ == "__main__":
    from reference_line import ReferenceLine
    reference_line = ReferenceLine("highway_out.txt")
    traj = TrajectoryGen()
    traj.generate_trajectories((np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])))
    out = traj.get_trajectory_xy(reference_line, 0)
    for p in out:
        print(p)


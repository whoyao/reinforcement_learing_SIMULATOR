import numpy as np

def normalize_angle(angle):
  a = np.mod(angle + np.pi, 2.0 * np.pi)
  if a < 0.0:
    a += 2.0 * np.pi
  return a - np.pi;

#input reference [rs, rx, ry, rtheta, rkappa, rdkappa]
#input current [x, y, v, a, theta, kappa]
def cartesian_to_frenet_array(reference, x, y, v, a, theta, kappa):
    res_s = np.array([0.]*3)
    res_d = np.array([0.]*3)

    # calculate d first
    delta_position = np.array([x, y]) - reference[1:3]
    cos_sin = np.array([np.cos(reference[3]), np.sin(reference[3])])
    cross_rd_nd = np.cross(delta_position,cos_sin)

    res_d[0] = np.copysign(np.linalg.norm(delta_position), cross_rd_nd)

    delta_theta = normalize_angle(theta - reference[3])
    tan_delta_theta = np.tan(delta_theta)
    cos_delta_theta = np.cos(delta_theta)
    one_minus_kappa_r_d = 1 - reference[4]*res_d[0]

    if np.fabs(delta_theta) < 3*np.pi/4 and np.fabs(delta_theta) > np.pi/4:
        res_d[1] = np.cos(np.pi/2 + reference[3] - theta)
    else:
        res_d[1] = one_minus_kappa_r_d * tan_delta_theta

    kappa_r_d_prime = reference[5] * res_d[0] + reference[4] * res_d[1]

    res_d[2] = -kappa_r_d_prime * tan_delta_theta + one_minus_kappa_r_d /(cos_delta_theta * cos_delta_theta) * \
                (kappa*one_minus_kappa_r_d/cos_delta_theta - reference[4])

    #calculate s now
    res_s[0] = reference[0]

    res_s[1] = v * cos_delta_theta / one_minus_kappa_r_d

    delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - reference[4]

    res_s[2] = (a*cos_delta_theta - res_s[1]*res_s[1]*(res_d[1]*delta_theta_prime-kappa_r_d_prime))/one_minus_kappa_r_d

    return res_s, res_d


def cartesian_to_frenet_point(reference, x, y):
    delta_position = np.array([x, y]) - reference[1:3]
    cos_sin = np.array([np.cos(reference[3]), np.sin(reference[3])])
    cross_rd_nd = np.cross(delta_position, cos_sin)
    return np.copysign(np.linalg.norm(delta_position), cross_rd_nd), reference[0]

#input reference [rs, rx, ry, rtheta, rkappa, rdkappa]
def frenet_to_cartesian( reference, s_condition, d_condition):
    sin_theta_r = np.sin(reference[3])
    cos_theta_r = np.cos(reference[3])
    res_x = reference[1] + sin_theta_r * d_condition[0]
    res_y = reference[2] - cos_theta_r * d_condition[0]
    one_minus_kappa_r_d = 1 - reference[4] * d_condition[0]
    tan_delta_theta = d_condition[1] / one_minus_kappa_r_d
    delta_theta = np.arctan2(d_condition[1], one_minus_kappa_r_d)
    cos_delta_theta = np.cos(delta_theta)
    res_theta = normalize_angle(delta_theta + reference[3])
    kappa_r_d_prime = reference[5] * d_condition[0] + reference[4] * d_condition[1]
    res_kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) * cos_delta_theta * cos_delta_theta) / \
                    one_minus_kappa_r_d + reference[4]) * cos_delta_theta / one_minus_kappa_r_d

    d_dot = d_condition[1] * s_condition[1]
    res_v = np.sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d * s_condition[1] * s_condition[1] + d_dot * d_dot)
    delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * res_kappa - reference[4]

    res_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta + s_condition[1] * s_condition[1] / cos_delta_theta *\
               (d_condition[1] * delta_theta_prime - kappa_r_d_prime)

    return res_x, res_y, res_theta, res_kappa, res_v, res_a


def frenet_to_cartesian_easy(reference, d_condition):
    sin_theta_r = np.sin(reference[3])
    cos_theta_r = np.cos(reference[3])
    res_x = reference[1] + sin_theta_r * d_condition[0]
    res_y = reference[2] - cos_theta_r * d_condition[0]

    return res_x, res_y


# debug usage
if __name__ == '__main__':
    # input = np.array([10.0, 0.0, 0.0, np.pi/4,0.1,0.01])
    # middle = cartesian_to_frenet_array(input, -1.0, 1.0, 2.0, 0.0, np.pi/3.0, 0.11)
    input = np.array([3285.35889, 2033.295975, 2999.8114241, -3.140654004923, 0.004263910, 0.000233226])
    print(frenet_to_cartesian(input, np.array([3285.35889, 20.21, -0.041244]), np.array([-6.0, 0.0, 0.0])))
import numpy as np

from bisect import bisect
from scipy.spatial import cKDTree
from frenet_utils import normalize_angle, cartesian_to_frenet_array, frenet_to_cartesian_easy, frenet_to_cartesian



def read_file(filename):
    raw_data = open(filename, 'rb').read()
    lines = raw_data.split(b'\n')
    points = []
    for line in lines:
        if len(line) == 0:
            continue
        data = [float(i) for i in line.split(b',')]
        points.append(data)
    return np.array(points)

def gen_sorted_bound(input):
    sorted_bound = []
    for point in input[1:, :]:
        sorted_bound.append(point[0])
    return sorted_bound

def gen_kd_tree(input):
    return cKDTree(input[:, 1:3])

def find_projection_point(p0, p1, point):
    v0 = point - p0[1:3]
    v1 = p1[1:3] - p0[1:3]
    delta_s = np.dot(v0, v1) / np.linalg.norm(v1)
    return interpolate_linear(p0, p1, p0[0]+delta_s)

# in s, x, y, theta, kappa, dkappa
# out [rs, rx, ry, rtheta, rkappa, rdkappa]
def interpolate_linear(p0, p1, s):
    s0 = p0[0]
    s1 = p1[0]
    if s0 > s1:
        return interpolate_linear(p1, p0, s)
    weight = (s - s0) / (s1 - s0)
    res_p = np.array([0.0]*6)
    res_p[0] = s
    res_p[1] = (1-weight)*p0[1] + weight*p1[1]
    res_p[2] = (1-weight)*p0[2] + weight*p1[2]
    res_p[3] = normalize_angle((1-weight)*p0[3]+weight*p1[3])
    res_p[4] = (1-weight)*p0[4] + weight*p1[4]
    res_p[5] = (1-weight)*p0[5] + weight*p1[5]
    return res_p


# x, y, s, theta, kappa, dkappa
class ReferenceLine:
    def __init__(self, file_name):
        self.reference_line = read_file(file_name)
        self.sorted_bound = gen_sorted_bound(self.reference_line)
        self.xy_tree = cKDTree(self.reference_line[:, 1:3])

    def get_match_index_s(self, s):
        return bisect(self.sorted_bound, s)

    def get_match_index_xy(self, point):
        return self.xy_tree.query(point, k=1)  ## index??

    def match_to_path_s(self, s):
        index_start = self.get_match_index_s(s)
        if index_start == len(self.reference_line)-1:
            return self.reference_line[index_start]
        return interpolate_linear(self.reference_line[index_start], self.reference_line[index_start+1], s)

    def match_to_path_xy(self, point):
        dis, index_min = self.get_match_index_xy(point)
        index_start = index_min
        index_end = index_min
        if index_min != 0:
            index_start = index_min - 1
        if index_min != len(self.reference_line) - 1:
            index_end = index_min + 1
        if index_end == index_start:
            return self.reference_line[index_start]
        return find_projection_point(self.reference_line[index_start], self.reference_line[index_end], point)

    def calculate_frenet(self, x, y, v, a, theta, kappa):
        return cartesian_to_frenet_array(self.match_to_path_xy(np.array([x, y])), x, y, v, a, theta, kappa)

    def calculate_cartesian_points(self, frenet_points):
        cartesian_points = []
        for point in frenet_points:
            reference_point = self.match_to_path_s(point[0])
            cartesian_points.append(frenet_to_cartesian_easy(reference_point, point[3:6]))
        return cartesian_points

    def calculate_cartesian(self, frenet):
        reference_point = self.match_to_path_s(frenet[0][0])
        return frenet_to_cartesian(reference_point, frenet[0], frenet[1])


# debug usage
if __name__ == '__main__':
    # tree = gen_kd_tree(read_file("highway_out.txt"))
    # print(tree.query([1765.1, 2972.9], k=1))
    refer = ReferenceLine("highway_out.txt")
    # print(refer.calculate_frenet(916.1138, 1126.566, 5.577, 18.15901, -0.0268993, 0.0))
    print(refer.calculate_frenet(966.23, 1138.63,  0.0, 0.0, 0.0, 0.0))
    print(refer.calculate_cartesian(refer.calculate_frenet(966.23, 1138.63, 0.0, 0.0, 0.0, 0.0)))

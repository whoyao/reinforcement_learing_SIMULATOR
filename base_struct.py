import numpy as np
import reference_line

TIME_HORIZON = 0.02
TIME_PREDICT = 4.0
TIME_KEEP = 0.5
NUM_KEEP = int(TIME_KEEP/TIME_HORIZON)
# TARGET_LON = np.array([[20.0, 0.0, 0.0], [40.0, 0.0, 0.0], [60.0, 0.0, 0.0]])
# TARGET_LAT = np.array([[-2.0, 0.0, 0.0], [0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
TARGET_LON = np.array([[60.0, 0.0, 0.0]])
TARGET_LAT = np.array([[2.0, 0.0, 0.0]])
TARGET = [(s, d) for s in TARGET_LON for d in TARGET_LAT]



class RoadPosition(object):
    def __init__(self):
        self.end_path_s = 0.
        self.end_path_d = 0.
        self.previous_path_x = []
        self.previous_path_y = []

    def update_road_position(self, whole_msgs):
        self.end_path_s = whole_msgs['end_path_s']
        self.end_path_d = whole_msgs['end_path_d']
        self.previous_path_x = whole_msgs['previous_path_x']
        self.previous_path_y = whole_msgs['previous_path_y']

    def get_end_sd(self):
        return np.array([self.end_path_s, self.end_path_d])

    def get_previous_path(self):
        return np.vstack((self.previous_path_x, self.previous_path_y))

    def get_start_point(self):
        index = NUM_KEEP - 1
        if len(self.previous_path_x) < 10:
            return None, None
        if len(self.previous_path_x) < NUM_KEEP:
            index = len(self.previous_path_x) - 1
        res_x = self.previous_path_x[index]
        res_y = self.previous_path_y[index]
        vx_t = (self.previous_path_x[index] - self.previous_path_x[index-1])/TIME_HORIZON
        vy_t = (self.previous_path_y[index] - self.previous_path_y[index-1])/TIME_HORIZON
        res_v = np.linalg.norm([vx_t, vy_t])
        res_theta = np.arctan(vy_t/vx_t)
        vx_t_minus_1 = (self.previous_path_x[index-1] - self.previous_path_x[index-2])/TIME_HORIZON
        vy_t_minus_1 = (self.previous_path_y[index-1] - self.previous_path_y[index-2])/TIME_HORIZON
        v_t_minus_1 = np.linalg.norm([vx_t_minus_1, vy_t_minus_1])
        res_a = (res_v - v_t_minus_1)/TIME_HORIZON
        return (res_x, res_y, res_v, res_a, res_theta, 0.0), np.hstack((self.previous_path_x[:index],
                                                                        self.previous_path_y[:index]))


# A 2d vector of cars and then that car's
# car's unique ID,
# car's x position in map coordinates,
# car's y position in map coordinates,
# car's x velocity in m/s,
# car's y velocity in m/s,
# car's s position in frenet coordinates,
# car's d position in frenet coordinates.
# def cartesian_to_frenet_point(reference, x, y, v, a, theta, kappa):
class SensorFusion(object):
    def __init__(self):
        self.other_cars = []

    def update_sensor_fusion(self, reference_line_in, whole_msgs):
        self.other_cars = np.array(whole_msgs['sensor_fusion'])
        car_frenet = []
        for car in self.other_cars:
            car_s, car_d = reference_line_in.calculate_frenet(car[1], car[2], np.linalg.norm([car[3], car[4]]), 0.0,
                                                              np.arctan(car[4]/max(car[3], np.finfo(float).eps)), 0.0)
            car_frenet.append(np.concatenate((car_s, car_d)))
        self.other_cars = np.hstack((self.other_cars, car_frenet))

    def generate_prediction(self, reference_line, start, end):
        reference_line = reference_line.ReferenceLine
        predictions = []
        for car in self.other_cars:
            car_s = car[7:10]
            car_d = car[10:]
            prediction_list = []
            t = start
            while t < end:
                prediction_list.append([car_s[0]+car_s[1]*t, car_d[0]+car_d[1]*t])
                t += TIME_HORIZON
            prediction = np.array(prediction_list)
            predictions.append(prediction)
        return predictions

    def get_surroud_car(self):
        pass


class CarStatus(object):
    def __init__(self):
        self.s = 0.0
        self.d = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0

    def update_car_status(self, whole_msgs):
        print(whole_msgs)
        self.s = whole_msgs['s']
        self.d = whole_msgs['d']
        self.x = whole_msgs['x']
        self.y = whole_msgs['y']
        self.yaw = whole_msgs['yaw']

    def get_car_speed(self):
        return self.speed

    def get_car_position_xyyaw(self):
        return np.array([self.x, self.y, self.yaw])

    def get_car_position_sd(self):
        return np.array([self.s, self.d])

    def get_car_start(self):
        return self.x, self.y, self.speed, 0.0, self.yaw, 0.0


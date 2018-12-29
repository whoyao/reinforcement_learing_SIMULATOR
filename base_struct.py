import numpy as np


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

#A 2d vector of cars and then that car's
# car's unique ID,
# car's x position in map coordinates,
# car's y position in map coordinates,
# car's x velocity in m/s,
# car's y velocity in m/s,
# car's s position in frenet coordinates,
# car's d position in frenet coordinates.
class SensorFusion(object):
    def __init__(self):
        self.other_cars = []

    def update_sensor_fusion(self, whole_msgs):
        self.other_cars = np.array(whole_msgs['sensor_fusion'])


class CarStatus(object):
    def __init__(self):
        self.s = 0.0
        self.d = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0

    def update_car_status(self, whole_msgs):
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


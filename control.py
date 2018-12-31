import numpy as np
from base_struct import RoadPosition, SensorFusion, CarStatus
from reference_line import ReferenceLine
from trajectory_gen import TrajectoryGen


class Control:
    def __init__(self, file_name):
        self.reference_line = ReferenceLine(file_name)
        self.car_status = CarStatus()
        self.sensor_fusion = SensorFusion()
        self.road_info = RoadPosition()
        self.trajectory_gen = TrajectoryGen()

    def update(self, detail_data):
        self.car_status.update_car_status(detail_data)
        self.sensor_fusion.update_sensor_fusion(self.reference_line, detail_data)
        self.road_info.update_road_position(detail_data)

        start_point_xy, keep_path = self.road_info.get_start_point()
        if start_point_xy is None:
            start_point_xy = self.car_status.get_car_start()
            keep_path = []
        start_point_sd = self.reference_line.calculate_frenet(start_point_xy)
        self.trajectory_gen.generate_trajectories(start_point_sd)
        return np.concatenate(keep_path, self.trajectory_gen.get_trajectory_xy(self.reference_line, 0))


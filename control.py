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
        start_point_sd = self.reference_line.calculate_frenet(start_point_xy[0], start_point_xy[1], start_point_xy[2],
                                                              start_point_xy[3], start_point_xy[4], start_point_xy[5])
        self.trajectory_gen.generate_trajectories(start_point_sd)
        trajectory_in_xy = self.trajectory_gen.get_trajectory_xy(self.reference_line, 0)
        if keep_path is None:
            return np.array(trajectory_in_xy)
        # print("keep_path:")
        # print(np.concatenate([keep_path, np.array(trajectory_in_xy)]))
        return np.concatenate([keep_path, np.array(trajectory_in_xy)])


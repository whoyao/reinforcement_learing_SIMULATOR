# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import asyncio
import json
import numpy as np
import websockets

from base_struct import CarStatus, RoadPosition, SensorFusion


car_status = CarStatus()
sensor_fusion = SensorFusion()
road_info = RoadPosition()


async def hello(websocket, path):
    while True:
        # await websocket.send(control_msg)
        try:
            message = await websocket.recv()
            if len(message)>2 and message.startswith("42"):
                message = message.lstrip("42")
                data = json.loads(message)
                status = data[0]
                detail_data = data[1]
                car_status.update_car_status(detail_data)
                sensor_fusion.update_sensor_fusion(detail_data)
                road_info.update_road_position(detail_data)
                print(car_status.get_car_position_xyyaw())
                print(sensor_fusion.other_cars)
                print(road_info.get_previous_path())
            else:
                continue
        except websockets.exceptions.ConnectionClosed:
            pass


start_server = websockets.serve(hello, 'localhost', 4567)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
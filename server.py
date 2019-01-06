# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import asyncio
import json
import numpy as np
import websockets

from base_struct import CarStatus, RoadPosition, SensorFusion
from control import Control


# car_status = CarStatus()
# sensor_fusion = SensorFusion()
# road_info = RoadPosition()

control = Control("highway_out.txt")

async def task_callback(websocket, path):
    while True:
        # await websocket.send(control_msg)
        try:
            message = await websocket.recv()
            if len(message)>2 and message.startswith("42"):
                message = message.lstrip("42")
                data = json.loads(message)
                status = data[0]
                detail_data = data[1]
                print(detail_data)
                new_path = control.update(detail_data)
                res_msg = json.dumps(['control', {'next_x': new_path[:, 0].tolist(), 'next_y':new_path[:, 1].tolist()}])
                print("42"+res_msg)
                await websocket.send("42"+res_msg)
            else:
                continue
        except websockets.exceptions.ConnectionClosed:
            pass


start_server = websockets.serve(task_callback, 'localhost', 4567)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
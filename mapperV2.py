#!/usr/bin/env python3
import asyncio
import numpy as np
import time
import threading
import cv2
from queue import Queue
import copy

from depth_receiver import DepthReceiver
from drone_control import Drone
from AvoidancePlanner import AvoidancePlanner
from get_position_with_task_v2 import Telemetry, position_monitor_task
#from SparseF import SparseFrontierMapper
from GlobalMapperV2  import GlobalMapper
import matplotlib.pyplot as plt

class SharedState:
    def __init__(self):
        self.bridgeque = Queue()
        self.lock = threading.Lock()
    def push(self, item):
        with self.lock:
            self.bridgeque.put(item)
    def pop(self):
        with self.lock:
            if not self.bridgeque.empty():
                return self.bridgeque.get()
            else:
                return None

class DroneNavigation:
    def __init__(self, bridge: SharedState,
                 depth_topic="/depth_camera",
                 loop_hz=20.0,):

        self.loop_hz = loop_hz
        self.running = True
        self.scale = 2.0
        self.bridge = bridge
        # =========================
        # GRID HEADING SYSTEM
        # =========================
        self.grid_headings = [0, 90, 180, -90]  # N, E, S, W
        self.current_heading_idx = 0
        self.target_yaw_deg = self.grid_headings[self.current_heading_idx]
        self.yaw_tolerance = 5.0

        # Camera intrinsics
        K = np.array([[433.0, 0.0, 320.0],
                      [0.0, 433.0, 240.0],
                      [0.0, 0.0, 1.0]])

        self.receiver = DepthReceiver(depth_topic)

        self.planner = AvoidancePlanner(
            K=K,
            width=640,
            height=480,
            safe_distance=4.0,
            critical_distance=1.5
        )

        self.drone = Drone()
        self.telemetry = Telemetry()
        self.mapper = GlobalMapper(K)    

    # =========================
    #  YAW UTILS
    # =========================
    def _yaw_error(self, target, current):
        error = target - current
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error

    async def align_to_grid(self):
        current_yaw = await self.drone.get_yaw()
        error = self._yaw_error(self.target_yaw_deg, current_yaw)

        if abs(error) > self.yaw_tolerance:
            print(f"Aligning to {self.target_yaw_deg}° (err={error:.2f})")
            await self.drone.rotate_to_yaw(self.target_yaw_deg)

    # =========================
    # 🔄 GRID TURNING
    # =========================
    async def rotate_next_direction(self):
        self.current_heading_idx = (self.current_heading_idx + 1) % 4
        self.target_yaw_deg = self.grid_headings[self.current_heading_idx]

        print(f"🔄 New heading: {self.target_yaw_deg}°")
        await self.drone.rotate_to_yaw(self.target_yaw_deg)

    # =========================
    # tHE MAIN LOOP WHERE THE PIPELINE COMES TOGETHER
    # =========================
    async def run(self):
        print("\nPOSITION-BASED AUTONOMOUS AvoidanceNAVIGATION\n")

        await self.drone.connect()
        print("Starting position monitor.")
        self.monitor_task = asyncio.create_task(position_monitor_task(self.drone, self.telemetry, asyncio.Event()))
        await self.drone.arm_and_takeoff()
        # Initial alignment
#        await self.drone.rotate_to_yaw(self.target_yaw_deg)
        n = self.telemetry.north
        e = self.telemetry.east
        print(F"Drone Origin Position: N: {n} E: {e} Yaw: {self.telemetry.yaw_deg}°")

        try:
            while self.running:
                depth_frame = self.receiver.get_frame()
                if depth_frame is not None:
                    t = copy.copy(self.telemetry)  # Get a snapshot of telemetry state
                    updated = self.mapper.update_frame(depth_frame, self.telemetry)
                    if updated is not None:
                        print(f"Map updated with new frame at pose N:{t.north:.2f} E:{t.east:.2f} Yaw:{t.yaw_deg:.2f}")
                        self.bridge.push(updated)
                        # Check for frontiers and decide if we need to turn
                await self.rotate_next_direction()
                await asyncio.sleep(1.0 / self.loop_hz)
        except Exception as e:
            print(f"Error in navigation loop: {type(e).__name__}: {e}")
        except asyncio.CancelledError:
            print("🛑 Navigation cancelled")
        finally:
            await self.drone.send_velocity(0, 0, 0, self.target_yaw_deg)
            print("Drone hovering safely")

    def stop(self):
        self.running = False

async def visualization_loop(bridge: SharedState):
    old_img = None
    while True:
        img = bridge.pop()
        if img is not None:
            old_img = img
        if old_img is not None:
            cv2.imshow("Map", old_img)
            cv2.waitKey(1)        
        await asyncio.sleep(0.1)

# =========================
#  ENTRY POINT
# =========================
async def main():
    bridge_state = SharedState() # to share the latest map image between the navigation and visualization tasks
    nav = DroneNavigation(bridge=bridge_state) # to run the navigation loop and to push map updates to the bridge
    vis_task = asyncio.create_task(visualization_loop(bridge=bridge_state)) # to display the map in almost real-time
    task = asyncio.create_task(nav.run())
    try:
        while True:
            await asyncio.sleep(1.0)
    except KeyboardInterrupt:
        print("\n Stopping...")
        nav.stop()
        await asyncio.gather(task, return_exceptions=True)


if __name__ == "__main__":
    asyncio.run(main())

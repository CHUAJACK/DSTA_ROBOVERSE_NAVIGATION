#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from drone_control import Drone
from mavsdk.offboard import VelocityNedYaw
import math
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
import asyncio

class Telemetry:
    """Thread-safe(ish) container for inter-task data in a single event loop."""
    def __init__(self):
        self.latest_position = None  # NED position from telemetry
        self.yaw_deg = None
        self.yaw_rad = None
        self.roll = None
        self.pitch = None
        self.is_armed = False
        self.control_active = False
        self.w = None
        self.x = None
        self.y = None
        self.z = None
        self.north = None
        self.east = None
        self.down = None

class ODOMtoBASELINKTF(Node):
    def __init__(self,drone:Drone,state: Telemetry,stop_event:asyncio.Event):
        super().__init__('ned_to_enu_converter')

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.ned_to_enu = R.from_matrix([
            [0,  1,  0],
            [1,  0,  0],
            [0,  0, -1]
        ])
        self.state = state
        self.drone = drone
        self.stop_event = stop_event

    async def position_monitor_task(self):
        async def stream_position():
            async for pos_vel in self.drone.drone.telemetry.position_velocity_ned():
                if self.stop_event.is_set():
                    break
                self.state.north = pos_vel.position.north_m
                self.state.east = pos_vel.position.east_m
                self.state.down = pos_vel.position.down_m
                if self.state.yaw_rad is not None :
                    self.odom_callback()

        async def stream_orientation():
            async for att in self.drone.drone.telemetry.attitude_euler():
                if self.stop_event.is_set():
                    break
                self.state.yaw_deg = att.yaw_deg
                self.state.roll = att.roll_deg
                self.state.pitch = att.pitch_deg
                self.state.w, self.state.x, self.state.y, self.state.z = euler_to_quaternion(self.state.roll, self.state.pitch, self.state.yaw_deg)
                self.state.yaw_rad = np.deg2rad(self.state.yaw_deg)
                
        try:
            await asyncio.gather(stream_position(),stream_orientation())
        except asyncio.CancelledError:
            print("position monitor task cancelled, ")
        except Exception as e:
            print(f"Monitor error: {type(e).__name__}: {e}")

    def odom_callback(self):
        # ... your existing NED→ENU conversion ...

        # ── Publish odom → base_link TF ───────────────────────
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        pos_ned = np.array([
            self.state.north,
            self.state.east,
            self.state.down
        ])
        pos_enu = self.ned_to_enu.apply(pos_ned)
        rot_ned = R.from_quat(
            [
                self.state.x,
                self.state.y,
                self.state.z,
                self.state.w
            ]
        )
        rot_enu = self.ned_to_enu * rot_ned
        q_enu = rot_enu.as_quat()  # [x, y, z, w]

        # Position (use already-converted ENU values)
        t.transform.translation.x = pos_enu[0]
        t.transform.translation.y = pos_enu[1]
        t.transform.translation.z = pos_enu[2]

        # Orientation (use already-converted ENU quaternion)
        t.transform.rotation.x = q_enu[0]
        t.transform.rotation.y = q_enu[1]
        t.transform.rotation.z = q_enu[2]
        t.transform.rotation.w = q_enu[3]

        self.tf_broadcaster.sendTransform(t)
    
    # async def start(self):
    #     try:
    #         await self.position_monitor_task()
    #     except asyncio.CancelledError:
    #         pass
    #     finally:
    #         self.destroy_node()


    # def run_node(self):
    #     asyncio.run(self.start())





def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    # Convert degrees to radians
    phi = math.radians(roll_deg)
    theta = math.radians(pitch_deg)
    psi = math.radians(yaw_deg)

    # Pre-calculate trig values for half-angles
    c_phi = math.cos(phi / 2)
    s_phi = math.sin(phi / 2)
    c_theta = math.cos(theta / 2)
    s_theta = math.sin(theta / 2)
    c_psi = math.cos(psi / 2)
    s_psi = math.sin(psi / 2)

    # Standard Hamilton convention (w, x, y, z)
    w = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi
    x = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi
    y = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi
    z = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi

    return w, x, y, z
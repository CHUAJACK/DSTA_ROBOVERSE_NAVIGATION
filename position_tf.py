#!/usr/bin/env python3
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from drone_control import Drone
from mavsdk.offboard import VelocityNedYaw
import math
import tf2_ros
import rclpy
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
    def __init__(self, Drone: Drone, state: Telemetry, stop_event: asyncio.Event):
        super().__init__('ned_to_enu_converter')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )])

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)


        self.ned_to_enu = R.from_matrix([
            [0,  1,  0],
            [1,  0,  0],
            [0,  0, -1]
        ])

        self.frd_to_flu = R.from_matrix([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ])
        self.state = state
        self.drone = Drone
        self.stop_event = stop_event

        # Wait until sim clock is received before doing anything
        self.get_logger().info('Waiting for sim clock...')
        while self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Sim clock ready: {self.get_clock().now().to_msg()}')

    async def position_monitor_task(self):
        async def stream_position():
            async for pos_vel in self.drone.telemetry.position_velocity_ned():
                if self.stop_event.is_set():
                    break
                self.state.north = pos_vel.position.north_m
                self.state.east = pos_vel.position.east_m
                self.state.down = pos_vel.position.down_m
                if self.state.yaw_rad is not None:
                    self.odom_callback()

        async def stream_orientation():
            async for att in self.drone.telemetry.attitude_euler():
                if self.stop_event.is_set():
                    break
                self.state.yaw_deg = att.yaw_deg
                self.state.roll = att.roll_deg
                self.state.pitch = att.pitch_deg
                self.state.yaw_rad = np.deg2rad(self.state.yaw_deg)
                
                # FIX 2: Use scipy with ZYX (aerospace convention) instead of
                # custom euler_to_quaternion which had sign errors
                rot = R.from_euler('ZYX', [
                    att.yaw_deg,
                    att.pitch_deg,
                    att.roll_deg
                ], degrees=True)
                q = rot.as_quat()  # [x, y, z, w]
                self.state.x = q[0]
                self.state.y = q[1]
                self.state.z = q[2]
                self.state.w = q[3]
        try:
            await asyncio.gather(stream_position(), stream_orientation())
        except asyncio.CancelledError:
            print("position monitor task cancelled")
        except Exception as e:
            print(f"Monitor error: {type(e).__name__}: {e}")

    def odom_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # FIX 3: Explicit axis swap instead of matrix multiply for position
        # NED → ENU: X=East=NED_Y, Y=North=NED_X, Z=Up=-NED_Down
        t.transform.translation.x =  self.state.east   # NED East  → ENU X
        t.transform.translation.y =  self.state.north  # NED North → ENU Y
        t.transform.translation.z = -self.state.down   # NED Down  → ENU Z (flip sign)
        test = R.from_euler('ZYX', [0, 0, 0], degrees=True)  # flat drone facing North
        result = self.ned_to_enu * test
        # Orientation: apply NED→ENU frame rotation to the body quaternion
        rot_ned = R.from_quat(
            [
                self.state.x,
                self.state.y,
                self.state.z,
                self.state.w
            ]
        )
        
        # Apply World frame conversion (NED->ENU) AND Body frame conversion (FRD->FLU)
        rot_enu = self.ned_to_enu * rot_ned * self.frd_to_flu
        
        q_enu = rot_enu.as_quat()  # [x, y, z, w]

        t.transform.rotation.x = q_enu[0]
        t.transform.rotation.y = q_enu[1]
        t.transform.rotation.z = q_enu[2]
        t.transform.rotation.w = q_enu[3]

        self.tf_broadcaster.sendTransform(t)
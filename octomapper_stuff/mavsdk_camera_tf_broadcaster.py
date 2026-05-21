#!/usr/bin/env python3

import asyncio
import math
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from mavsdk import System


def rotation_matrix_to_quaternion(R):
    """
    Convert a 3x3 rotation matrix to quaternion x, y, z, w.
    """

    trace = np.trace(R)

    if trace > 0.0:
        S = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S

    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S

    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S

    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    return qx, qy, qz, qw


def camera_rotation_matrix_from_rpy(roll_deg, pitch_deg, yaw_deg):
    """
    Build camera-to-map rotation using full drone attitude.

    PX4 / MAVSDK body frame:
        body x = forward
        body y = right
        body z = down

    Camera frame from your generated point cloud:
        camera x = right
        camera y = down
        camera z = forward

    NED frame:
        x = north
        y = east
        z = down

    RViz / map frame:
        x = east
        y = north
        z = up
    """

    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll)
    sr = math.sin(roll)

    cp = math.cos(pitch)
    sp = math.sin(pitch)

    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # Rotation from body FRD to NED.
    # Body frame:
    #   x = forward
    #   y = right
    #   z = down
    R_body_to_ned = np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp,     sr * cp,                cr * cp],
    ])

    # Camera frame to body frame.
    #
    # Camera:
    #   x = right
    #   y = down
    #   z = forward
    #
    # Body:
    #   x = forward = camera z
    #   y = right   = camera x
    #   z = down    = camera y
    R_camera_to_body = np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ])

    # Camera to NED
    R_camera_to_ned = R_body_to_ned @ R_camera_to_body

    # NED to RViz/map.
    #
    # map x = east
    # map y = north
    # map z = up = -down
    R_ned_to_map = np.array([
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ])

    R_camera_to_map = R_ned_to_map @ R_camera_to_ned

    return R_camera_to_map


class CameraTFBroadcaster(Node):
    def __init__(
        self,
        parent_frame="map",
        child_frame="camera_link",
        camera_forward_offset=0.0,
        camera_right_offset=0.0,
        camera_down_offset=0.0,
    ):
        super().__init__("mavsdk_camera_tf_broadcaster")

        # Use Gazebo / ROS simulation time by default.
        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])

        self.parent_frame = parent_frame
        self.child_frame = child_frame

        # Camera offset relative to drone body frame.
        # Units: metres
        # Convention:
        #   forward = body x
        #   right   = body y
        #   down    = body z
        self.camera_forward_offset = camera_forward_offset
        self.camera_right_offset = camera_right_offset
        self.camera_down_offset = camera_down_offset

        self.tf_broadcaster = TransformBroadcaster(self)

        self.north = None
        self.east = None
        self.down = None

        self.roll_deg = None
        self.pitch_deg = None
        self.yaw_deg = None

    def update_position(self, north, east, down):
        self.north = north
        self.east = east
        self.down = down

    def update_attitude(self, roll_deg, pitch_deg, yaw_deg):
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.yaw_deg = yaw_deg

    def publish_tf(self):
        if (
            self.north is None or
            self.east is None or
            self.down is None or
            self.roll_deg is None or
            self.pitch_deg is None or
            self.yaw_deg is None
        ):
            return

        yaw_rad = math.radians(self.yaw_deg)
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)

        # Offset calculation still uses yaw only.
        # This is acceptable for small camera mounting offsets.
        # If the camera is far from the drone centre, this can also be upgraded
        # to use full roll/pitch/yaw.
        offset_north = (
            self.camera_forward_offset * c -
            self.camera_right_offset * s
        )
        offset_east = (
            self.camera_forward_offset * s +
            self.camera_right_offset * c
        )
        offset_down = self.camera_down_offset

        cam_north = self.north + offset_north
        cam_east = self.east + offset_east
        cam_down = self.down + offset_down

        # Full roll, pitch, yaw camera-frame rotation.
        R = camera_rotation_matrix_from_rpy(
            self.roll_deg,
            self.pitch_deg,
            self.yaw_deg,
        )
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # Convert NED to RViz/map:
        #   map x = east
        #   map y = north
        #   map z = up = -down
        t.transform.translation.x = float(cam_east)
        t.transform.translation.y = float(cam_north)
        t.transform.translation.z = float(-cam_down)

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)


async def mavsdk_position_task(drone, tf_node):
    async for pos in drone.telemetry.position_velocity_ned():
        tf_node.update_position(
            north=pos.position.north_m,
            east=pos.position.east_m,
            down=pos.position.down_m,
        )


async def mavsdk_attitude_task(drone, tf_node):
    async for att in drone.telemetry.attitude_euler():
        tf_node.update_attitude(
            roll_deg=att.roll_deg,
            pitch_deg=att.pitch_deg,
            yaw_deg=att.yaw_deg,
        )


async def ros_tf_publish_task(tf_node):
    while rclpy.ok():
        tf_node.publish_tf()
        rclpy.spin_once(tf_node, timeout_sec=0.0)
        await asyncio.sleep(0.02)  # 50 Hz


async def main():
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("--child-frame", default="camera_link")
    args = parser.parse_args()

    # IMPORTANT:
    # child_frame must match the frame_id from:
    # ros2 topic echo /depth_camera/points_fast --once | grep frame_id
    tf_node = CameraTFBroadcaster(
        parent_frame="map",
        child_frame=args.child_frame,
        camera_forward_offset=0.0,
        camera_right_offset=0.0,
        camera_down_offset=0.0,
    )

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for PX4 connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("PX4 connected.")
            break

    await asyncio.gather(
        mavsdk_position_task(drone, tf_node),
        mavsdk_attitude_task(drone, tf_node),
        ros_tf_publish_task(tf_node),
    )


if __name__ == "__main__":
    asyncio.run(main())
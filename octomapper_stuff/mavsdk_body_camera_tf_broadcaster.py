#!/usr/bin/env python3

import asyncio
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from mavsdk import System
from scipy.spatial.transform import Rotation as R


class Telemetry:
    """
    Shared telemetry container.
    MAVSDK position is NED:
        north, east, down

    MAVSDK body frame is FRD:
        forward, right, down

    ROS base_link convention is FLU:
        forward, left, up
    """

    def __init__(self):
        self.north = None
        self.east = None
        self.down = None

        self.roll_deg = None
        self.pitch_deg = None
        self.yaw_deg = None


class MavsdkBodyCameraTFBroadcaster(Node):
    def __init__(
        self,
        drone: System,
        state: Telemetry,
        stop_event: asyncio.Event,
        parent_frame="map",
        base_frame="base_link",
        camera_frame="camera_link",
        camera_forward_offset=0.0,
        camera_left_offset=0.0,
        camera_up_offset=-0.10,
    ):
        super().__init__("mavsdk_body_camera_tf_broadcaster")

        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])

        self.drone = drone
        self.state = state
        self.stop_event = stop_event

        self.parent_frame = parent_frame
        self.base_frame = base_frame
        self.camera_frame = camera_frame

        # Camera position relative to base_link.
        #
        # base_link convention:
        #   x = forward
        #   y = left
        #   z = up
        #
        # If camera is below the drone, camera_up_offset should be negative.
        self.camera_forward_offset = camera_forward_offset
        self.camera_left_offset = camera_left_offset
        self.camera_up_offset = camera_up_offset

        self.tf_broadcaster = TransformBroadcaster(self)

        # NED world to ROS map/ENU-like world:
        # map x = east
        # map y = north
        # map z = up = -down
        self.ned_to_map = R.from_matrix([
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

        # Body FRD to ROS base_link FLU:
        # FRD:
        #   x = forward
        #   y = right
        #   z = down
        #
        # FLU:
        #   x = forward
        #   y = left  = -right
        #   z = up    = -down
        self.frd_to_flu = R.from_matrix([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

        # Camera frame from your point cloud:
        #   camera x = right
        #   camera y = down
        #   camera z = forward
        #
        # base_link frame:
        #   base x = forward
        #   base y = left
        #   base z = up
        #
        # Therefore:
        #   camera x right   -> base -y
        #   camera y down    -> base -z
        #   camera z forward -> base +x
        self.camera_to_base = R.from_matrix([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ])

        self.get_logger().info("TF broadcaster initialized.")
        self.get_logger().info(
            f"Publishing {self.parent_frame} -> {self.base_frame} -> {self.camera_frame}"
        )

    async def telemetry_task(self):
        async def stream_position():
            async for pos_vel in self.drone.telemetry.position_velocity_ned():
                if self.stop_event.is_set():
                    break

                self.state.north = pos_vel.position.north_m
                self.state.east = pos_vel.position.east_m
                self.state.down = pos_vel.position.down_m

        async def stream_attitude():
            async for att in self.drone.telemetry.attitude_euler():
                if self.stop_event.is_set():
                    break

                self.state.roll_deg = att.roll_deg
                self.state.pitch_deg = att.pitch_deg
                self.state.yaw_deg = att.yaw_deg

        try:
            await asyncio.gather(
                stream_position(),
                stream_attitude(),
            )

        except asyncio.CancelledError:
            self.get_logger().info("Telemetry task cancelled.")

        except Exception as e:
            self.get_logger().error(
                f"Telemetry error: {type(e).__name__}: {e}"
            )

    def publish_tf(self):
        if (
            self.state.north is None or
            self.state.east is None or
            self.state.down is None or
            self.state.roll_deg is None or
            self.state.pitch_deg is None or
            self.state.yaw_deg is None
        ):
            return

        now = self.get_clock().now().to_msg()

        # =====================================================
        # map → base_link
        # =====================================================

        # MAVSDK attitude is roll, pitch, yaw in NED/FRD convention.
        #
        # This creates body FRD relative to NED.
        rot_body_frd_in_ned = R.from_euler(
            "ZYX",
            [
                self.state.yaw_deg,
                self.state.pitch_deg,
                self.state.roll_deg,
            ],
            degrees=True,
        )

        # Convert:
        # NED world → ROS map
        # FRD body  → FLU base_link
        #
        # Result is base_link orientation in map frame.
        rot_base_in_map = (
            self.ned_to_map *
            rot_body_frd_in_ned *
            self.frd_to_flu
        )

        q_base = rot_base_in_map.as_quat()  # [x, y, z, w]

        base_tf = TransformStamped()
        base_tf.header.stamp = now
        base_tf.header.frame_id = self.parent_frame
        base_tf.child_frame_id = self.base_frame

        # NED position to ROS map position
        base_tf.transform.translation.x = float(self.state.east)
        base_tf.transform.translation.y = float(self.state.north)
        base_tf.transform.translation.z = float(-self.state.down)

        base_tf.transform.rotation.x = float(q_base[0])
        base_tf.transform.rotation.y = float(q_base[1])
        base_tf.transform.rotation.z = float(q_base[2])
        base_tf.transform.rotation.w = float(q_base[3])

        self.tf_broadcaster.sendTransform(base_tf)

        # =====================================================
        # base_link → camera_link
        # =====================================================

        q_camera = self.camera_to_base.as_quat()  # [x, y, z, w]

        camera_tf = TransformStamped()
        camera_tf.header.stamp = now
        camera_tf.header.frame_id = self.base_frame
        camera_tf.child_frame_id = self.camera_frame

        camera_tf.transform.translation.x = float(self.camera_forward_offset)
        camera_tf.transform.translation.y = float(self.camera_left_offset)
        camera_tf.transform.translation.z = float(self.camera_up_offset)

        camera_tf.transform.rotation.x = float(q_camera[0])
        camera_tf.transform.rotation.y = float(q_camera[1])
        camera_tf.transform.rotation.z = float(q_camera[2])
        camera_tf.transform.rotation.w = float(q_camera[3])

        self.tf_broadcaster.sendTransform(camera_tf)

        self.get_logger().info(
            f"Published TF | "
            f"N={self.state.north:.2f}, "
            f"E={self.state.east:.2f}, "
            f"D={self.state.down:.2f}, "
            f"R={self.state.roll_deg:.1f}, "
            f"P={self.state.pitch_deg:.1f}, "
            f"Y={self.state.yaw_deg:.1f}",
            throttle_duration_sec=1.0
        )


async def ros_spin_task(node: MavsdkBodyCameraTFBroadcaster):
    while rclpy.ok() and not node.stop_event.is_set():
        node.publish_tf()
        rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(0.02)  # 50 Hz


async def main():
    rclpy.init()

    stop_event = asyncio.Event()
    state = Telemetry()

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for PX4 connection...")
    async for connection_state in drone.core.connection_state():
        if connection_state.is_connected:
            print("PX4 connected.")
            break

    tf_node = MavsdkBodyCameraTFBroadcaster(
        drone=drone,
        state=state,
        stop_event=stop_event,

        parent_frame="map",
        base_frame="base_link",
        camera_frame="camera_link",

        # Tune these if needed.
        # base_link convention:
        #   x = forward
        #   y = left
        #   z = up
        camera_forward_offset=0.10,
        camera_left_offset=0.0,
        camera_up_offset=-0.0,
    )

    try:
        await asyncio.gather(
            tf_node.telemetry_task(),
            ros_spin_task(tf_node),
        )

    except KeyboardInterrupt:
        print("\nKeyboard interrupt. Shutting down...")

    finally:
        stop_event.set()
        tf_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
#!/usr/bin/env python3

import asyncio
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image


class FastDepthToPointCloud(Node):
    def __init__(
        self,
        depth_topic="/depth_camera",
        output_topic="/depth_camera/points_fast",
        frame_id="camera_link",
        width=640,
        height=480,
        fx=433.0,
        fy=433.0,
        cx=320.0,
        cy=240.0,
        stride=8,
        min_depth=0.1,
        max_depth=10.0,
        publish_rate_hz=5.0,
    ):
        super().__init__("fast_depth_to_pointcloud")

        # Use Gazebo sim time by default
        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])

        self.depth_topic = depth_topic
        self.output_topic = output_topic
        self.frame_id = frame_id

        self.width = width
        self.height = height

        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

        self.stride = stride
        self.min_depth = min_depth
        self.max_depth = max_depth

        self.latest_depth = None

        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.gz_node = GzNode()
        ok = self.gz_node.subscribe(Image, depth_topic, self.gz_callback)

        if ok:
            self.get_logger().info(f"Subscribed to Gazebo depth topic: {depth_topic}")
        else:
            self.get_logger().error(f"Failed to subscribe to Gazebo topic: {depth_topic}")

        timer_period = 1.0 / publish_rate_hz
        self.timer = self.create_timer(timer_period, self.publish_pointcloud)

        self.get_logger().info(
            f"Publishing fast point cloud to {output_topic}, "
            f"frame_id={frame_id}, stride={stride}, max_depth={max_depth}"
        )

    def gz_callback(self, msg: Image):
        """
        Receives Gazebo depth image.
        Assumes depth image is float32 metres.
        """

        try:
            depth = np.frombuffer(msg.data, dtype=np.float32)

            expected = msg.width * msg.height

            if depth.size != expected:
                self.get_logger().warn(
                    f"Unexpected depth size. Got {depth.size}, expected {expected}. "
                    f"width={msg.width}, height={msg.height}"
                )
                return

            depth = depth.reshape((msg.height, msg.width))
            self.latest_depth = depth.copy()

        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {type(e).__name__}: {e}")

    def depth_to_points(self, depth):
        """
        Converts depth image to downsampled point cloud.

        Camera frame convention:
        x = right
        y = down
        z = forward

        Filters out false obstacle points between 0.9 m and 1.1 m
        in front of the camera.
        """

        h, w = depth.shape

        v_coords = np.arange(0, h, self.stride)
        u_coords = np.arange(0, w, self.stride)

        uu, vv = np.meshgrid(u_coords, v_coords)

        z = depth[vv, uu]

        # Basic valid depth mask
        valid = (
            np.isfinite(z) &
            (z > self.min_depth) &
            (z < self.max_depth)
        )

        # -------------------------------------------------
        # Exclude false obstacle band in front of camera
        # Camera z = forward distance
        # Remove points between 0.9 m and 1.1 m
        # -------------------------------------------------
        false_obstacle_band = (z >= 0.9) & (z <= 1.1)

        valid = valid & (~false_obstacle_band)

        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        if z.size == 0:
            return []

        x = (uu - self.cx) * z / self.fx
        y = (vv - self.cy) * z / self.fy

        points = np.column_stack((x, y, z)).astype(np.float32)

        return points.tolist()

    def publish_pointcloud(self):
        if self.latest_depth is None:
            return

        points = self.depth_to_points(self.latest_depth)

        if len(points) == 0:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, points)
        self.pub.publish(cloud_msg)

        self.get_logger().info(
            f"Published {len(points)} points to {self.output_topic}",
            throttle_duration_sec=2.0
        )


async def ros_spin(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(0.001)


async def main():
    rclpy.init()

    node = FastDepthToPointCloud(
        depth_topic="/depth_camera",
        output_topic="/depth_camera/points_fast",
        frame_id="camera_link",
        stride=8,
        min_depth=0.3,
        max_depth=10.0,
        publish_rate_hz=5.0,
    )

    try:
        await ros_spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
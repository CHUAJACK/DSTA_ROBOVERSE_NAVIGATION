#!/usr/bin/env python3
"""
Stride-based PointCloud2 downsampler.
Takes every STRIDE-th point in each dimension of a structured depth cloud,
reducing point count by STRIDE² without affecting spatial coverage.
Publishes to /depth_camera/points_filtered for OctoMap consumption.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

STRIDE = 2   # take every 2nd point → 4× fewer points


class PointCloudDownsampler(Node):
    def __init__(self):
        super().__init__('pointcloud_downsampler')
        self.sub = self.create_subscription(
            PointCloud2, '/depth_camera/points', self._cb, 2)
        self.pub = self.create_publisher(
            PointCloud2, '/depth_camera/points_filtered', 2)

    def _cb(self, msg: PointCloud2):
        w, h = msg.width, msg.height
        if w == 0 or h == 0:
            return

        # Slice rows and columns by STRIDE
        new_h = (h + STRIDE - 1) // STRIDE
        new_w = (w + STRIDE - 1) // STRIDE

        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w * msg.point_step)
        downsampled = raw[::STRIDE, :].reshape(new_h, w, msg.point_step)[:, ::STRIDE, :]
        flat = np.ascontiguousarray(downsampled).tobytes()

        out = PointCloud2()
        out.header = msg.header
        out.height = new_h
        out.width = new_w
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = new_w * msg.point_step
        out.is_dense = msg.is_dense
        out.data = flat
        try:
            self.pub.publish(out)
        except Exception:
            pass


def main():
    rclpy.init()
    node = PointCloudDownsampler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

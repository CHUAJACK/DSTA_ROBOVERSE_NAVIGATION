#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import octomap
import numpy as np
from octomap_msgs.msg import Octomap
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import matplotlib.pyplot as plt
import threading

class OctoMapSlicer(Node):
    def __init__(self):
        super().__init__('octomap_slicer')

        # Declare parameters (set via CLI or YAML)
        self.declare_parameter('slice_z', 0.5)        # height to slice (meters)
        self.declare_parameter('z_tolerance', 0.2)    # ± band around slice height
        self.declare_parameter('frame_id', 'map')

        self.slice_z     = self.get_parameter('slice_z').value
        self.z_tolerance = self.get_parameter('z_tolerance').value
        self.frame_id    = self.get_parameter('frame_id').value
        self.lock = threading.Lock()
        self.latest = None
        self.sub = self.create_subscription(
            Octomap,
            '/octomap_binary',
            self.callback,
            10
        )

        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            '/octomap_2d_slice',
            10
        )

        self.get_logger().info(f'Slicing at Z={self.slice_z} ± {self.z_tolerance}m')

    def callback(self, msg: Octomap):
        # 1. Deserialize binary OctoMap
        tree = octomap.OcTree(msg.resolution)
        tree.readBinary(bytes(msg.data))

        # 2. Get metric bounds
        aabb_min = np.zeros(3)
        aabb_max = np.zeros(3)
        tree.getMetricMin(aabb_min[0], aabb_min[1], aabb_min[2])
        tree.getMetricMax(aabb_max[0], aabb_max[1], aabb_max[2])

        x_min, y_min = aabb_min[0], aabb_min[1]
        x_max, y_max = aabb_max[0], aabb_max[1]
        res = msg.resolution

        # 3. Allocate 2D grid (X-Y plane, fixed Z slice)
        x_cells = int((x_max - x_min) / res) + 1
        y_cells = int((y_max - y_min) / res) + 1
        # -1 = unknown, 0 = free, 100 = occupied
        grid = np.full((y_cells, x_cells), -1, dtype=np.int8)

        # 4. Query only the Z band using bounding box iterator
        bbx_min = np.array([x_min, y_min, self.slice_z - self.z_tolerance])
        bbx_max = np.array([x_max, y_max, self.slice_z + self.z_tolerance])

        for node, x, y, z in tree.begin_leafs_bbx(bbx_min, bbx_max):
            xi = int((x - x_min) / res)
            yi = int((y - y_min) / res)

            # Clamp to grid bounds (float precision safety)
            xi = max(0, min(xi, x_cells - 1))
            yi = max(0, min(yi, y_cells - 1))

            if tree.isNodeOccupied(node):
                grid[yi, xi] = 100   # occupied
            else:
                # Only mark free if not already occupied (don't overwrite)
                if grid[yi, xi] != 100:
                    grid[yi, xi] = 0

        # 5. Publish as OccupancyGrid
        self.publish_grid(grid, x_min, y_min, res, msg.header.stamp)
        with self.lock:
            self.latest = np.copy(grid)
    def save(self, filename="octomap.png"):
        with self.lock:
            if self.latest is None:
                self.get_logger().warning("No map available.")
                return

            grid = np.copy(self.latest)

        display = np.zeros_like(grid, dtype=np.uint8)

        # unknown -> gray
        display[grid == -1] = 127

        # free -> white
        display[grid == 0] = 255

        # occupied -> black
        display[grid == 100] = 0

        plt.figure(figsize=(10, 10))
        plt.imshow(display, cmap='gray', origin='lower')
        plt.title(f"OctoMap Slice Z={self.slice_z}")
        plt.savefig(filename, bbox_inches='tight')
        plt.close()

        self.get_logger().info(f"Saved map to {filename}")

    def publish_grid(self, grid, x_origin, y_origin, res, stamp):
        return
        og = OccupancyGrid()
        og.header.frame_id = self.frame_id
        og.header.stamp    = stamp  # use octomap's timestamp

        og.info.resolution = res
        og.info.width      = grid.shape[1]
        og.info.height     = grid.shape[0]

        og.info.origin          = Pose()
        og.info.origin.position.x = x_origin
        og.info.origin.position.y = y_origin
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0  # no rotation

        # OccupancyGrid expects row-major, flattened
        og.data = grid.flatten().tolist()

        self.grid_pub.publish(og)
        self.get_logger().info(
            f'Published 2D grid {grid.shape[1]}x{grid.shape[0]} '
            f'@ Z={self.slice_z}m, origin=({x_origin:.2f}, {y_origin:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OctoMapSlicer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
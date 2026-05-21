#!/usr/bin/env python3

import os
import sys
import math
import asyncio
import heapq
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from mavsdk import System

# Optional portable library support
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PORTABLE_LIBS = os.path.join(CURRENT_DIR, "pathfinding3d")

if os.path.isdir(PORTABLE_LIBS) and PORTABLE_LIBS not in sys.path:
    sys.path.insert(0, PORTABLE_LIBS)

from pathfinding3d.core.grid import Grid
from pathfinding3d.finder.a_star import AStarFinder
from pathfinding3d.core.diagonal_movement import DiagonalMovement

from octomap_to_numpy_grid import OctomapToNumpyGrid


class DroneState:
    def __init__(self):
        self.north = None
        self.east = None
        self.down = None
        self.yaw_deg = None


class FrontierPathfinder(Node):
    def __init__(self, grid_node: OctomapToNumpyGrid):
        super().__init__("frontier_pathfinder")

        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])

        self.grid_node = grid_node

        self.path_pub = self.create_publisher(
            Path,
            "/frontier_path",
            10
        )

        self.target_pub = self.create_publisher(
            MarkerArray,
            "/selected_frontier_target",
            10
        )

        self.get_logger().info("Frontier pathfinder ready.")

    # =========================================================
    # Utility
    # =========================================================

    def distance_ned(self, a, b):
        """
        a, b are [north, east, down]
        """
        return math.sqrt(
            (a[0] - b[0]) ** 2 +
            (a[1] - b[1]) ** 2 +
            (a[2] - b[2]) ** 2
        )

    def grid_node_to_tuple(self, node):
        """
        pathfinding3D returns node objects.
        This converts them to ix, iy, iz.
        """
        return int(node.x), int(node.y), int(node.z)

    def is_valid_grid_index(self, ix, iy, iz):
        return self.grid_node.is_in_bounds(ix, iy, iz)

    # =========================================================
    # Frontier priority queue
    # =========================================================

    def build_frontier_priority_queue(self, drone_ned):
        """
        Priority queue of frontier clusters, nearest first.

        Returns:
            heap of (distance, index, frontier_ned)
        """

        clusters = self.grid_node.get_frontier_clusters(
            min_cluster_size=5
        )

        heap = []

        for i, frontier_ned in enumerate(clusters):
            dist = self.distance_ned(drone_ned, frontier_ned)
            heapq.heappush(heap, (dist, i, frontier_ned))

        return heap

    # =========================================================
    # Pathfinding3D
    # =========================================================

    def plan_to_frontier(self, drone_ned, frontier_ned):
        """
        Try planning from drone position to one frontier.

        Returns:
            path_ned: np.ndarray of shape (N, 3), or None
        """

        cost_grid = self.grid_node.get_pathfinding_grid()

        start_ix, start_iy, start_iz = self.grid_node.ned_position_to_grid(
            north=drone_ned[0],
            east=drone_ned[1],
            down=drone_ned[2],
        )

        goal_ix, goal_iy, goal_iz = self.grid_node.ned_position_to_grid(
            north=frontier_ned[0],
            east=frontier_ned[1],
            down=frontier_ned[2],
        )

        if not self.is_valid_grid_index(start_ix, start_iy, start_iz):
            self.get_logger().warn(
                f"Start outside grid: {start_ix}, {start_iy}, {start_iz}"
            )
            return None

        if not self.is_valid_grid_index(goal_ix, goal_iy, goal_iz):
            self.get_logger().warn(
                f"Goal outside grid: {goal_ix}, {goal_iy}, {goal_iz}"
            )
            return None

        if cost_grid[start_ix, start_iy, start_iz] <= 0.0:
            self.get_logger().warn(
                f"Start is blocked: {start_ix}, {start_iy}, {start_iz}"
            )
            return None

        if cost_grid[goal_ix, goal_iy, goal_iz] <= 0.0:
            self.get_logger().warn(
                f"Goal is blocked: {goal_ix}, {goal_iy}, {goal_iz}"
            )
            return None

        grid = Grid(matrix=cost_grid)

        start = grid.node(start_ix, start_iy, start_iz)
        end = grid.node(goal_ix, goal_iy, goal_iz)

        if start is None or end is None:
            self.get_logger().warn("pathfinding3D returned invalid start/end node.")
            return None

        finder = AStarFinder(
            diagonal_movement=DiagonalMovement.always
        )

        path, runs = finder.find_path(start, end, grid)

        if len(path) == 0:
            return None

        path_ned = []

        for node in path:
            ix, iy, iz = self.grid_node_to_tuple(node)
            north, east, down = self.grid_node.grid_to_ned_position(ix, iy, iz)
            path_ned.append([north, east, down])

        path_ned = np.array(path_ned, dtype=np.float32)

        self.get_logger().info(
            f"Found path to frontier. "
            f"nodes={len(path)}, runs={runs}"
        )

        return path_ned

    def find_best_frontier_path(self, drone_ned, max_attempts=20):
        """
        Try frontiers from nearest to furthest until a valid path is found.
        """

        heap = self.build_frontier_priority_queue(drone_ned)

        if len(heap) == 0:
            self.get_logger().warn("No frontier clusters found.")
            return None, None

        attempts = 0

        while heap and attempts < max_attempts:
            dist, _, frontier_ned = heapq.heappop(heap)
            attempts += 1

            self.get_logger().info(
                f"Trying frontier {attempts}, distance={dist:.2f} m, "
                f"N={frontier_ned[0]:.2f}, "
                f"E={frontier_ned[1]:.2f}, "
                f"D={frontier_ned[2]:.2f}"
            )

            path = self.plan_to_frontier(drone_ned, frontier_ned)

            if path is not None:
                return path, frontier_ned

        self.get_logger().warn("No reachable frontier found.")
        return None, None

    # =========================================================
    # RViz publishing
    # =========================================================

    def publish_path(self, path_ned):
        """
        Publish path as nav_msgs/Path in map frame.
        """

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for north, east, down in path_ned:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = msg.header.stamp

            # NED to map
            pose.pose.position.x = float(east)
            pose.pose.position.y = float(north)
            pose.pose.position.z = float(-down)

            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def publish_selected_target(self, frontier_ned):
        """
        Publish selected frontier target as a red sphere.
        """

        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "selected_frontier"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(frontier_ned[1])   # east
        marker.pose.position.y = float(frontier_ned[0])   # north
        marker.pose.position.z = float(-frontier_ned[2])  # up
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.7
        marker.scale.y = 0.7
        marker.scale.z = 0.7

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker_array.markers.append(marker)
        self.target_pub.publish(marker_array)


# =========================================================
# MAVSDK telemetry
# =========================================================

async def mavsdk_position_task(drone, state: DroneState):
    async for pv in drone.telemetry.position_velocity_ned():
        state.north = pv.position.north_m
        state.east = pv.position.east_m
        state.down = pv.position.down_m


async def mavsdk_yaw_task(drone, state: DroneState):
    async for att in drone.telemetry.attitude_euler():
        state.yaw_deg = att.yaw_deg


async def ros_spin_task(nodes):
    while rclpy.ok():
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(0.01)


async def planner_loop(pathfinder: FrontierPathfinder, drone_state: DroneState):
    """
    Replans periodically to the nearest reachable frontier.
    """

    while rclpy.ok():
        if not pathfinder.grid_node.has_map:
            pathfinder.get_logger().info(
                "Waiting for numpy grid map...",
                throttle_duration_sec=2.0
            )
            await asyncio.sleep(0.5)
            continue

        if (
            drone_state.north is None or
            drone_state.east is None or
            drone_state.down is None
        ):
            pathfinder.get_logger().info(
                "Waiting for MAVSDK drone position...",
                throttle_duration_sec=2.0
            )
            await asyncio.sleep(0.5)
            continue

        drone_ned = np.array([
            drone_state.north,
            drone_state.east,
            drone_state.down,
        ], dtype=np.float32)

        path, target = pathfinder.find_best_frontier_path(
            drone_ned,
            max_attempts=20
        )

        if path is not None:
            pathfinder.publish_path(path)
            pathfinder.publish_selected_target(target)

        await asyncio.sleep(2.0)


async def main():
    rclpy.init()

    # This node builds the live numpy grid and detects frontiers.
    grid_node = OctomapToNumpyGrid(
        occupied_topic="/occupied_cells_vis_array",
        free_topic="/free_cells_vis_array",

        resolution=0.5,

        size_x=40.0,
        size_y=40.0,
        size_z=8.5,

        origin_x=0.0,
        origin_y=0.0,
        origin_z=0.0,

        inflation_radius=1.0,
        soft_radius=2.0,
        unknown_cost=5.0,
        roof_height=8.0,
    )

    pathfinder = FrontierPathfinder(grid_node)

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for PX4 connection...")
    async for connection_state in drone.core.connection_state():
        if connection_state.is_connected:
            print("PX4 connected.")
            break

    drone_state = DroneState()

    try:
        await asyncio.gather(
            mavsdk_position_task(drone, drone_state),
            mavsdk_yaw_task(drone, drone_state),
            ros_spin_task([grid_node, pathfinder]),
            planner_loop(pathfinder, drone_state),
        )

    finally:
        grid_node.destroy_node()
        pathfinder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
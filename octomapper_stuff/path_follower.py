#!/usr/bin/env python3

import asyncio
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav_msgs.msg import Path

from drone_control import Drone


class PathFollower(Node):
    def __init__(
        self,
        drone: Drone,
        path_topic="/frontier_path",
        waypoint_spacing=1.0,
        max_waypoints_per_path=3,
        pos_tolerance=0.35,
        yaw_tolerance=10.0,
    ):
        super().__init__("path_follower")

        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])

        self.drone = drone
        self.path_topic = path_topic

        self.waypoint_spacing = waypoint_spacing
        self.max_waypoints_per_path = max_waypoints_per_path
        self.pos_tolerance = pos_tolerance
        self.yaw_tolerance = yaw_tolerance

        self.latest_path_ned = None
        self.latest_path_stamp = None
        self.path_counter = 0
        self.following = False

        self.sub = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            10
        )

        self.get_logger().info(f"Listening for path on {path_topic}")

    def path_callback(self, msg: Path):
        """
        Receives nav_msgs/Path in map frame.

        map frame:
            x = East
            y = North
            z = Up

        Converts to MAVSDK NED:
            north = y
            east = x
            down = -z
        """

        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path.")
            return

        path_ned = []

        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position

            east = p.x
            north = p.y
            down = -p.z

            path_ned.append([north, east, down])

        self.latest_path_ned = np.array(path_ned, dtype=np.float32)
        self.latest_path_stamp = self.get_clock().now()
        self.path_counter += 1

        self.get_logger().info(
            f"Received new path with {len(path_ned)} points. "
            f"path_counter={self.path_counter}"
        )

    def distance_ned(self, a, b):
        return math.sqrt(
            (a[0] - b[0]) ** 2 +
            (a[1] - b[1]) ** 2 +
            (a[2] - b[2]) ** 2
        )

    def simplify_path_by_distance(self, path_ned):
        """
        Converts dense grid path into fewer waypoints.
        """

        if path_ned is None or len(path_ned) == 0:
            return None

        simplified = [path_ned[0]]
        last = path_ned[0]

        for point in path_ned[1:]:
            if self.distance_ned(last, point) >= self.waypoint_spacing:
                simplified.append(point)
                last = point

        if not np.allclose(simplified[-1], path_ned[-1]):
            simplified.append(path_ned[-1])

        return np.array(simplified, dtype=np.float32)

    def yaw_towards_next_point(self, current_ned, target_ned):
        """
        MAVSDK yaw convention:
            0 deg = North
            90 deg = East
        """

        dn = target_ned[0] - current_ned[0]
        de = target_ned[1] - current_ned[1]

        yaw_rad = math.atan2(de, dn)
        yaw_deg = math.degrees(yaw_rad)

        while yaw_deg > 180:
            yaw_deg -= 360
        while yaw_deg < -180:
            yaw_deg += 360

        return yaw_deg

    async def get_current_ned(self):
        """
        Uses your Drone wrapper.
        Expected:
            await drone.get_position()
            returns north, east, down
        """

        north, east, down = await self.drone.get_position()
        return np.array([north, east, down], dtype=np.float32)

    async def wait_until_position_reached(
        self,
        target_north,
        target_east,
        target_down,
        target_yaw=None,
        hold_time=0.3,
        timeout=10.0,
    ):
        start_time = time.monotonic()
        stable_since = None

        while True:
            if time.monotonic() - start_time > timeout:
                return False

            north, east, down = await self.drone.get_position()

            distance_error = math.sqrt(
                (target_north - north) ** 2 +
                (target_east - east) ** 2 +
                (target_down - down) ** 2
            )

            yaw_ok = True

            if target_yaw is not None:
                yaw = await self.drone.get_yaw()
                yaw_error = target_yaw - yaw

                while yaw_error > 180:
                    yaw_error -= 360
                while yaw_error < -180:
                    yaw_error += 360

                yaw_ok = abs(yaw_error) < self.yaw_tolerance

            pos_ok = distance_error < self.pos_tolerance

            now = time.monotonic()

            if pos_ok and yaw_ok:
                if stable_since is None:
                    stable_since = now

                if now - stable_since >= hold_time:
                    return True
            else:
                stable_since = None

            await asyncio.sleep(0.1)

    async def follow_latest_path_once(self):
        """
        Follow only the first few waypoints of the latest path.

        This is intentional:
        after a short segment, the pathfinder can publish a newer path
        based on updated OctoMap/frontiers.
        """

        if self.latest_path_ned is None:
            return False

        path_id = self.path_counter
        raw_path = self.latest_path_ned.copy()

        waypoints = self.simplify_path_by_distance(raw_path)

        if waypoints is None or len(waypoints) < 2:
            self.get_logger().warn("Path has too few waypoints to follow.")
            return False

        current_ned = await self.get_current_ned()

        # Replace first waypoint with actual current position for yaw calculation
        waypoints[0] = current_ned

        end_index = min(len(waypoints), self.max_waypoints_per_path + 1)

        self.get_logger().info(
            f"Following path segment: {end_index - 1} waypoints "
            f"from path_id={path_id}"
        )

        for i in range(1, end_index):
            # If a newer path appears while following, stop and use the new one
            if self.path_counter != path_id:
                self.get_logger().info("New path received. Replanning segment.")
                return False

            current = await self.get_current_ned()
            target = waypoints[i]

            yaw_deg = self.yaw_towards_next_point(current, target)

            north = float(target[0])
            east = float(target[1])
            down = float(target[2])

            self.get_logger().info(
                f"Sending waypoint {i}/{end_index - 1}: "
                f"N={north:.2f}, E={east:.2f}, D={down:.2f}, Yaw={yaw_deg:.1f}"
            )

            # Use your Drone wrapper movement function
            await self.drone.send_position_setpoint(
                north=north,
                east=east,
                down=down,
                yaw_deg=yaw_deg,
            )

            # If custom_position_setpoint already waits until reached,
            # this wait is still okay but can be removed later.
            reached = await self.wait_until_position_reached(
                target_north=north,
                target_east=east,
                target_down=down,
                target_yaw=yaw_deg,
                hold_time=0.3,
                timeout=10.0,
            )

            if not reached:
                self.get_logger().warn("Waypoint timeout. Waiting for new path.")
                return False

        self.get_logger().info("Completed short path segment.")
        return True


async def ros_spin_task(node: PathFollower):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(0.01)


async def follow_loop(node: PathFollower):
    while rclpy.ok():
        if node.latest_path_ned is None:
            node.get_logger().info(
                "Waiting for /frontier_path...",
                throttle_duration_sec=2.0
            )
            await asyncio.sleep(0.5)
            continue

        if node.following:
            await asyncio.sleep(0.1)
            continue

        node.following = True

        try:
            await node.follow_latest_path_once()

        except Exception as e:
            node.get_logger().error(
                f"Path following error: {type(e).__name__}: {e}"
            )

        finally:
            node.following = False

        # Small pause before following next updated path
        await asyncio.sleep(0.5)


async def main():
    rclpy.init()

    drone = Drone()
    await drone.connect()

    node = PathFollower(
        drone=drone,
        path_topic="/frontier_path",
        waypoint_spacing=1.0,
        max_waypoints_per_path=3,
        pos_tolerance=0.35,
        yaw_tolerance=10.0,
    )

    try:
        await asyncio.gather(
            ros_spin_task(node),
            follow_loop(node),
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
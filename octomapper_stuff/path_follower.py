#!/usr/bin/env python3

import asyncio
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from drone_control import Drone

class VelocityPathFollower(Node):
    def __init__(
        self,
        drone: Drone,
        path_topic="/frontier_path",
        lookahead_distance=1.5,
        goal_tolerance=0.7,
        max_horizontal_speed=1.2,
        max_vertical_speed=0.5,
        kp_velocity=0.8,
        control_rate_hz=10.0,
        lock_path_until_goal=True,
    ):
        super().__init__("velocity_path_follower")

        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])

        self.drone = drone
        self.path_topic = path_topic

        self.lookahead_distance = float(lookahead_distance)
        self.goal_tolerance = float(goal_tolerance)
        self.max_horizontal_speed = float(max_horizontal_speed)
        self.max_vertical_speed = float(max_vertical_speed)
        self.kp_velocity = float(kp_velocity)
        self.control_rate_hz = float(control_rate_hz)

        # If True:
        #   once the drone starts following a path, it ignores normal new paths
        #   until the final goal is reached.
        self.lock_path_until_goal = bool(lock_path_until_goal)

        self.active_path_ned = None
        self.pending_path_ned = None
        self.path_counter = 0
        self.active_path_id = None

        self.has_sent_stop = False

        self.sub = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            10,
        )

        self.active_target_pub = self.create_publisher(
            Marker,
            "/active_velocity_target",
            10
        )

        self.get_logger().info(f"Listening for path on {path_topic}")
        self.get_logger().info(
            f"Velocity follower settings: "
            f"lookahead={self.lookahead_distance:.2f} m, "
            f"max_horizontal_speed={self.max_horizontal_speed:.2f} m/s, "
            f"max_vertical_speed={self.max_vertical_speed:.2f} m/s"
        )

    def publish_active_target(self, target_ned):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "active_velocity_target"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # NED to map
        marker.pose.position.x = float(target_ned[1])   # east
        marker.pose.position.y = float(target_ned[0])   # north
        marker.pose.position.z = float(-target_ned[2])  # up
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.active_target_pub.publish(marker)

    # =========================================================
    # Path callback
    # =========================================================

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
            self.get_logger().warn("Received empty /frontier_path.")
            return

        path_ned = []

        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position

            east = p.x
            north = p.y
            down = -p.z

            path_ned.append([north, east, down])

        path_ned = np.array(path_ned, dtype=np.float32)

        self.path_counter += 1

        if self.active_path_ned is not None and self.lock_path_until_goal:
            self.pending_path_ned = path_ned
            self.get_logger().info(
                "Received new path while following. Stored as pending path.",
                throttle_duration_sec=2.0,
            )
            return

        self.active_path_ned = path_ned
        self.active_path_id = self.path_counter
        self.has_sent_stop = False

        self.get_logger().info(
            f"Accepted new path with {len(path_ned)} points. "
            f"path_id={self.active_path_id}"
        )

    # =========================================================
    # Coordinate / geometry helpers
    # =========================================================

    def distance_ned(self, a, b):
        return math.sqrt(
            (a[0] - b[0]) ** 2 +
            (a[1] - b[1]) ** 2 +
            (a[2] - b[2]) ** 2
        )

    def yaw_towards_point(self, current_ned, target_ned):
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

    def find_closest_path_index(self, current_ned, path_ned):
        distances = np.linalg.norm(path_ned - current_ned, axis=1)
        return int(np.argmin(distances))

    def get_lookahead_target(self, current_ned, path_ned):
        """
        Finds a target point ahead along the path.

        1. Find the closest path point to the drone.
        2. Walk forward along the path until lookahead_distance is reached.
        3. Return that point.
        """

        if path_ned is None or len(path_ned) == 0:
            return None

        closest_idx = self.find_closest_path_index(current_ned, path_ned)

        accumulated = 0.0
        last = path_ned[closest_idx]

        for i in range(closest_idx + 1, len(path_ned)):
            point = path_ned[i]
            accumulated += self.distance_ned(last, point)

            if accumulated >= self.lookahead_distance:
                return point

            last = point

        # If near the end, use final goal
        return path_ned[-1]

    def compute_velocity_command(self, current_ned, target_ned):
        """
        P-controller toward lookahead point.

        Returns:
            vn, ve, vd
        """

        error = target_ned - current_ned

        vn = self.kp_velocity * float(error[0])
        ve = self.kp_velocity * float(error[1])
        vd = self.kp_velocity * float(error[2])

        # Limit horizontal speed
        horizontal_speed = math.sqrt(vn ** 2 + ve ** 2)

        if horizontal_speed > self.max_horizontal_speed:
            scale = self.max_horizontal_speed / horizontal_speed
            vn *= scale
            ve *= scale

        # Limit vertical speed
        vd = max(min(vd, self.max_vertical_speed), -self.max_vertical_speed)

        return vn, ve, vd

    # =========================================================
    # Drone helpers
    # =========================================================

    async def get_current_ned(self):
        north, east, down = await self.drone.get_position()
        return np.array([north, east, down], dtype=np.float32)
    
    async def send_velocity(self, vn, ve, vd, yaw_deg):
        """
        Sends velocity using your Drone wrapper.

        Drone wrapper:
            send_velocity(vx, vy, vz, yaw_deg)

        Where:
            vx = north velocity
            vy = east velocity
            vz = down velocity
        """

        await self.drone.send_velocity(
            vx=float(vn),
            vy=float(ve),
            vz=float(vd),
            yaw_deg=float(yaw_deg),
        )

    async def stop_drone(self):
        """
        Sends zero velocity to stop the drone.
        """

        yaw_deg = 0.0

        try:
            yaw_deg = await self.drone.get_yaw()
        except Exception:
            pass

        await self.send_velocity(
            vn=0.0,
            ve=0.0,
            vd=0.0,
            yaw_deg=yaw_deg,
        )

    # =========================================================
    # Main control step
    # =========================================================

    async def control_step(self):
        if self.active_path_ned is None:
            if not self.has_sent_stop:
                await self.stop_drone()
                self.has_sent_stop = True

            self.get_logger().info(
                "Waiting for /frontier_path...",
                throttle_duration_sec=2.0,
            )
            return

        current_ned = await self.get_current_ned()
        goal_ned = self.active_path_ned[-1]

        distance_to_goal = self.distance_ned(current_ned, goal_ned)

        if distance_to_goal <= self.goal_tolerance:
            self.get_logger().info(
                f"Reached frontier goal. distance={distance_to_goal:.2f} m"
            )

            await self.stop_drone()

            self.active_path_ned = None
            self.active_path_id = None
            self.has_sent_stop = True

            # If a path was published while following, accept it now.
            if self.pending_path_ned is not None:
                self.active_path_ned = self.pending_path_ned
                self.pending_path_ned = None
                self.active_path_id = self.path_counter
                self.has_sent_stop = False

                self.get_logger().info("Accepted pending path after reaching goal.")

            return

        target_ned = self.get_lookahead_target(
            current_ned=current_ned,
            path_ned=self.active_path_ned,
        )

        self.publish_active_target(target_ned)

        if target_ned is None:
            self.get_logger().warn("No valid lookahead target.")
            await self.stop_drone()
            return

        vn, ve, vd = self.compute_velocity_command(
            current_ned=current_ned,
            target_ned=target_ned,
        )

        yaw_deg = self.yaw_towards_point(current_ned, target_ned)

        await self.send_velocity(vn, ve, vd, yaw_deg)

        self.get_logger().info(
            f"Following path | "
            f"goal_dist={distance_to_goal:.2f} m, "
            f"target N={target_ned[0]:.2f}, "
            f"E={target_ned[1]:.2f}, "
            f"D={target_ned[2]:.2f}, "
            f"vel N={vn:.2f}, E={ve:.2f}, D={vd:.2f}, "
            f"yaw={yaw_deg:.1f}",
            throttle_duration_sec=0.5,
        )


async def ros_spin_task(node: VelocityPathFollower):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(0.01)


async def control_loop(node: VelocityPathFollower):
    period = 1.0 / node.control_rate_hz

    while rclpy.ok():
        try:
            await node.control_step()

        except Exception as e:
            node.get_logger().error(
                f"Velocity follower error: {type(e).__name__}: {e}"
            )

        await asyncio.sleep(period)


async def main():
    rclpy.init()

    drone = Drone()
    await drone.connect()

    node = VelocityPathFollower(
        drone=drone,
        path_topic="/frontier_path",

        # Tune these first
        lookahead_distance=1,
        goal_tolerance=0.25,

        # Speed tuning
        max_horizontal_speed=5,
        max_vertical_speed=0.5,
        kp_velocity=1.2,

        control_rate_hz=10.0,

        # True means it commits to one frontier path until it reaches the goal.
        # New paths are stored and used only after reaching the current goal.
        lock_path_until_goal=True,
    )

    try:
        await asyncio.gather(
            ros_spin_task(node),
            control_loop(node),
        )

    finally:
        try:
            await node.stop_drone()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
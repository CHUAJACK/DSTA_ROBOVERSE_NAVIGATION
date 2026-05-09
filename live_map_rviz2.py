"""
live_map_rviz2.py
==================
Live Global Mapping streamed to RViz2 + Keyboard / Autonomous Flight

Publishes two ROS 2 topics for RViz2:
  /global_map   sensor_msgs/PointCloud2   – accumulated obstacle cloud
  /drone_pose   geometry_msgs/PoseStamped – drone position + heading

RViz2 is launched automatically from within this script.

Run with:
  source /opt/ros/humble/setup.bash
  cd ~/Codes
  python3 live_map_rviz2.py

Controls:
  T         Arm + Takeoff (enters offboard automatically)
  W / S     Climb / Descend
  A / D     Yaw CCW / CW
  U / J     Forward / Backward
  H / K     Left / Right
  SPACE     Full stop / hover
  P         Toggle autonomous survey pattern
  L         Land
  Q         Quit + save map
"""

import asyncio
import atexit
import math
import os
import select
import signal
import struct
import subprocess
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import numpy as np
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# ROS 2 imports (requires sourced /opt/ros/humble/setup.bash)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path as NavPath

from depth_receiver import DepthReceiver
from drone_control_new import Drone
from get_position_with_task import SharedState, position_monitor_task
from GlobalMapper_new import GlobalMapper

# ── Tunable parameters ────────────────────────────────────────────────────────
MAVSDK_ADDRESS   = "udpin://0.0.0.0:14540"
TAKEOFF_ALTITUDE = 2.5        # metres

SPEED_XY         = 1.5        # m/s horizontal
SPEED_Z          = 1.0        # m/s vertical
YAW_RATE         = 30.0       # deg/s

CONTROL_HZ       = 20         # velocity setpoint rate
MAP_HZ           = 5          # depth capture + publish rate
POSE_HZ          = 20         # drone pose publish rate

KEY_HOLD_TIMEOUT = 0.50       # s — must exceed OS key-repeat initial delay (~400 ms)

# Camera intrinsics (match your Gazebo depth camera)
K = np.array([
    [433.0,   0.0, 320.0],
    [  0.0, 433.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float32)

# Autonomous survey waypoints (north_offset, east_offset) in metres
SURVEY_WAYPOINTS    = [( 5.0, 0.0), ( 5.0, 5.0), ( 0.0, 5.0), ( 0.0, 0.0)]
WAYPOINT_REACH_DIST = 0.8    # metres
SURVEY_SPEED        = 1.5    # m/s

# Path to the RViz2 config shipped alongside this script
RVIZ_CONFIG = str(Path(__file__).parent / "rviz2_map.rviz")

# ── Shared flight state ───────────────────────────────────────────────────────
class FlightState:
    running         : bool = True
    takeoff_req     : bool = False
    land_req        : bool = False
    offboard_active : bool = False
    autonomous      : bool = False


flight = FlightState()

_key_lock      = threading.Lock()
_active_key    = ''
_active_key_ts = 0.0

VEL_MAP = {
    'u': ( SPEED_XY,  0.0,      0.0,       0.0     ),
    'j': (-SPEED_XY,  0.0,      0.0,       0.0     ),
    'h': ( 0.0,      -SPEED_XY, 0.0,       0.0     ),
    'k': ( 0.0,       SPEED_XY, 0.0,       0.0     ),
    'w': ( 0.0,       0.0,     -SPEED_Z,   0.0     ),
    's': ( 0.0,       0.0,      SPEED_Z,   0.0     ),
    'a': ( 0.0,       0.0,      0.0,      -YAW_RATE),
    'd': ( 0.0,       0.0,      0.0,       YAW_RATE),
}


def _update_active_key(k: str):
    global _active_key, _active_key_ts
    with _key_lock:
        _active_key    = k
        _active_key_ts = time.monotonic()


def _get_active_key() -> str:
    with _key_lock:
        if _active_key and (time.monotonic() - _active_key_ts) < KEY_HOLD_TIMEOUT:
            return _active_key
        return ''


# ── Terminal helpers ──────────────────────────────────────────────────────────
_saved_term_fd: int | None = None
_saved_term_attrs = None

def _restore_terminal():
    """Always restore the terminal — called by atexit and signal handlers."""
    if _saved_term_fd is not None and _saved_term_attrs is not None:
        try:
            termios.tcsetattr(_saved_term_fd, termios.TCSADRAIN, _saved_term_attrs)
        except Exception:
            pass

atexit.register(_restore_terminal)


class RawTerminal:
    def __enter__(self):
        global _saved_term_fd, _saved_term_attrs
        self.fd  = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        _saved_term_fd    = self.fd
        _saved_term_attrs = self.old
        tty.setraw(self.fd)
        return self

    def __exit__(self, *_):
        global _saved_term_fd, _saved_term_attrs
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        _saved_term_fd    = None
        _saved_term_attrs = None

    def read_key(self, timeout=0.05) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            ch = os.read(self.fd, 1)
            if ch == b'\x03':   # Ctrl+C in raw mode
                return '\x03'
            return ch.decode('utf-8', errors='ignore').lower()
        return ''


def out(msg: str):
    sys.stdout.write(msg)
    sys.stdout.flush()


# ── Keyboard thread ───────────────────────────────────────────────────────────
def keyboard_thread():
    out("\n====================================================\n")
    out("  LIVE MAP -> RVIZ2 + KEYBOARD CONTROLLER\n")
    out("====================================================\n")
    out("  T         Arm + Takeoff\n")
    out("  W / S     Climb / Descend\n")
    out("  A / D     Yaw CCW / CW\n")
    out("  U / J     Forward / Backward\n")
    out("  H / K     Left / Right\n")
    out("  SPACE     Hover (full stop)\n")
    out("  P         Toggle autonomous survey\n")
    out("  L         Land\n")
    out("  Q         Quit + save map\n")
    out("====================================================\n\n")

    with RawTerminal() as term:
        while flight.running:
            key = term.read_key(timeout=0.05)
            if not key:
                continue

            if key in VEL_MAP:
                _update_active_key(key)
            elif key == ' ':
                _update_active_key('')
                out("\r[HOVER]                    \n")
            elif key == 't':
                flight.takeoff_req = True
                out("\r[TAKEOFF requested]\n")
            elif key == 'l':
                flight.land_req = True
                out("\r[LAND requested]\n")
            elif key == 'p':
                flight.autonomous = not flight.autonomous
                mode = "AUTONOMOUS survey" if flight.autonomous else "KEYBOARD"
                out(f"\r[MODE -> {mode}]\n")
            elif key in ('q', '\x03'):   # Q or Ctrl+C
                flight.running = False
                out("\r[QUIT]\n")
                break


# ── ROS 2 publisher node (runs in its own thread) ─────────────────────────────
class MapPublisher(Node):
    def __init__(self):
        super().__init__('live_map_publisher')
        self.pc_pub   = self.create_publisher(PointCloud2, '/global_map',  10)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone_pose', 10)
        self.odom_pub = self.create_publisher(Odometry,    '/drone_odom', 10)
        self.path_pub = self.create_publisher(NavPath,     '/drone_path', 10)
        self._path_msg = NavPath()
        self._path_msg.header.frame_id = 'map'

    def publish_pointcloud(self, points_ne: np.ndarray):
        """
        points_ne: (N, 2) float32 array of [north, east] in metres.
        Published in RViz2 'map' frame as (x=east, y=north, z=0).
        """
        if points_ne.shape[0] == 0:
            return

        # Build xyz float32 array: x=east, y=north, z=0
        n = points_ne.shape[0]
        xyz = np.zeros((n, 3), dtype=np.float32)
        xyz[:, 0] = points_ne[:, 1]   # east  → x
        xyz[:, 1] = points_ne[:, 0]   # north → y
        # z stays 0 (top-down map)

        # intensity = distance from origin (used for rainbow colouring in RViz2)
        intensity = np.linalg.norm(points_ne, axis=1).astype(np.float32)
        data = np.column_stack([xyz, intensity])   # (N, 4) float32

        msg             = PointCloud2()
        msg.header      = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height      = 1
        msg.width       = n
        msg.is_dense    = True
        msg.is_bigendian = False
        msg.point_step  = 16   # 4 fields × 4 bytes
        msg.row_step    = msg.point_step * n

        msg.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = data.tobytes()
        self.pc_pub.publish(msg)

    def publish_pose(self, north: float, east: float, yaw_deg: float):
        """Publishes drone position as PoseStamped in the map frame."""
        msg             = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = east    # x = east
        msg.pose.position.y = north   # y = north
        msg.pose.position.z = 0.0

        # yaw in NED (clockwise from North) → ROS quaternion (CCW from x-axis)
        yaw_ros = math.pi / 2.0 - math.radians(yaw_deg)
        msg.pose.orientation.z = math.sin(yaw_ros / 2.0)
        msg.pose.orientation.w = math.cos(yaw_ros / 2.0)

        self.pose_pub.publish(msg)

    def publish_odom(self, north: float, east: float, down: float, yaw_deg: float):
        """Publishes Odometry and appends a point to the internal path buffer."""
        now = self.get_clock().now().to_msg()
        yaw_ros = math.pi / 2.0 - math.radians(yaw_deg)
        qz = math.sin(yaw_ros / 2.0)
        qw = math.cos(yaw_ros / 2.0)

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'map'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x    = east
        odom.pose.pose.position.y    = north
        odom.pose.pose.position.z    = -down
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        # Accumulate path — published separately at a lower rate
        ps = PoseStamped()
        ps.header.stamp    = now
        ps.header.frame_id = 'map'
        ps.pose.position.x    = east
        ps.pose.position.y    = north
        ps.pose.position.z    = 0.0
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self._path_msg.poses.append(ps)
        if len(self._path_msg.poses) > 2000:
            self._path_msg.poses = self._path_msg.poses[-2000:]
        self._path_msg.header.stamp = now

    def publish_path(self):
        """Publishes the accumulated path trail. Call at a low rate (≤2 Hz)."""
        self.path_pub.publish(self._path_msg)


def ros_spin_thread(node: MapPublisher):
    """Runs rclpy.spin in a daemon thread so it doesn't block asyncio."""
    rclpy.spin(node)


# ── RViz2 launcher ────────────────────────────────────────────────────────────
def launch_rviz2() -> subprocess.Popen:
    env = os.environ.copy()
    # ensure ROS2 is sourced even if the user forgot
    ros_setup = "/opt/ros/humble/setup.bash"
    cmd = (
        f"source {ros_setup} && "
        f"rviz2 -d {RVIZ_CONFIG}"
    )
    proc = subprocess.Popen(
        cmd, shell=True, executable="/bin/bash",
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        env=env,
    )
    out(f"[RViz2] Launched (PID {proc.pid}) with config {RVIZ_CONFIG}\n")
    return proc


# ── Mapping task ──────────────────────────────────────────────────────────────
async def mapping_task(receiver: DepthReceiver, mapper: GlobalMapper,
                       node: MapPublisher, state: SharedState,
                       stop_event: asyncio.Event):
    dt = 1.0 / MAP_HZ
    while not stop_event.is_set() and flight.running:
        if state.latest_position is not None and state.latest_yaw is not None:
            depth_img = receiver.get_frame()
            if depth_img is not None:
                pose = {
                    "north": float(state.latest_position.north_m),
                    "east":  float(state.latest_position.east_m),
                    "down":  float(state.latest_position.down_m),
                    "yaw":   float(state.latest_yaw),
                }
                loop = asyncio.get_event_loop()
                await loop.run_in_executor(None, mapper.update_frame, depth_img, pose)
                pts = await loop.run_in_executor(None, mapper.get_global_points)
                # numpy serialisation (tobytes) is CPU-heavy — keep off event loop
                await loop.run_in_executor(None, node.publish_pointcloud, pts)

        await asyncio.sleep(dt)


# ── Pose publish task ─────────────────────────────────────────────────────────
async def pose_task(node: MapPublisher, state: SharedState,
                    stop_event: asyncio.Event):
    dt           = 1.0 / POSE_HZ
    last_path    = 0.0
    last_status  = 0.0
    while not stop_event.is_set() and flight.running:
        if state.latest_position is not None:
            north = float(state.latest_position.north_m)
            east  = float(state.latest_position.east_m)
            down  = float(state.latest_position.down_m)
            yaw   = float(state.latest_yaw or 0.0)
            now   = time.monotonic()

            node.publish_pose(north=north, east=east, yaw_deg=yaw)
            node.publish_odom(north=north, east=east, down=down, yaw_deg=yaw)

            # Path is a large growing message — publish at 2 Hz, not 20 Hz
            if now - last_path >= 0.5:
                node.publish_path()
                last_path = now

            if now - last_status >= 2.0:
                out(f"\r[POS] N={north:+.2f}m  E={east:+.2f}m  Alt={-down:.2f}m  Yaw={yaw:.1f}°    \n")
                last_status = now
        await asyncio.sleep(dt)


# ── Autonomous waypoint follower ──────────────────────────────────────────────
async def autonomous_task(drone: Drone, state: SharedState,
                          stop_event: asyncio.Event):
    while state.latest_position is None and not stop_event.is_set():
        await asyncio.sleep(0.2)
    if stop_event.is_set():
        return

    origin_n = float(state.latest_position.north_m)
    origin_e = float(state.latest_position.east_m)
    origin_d = float(state.latest_position.down_m)
    wp_idx   = 0
    dt       = 1.0 / CONTROL_HZ

    out(f"\r[AUTO] Survey origin N={origin_n:.1f} E={origin_e:.1f}\n")

    while not stop_event.is_set() and flight.running and flight.autonomous:
        if not flight.offboard_active:
            await asyncio.sleep(dt)
            continue

        target_n = origin_n + SURVEY_WAYPOINTS[wp_idx][0]
        target_e = origin_e + SURVEY_WAYPOINTS[wp_idx][1]

        pos  = state.latest_position
        dn   = target_n - float(pos.north_m)
        de   = target_e - float(pos.east_m)
        dist = math.hypot(dn, de)

        if dist < WAYPOINT_REACH_DIST:
            out(f"\r[AUTO] Waypoint {wp_idx} reached\n")
            wp_idx = (wp_idx + 1) % len(SURVEY_WAYPOINTS)
            await asyncio.sleep(0.5)
            continue

        scale = min(SURVEY_SPEED / max(dist, 0.01), SURVEY_SPEED)
        vn    = dn * scale
        ve    = de * scale

        yaw_r = math.radians(float(state.latest_yaw or 0.0))
        fwd   =  vn * math.cos(yaw_r) + ve * math.sin(yaw_r)
        rgt   = -vn * math.sin(yaw_r) + ve * math.cos(yaw_r)
        vd    =  0.5 * (float(pos.down_m) - origin_d)

        await drone.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(float(fwd), float(rgt), float(vd), 0.0)
        )
        await asyncio.sleep(dt)


# ── Main control loop ─────────────────────────────────────────────────────────
async def control_loop(drone: Drone, state: SharedState,
                       stop_event: asyncio.Event):
    dt        = 1.0 / CONTROL_HZ
    auto_task = None

    while flight.running:
        if flight.takeoff_req:
            flight.takeoff_req = False
            out("\n[CONTROL] Arming and taking off ...\n")
            try:
                await drone.arm_and_takeoff()
                await drone.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                )
                await drone.drone.offboard.start()
                flight.offboard_active = True
                out("[CONTROL] Offboard ACTIVE. Fly with keys or press P for auto.\n")
            except OffboardError as e:
                out(f"[ERROR] Offboard start failed: {e._result.result}\n[INFO] Press T to retry.\n")
            except Exception as e:
                out(f"[ERROR] Takeoff failed: {e}\n[INFO] Press T to retry.\n")

        if flight.land_req:
            flight.land_req         = False
            flight.offboard_active  = False
            flight.autonomous       = False
            _update_active_key('')
            out("\n[CONTROL] Landing ...\n")
            try:
                await drone.drone.offboard.stop()
            except Exception:
                pass
            try:
                await drone.land()
                out("[CONTROL] Landed.\n")
            except Exception as e:
                out(f"[ERROR] Land failed: {e}\n")

        if not flight.offboard_active:
            await asyncio.sleep(dt)
            continue

        # autonomous mode
        if flight.autonomous:
            if auto_task is None or auto_task.done():
                auto_task = asyncio.create_task(
                    autonomous_task(drone, state, stop_event)
                )
            await asyncio.sleep(dt)
            continue
        else:
            if auto_task is not None and not auto_task.done():
                auto_task.cancel()
                try:
                    await auto_task
                except asyncio.CancelledError:
                    pass
                auto_task = None

        # keyboard velocity
        active        = _get_active_key()
        fwd, rgt, dwn, yaw = VEL_MAP.get(active, (0.0, 0.0, 0.0, 0.0))
        try:
            await drone.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(fwd, rgt, dwn, yaw)
            )
        except Exception as e:
            out(f"[WARN] Velocity command failed: {e}\n")
            flight.offboard_active = False
        await asyncio.sleep(dt)

    stop_event.set()


# ── Entry point ───────────────────────────────────────────────────────────────
async def main():
    drone    = Drone(system_address=MAVSDK_ADDRESS,
                     takeoff_altitude=TAKEOFF_ALTITUDE)
    receiver = DepthReceiver("/depth_camera")
    mapper   = GlobalMapper(
        K,
        cam_height=1.0, obs_h_min=0.1, obs_h_max=1.5,
        z_min=0.3, z_max=6.0,
        yaw_in_degrees=True, yaw_smoothing=0.9,
    )
    state      = SharedState()
    stop_event = asyncio.Event()

    # Handle Ctrl+C before keyboard thread starts (terminal not yet in raw mode)
    def _sigint_handler(sig, frame):
        flight.running = False
        stop_event.set()
        _restore_terminal()

    signal.signal(signal.SIGINT,  _sigint_handler)
    signal.signal(signal.SIGTERM, _sigint_handler)

    # Init ROS 2
    rclpy.init()
    node = MapPublisher()
    threading.Thread(target=ros_spin_thread, args=(node,), daemon=True).start()

    # Launch RViz2 in background
    rviz_proc = launch_rviz2()

    out("Connecting to drone ...\n")
    await drone.connect()
    out("Connected. Waiting for PX4 to be ready ...\n")
    try:
        await drone.wait_until_ready(timeout_s=60.0)
        out("PX4 ready. Starting tasks ...\n")
    except TimeoutError:
        out("[WARN] Readiness timeout — starting anyway.\n")
    await asyncio.sleep(1)

    threading.Thread(target=keyboard_thread, daemon=True).start()

    async def resilient_monitor():
        """Restarts position monitor on connection drop. Suppresses spam."""
        fail_count = 0
        while not stop_event.is_set() and flight.running:
            try:
                fail_count = 0
                await position_monitor_task(drone, state, stop_event)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                fail_count += 1
                if not stop_event.is_set():
                    if fail_count == 1:
                        out(f"[WARN] Position monitor lost connection, retrying ...\n")
                    elif fail_count % 10 == 0:
                        out(f"[WARN] Position monitor still reconnecting (attempt {fail_count}) ...\n")
                    await asyncio.sleep(3.0)

    tasks = [
        asyncio.create_task(resilient_monitor()),
        asyncio.create_task(mapping_task(receiver, mapper, node, state, stop_event)),
        asyncio.create_task(pose_task(node, state, stop_event)),
        asyncio.create_task(control_loop(drone, state, stop_event)),
    ]

    try:
        # return_exceptions=True means one task crashing won't cancel the others
        await asyncio.gather(*tasks, return_exceptions=True)
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    finally:
        stop_event.set()
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

        # Save map
        pts = mapper.get_global_points()
        if len(pts) > 0:
            mapper.save_points("global_obstacles.npy")
            out(f"Map saved: {len(pts)} points -> global_obstacles.npy\n")

        # Safe landing
        if flight.offboard_active:
            try:
                await drone.drone.offboard.stop()
            except Exception:
                pass
        try:
            await drone.land()
        except Exception:
            pass

        # Teardown ROS 2 and RViz2
        node.destroy_node()
        rclpy.shutdown()
        rviz_proc.terminate()
        out("Shutdown complete.\n")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        out("\nInterrupted.\n")

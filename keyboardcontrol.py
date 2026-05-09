#!/usr/bin/env python3
"""
PX4 Keyboard Controller + SlamMapper
=====================================
Fly manually with keyboard while SlamMapper builds a point-cloud map
in a background thread from the Gazebo depth camera.

Extra keys added:
  M     Print current map point count
  P     Save map to slam_map.pcd  (also saved automatically on quit)

Everything else is identical to the original keyboard controller.
"""

import asyncio
import sys
import os
import termios
import tty
import threading
import time
import select
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from depth_receiver import DepthReceiver

# ── Tunable parameters ──────────────────────────────────────────────────────
MAVSDK_ADDRESS   = "udp://:14540"
TAKEOFF_ALTITUDE = 2.5          # metres
DEPTH_TOPIC      = "/depth_camera"

SPEED_XY  = 1.0     # m/s  horizontal body velocity
SPEED_Z   = 1.0     # m/s  vertical velocity
YAW_RATE  = 30.0    # deg/s

KEY_HOLD_TIMEOUT = 0.12   # seconds – key considered released after this


# =============================================================================
#  SLAM MAPPER
# =============================================================================

class SlamMapper:
    """
    Runs in its own daemon thread.
    Reads pose from _PoseState (populated by a background telemetry coroutine)
    and depth frames from DepthReceiver, fusing them into an Open3D point cloud.

    Coordinate convention
    ----------------------
    PX4 NED position + attitude_euler (roll/pitch/yaw in degrees, ZYX extrinsic).
    Internally converted to a right-handed Z-up world frame:
        world_x = east_m
        world_y = north_m
        world_z = -down_m
    """

    # ── Camera intrinsics ──────────────────────────────────────────────────
    FX, FY   = 433.0, 433.0
    CX, CY   = 320.0, 240.0

    # ── Depth clip (metres) ────────────────────────────────────────────────
    DEPTH_MIN = 0.15
    DEPTH_MAX = 8.0

    # ── Map resolution ─────────────────────────────────────────────────────
    VOXEL_SIZE = 0.08   # metres – voxel down-sample of global map

    # ── Thread period ──────────────────────────────────────────────────────
    PERIOD = 0.10       # seconds (10 Hz)

    # ── Extrinsic: camera frame → drone body frame ─────────────────────────
    # Camera faces forward (+X body). Adjust translation for your mount.
    T_BODY_CAM = np.array([
        [ 0,  0,  1,  0.10],   # cam-x = body-forward
        [-1,  0,  0,  0.00],   # cam-y = body-left
        [ 0, -1,  0,  0.05],   # cam-z = body-up
        [ 0,  0,  0,  1.00]
    ], dtype=float)

    def __init__(self, pose_state: "_PoseState", receiver: DepthReceiver):
        self.pose_state = pose_state
        self.receiver   = receiver

        self.global_map = o3d.geometry.PointCloud()
        self._map_lock  = threading.Lock()

        self._running       = False
        self._last_frame_id = None
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="SlamMapper"
        )

    # ── Public API ─────────────────────────────────────────────────────────

    def start(self):
        self._running = True
        self._thread.start()
        print("[SlamMapper] Thread started.")

    def stop(self):
        self._running = False
        self._thread.join(timeout=3.0)
        print("[SlamMapper] Thread stopped.")

    def point_count(self) -> int:
        with self._map_lock:
            return len(self.global_map.points)

    def save_map(self, path: str = "slam_map.pcd"):
        with self._map_lock:
            snapshot = o3d.geometry.PointCloud(self.global_map)
        o3d.io.write_point_cloud(path, snapshot)
        print(f"[SlamMapper] Map saved → {path}  ({len(snapshot.points)} pts)")

    # ── Internal helpers ───────────────────────────────────────────────────

    def _pose_ready(self) -> bool:
        ps = self.pose_state
        return (ps.north_m    is not None and
                ps.roll_deg   is not None)

    def _build_world_transform(self) -> np.ndarray:
        ps = self.pose_state

        # NED → Z-up world
        north =  ps.north_m
        east  =  ps.east_m
        up    = -ps.down_m

        # PX4 attitude_euler: ZYX extrinsic, yaw CW-positive → negate for scipy
        R = Rotation.from_euler(
            'ZYX',
            [-ps.yaw_deg, ps.pitch_deg, ps.roll_deg],
            degrees=True
        ).as_matrix()

        T_world_body = np.eye(4)
        T_world_body[:3, :3] = R
        T_world_body[:3,  3] = [east, north, up]

        return T_world_body @ self.T_BODY_CAM   # T_world_cam

    def _depth_to_points_cam(self, depth: np.ndarray) -> np.ndarray:
        h, w   = depth.shape
        u, v   = np.meshgrid(np.arange(w), np.arange(h))
        z      = depth.astype(np.float64)
        valid  = (z > self.DEPTH_MIN) & (z < self.DEPTH_MAX) & np.isfinite(z)

        z, u, v = z[valid], u[valid].astype(float), v[valid].astype(float)
        x = (u - self.CX) * z / self.FX
        y = (v - self.CY) * z / self.FY
        return np.column_stack([x, y, z])       # Nx3 in camera frame

    @staticmethod
    def _apply_transform(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
        ones  = np.ones((len(pts), 1))
        return (T @ np.hstack([pts, ones]).T).T[:, :3]

    # ── Thread loop ────────────────────────────────────────────────────────

    def _loop(self):
        while self._running:
            t0 = time.monotonic()

            if not self._pose_ready():
                time.sleep(0.05)
                continue

            depth = self.receiver.get_frame()
            if depth is None:
                time.sleep(self.PERIOD)
                continue

            fid = id(depth)
            if fid == self._last_frame_id:
                time.sleep(self.PERIOD)
                continue
            self._last_frame_id = fid

            pts_cam = self._depth_to_points_cam(depth)
            if len(pts_cam) == 0:
                time.sleep(self.PERIOD)
                continue

            T_wc       = self._build_world_transform()
            pts_world  = self._apply_transform(pts_cam, T_wc)

            frame_pcd        = o3d.geometry.PointCloud()
            frame_pcd.points = o3d.utility.Vector3dVector(pts_world)

            with self._map_lock:
                self.global_map += frame_pcd
                self.global_map  = self.global_map.voxel_down_sample(self.VOXEL_SIZE)

            time.sleep(max(0.0, self.PERIOD - (time.monotonic() - t0)))


# =============================================================================
#  POSE STATE  – populated by a background asyncio task, read by SlamMapper
# =============================================================================

class _PoseState:
    """Lightweight container updated by two concurrent telemetry streams."""
    def __init__(self):
        # position (metres, NED)
        self.north_m: float | None = None
        self.east_m:  float | None = None
        self.down_m:  float | None = None
        # attitude (degrees)
        self.roll_deg:  float | None = None
        self.pitch_deg: float | None = None
        self.yaw_deg:   float | None = None


async def _pose_monitor(drone: System, ps: _PoseState, stop: asyncio.Event):
    """Stream position_velocity_ned + attitude_euler into _PoseState."""

    async def _pos():
        async for pv in drone.telemetry.position_velocity_ned():
            if stop.is_set():
                break
            ps.north_m = pv.position.north_m
            ps.east_m  = pv.position.east_m
            ps.down_m  = pv.position.down_m

    async def _att():
        async for att in drone.telemetry.attitude_euler():
            if stop.is_set():
                break
            ps.roll_deg  = att.roll_deg
            ps.pitch_deg = att.pitch_deg
            ps.yaw_deg   = att.yaw_deg

    try:
        await asyncio.gather(_pos(), _att())
    except asyncio.CancelledError:
        pass


# =============================================================================
#  KEYBOARD CONTROLLER  (original logic, SlamMapper injected)
# =============================================================================

class State:
    forward_m_s     : float = 0.0
    right_m_s       : float = 0.0
    down_m_s        : float = 0.0
    yaw_deg_s       : float = 0.0
    running         : bool  = True
    takeoff         : bool  = False
    land            : bool  = False
    offboard_active : bool  = False
    print_map_count : bool  = False
    save_map        : bool  = False

state = State()

_key_lock      = threading.Lock()
_active_key    = ''
_active_key_ts = 0.0

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

VEL_MAP = {
    'u': ( SPEED_XY,  0.0,      0.0,      0.0     ),
    'j': (-SPEED_XY,  0.0,      0.0,      0.0     ),
    'h': ( 0.0,      -SPEED_XY, 0.0,      0.0     ),
    'k': ( 0.0,       SPEED_XY, 0.0,      0.0     ),
    'w': ( 0.0,       0.0,     -SPEED_Z,  0.0     ),
    's': ( 0.0,       0.0,      SPEED_Z,  0.0     ),
    'a': ( 0.0,       0.0,      0.0,     -YAW_RATE),
    'd': ( 0.0,       0.0,      0.0,      YAW_RATE),
}


class RawTerminal:
    def __enter__(self):
        self.fd  = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self, timeout=0.05) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            return os.read(self.fd, 1).decode('utf-8', errors='ignore').lower()
        return ''

def out(msg: str):
    sys.stdout.write(msg)
    sys.stdout.flush()

def print_banner():
    out("\n" + "=" * 54 + "\n")
    out("  PX4 KEYBOARD CONTROLLER + SLAM MAPPER\n")
    out("=" * 54 + "\n")
    out("  W / S       Climb / Descend\n")
    out("  A / D       Yaw CCW / CW\n")
    out("  U / J       Forward / Backward\n")
    out("  H / K       Left / Right\n")
    out("  SPACE       Full stop\n")
    out("  T           Arm + Takeoff\n")
    out("  L           Land\n")
    out("  M           Print map point count\n")
    out("  P           Save map to slam_map.pcd\n")
    out("  Q           Quit  (map auto-saved)\n")
    out("=" * 54 + "\n\n")


def keyboard_thread():
    print_banner()
    with RawTerminal() as term:
        while state.running:
            key = term.read_key(timeout=0.05)
            if not key:
                continue

            if key in VEL_MAP:
                _update_active_key(key)
                fwd, rgt, dwn, yaw = VEL_MAP[key]
                state.forward_m_s += fwd
                state.right_m_s   += rgt
                state.down_m_s    += dwn
                out(f"\r[KEY] {key.upper()}  fwd={state.forward_m_s:+.1f} "
                    f"rgt={state.right_m_s:+.1f} dwn={state.down_m_s:+.1f} "
                    f"yaw={yaw:+.1f}   ")

            elif key == ' ':
                _update_active_key('')
                state.forward_m_s = state.right_m_s = state.down_m_s = 0.0
                out("\n[KEY] SPACE -> Full stop\n")

            elif key == 't':
                state.takeoff = True
                out("\n[KEY] T -> Takeoff requested\n")

            elif key == 'l':
                state.land = True
                out("\n[KEY] L -> Land requested\n")

            elif key == 'm':
                state.print_map_count = True
                out("\n[KEY] M -> Map point count requested\n")

            elif key == 'p':
                state.save_map = True
                out("\n[KEY] P -> Save map requested\n")

            elif key == 'q':
                state.running = False
                out("\n[KEY] Q -> Quit\n")
                break


# ── MAVSDK helpers (unchanged) ────────────────────────────────────────────────

async def connect(drone: System):
    print(f"[MAVSDK] Connecting to {MAVSDK_ADDRESS} ...")
    await drone.connect(system_address=MAVSDK_ADDRESS)
    async for health in drone.telemetry.health():
        print(f"[HEALTH] GPS={health.is_global_position_ok}  "
              f"Home={health.is_home_position_ok}  "
              f"Arm={health.is_armable}")
        if health.is_global_position_ok and health.is_home_position_ok:
            break
    print("[MAVSDK] Connected and healthy.")


async def arm_and_takeoff(drone: System):
    print("[MAVSDK] Arming ...")
    await drone.action.arm()
    print(f"[MAVSDK] Taking off to {TAKEOFF_ALTITUDE} m ...")
    await drone.action.takeoff()
    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m
        sys.stdout.write(f"\r[MAVSDK] Alt: {alt:.2f} / {TAKEOFF_ALTITUDE:.2f} m   ")
        sys.stdout.flush()
        if alt >= TAKEOFF_ALTITUDE - 0.20:
            break
    print(f"\n[MAVSDK] Reached {alt:.2f} m – takeoff complete.")


async def start_offboard(drone: System):
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    try:
        await drone.offboard.start()
        state.offboard_active = True
        print("[MAVSDK] Offboard mode ACTIVE.")
    except OffboardError as e:
        print(f"[ERROR] Offboard start failed: {e._result.result}")
        raise


# ── Main control loop ─────────────────────────────────────────────────────────

async def control_loop(drone: System, slam: SlamMapper):
    print("[MAVSDK] Control loop running at 20 Hz ...")
    dt       = 0.05
    prev_key = ''

    while state.running:

        # ── Takeoff ────────────────────────────────────────────────────────
        if state.takeoff:
            state.takeoff = False
            await arm_and_takeoff(drone)
            await start_offboard(drone)
            slam.start()   # begin mapping once the drone is airborne

        # ── Land ───────────────────────────────────────────────────────────
        if state.land:
            state.land            = False
            state.offboard_active = False
            _update_active_key('')
            print("[MAVSDK] Landing ...")
            try:
                await drone.offboard.stop()
            except Exception:
                pass
            await drone.action.land()
            await asyncio.sleep(8)
            print("[MAVSDK] Landed.")

        # ── Map diagnostics ────────────────────────────────────────────────
        if state.print_map_count:
            state.print_map_count = False
            print(f"\n[SLAM] Current map: {slam.point_count()} points")

        if state.save_map:
            state.save_map = False
            # run blocking file I/O in a thread so asyncio isn't blocked
            await asyncio.get_event_loop().run_in_executor(
                None, slam.save_map, "slam_map.pcd"
            )

        if not state.offboard_active:
            await asyncio.sleep(dt)
            continue

        # ── Velocity command ───────────────────────────────────────────────
        active = _get_active_key()
        fwd, rgt, dwn, yaw = VEL_MAP.get(active, (0.0, 0.0, 0.0, 0.0))

        if active != prev_key:
            if active:
                print(f"\n[CTL] '{active.upper()}' ACTIVE  "
                      f"fwd={state.forward_m_s:+.1f} rgt={state.right_m_s:+.1f} "
                      f"dwn={state.down_m_s:+.1f} yaw={yaw:+.1f}")
            else:
                print("\n[CTL] Released – hovering")
            prev_key = active

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                forward_m_s    = state.forward_m_s,
                right_m_s      = state.right_m_s,
                down_m_s       = state.down_m_s,
                yawspeed_deg_s = yaw,
            )
        )

        await asyncio.sleep(dt)


async def shutdown(drone: System, slam: SlamMapper, stop_event: asyncio.Event):
    print("[MAVSDK] Shutting down ...")
    state.offboard_active = False
    stop_event.set()

    try:
        await drone.offboard.stop()
    except Exception:
        pass
    try:
        await drone.action.disarm()
    except Exception:
        pass

    slam.stop()
    await asyncio.get_event_loop().run_in_executor(
        None, slam.save_map, "slam_map.pcd"
    )
    print("[MAVSDK] Done.")


# ── Entry point ───────────────────────────────────────────────────────────────

async def main():
    drone      = System()
    await connect(drone)

    # ── Pose state shared between telemetry task and SlamMapper ───────────
    pose_state = _PoseState()
    stop_event = asyncio.Event()

    # Start telemetry streams immediately (before takeoff)
    # so pose_state is warm when mapping begins
    asyncio.create_task(_pose_monitor(drone, pose_state, stop_event))

    # ── Depth receiver + mapper ───────────────────────────────────────────
    receiver = DepthReceiver(DEPTH_TOPIC)
    slam     = SlamMapper(pose_state, receiver)
    # slam.start() is called inside control_loop after takeoff

    # ── Keyboard thread ───────────────────────────────────────────────────
    kb = threading.Thread(target=keyboard_thread, daemon=True)
    kb.start()

    print("[INFO] Press T to arm & take off, then fly. M=map info, P=save, Q=quit.\n")

    try:
        await control_loop(drone, slam)
    except asyncio.CancelledError:
        pass
    finally:
        await shutdown(drone, slam, stop_event)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
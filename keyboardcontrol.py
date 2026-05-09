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
from get_position_with_task import OdomTFBroadcaster

# ── Tunable parameters ──────────────────────────────────────────────────────
MAVSDK_ADDRESS   = "udp://:14540"
TAKEOFF_ALTITUDE = 2.5          # metres
DEPTH_TOPIC      = "/depth_camera"

SPEED_XY  = 1.0     # m/s  horizontal body velocity
SPEED_Z   = 1.0     # m/s  vertical velocity
YAW_RATE  = 30.0    # deg/s

KEY_HOLD_TIMEOUT = 0.12   # seconds – key considered released after this


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
            print(f'pitch:{ps.pitch_deg} | yaw:{ps.yaw_deg} | roll:{ps.roll_deg}')
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
    rclpy.init()
    # ── Pose state shared between telemetry task and SlamMapper ───────────
    pose_state = _PoseState()
    stop_event = asyncio.Event()

    # Start telemetry streams immediately (before takeoff)
    # so pose_state is warm when mapping begins
    asyncio.create_task(_pose_monitor(drone, pose_state, stop_event))

    # ── Depth receiver + mapper ───────────────────────────────────────────
    receiver = DepthReceiver(DEPTH_TOPIC)
    
    # slam.start() is called inside control_loop after takeoff
    tf2_node = OdomTFBroadcaster(Drone=drone,stop_event=stop_event)
    # ── Keyboard thread ───────────────────────────────────────────────────
    kb = threading.Thread(target=keyboard_thread, daemon=True)
    
    kb.start()
    ros_thread = threading.Thread(target=rclpy.spin, args=(tf2_node,), daemon=True)
    ros_thread.start()
    
    print("[INFO] Press T to arm & take off, then fly. M=map info, P=save, Q=quit.\n")

    try:
        await control_loop(drone, slam)
    except asyncio.CancelledError:
        pass
    finally:
        tf2_node.destroy_node()
        rclpy.shutdown()
        await shutdown(drone, slam, stop_event)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
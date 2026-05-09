"""
live_map_keyboard.py
=====================
Live Global Mapping + Keyboard Flight Control + Autonomous Survey Mode

Runs simultaneously:
  1. Live matplotlib map (RViz-style) updating at ~4 Hz
  2. Keyboard velocity control (body-frame)
  3. Continuous depth-based GlobalMapper at ~2 Hz
  4. Optional autonomous waypoint survey (toggle with P)

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
import os
import select
import sys
import termios
import threading
import time
import tty

import matplotlib.pyplot as plt
import numpy as np
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

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
MAP_HZ           = 2          # depth capture rate
PLOT_HZ          = 4          # map redraw rate

KEY_HOLD_TIMEOUT = 0.12       # s – key released after this

# Camera intrinsics (match your Gazebo depth camera)
K = np.array([
    [433.0,   0.0, 320.0],
    [  0.0, 433.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float32)

# Autonomous survey: (north_offset, east_offset) relative to takeoff point
SURVEY_WAYPOINTS = [
    ( 5.0,  0.0),
    ( 5.0,  5.0),
    ( 0.0,  5.0),
    ( 0.0,  0.0),
]
WAYPOINT_REACH_DIST = 0.8   # metres – "arrived" threshold
SURVEY_SPEED        = 1.5   # m/s

# ── Shared flight state ───────────────────────────────────────────────────────
class FlightState:
    running         : bool = True
    takeoff_req     : bool = False
    land_req        : bool = False
    offboard_active : bool = False
    autonomous      : bool = False   # toggled by P key


flight = FlightState()

_key_lock      = threading.Lock()
_active_key    = ''
_active_key_ts = 0.0

VEL_MAP = {
    'u': ( SPEED_XY,  0.0,      0.0,       0.0     ),  # forward
    'j': (-SPEED_XY,  0.0,      0.0,       0.0     ),  # backward
    'h': ( 0.0,      -SPEED_XY, 0.0,       0.0     ),  # left
    'k': ( 0.0,       SPEED_XY, 0.0,       0.0     ),  # right
    'w': ( 0.0,       0.0,     -SPEED_Z,   0.0     ),  # climb
    's': ( 0.0,       0.0,      SPEED_Z,   0.0     ),  # descend
    'a': ( 0.0,       0.0,      0.0,      -YAW_RATE),  # yaw CCW
    'd': ( 0.0,       0.0,      0.0,       YAW_RATE),  # yaw CW
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


# ── Keyboard thread ───────────────────────────────────────────────────────────
def keyboard_thread():
    out("\n====================================================\n")
    out("  LIVE MAP + KEYBOARD CONTROLLER\n")
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
            elif key == 'q':
                flight.running = False
                out("\r[QUIT]\n")
                break


# ── Mapping task ──────────────────────────────────────────────────────────────
async def mapping_task(receiver: DepthReceiver, mapper: GlobalMapper,
                       state: SharedState, stop_event: asyncio.Event):
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
                mapper.update_frame(depth_img, pose)
        await asyncio.sleep(dt)


# ── Live plot task (the RViz equivalent) ──────────────────────────────────────
async def plot_task(mapper: GlobalMapper, state: SharedState,
                    stop_event: asyncio.Event):
    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 9))
    fig.canvas.manager.set_window_title("Live Global Map")
    dt = 1.0 / PLOT_HZ

    trail_n: list = []
    trail_e: list = []

    while not stop_event.is_set() and flight.running:
        pts = mapper.get_global_points()
        ax.clear()

        # obstacle point cloud
        if len(pts) > 0:
            dists = np.linalg.norm(pts, axis=1)
            ax.scatter(pts[:, 1], pts[:, 0],
                       c=dists, s=2, cmap='viridis',
                       edgecolors='none', alpha=0.7)

        # drone position + trail + heading arrow
        if state.latest_position is not None:
            n = float(state.latest_position.north_m)
            e = float(state.latest_position.east_m)

            trail_n.append(n)
            trail_e.append(e)
            if len(trail_n) > 300:
                trail_n.pop(0)
                trail_e.pop(0)

            ax.plot(trail_e, trail_n, '-', color='orange',
                    linewidth=0.8, alpha=0.5, label='Trail')
            ax.plot(e, n, 'r*', markersize=14, zorder=6, label='Drone')

            if state.latest_yaw is not None:
                yaw_r = np.deg2rad(float(state.latest_yaw))
                ax.annotate(
                    '',
                    xy=(e + 1.5 * np.sin(yaw_r), n + 1.5 * np.cos(yaw_r)),
                    xytext=(e, n),
                    arrowprops=dict(arrowstyle='->', color='red', lw=2),
                    zorder=7,
                )

        mode_str = "AUTO" if flight.autonomous else "KEYBOARD"
        ax.set_title(
            f"Live Global Map  |  points={len(pts):,}  |  mode={mode_str}",
            fontsize=11,
        )
        ax.set_xlabel("East [m]")
        ax.set_ylabel("North [m]")
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        if state.latest_position is not None or len(pts) > 0:
            ax.legend(loc='upper left', fontsize=9)

        # Non-blocking GUI update – do NOT use plt.pause() here,
        # it calls time.sleep() which blocks the asyncio event loop
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        await asyncio.sleep(dt)

    plt.ioff()


# ── Autonomous waypoint follower ──────────────────────────────────────────────
async def autonomous_task(drone: Drone, state: SharedState,
                          stop_event: asyncio.Event):
    # wait for first position fix
    while state.latest_position is None and not stop_event.is_set():
        await asyncio.sleep(0.2)
    if stop_event.is_set():
        return

    origin_n = float(state.latest_position.north_m)
    origin_e = float(state.latest_position.east_m)
    origin_d = float(state.latest_position.down_m)

    wp_idx = 0
    dt = 1.0 / CONTROL_HZ

    out(f"\r[AUTO] Starting survey from origin N={origin_n:.1f} E={origin_e:.1f}\n")

    while not stop_event.is_set() and flight.running and flight.autonomous:
        if not flight.offboard_active:
            await asyncio.sleep(dt)
            continue

        target_n = origin_n + SURVEY_WAYPOINTS[wp_idx][0]
        target_e = origin_e + SURVEY_WAYPOINTS[wp_idx][1]

        pos = state.latest_position
        dn = target_n - float(pos.north_m)
        de = target_e - float(pos.east_m)
        dist = (dn ** 2 + de ** 2) ** 0.5

        if dist < WAYPOINT_REACH_DIST:
            out(f"\r[AUTO] Waypoint {wp_idx} reached, next ...\n")
            wp_idx = (wp_idx + 1) % len(SURVEY_WAYPOINTS)
            await asyncio.sleep(0.5)
            continue

        # proportional velocity, capped at SURVEY_SPEED
        scale = min(SURVEY_SPEED / max(dist, 0.01), SURVEY_SPEED)
        vn = dn * scale
        ve = de * scale

        # rotate global NED velocity into body frame
        yaw_r = np.deg2rad(float(state.latest_yaw or 0.0))
        fwd =  vn * np.cos(yaw_r) + ve * np.sin(yaw_r)
        rgt = -vn * np.sin(yaw_r) + ve * np.cos(yaw_r)

        # gentle altitude hold at takeoff down value
        vd = 0.5 * (float(pos.down_m) - origin_d)

        await drone.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                forward_m_s    = float(fwd),
                right_m_s      = float(rgt),
                down_m_s       = float(vd),
                yawspeed_deg_s = 0.0,
            )
        )
        await asyncio.sleep(dt)


# ── Main control loop ─────────────────────────────────────────────────────────
async def control_loop(drone: Drone, state: SharedState,
                       stop_event: asyncio.Event):
    dt = 1.0 / CONTROL_HZ
    auto_task = None

    while flight.running:
        # takeoff request
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

        # land request
        if flight.land_req:
            flight.land_req = False
            flight.offboard_active = False
            flight.autonomous = False
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

        # autonomous mode: spawn / cancel auto task as needed
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

        # keyboard velocity command
        active = _get_active_key()
        fwd, rgt, dwn, yaw = VEL_MAP.get(active, (0.0, 0.0, 0.0, 0.0))
        try:
            await drone.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    forward_m_s    = fwd,
                    right_m_s      = rgt,
                    down_m_s       = dwn,
                    yawspeed_deg_s = yaw,
                )
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
        cam_height=1.0,
        obs_h_min=0.1,
        obs_h_max=1.5,
        z_min=0.3,
        z_max=6.0,
        yaw_in_degrees=True,
        yaw_smoothing=0.9,
    )
    state      = SharedState()
    stop_event = asyncio.Event()

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
        asyncio.create_task(mapping_task(receiver, mapper, state, stop_event)),
        asyncio.create_task(plot_task(mapper, state, stop_event)),
        asyncio.create_task(control_loop(drone, state, stop_event)),
    ]

    try:
        await asyncio.gather(*tasks, return_exceptions=True)
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    finally:
        stop_event.set()
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

        if len(mapper.get_global_points()) > 0:
            mapper.save_points("global_obstacles.npy")
            out("Map saved to global_obstacles.npy\n")

        if flight.offboard_active:
            try:
                await drone.drone.offboard.stop()
            except Exception:
                pass
        try:
            await drone.land()
        except Exception:
            pass

        out("Shutdown complete.\n")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        out("\nInterrupted.\n")

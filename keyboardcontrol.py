"""
PX4 Keyboard Controller using MAVSDK – VelocityBodyYawspeed
============================================================
Commands velocity in the drone body frame so movement keys work
regardless of which direction the drone is facing.

Body frame:
  +forward_m_s  = nose direction
  +right_m_s    = right of nose
  +down_m_s     = downward  (NED convention – negative = climb)

Controls:
  W / S     Throttle up / down   (climb / descend)
  A / D     Yaw CCW / CW
  U / J     Pitch forward / backward
  H / K     Roll left / right
  SPACE     Full stop hover
  T         Arm + Takeoff
  L         Land
  M         Toggle mapping on / off
  Q         Quit

Threading / async model
-----------------------
┌─────────────────────────────────────────┐
│            Main Thread                  │
│   asyncio.run(main())                   │
│                                         │
│  ┌─────────────────────────────────┐    │
│  │        asyncio event loop       │    │
│  │  • Drone.connect()              │    │
│  │  • Drone.arm_and_takeoff()      │    │
│  │  • control_loop()  (20 Hz cmd)  │    │
│  │  • broadcaster                  │    │
│  │    .odom_monitor_task()         │    │
│  │    streams NED pos + attitude   │    │
│  │    → calls publish_tf()         │    │
│  │  • mapping_loop() (async task)  │    │
│  │  • visualization_loop() (task)  │    │
│  └─────────────────────────────────┘    │
└─────────────────────────────────────────┘
┌─────────────────────────────────────────┐
│         keyboard_thread (daemon)        │
│   reads raw terminal input              │
└─────────────────────────────────────────┘
┌─────────────────────────────────────────┐
│         ros_spin_thread (daemon)        │
│   rclpy.spin(broadcaster)               │
│   keeps ROS2 executor alive             │
└─────────────────────────────────────────┘

Install:
  pip install mavsdk
  sudo apt install ros-<distro>-tf2-ros python3-tf-transformations
"""

import asyncio
import sys
import os
import termios
import tty
import threading
import time
import select
import copy

import cv2
import numpy as np
import rclpy
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# ── Imports from sibling modules (no changes to those files) ──────────────────
from drone_control import Drone                          # owns mavsdk.System()
from get_position_with_task import OdomTFBroadcaster    # ROS2 TF broadcaster node
from get_position_with_task_v2 import Telemetry, position_monitor_task
from GlobalMapperV2 import GlobalMapper
from depth_receiver import DepthReceiver

# Reuse SharedState and visualization_loop exactly as defined in mapperV2
from mapperV2 import SharedState, visualization_loop

# ── Tunable parameters ────────────────────────────────────────────────────────
TAKEOFF_ALTITUDE = 2.5    # metres  (Drone.arm_and_takeoff waits 20 s – see note)
SPEED_XY         = 1.0    # m/s  horizontal body velocity
SPEED_Z          = 1.0    # m/s  vertical velocity
YAW_RATE         = 30.0   # deg/s
KEY_HOLD_TIMEOUT = 0.12   # seconds – key considered released after this
MAPPING_HZ       = 2.0    # how often to capture a depth frame for the map

# ── Camera intrinsics (must match what GlobalMapper / depth sensor expect) ────
K = np.array([[433.0, 0.0, 320.0],
              [0.0,   433.0, 240.0],
              [0.0,   0.0,   1.0]])

# ── Shared flight state ───────────────────────────────────────────────────────
class State:
    forward_m_s     : float = 0.0
    right_m_s       : float = 0.0
    down_m_s        : float = 0.0
    running         : bool  = True
    takeoff         : bool  = False
    land            : bool  = False
    offboard_active : bool  = False
    mapping_active  : bool  = False   # NEW – toggled by 'M' key

state = State()

# ── Active-key tracking (keyboard thread → asyncio control loop) ──────────────
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

# Maps key → (forward_m_s, right_m_s, down_m_s, yawspeed_deg_s)
VEL_MAP = {
    'u': ( SPEED_XY,  0.0,       0.0,       0.0      ),  # pitch forward
    'j': (-SPEED_XY,  0.0,       0.0,       0.0      ),  # pitch backward
    'h': ( 0.0,      -SPEED_XY,  0.0,       0.0      ),  # roll left
    'k': ( 0.0,       SPEED_XY,  0.0,       0.0      ),  # roll right
    'w': ( 0.0,       0.0,      -SPEED_Z,   0.0      ),  # throttle up (climb)
    's': ( 0.0,       0.0,       SPEED_Z,   0.0      ),  # throttle down (descend)
    'a': ( 0.0,       0.0,       0.0,      -YAW_RATE ),  # yaw CCW
    'd': ( 0.0,       0.0,       0.0,       YAW_RATE ),  # yaw CW
}

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

def print_banner():
    out("\n" + "=" * 54 + "\n")
    out("  PX4 KEYBOARD CONTROLLER – VelocityBodyYawspeed\n")
    out("=" * 54 + "\n")
    out("  W / S       Climb / Descend\n")
    out("  A / D       Yaw CCW / CW\n")
    out("  U / J       Forward / Backward\n")
    out("  H / K       Left / Right\n")
    out("  SPACE       Full stop\n")
    out("  T           Arm + Takeoff\n")
    out("  L           Land\n")
    out("  M           Toggle mapping ON / OFF\n")
    out("  Q           Quit\n")
    out("=" * 54 + "\n\n")

# ── Keyboard thread ───────────────────────────────────────────────────────────
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
                out(f"\r[KEY] {key.upper()}  fwd={state.forward_m_s:+.1f} rgt={state.right_m_s:+.1f} "
                    f"dwn={state.down_m_s:+.1f} yaw={yaw:+.1f}   ")

            elif key == ' ':
                state.forward_m_s = 0.0
                state.right_m_s   = 0.0
                state.down_m_s    = 0.0
                _update_active_key('')
                out("\n[KEY] SPACE -> Full stop\n")

            elif key == 't':
                state.takeoff = True
                out("\n[KEY] T -> Takeoff requested\n")

            elif key == 'l':
                state.land = True
                out("\n[KEY] L -> Land requested\n")

            elif key == 'm':
                # Toggle mapping on/off at runtime
                state.mapping_active = not state.mapping_active
                status = "ON" if state.mapping_active else "OFF"
                out(f"\n[KEY] M -> Mapping {status}\n")

            elif key == 'q':
                state.running = False
                out("\n[KEY] Q -> Quit\n")
                break

# ── ROS2 spin thread ──────────────────────────────────────────────────────────
def ros_spin_thread(broadcaster: OdomTFBroadcaster):
    """Keeps the ROS2 executor alive. Runs until rclpy.shutdown() is called."""
    try:
        rclpy.spin(broadcaster)
    except Exception:
        pass  # normal on shutdown

# ── Offboard bootstrap ────────────────────────────────────────────────────────
async def start_offboard(drone: Drone):
    """Send a zero setpoint then enable offboard mode."""
    await drone.drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    try:
        await drone.drone.offboard.start()
        state.offboard_active = True
        print("[MAVSDK] Offboard mode ACTIVE.")
    except OffboardError as e:
        print(f"[ERROR] Offboard start failed: {e._result.result}")
        raise

# ── Mapping loop (NEW) ────────────────────────────────────────────────────────
async def mapping_loop(mapper: GlobalMapper,
                       receiver: DepthReceiver,
                       telemetry: Telemetry,
                       bridge: SharedState):
    """
    Runs as an asyncio task alongside control_loop.
    Captures depth frames and feeds them into GlobalMapper whenever
    state.mapping_active is True and telemetry is available.
    Uses the same update_frame() call pattern as mapperV2.DroneNavigation.run().
    """
    print("[MAP] Mapping loop started. Press M to enable.")
    interval = 1.0 / MAPPING_HZ

    while state.running:
        if not state.mapping_active:
            await asyncio.sleep(interval)
            continue

        # Guard: telemetry must be populated before we can map
        if telemetry.north is None or telemetry.yaw_rad is None:
            await asyncio.sleep(interval)
            continue

        depth_frame = receiver.get_frame()
        if depth_frame is not None:
            t = copy.copy(telemetry)          # snapshot – same pattern as mapperV2
            updated = mapper.update_frame(depth_frame, t)
            if updated is not None:
                print(f"[MAP] Frame captured at "
                      f"N:{t.north:.2f} E:{t.east:.2f} Yaw:{t.yaw_deg:.2f}°")
                bridge.push(updated)          # hand off to visualization_loop

        await asyncio.sleep(interval)

    print("[MAP] Mapping loop exiting.")

# ── Main control loop ─────────────────────────────────────────────────────────
async def control_loop(drone: Drone):
    """20 Hz velocity-body setpoint loop."""
    print("[MAVSDK] Control loop running at 20 Hz ...")
    dt       = 0.05
    prev_key = ''

    while state.running:
        # ── T pressed: arm, takeoff via Drone class, then enable offboard ──
        if state.takeoff:
            state.takeoff = False
            await drone.arm_and_takeoff()   # waits ~20 s inside Drone class
            await start_offboard(drone)

        # ── L pressed: stop offboard, land, disarm via Drone class ─────────
        if state.land:
            state.land            = False
            state.offboard_active = False
            state.mapping_active  = False   # also pause mapping on land
            state.forward_m_s     = 0.0
            state.right_m_s       = 0.0
            state.down_m_s        = 0.0
            _update_active_key('')
            print("[MAVSDK] Landing ...")
            await drone.land()              # stops offboard + lands + disarms
            print("[MAVSDK] Landed.")

        if not state.offboard_active:
            await asyncio.sleep(dt)
            continue

        active = _get_active_key()
        _, _, _, yaw = VEL_MAP.get(active, (0.0, 0.0, 0.0, 0.0))

        if active != prev_key:
            if active:
                print(f"\n[CTL] '{active.upper()}' ACTIVE  "
                      f"fwd={state.forward_m_s:+.1f} rgt={state.right_m_s:+.1f} "
                      f"dwn={state.down_m_s:+.1f} yaw={yaw:+.1f}")
            else:
                print("\n[CTL] Released – hovering")
            prev_key = active

        await drone.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                forward_m_s    = state.forward_m_s,
                right_m_s      = state.right_m_s,
                down_m_s       = state.down_m_s,
                yawspeed_deg_s = yaw,
            )
        )

        await asyncio.sleep(dt)

# ── Shutdown ──────────────────────────────────────────────────────────────────
async def shutdown(drone: Drone, odom_task: asyncio.Task,
                   broadcaster: OdomTFBroadcaster):
    print("[MAVSDK] Shutting down ...")
    state.offboard_active = False

    odom_task.cancel()
    try:
        await odom_task
    except asyncio.CancelledError:
        pass

    try:
        await drone.drone.offboard.stop()
    except Exception:
        pass
    try:
        await drone.drone.action.disarm()
    except Exception:
        pass

    broadcaster.destroy_node()
    rclpy.shutdown()
    print("[MAVSDK] Done.")

# ── Entry point ───────────────────────────────────────────────────────────────
async def main():
    # 1. ROS2 init
    rclpy.init()

    # 2. Instantiate Drone and connect
    drone = Drone()
    await drone.connect()

    # 3. OdomTFBroadcaster for ROS2 TF
    stop_event = asyncio.Event()
    broadcaster = OdomTFBroadcaster(drone, stop_event)

    # 4. rclpy.spin in a daemon thread
    spin_thread = threading.Thread(
        target=ros_spin_thread,
        args=(broadcaster,),
        daemon=True,
        name="ros_spin",
    )
    spin_thread.start()
    print("[ROS2] Spin thread started.")

    # 5. Keyboard input in a daemon thread
    kb = threading.Thread(target=keyboard_thread, daemon=True, name="keyboard")
    kb.start()

    # 6. Telemetry (same Telemetry class used by mapperV2 / GlobalMapper)
    telemetry = Telemetry()
    telemetry_task = asyncio.create_task(
        position_monitor_task(drone, telemetry, stop_event),
        name="telemetry_monitor",
    )

    # 7. Mapping infrastructure – mirrors mapperV2.main() setup
    bridge   = SharedState()
    mapper   = GlobalMapper(K)
    receiver = DepthReceiver("/depth_camera")

    map_task = asyncio.create_task(
        mapping_loop(mapper, receiver, telemetry, bridge),
        name="mapping_loop",
    )
    vis_task = asyncio.create_task(
        visualization_loop(bridge),          # reused verbatim from mapperV2
        name="visualization_loop",
    )

    # 8. odom_monitor_task (original keyboardcontrol requirement)
    odom_task = asyncio.create_task(
        broadcaster.odom_monitor_task(),
        name="odom_monitor",
    )

    print("[INFO] Press T to arm & take off, then use keys to fly.\n"
          "[INFO] Press M to toggle mapping on/off.\n")

    try:
        await control_loop(drone)
    except asyncio.CancelledError:
        pass
    finally:
        stop_event.set()
        state.running = False

        # Cancel mapping / vis tasks gracefully
        for task in (map_task, vis_task, telemetry_task):
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

        await shutdown(drone, odom_task, broadcaster)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
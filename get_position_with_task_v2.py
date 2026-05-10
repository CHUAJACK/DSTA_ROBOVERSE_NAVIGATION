# main.py - Integration of Drone class with async position monitoring
import asyncio
from drone_control import Drone  # Your provided class
from mavsdk.offboard import VelocityNedYaw
import math
import numpy as np

class Telemetry:
    """Thread-safe(ish) container for inter-task data in a single event loop."""
    def __init__(self):
        self.latest_position = None  # NED position from telemetry
        self.yaw_deg = None
        self.yaw_rad = None
        self.roll = None
        self.pitch = None
        self.is_armed = False
        self.control_active = False
        self.w = None
        self.x = None
        self.y = None
        self.z = None
        self.north = None
        self.east = None
        self.down = None
        self.w = None
        self.x = None
        self.y = None
        self.z = None

async def position_monitor_task(drone: Drone, state: Telemetry, stop_event: asyncio.Event):
    """
    Background task streaming NED position and Yaw updates concurrently.
    """
    print("Position monitor task started...")

    async def stream_position():
        async for pos_vel in drone.drone.telemetry.position_velocity_ned():
            if stop_event.is_set():
                break
            state.north = pos_vel.position.north_m
            state.east = pos_vel.position.east_m
            state.down = pos_vel.position.down_m

    async def stream_orientation():
        async for att in drone.drone.telemetry.attitude_euler():
            if stop_event.is_set():
                break
            state.yaw_deg = att.yaw_deg
            state.roll = att.roll_deg
            state.pitch = att.pitch_deg
            state.w, state.x, state.y, state.z = euler_to_quaternion(state.roll, state.pitch, state.yaw_deg)
            state.yaw_rad = np.deg2rad(state.yaw_deg)

    try:
        # Run both streams concurrently
        await asyncio.gather(stream_position(), stream_orientation())

    except asyncio.CancelledError:
        print("📡 Position monitor task cancelled.")
    except Exception as e:
        print(f"Monitor error: {type(e).__name__}: {e}")

def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    # Convert degrees to radians
    phi = math.radians(roll_deg)
    theta = math.radians(pitch_deg)
    psi = math.radians(yaw_deg)

    # Pre-calculate trig values for half-angles
    c_phi = math.cos(phi / 2)
    s_phi = math.sin(phi / 2)
    c_theta = math.cos(theta / 2)
    s_theta = math.sin(theta / 2)
    c_psi = math.cos(psi / 2)
    s_psi = math.sin(psi / 2)

    # Standard Hamilton convention (w, x, y, z)
    w = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi
    x = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi
    y = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi
    z = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi

    return w, x, y, z


############################################################
# the following are tester codes #
############################################################

async def control_loop(drone: Drone, state: Telemetry, stop_event: asyncio.Event):
    """
    Main offboard control loop using your Drone class methods.
    Example: simple position-hold with telemetry logging.
    """
    print("\nStarting main control loop...")
    
    # Ensure offboard is active (your class handles this in arm_and_takeoff)
    await drone.arm_and_takeoff()
    state.is_armed = True
    state.control_active = True

    # PID gains for position hold (tune for your platform)
    dt = 0.1  # 10 Hz control loop

    try:
        # Record takeoff position as hold reference (NED frame)
        while state.latest_position is None and not stop_event.is_set():
            await asyncio.sleep(0.1)
        
        if stop_event.is_set():
            return
            
        hold_north = state.latest_position.north_m
        hold_east = state.latest_position.east_m
        hold_down = state.latest_position.down_m  # Negative = up
        print(f" Hold position set: N={hold_north:.2f}m, E={hold_east:.2f}m, D={hold_down:.2f}m")

        while not stop_event.is_set():
            pos = state.latest_position
            yaw = state.latest_yaw
            # your control logic here using pos and yaw, e.g. compute errors and send velocity commands


            # your control logic ends here using pos and yaw, e.g. compute errors and send velocity commands

            await asyncio.sleep(dt)
            
    except asyncio.CancelledError:
        print("\n Control loop cancelled.")
    except Exception as e:
        print(f"\n Control error: {type(e).__name__}: {e}")
    finally:
        state.control_active = False

async def run():
    # Initialize your Drone class
    drone = Drone()
    await drone.connect()

    stop_event = asyncio.Event()

    # Setup shared state & cancellation
    state = Telemetry()    
    # Start background position monitor task
    monitor_task = asyncio.create_task(
        position_monitor_task(drone, state, stop_event)
    )
    
    try:
        # Run main control loop (blocks until done/cancelled)
        await control_loop(drone, state, stop_event)
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt - initiating shutdown...")
    finally:
        # Graceful shutdown sequence
        stop_event.set()
        
        # Cancel and await monitor task
        monitor_task.cancel()
        try:
            await monitor_task
        except asyncio.CancelledError:
            pass
        
        # Land and disarm using your class
        if state.is_armed and state.control_active:
            print("\nLanding...")
            await drone.land()
        
        print("Shutdown complete.")

if __name__ == "__main__":
    asyncio.run(run())
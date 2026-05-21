import asyncio
import time
from drone_control import Drone  # Your provided class
from mavsdk.offboard import VelocityNedYaw

class SharedState:
    """Thread-safe(ish) container for inter-task data in a single event loop."""
    def __init__(self):
        self.is_armed = False
        self.control_active = False

async def control_loop(drone: Drone, state: SharedState, stop_event: asyncio.Event):
    """
    Main offboard control loop using your Drone class methods.
    Example: simple position-hold with telemetry logging.
    """
    print("\nStarting main control loop...")
    
    # Ensure offboard is active (your class handles this in arm_and_takeoff)
    await drone.arm_and_takeoff()
    state.is_armed = True
    state.control_active = True

    try:

        if stop_event.is_set():
            return
        
        while not stop_event.is_set():
            line = await asyncio.to_thread(input, "N E D Y: ")
            if line == "land":
                await drone.land()
                stop_event.set()
            N, E, D, Y = map(float, line.split())
            await drone.custom_position_setpoint(
                north=N,
                east=E,
                down=D,
                yaw_deg=Y,
            )
            print("waiting")
            # Hold new position
            await drone.send_position_setpoint(
                north=N,
                east=E,
                down=D,
                yaw_deg=Y,
            )
            # your control logic ends here using pos and yaw, e.g. compute errors and send velocity commands

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
    state = SharedState()    
    
    try:
        # Run main control loop (blocks until done/cancelled)
        await control_loop(drone, state, stop_event)
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt - initiating shutdown...")
    finally:
        # Graceful shutdown sequence
        stop_event.set()
        
        # Land and disarm using your class
        if state.is_armed and state.control_active:
            print("\nLanding...")
            await drone.land()
        
        print("Shutdown complete.")

if __name__ == "__main__":
    asyncio.run(run())
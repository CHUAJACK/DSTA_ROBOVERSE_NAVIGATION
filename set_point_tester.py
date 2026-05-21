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
    start = time.monotonic()
    await drone.arm_and_takeoff()
    state.is_armed = True
    state.control_active = True

    try:

        if stop_event.is_set():
            return
        
        while not stop_event.is_set():
            # coords format: label,N,E,D,Y
            coords = [

                # Clear Start Zone
                ["Starting position",0, 0,-4.5,0],
                ["Short Tower Hole",0,4.4,-2.5,0],
                ["Origin",0,0,-1,0],
                ["Move N",9,0,-1,0],
                ["Check E",9,0,-1,90],
                ["NW corner",16,0,-1,0],
                ["Face E",16,0,-1,90],
                ["N middle",16,8,-1,90],
                ["Face S",16,8,-1,180],
                ["Middle S",2,8,-1,180],
                ["Face W",2,12,-1,-90],
                ["Check fake barrels",-0.9,12,-1,-90],
                ["Face N",0,12,-1,0],
                ["Face E",1,12,-1,90],
                ["SE corner",2,15.5,-1,0],
                ["E middle",8,15.5,-1,0],
                ["Tall Tower Hole",8.5,12.5,-5.5,0],
                ["East Room Entrance",12,15.5,-1,90],
                
                # Travel to Small Room
                
                ["East Corridor",12,28,-1,90],
                ["Face North",12,28,-1,0],
                
                # Clear Small Room
                
                ["SW Corner",15.25,28,-1,90],
                ["SE Corner",15.25,36,-1,0],
                ["NE Corner",20,36,-1,-90],
                ["Box Top",21,32,-2.7,180],
                ["Face N",20,32,-1,0],
                
                # Travel to Weird Bend
                
                ["N Corridor",28,32,-1,0],
                ["Face W",28,32,-1,-90],
                ["Travel halfway",28,20,-1,-90],
                ["Face N Corridor",28,20,-1,0],
                
                # Clear Weird Bend
                
                ["Travel N",36,20,-1,-90],
                ["Travel W",35,16,-1,0],
                # Exit Weird Bend
                ["Travel E",36,20,-1,180],
                ["Travel S",28,20,-1,-90],
                
                # Clear Corridor
                
                ["Rest of Corridor",28,8,-1,180],
                ]

            # Clear short tower hole
            for coord in coords:
                print(coord[0])
                await drone.custom_position_setpoint(
                    north=coord[1],
                    east=coord[2],
                    down=coord[3],
                    yaw_deg=coord[4],
                )
                print("waiting")
                # Hold new position
                await drone.send_position_setpoint(
                    north=coord[1],
                    east=coord[2],
                    down=coord[3],
                    yaw_deg=coord[4],
                )
            stop_event.set()
            # your control logic ends here using pos and yaw, e.g. compute errors and send velocity commands
        
        await drone.land()
        end = time.monotonic()
        time_taken = end - start
        print(f"Run Time: {time_taken}")  

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
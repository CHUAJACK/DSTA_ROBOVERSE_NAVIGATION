# main.py - Integration of Drone class with async position monitoring
import asyncio
from drone_control import Drone  # Your provided class
from mavsdk.offboard import VelocityNedYaw
import math
from tf_transformations import quaternion_from_euler
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import asyncio
import threading

class SharedState:
    """Thread-safe(ish) container for inter-task data in a single event loop."""
    def __init__(self):
        self.latest_position = None  # NED position from telemetry
        self.latest_PitchRollYaw = None
        self.is_armed = False
        self.control_active = False



class OdomTFBroadcaster(Node):
    def __init__(self,Drone,stop_event):
        super().__init__('px4_tf_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.stop_event = stop_event
        self.drone = Drone
        self.get_logger().info("Node started")
        self.state = SharedState()

    def publish_tf(self, position, attitude):
        """Call when new position/attitude is obtained from MAVSDK."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # NED → ROS (ENU): x=east=y_ned, y=north=x_ned, z=up=-z_ned
        t.transform.translation.x = float(position.east_m)
        t.transform.translation.y = float(position.north_m)
        t.transform.translation.z = float(-position.down_m)

        # Yaw: NED yaw → ENU yaw
        roll_enu  = math.radians(attitude.pitch_deg)
        pitch_enu = math.radians(attitude.roll_deg)
        yaw_enu   = -math.radians(attitude.yaw_deg) + math.pi / 2
        q = quaternion_from_euler(roll_enu, pitch_enu, yaw_enu)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
    
    async def odom_monitor_task(self):
        """
        Background task streaming NED position and Yaw updates concurrently.
        """
        print("Position monitor task started...")

        async def stream_position():
            async for pos_vel in self.drone.drone.telemetry.position_velocity_ned():
                if self.stop_event.is_set():
                    break
                self.state.latest_position = pos_vel.position

        async def stream_yaw():
            async for att in self.drone.drone.telemetry.attitude_euler():
                if self.stop_event.is_set():
                    break
                self.state.latest_PitchRollYaw = att
                if self.state.latest_position is not None:
                    self.publish_tf(self.state.latest_position, self.state.latest_PitchRollYaw)

        

        try:
            # Run both streams concurrently
            await asyncio.gather(stream_position(), stream_yaw())

        except asyncio.CancelledError:
            print("📡 Position monitor task cancelled.")
        except Exception as e:
            print(f"Monitor error: {type(e).__name__}: {e}")



async def position_monitor_task(drone: Drone, state: SharedState, stop_event: asyncio.Event):
    """
    Background task streaming NED position and Yaw updates concurrently.
    """
    print("Position monitor task started...")

    async def stream_position():
        async for pos_vel in drone.drone.telemetry.position_velocity_ned():
            if stop_event.is_set():
                break
            state.latest_position = pos_vel.position

    async def stream_yaw():
        async for att in drone.drone.telemetry.attitude_euler():
            if stop_event.is_set():
                break
            state.latest_PitchRollYaw = att

    try:
        # Run both streams concurrently
        await asyncio.gather(stream_position(), stream_yaw())

    except asyncio.CancelledError:
        print("📡 Position monitor task cancelled.")
    except Exception as e:
        print(f"Monitor error: {type(e).__name__}: {e}")


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
    state = SharedState()    
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
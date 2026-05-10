import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from top_down import depth_to_xy_map
from depth_receiver import DepthReceiver
import time
from drone_control import Drone
import asyncio
from get_position_with_task_v2 import Telemetry, position_monitor_task, run
import io
import cv2

class GlobalMapper:
    """
    Incremental top-down occupancy grid mapper using NED (North-East-Down) pose.
    
    Coordinate Conventions:
    - Pose: {'north': float, 'east': float, 'yaw': float}
    - Yaw: radians, clockwise from North (standard NED heading)
    - Camera: Forward-facing, level mount assumed (X_cam=right, Z_cam=forward)
    - Grid: row=North, col=East (matches standard map orientation)
    """
    def __init__(self, K,
                 cam_height=1.0, obs_h_min=0.1, obs_h_max=1.5,
                 z_min=0.2, z_max=15.0, yaw_clockwise=True):
        self.K = K
        self.cam_height = cam_height
        self.obs_h_min = obs_h_min
        self.obs_h_max = obs_h_max
        self.z_min = z_min
        self.z_max = z_max
        
        # Yaw handling
        self.yaw_clockwise = yaw_clockwise
        
        # Global point storage: (N, 2) array [north, east] in meters
        self.global_points = np.empty((0, 2), dtype=np.float32)
        
    def _local_to_ned_global(self, local_xy, north, east, yaw_rad):
        """Transform local (X_cam=right, Z_cam=forward) to NED global (north, east)"""
        X_cam = local_xy[:, 0]
        Z_cam = local_xy[:, 1]
        X = Z_cam
        Y = X_cam
        c, s = np.cos(yaw_rad), np.sin(yaw_rad)
       
        X_new = X * c - Y * s
        Y_new = X * s + Y * c
        return np.column_stack([north+X_new, east+Y_new])
    
    def update_frame(self, depth_img, telemetry):
        north = float(telemetry.north)
        east = float(telemetry.east)
        down = float(telemetry.down)
        yaw_rad = float(telemetry.yaw_rad)
        yaw_deg = float(telemetry.yaw_deg)
        yaw_rad = np.arctan2(np.sin(yaw_rad),np.cos(yaw_rad))
        print(f"Drone Pose - North: {north:.2f} m, East: {east:.2f} m, Yaw: {yaw_deg} deg {yaw_rad:.2f} rad")

        if yaw_rad is None:
            print("Warning: yaw is None, using 0.0")
            yaw_rad = 0.0
        # convert yaw
        if not self.yaw_clockwise:
            yaw_rad = -yaw_rad

        yaw = yaw_rad
        # get local obstacle coordinates 
        xy_obstacles = depth_to_xy_map(
            depth_img,
            self.K,
            cam_height=self.cam_height,
            obs_h_min=self.obs_h_min,
            obs_h_max=self.obs_h_max,
            z_min=self.z_min,
            z_max=self.z_max,
        )
        if xy_obstacles is None or xy_obstacles.shape[0] == 0:
            return None

        # transform and accumulate
        global_pts = self._local_to_ned_global(xy_obstacles, north, east, yaw)
        self.global_points = np.vstack([self.global_points, global_pts.astype(np.float32)])
        print(len(self.global_points))
        x = self.draw_map()
        return x
    
    def get_global_points(self):
        """Returns copy of accumulated (north, east) points in meters"""
        return self.global_points.copy()
    
    def save_points(self, filename="global_obstacles.npy"):
        np.save(filename, self.global_points)
        print(f"✅ Saved {len(self.global_points)} points to {filename}")
    
    def draw_map(self):
        buf = io.BytesIO()
        # Assuming x_cam and z_fwd are your numpy arrays
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.scatter(
            self.global_points[:, 1], # All x values
            self.global_points[:, 0] # All y values
        )  
        ax.set_xlabel('East [m]')
        ax.set_ylabel('north [m]')
        ax.set_title('Basic Scatter Plot')
        plt.savefig(buf, format='png')
        buf.seek(0)
        img_arr = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        cv_image = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
        return cv_image


# ================= Sample usage EXAMPLE =================
async def run():
    K = np.array([[433.0, 0.0, 320.0],
                  [0.0, 433.0, 240.0],
                  [0.0, 0.0, 1.0]])
    receiver = DepthReceiver("/depth_camera")
    time.sleep(5)

    mapper = GlobalMapper(
        K, cam_height=1.0, obs_h_min=0.1, obs_h_max=1.5,
        yaw_in_degrees=True, yaw_smoothing=1.0,z_min=0,z_max=5.0
    )
        
    fig, ax = plt.subplots(figsize=(8, 8))
    
    cmap = plt.cm.colors.ListedColormap(['#808080', '#FFFFFF', '#000000'])
    
    stop_event = asyncio.Event()
    # 1. SETUP THE DRONE 
    drone = Drone()
    await drone.connect()
    await drone.arm_and_takeoff()

    # 2. Setup shared state & cancellation
    state = SharedState()    
    # Start background position monitor task
    monitor_task = asyncio.create_task(position_monitor_task(drone, state, stop_event))
    await asyncio.sleep(3)

#    for i in range(3):
    pose = {}
    pose['yaw'] = state.latest_yaw
    pose['north'] = state.latest_position.north_m
    pose['east'] = state.latest_position.east_m
    pose['down'] = state.latest_position.down_m
    depth_img = receiver.get_frame()
    await drone.send_position_setpoint(north=pose['north'], east=pose['east'], down=pose['down'], yaw_deg=0)
    await asyncio.sleep(5)
    if depth_img is None:
        print("No depth data received yet.")
    else:
        mapper.update_frame(depth_img, pose)
        print(F"N: {pose['north']} E:{pose['east']} Yaw:{pose['yaw']}")
    await drone.send_position_setpoint(north=pose['north'], east=pose['east'], down=pose['down'], yaw_deg=270)
    await asyncio.sleep(5)
    pose = {}
    pose['yaw'] = state.latest_yaw
    pose['north'] = state.latest_position.north_m
    pose['east'] = state.latest_position.east_m
    pose['down'] = state.latest_position.down_m
    depth_img = receiver.get_frame()
    if depth_img is None:
        print("No depth data received yet.")
    else:
        mapper.update_frame(depth_img, pose)
        print(F"N: {pose['north']} E:{pose['east']} Yaw:{pose['yaw']}")
    await drone.send_position_setpoint(north=pose['north'], east=pose['east'], down=pose['down'], yaw_deg=90)
    await asyncio.sleep(5)
    pose = {}
    pose['yaw'] = state.latest_yaw
    pose['north'] = state.latest_position.north_m
    pose['east'] = state.latest_position.east_m
    pose['down'] = state.latest_position.down_m
    depth_img = receiver.get_frame()
    if depth_img is None:
        print("No depth data received yet.")
    else:
        mapper.update_frame(depth_img, pose)
        print(F"N: {pose['north']} E:{pose['east']} Yaw:{pose['yaw']}")

    pts = mapper.get_global_points()
    ax.clear()
    if len(pts) > 0:
        # Color by distance from origin for depth perception
        dists = np.linalg.norm(pts, axis=1)
        ax.scatter(pts[:, 1], pts[:, 0], c=dists, s=4, cmap='viridis', edgecolors='none')
    
    ax.plot(pose['east'], pose['north'], 'r*', markersize=12, label='Drone')
    ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
    ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend()
    plt.pause(0.05)
    
    plt.show()

    await drone.land()

if __name__ == "__main__":
    asyncio.run(run())
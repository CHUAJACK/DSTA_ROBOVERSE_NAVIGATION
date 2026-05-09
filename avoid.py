#!/usr/bin/env python3
import asyncio
import numpy as np
import time
import threading
import open3d as o3d
from scipy.spatial.transform import Rotation
 
from depth_receiver import DepthReceiver
from drone_control import Drone
from AvoidancePlanner import AvoidancePlanner
from get_position_with_task import SharedState, position_monitor_task

class DroneNavigation:
    def __init__(self,
                 depth_topic="/depth_camera",
                 loop_hz=20.0):

        self.loop_hz = loop_hz
        self.running = True

        # =========================
        # GRID HEADING SYSTEM
        # =========================
        self.grid_headings = [0, 90, 180, -90]  # N, E, S, W
        self.current_heading_idx = 0
        self.target_yaw_deg = self.grid_headings[self.current_heading_idx]
        self.yaw_tolerance = 5.0

        # =========================
        #  NED POSE TRACKING
        # =========================
        self.pose = {
            "north": 0.0,
            "east": 0.0,
            "down": -2.0,
            "yaw": 0.0,
            "yaw_deg": 0.0
        }

        # Camera intrinsics
        K = np.array([[433.0, 0.0, 320.0],
                      [0.0, 433.0, 240.0],
                      [0.0, 0.0, 1.0]])

        self.receiver = DepthReceiver(depth_topic)

        self.planner = AvoidancePlanner(
            K=K,
            width=640,
            height=480,
            safe_distance=4.0,
            critical_distance=1.5
        )

        self.drone = Drone()
        self.position_state = SharedState()    


    # =========================
    #  YAW UTILS
    # =========================
    def _yaw_error(self, target, current):
        error = target - current
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error

    async def update_pose(self):
        """
        Get pose from drone from state shared by position monitor task. This is critical to ensure the planner has the latest pose for decision making.
        """
        self.pose["north"] = self.position_state.latest_position.north_m
        self.pose["east"]  = self.position_state.latest_position.east_m
        self.pose["down"]  = self.position_state.latest_position.down_m
        self.pose["yaw_deg"]   = self.position_state.latest_PitchRollYaw.yaw_deg
        self.pose["yaw"] = np.deg2rad(self.pose["yaw_deg"])

    async def align_to_grid(self):
        current_yaw = await self.drone.get_yaw()
        error = self._yaw_error(self.target_yaw_deg, current_yaw)

        if abs(error) > self.yaw_tolerance:
            print(f"Aligning to {self.target_yaw_deg}° (err={error:.2f})")
            await self.drone.rotate_to_yaw(self.target_yaw_deg)

    # =========================
    # 🔄 GRID TURNING
    # =========================
    async def rotate_next_direction(self):
        self.current_heading_idx = (self.current_heading_idx + 1) % 4
        self.target_yaw_deg = self.grid_headings[self.current_heading_idx]

        print(f"🔄 New heading: {self.target_yaw_deg}°")
        await self.drone.rotate_to_yaw(self.target_yaw_deg)

    # =========================
    # tHE MAIN LOOP WHERE THE PIPELINE COMES TOGETHER
    # =========================
    async def run(self):
        print("\nPOSITION-BASED AUTONOMOUS AvoidanceNAVIGATION\n")

        await self.drone.connect()
        await asyncio.sleep(3)
        print("Starting position monitor.")
        self.monitor_task = asyncio.create_task(position_monitor_task(self.drone, self.position_state, asyncio.Event()))
        await self.drone.arm_and_takeoff()

        self.slam_mapper.start()

        # Initial alignment
        await self.drone.rotate_to_yaw(self.target_yaw_deg)

        try:
            while self.running:
                t_start = time.monotonic()

                # -----------------------------------
                # UPDATE POSE (CRITICAL)
                # -----------------------------------
                await self.update_pose()

                depth_frame = self.receiver.get_frame()

                # -----------------------------------
                # POSITION PLANNER
                # -----------------------------------
                north, east, down, info = self.planner.compute_position_ned(
                    depth_frame,
                    self.pose,
                    step_size=1.5
                )

                c = info['clearance']

                print(f"Blocked={info['blocked']} | "
                      f"Target N={north:.2f}, E={east:.2f} | "
                      f"L={c['left']:.2f} C={c['center']:.2f} R={c['right']:.2f}")

                # ===================================
                #  BLOCK HANDLING
                # ===================================
                if info['blocked']:
                    await self.drone.send_velocity(0, 0, 0, self.target_yaw_deg)
                    await self.rotate_next_direction()
                else:
                    # Ensure alignment before motion
                    await self.align_to_grid()

                    # -----------------------------------
                    #  SEND POSITION SETPOINT
                    # -----------------------------------
                    await self.drone.send_position_setpoint(
                        north=north,
                        east=east,
                        down=down,
                        yaw_deg=self.target_yaw_deg
                    )

                # Maintain loop timing
                elapsed = time.monotonic() - t_start
                sleep_time = (1.0 / self.loop_hz) - elapsed
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)

        except asyncio.CancelledError:
            print("🛑 Navigation cancelled")

        finally:
            await self.drone.send_velocity(0, 0, 0, self.target_yaw_deg)
            print("Drone hovering safely")

    def stop(self):
        self.running = False

# =========================
#  SLAM MAPPER
# =========================

class SlamMapper:
    """
    Threaded point-cloud map builder.
 
    Coordinate convention
    ----------------------
    SharedState gives NED position + yaw (degrees, North = 0, CW positive).
    We convert to a right-handed ENU/world frame for Open3D:
        world_x =  east_m
        world_y =  north_m
        world_z = -down_m   (up is positive)
 
    The camera is assumed to face the drone's forward (North when yaw=0) axis.
    Adjust T_BODY_CAM if your camera is mounted differently.
    """

    # ---------- camera intrinsics (same K used by AvoidancePlanner) ----------
    FX, FY = 433.0, 433.0
    CX, CY = 320.0, 240.0
    WIDTH,  HEIGHT = 640, 480

    # Depth clip (metres) – ignore pixels outside this range
    DEPTH_MIN = 0.15
    DEPTH_MAX = 8.0
 
    # Voxel size for down-sampling the global map (metres)
    VOXEL_SIZE = 0.08
 
    # How often the mapper thread wakes up to process a new frame (seconds)
    PERIOD = 0.1   # 10 Hz is plenty for mapping
 
    # ---------- extrinsic: camera frame → drone body frame -------------------
    # Camera faces forward (+X body), Y body is left, Z body is up.
    # Adjust translation [x, y, z] to where the camera sits on the frame (metres).
    T_BODY_CAM = np.array([
        [ 0,  0,  1,  0.10],   # cam-x  = body-forward
        [-1,  0,  0,  0.00],   # cam-y  = body-left
        [ 0, -1,  0,  0.05],   # cam-z  = body-up
        [ 0,  0,  0,  1.00]
    ], dtype=float)

    def __init__(self, state: SharedState, receiver: DepthReceiver):
        self.state    = state
        self.receiver = receiver
 
        self.global_map = o3d.geometry.PointCloud()
        self._map_lock  = threading.Lock()   # protects global_map for safe reads
 
        self._running = False
        self._thread  = threading.Thread(target=self._loop, daemon=True, name="SlamMapper")
 
        # Track the last depth frame we processed to avoid re-processing duplicates
        self._last_frame_id: int | None = None
 
    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #
 
    def start(self):
        """Start the background mapping thread."""
        self._running = True
        self._thread.start()
        print("[SlamMapper] Thread started.")
 
    def stop(self):
        """Signal the thread to stop and wait for it to finish."""
        self._running = False
        self._thread.join(timeout=3.0)
        print("[SlamMapper] Thread stopped.")
 
    def get_map(self) -> o3d.geometry.PointCloud:
        """Return a snapshot of the current global map (thread-safe copy)."""
        with self._map_lock:
            return o3d.geometry.PointCloud(self.global_map)
 
    def save_map(self, path: str = "slam_map.pcd"):
        """Save the current map to a PCD file."""
        snapshot = self.get_map()
        o3d.io.write_point_cloud(path, snapshot)
        print(f"[SlamMapper] Map saved → {path}  ({len(snapshot.points)} points)")
 
    # ------------------------------------------------------------------ #
    #  Internal helpers                                                    #
    # ------------------------------------------------------------------ #
 
    def _pose_ready(self) -> bool:
        """Return True only when SharedState has been populated at least once."""
        return (self.state.latest_position is not None and
                self.state.latest_PitchRollYaw is not None)
 
    def _build_world_transform(self) -> np.ndarray:
        """
        Build a 4×4 homogeneous transform  T_world_cam  from current SharedState.

        attitude_euler() gives roll/pitch/yaw in degrees.
        PX4 convention: ZYX extrinsic (yaw → pitch → roll applied in that order),
        yaw = 0 is North, clockwise positive.

        We negate yaw to convert CW→CCW for scipy, and use 'ZYX' intrinsic
        which is equivalent to extrinsic XYZ (roll→pitch→yaw).
        """
        pos = self.state.latest_position
        att = self.state.latest_PitchRollYaw

        # NED → world (Z-up, right-handed)
        north =  pos.north_m
        east  =  pos.east_m
        up    = -pos.down_m

        roll_deg  =  att.roll_deg          # +right wing down
        pitch_deg =  att.pitch_deg         # +nose up
        yaw_deg   = -att.yaw_deg           # negate: PX4 CW → scipy CCW

        # scipy 'ZYX' intrinsic == extrinsic yaw→pitch→roll (PX4 convention)
        R_world_body = Rotation.from_euler(
            'ZYX',
            [yaw_deg, pitch_deg, roll_deg],
            degrees=True
        ).as_matrix()

        T_world_body = np.eye(4)
        T_world_body[:3, :3] = R_world_body
        T_world_body[:3,  3] = [east, north, up]

        return T_world_body @ self.T_BODY_CAM    # T_world_cam
 
    def _depth_to_pointcloud(self, depth: np.ndarray) -> np.ndarray:
        """
        Convert a float32 depth image (metres) to an Nx3 array of 3-D points
        expressed in the camera frame.
        """
        h, w = depth.shape
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))
 
        z = depth.astype(np.float64)
        valid = (z > self.DEPTH_MIN) & (z < self.DEPTH_MAX) & np.isfinite(z)
 
        z = z[valid]
        u = u_grid[valid].astype(np.float64)
        v = v_grid[valid].astype(np.float64)
 
        x = (u - self.CX) * z / self.FX
        y = (v - self.CY) * z / self.FY
 
        return np.column_stack([x, y, z])   # Nx3 in camera frame
 
    def _transform_points(self, pts_cam: np.ndarray, T: np.ndarray) -> np.ndarray:
        """Apply a 4×4 transform to an Nx3 point array."""
        ones  = np.ones((len(pts_cam), 1))
        pts_h = np.hstack([pts_cam, ones])          # Nx4
        return (T @ pts_h.T).T[:, :3]              # Nx3
 
    # ------------------------------------------------------------------ #
    #  Thread main loop                                                    #
    # ------------------------------------------------------------------ #
 
    def _loop(self):
        while self._running:
            t0 = time.monotonic()
 
            # Wait until telemetry is populated
            if not self._pose_ready():
                time.sleep(0.05)
                continue
 
            depth = self.receiver.get_frame()
 
            # Skip if no frame or same frame as last iteration
            if depth is None:
                time.sleep(self.PERIOD)
                continue
 
            frame_id = id(depth)            # object identity changes on every copy()
            if frame_id == self._last_frame_id:
                time.sleep(self.PERIOD)
                continue
            self._last_frame_id = frame_id
 
            # --- build point cloud for this frame ---
            pts_cam = self._depth_to_pointcloud(depth)
            if len(pts_cam) == 0:
                time.sleep(self.PERIOD)
                continue
 
            T_world_cam = self._build_world_transform()
            pts_world   = self._transform_points(pts_cam, T_world_cam)
 
            # --- merge into global map ---
            frame_pcd = o3d.geometry.PointCloud()
            frame_pcd.points = o3d.utility.Vector3dVector(pts_world)
 
            with self._map_lock:
                self.global_map += frame_pcd
                # Downsample to keep memory bounded
                self.global_map = self.global_map.voxel_down_sample(self.VOXEL_SIZE)
 
            elapsed = time.monotonic() - t0
            sleep   = max(0.0, self.PERIOD - elapsed)
            time.sleep(sleep)

# =========================
#  ENTRY POINT
# =========================
async def main():
    nav = DroneNavigation()

    task = asyncio.create_task(nav.run())

    try:
        await task
    except KeyboardInterrupt:
        print("\n⌨️ Stopping...")
        nav.stop()
        await asyncio.gather(task, return_exceptions=True)


if __name__ == "__main__":
    asyncio.run(main())
# Project Context

## What this is
PX4 drone simulation with Gazebo (SITL) using MAVSDK Python. The drone has a forward-facing depth camera. We are building live mapping + flight control scripts.

## Stack
- **Simulator**: PX4 SITL + Gazebo (launched via `~/start_px4.sh`)
- **Flight control**: MAVSDK Python (`mavsdk` library, auto-starts `mavsdk_server` on port 50051)
- **Drone model**: `x500_depth` (has depth camera), world: `roboverse` or `aprilworld`
- **ROS 2**: Humble (source `/opt/ros/humble/setup.bash` before running RViz2 version)
- **Connection address**: `udpin://0.0.0.0:14540`

## Key files in ~/Codes/

| File | Purpose |
|------|---------|
| `live_map_rviz2.py` | **Main script** — keyboard + autonomous flight + live mapping streamed to RViz2 |
| `live_map_keyboard.py` | Same but uses matplotlib window instead of RViz2 (no ROS 2 needed) |
| `rviz2_map.rviz` | RViz2 config auto-loaded by `live_map_rviz2.py` |
| `GlobalMapper_new.py` | Accumulates depth frames into a global NED obstacle point cloud |
| `GlobalMapper.py` | Older version of GlobalMapper |
| `drone_control_new.py` | `Drone` class wrapping MAVSDK System (connect, arm_and_takeoff, land, send_position_setpoint) |
| `drone_control.py` | Older version |
| `keyboardcontrol.py` | Standalone keyboard velocity control (no mapping) |
| `get_position_with_task.py` | `SharedState` + `position_monitor_task` (streams NED position + yaw) |
| `depth_receiver.py` | Receives depth images from Gazebo via gz-transport |
| `top_down.py` | `depth_to_xy_map()` — converts depth image to 2D obstacle coordinates |
| `AvoidancePlanner.py`, `VelocityPlanner.py`, etc. | Other planners (not yet integrated) |

## Architecture of live_map_rviz2.py

```
asyncio event loop (main thread)
├── resilient_monitor()       — streams NED pos + yaw into SharedState, auto-restarts on drop
├── mapping_task()            — 2 Hz: depth frame → GlobalMapper → publish /global_map PointCloud2
├── pose_task()               — 10 Hz: publish /drone_pose PoseStamped to RViz2
└── control_loop()            — 20 Hz: keyboard OR autonomous velocity setpoints

Thread: keyboard_thread()     — raw terminal input, sets FlightState flags
Thread: ros_spin_thread()     — rclpy.spin for the MapPublisher node
Subprocess: rviz2             — launched automatically with rviz2_map.rviz config
```

## RViz2 topics
- `/global_map` — `sensor_msgs/PointCloud2` — obstacle cloud (x=east, y=north, z=0, intensity=distance)
- `/drone_pose` — `geometry_msgs/PoseStamped` — drone position + heading arrow
- Frame: `map` (top-down ortho view)

## Controls (both scripts)
| Key | Action |
|-----|--------|
| T | Arm + Takeoff + enter offboard |
| U/J | Forward / Backward |
| H/K | Left / Right |
| W/S | Climb / Descend |
| A/D | Yaw CCW / CW |
| SPACE | Hover |
| P | Toggle autonomous survey (5x5m square) |
| L | Land |
| Q | Quit + save map to `global_obstacles.npy` |

## How to run

**Terminal 1** — start simulation:
```bash
cd ~/
bash start_px4.sh
# pick: x500_depth, roboverse, No QGC
```

**Terminal 2** — run script:
```bash
source /opt/ros/humble/setup.bash   # only needed for live_map_rviz2.py
cd ~/Codes
python3 live_map_rviz2.py           # RViz2 version
# OR
python3 live_map_keyboard.py        # matplotlib version (no source needed)
```

No MicroXRCE agent needed — MAVSDK Python manages its own mavsdk_server.

## Known issues / fixes applied

- **"Connection refused :50051" spam**: Fixed by calling `drone.wait_until_ready()` before starting tasks. Ensures mavsdk_server has an active MAVLink link to PX4 before telemetry streams open.
- **Script crash on arm failure**: Fixed with try/except around `arm_and_takeoff()` — prints error and allows pressing T to retry.
- **One task crash killing everything**: Fixed with `asyncio.gather(..., return_exceptions=True)`.
- **Velocity command killing control loop**: Fixed with try/except around `set_velocity_body()` — resets offboard state on failure.
- **RViz2 drone pose crossed out**: Caused by position monitor never connecting (state.latest_position stays None). Resolved by the wait_until_ready fix.

## Coordinate conventions
- NED frame: North=+X, East=+Y, Down=+Z
- Yaw: degrees, clockwise from North
- RViz2 map frame: x=East, y=North (remapped for standard ROS convention)
- Camera intrinsics K: fx=fy=433, cx=320, cy=240 (640x480 depth image)

# Drone Autonomy Codebase

A Python codebase for autonomous drone navigation using PX4/MAVSDK, Gazebo simulation, depth cameras, and computer vision. All scripts connect to PX4 SITL over UDP (`udpin://0.0.0.0:14540`) unless noted otherwise.

---

## Architecture Overview

```
Gazebo Simulation
    ├── Depth Camera  ──► DepthReceiver ──► AvoidancePlanner / VelocityPlanner / GlobalMapper
    └── RGB Camera    ──► get_video / save_photo / Detector

PX4 SITL (MAVSDK)
    ├── Telemetry     ──► get_position_with_task (SharedState)
    └── Commands      ──► Drone (drone_control.py)
```

---

## File Reference

### Core Drone Control

| File | Description |
|------|-------------|
| `drone_control.py` | **Primary `Drone` class.** Connects via MAVSDK, arms, takes off, and exposes offboard velocity/position NED setpoints. Includes a PID yaw controller (`rotate_to_yaw`) and helper methods (`turn_cw_90`, `turn_ccw_90`, `turn_cw_180`). |
| `drone_control_new.py` | Alternative `Drone` class that uses the `goto_location` action API instead of offboard NED setpoints. Handles lat/lon offset math internally. Better suited for GPS-based waypoint navigation. |

---

### Telemetry & Diagnostics

| File | Description |
|------|-------------|
| `get_battery.py` | Continuously streams battery %, GPS info, in-air status, and position to the terminal using parallel async tasks. |
| `drone_diagnostics.py` | One-shot health check — prints battery voltage, GPS satellite count, and global/home position status then exits. |
| `get_flightmode.py` | Continuously prints the current PX4 flight mode (e.g., OFFBOARD, HOLD, MANUAL). |
| `is_arm_air.py` | Streams armed and in-air boolean flags continuously. |
| `imu.py` | Streams high-rate IMU data at 200 Hz — acceleration, gyro, and magnetometer in FRD frame. |
| `imutest.py` | Simpler IMU streamer at 10 Hz, formatted cleanly per frame. |
| `get_position.py` | Subscribes to `position_velocity_ned` telemetry and prints NED position live. Also runs a minimal offboard position-hold loop concurrently. |

---

### Shared Async Infrastructure

| File | Description |
|------|-------------|
| `get_position_with_task.py` | **Core concurrency module.** Provides `SharedState` (stores latest NED position + yaw) and `position_monitor_task` (background async task that keeps `SharedState` up-to-date). Used by most navigation scripts. Also includes a `control_loop` template for writing position-based controllers. |

---

### Movement & Manual Control

| File | Description |
|------|-------------|
| `takeoff_and_land.py` | Minimal script: connect → arm → takeoff → hover 5 s → land. Good starting point. |
| `basic_offboard.py` | Demonstrates offboard velocity control: fly north at 1 m/s for 3 s, hover, then ascend. |
| `go_to.py` | GPS `goto_location` with pre-flight system checks (GPS lock, battery, flight mode) and haversine-based arrival monitoring. |
| `keyboardcontrol.py` | Full **keyboard controller** using `VelocityBodyYawspeed` offboard mode. Keys: `W/S` climb/descend, `A/D` yaw, `U/J` forward/back, `H/K` left/right, `Space` hover, `T` takeoff, `L` land, `Q` quit. |

---

### Depth Camera

| File | Description |
|------|-------------|
| `depth_receiver.py` | **`DepthReceiver` class.** Subscribes to a Gazebo depth image topic via `gz.transport` and exposes a thread-safe `get_frame()` method returning a float32 NumPy depth map (meters). |
| `depthtest.py` | Subscribes to the depth camera and prints the minimum valid depth per frame. Validates the depth pipeline. |
| `depthcloud.py` | Subscribes to the depth topic and displays the depth image using OpenCV with a JET colormap for visualization. Also contains a standalone occupancy mapping + frontier exploration system (DFS path planner, log-odds map, visualizer). |
| `top_down.py` | **`depth_to_xy_map()` function.** Back-projects a depth image to a top-down 2D obstacle point cloud using camera intrinsics. Filters by valid depth range and obstacle height band. Returns an Mx2 array of `[X_lateral, Z_forward]` obstacle coordinates in meters. |

---

### Obstacle Avoidance

| File | Description |
|------|-------------|
| `AvoidancePlanner.py` | **`AvoidancePlanner` class.** Polar histogram-based obstacle avoidance from a depth map. Divides the image into angular bins, computes obstacle density per bin, selects the clearest direction, and applies emergency override when blocked. Outputs either NED position setpoints (`compute_position_ned`) or body-frame velocity (`compute_velocity`). Includes exponential smoothing on both velocity and position outputs. |
| `VelocityPlanner.py` | Identical logic to `AvoidancePlanner` but only exposes the velocity output (`compute_velocity`). Slightly simpler interface for velocity-based controllers. |
| `avoid.py` | **Position-based autonomous navigation.** Combines `DepthReceiver` + `AvoidancePlanner` + `drone_control_new.Drone`. Runs at 20 Hz with a grid heading system (N → E → S → W). When blocked, rotates to the next cardinal direction. Uses `SharedState` for real-time pose updates. |
| `avoid_with_detect.py` | Same as `avoid.py` (identical implementation). |
| `vel_avoidance.py` | **Velocity-based autonomous navigation.** Uses `VelocityPlanner` instead of `AvoidancePlanner`. Converts camera-frame velocity to NED frame using the current yaw before sending. Grid heading system with rotate-on-block fallback. |

---

### Mapping

| File | Description |
|------|-------------|
| `GlobalMapper.py` | **`GlobalMapper` class (v1).** Accumulates depth frames into a global NED obstacle point cloud. Each frame is back-projected using `depth_to_xy_map`, then rotated from camera frame to NED global frame using the drone's yaw. Includes exponential yaw smoothing. Sample `run()` moves the drone north 3× then east and plots the result. |
| `GlobalMapper_new.py` | **`GlobalMapper` class (v2, preferred).** Refactored version with null-safe pose handling (`latest_pose_from_state`), cleaner frame collection helper (`collect_and_map_frame`), graceful shutdown, and saves the final point cloud to `global_obstacles.npy`. |

---

### Path Planning

| File | Description |
|------|-------------|
| `PointCloudPlanner.py` | **`PointCloudPlanner` class (v1).** Wraps a KDTree built from a 2D obstacle point cloud. Provides `is_collision_free(point)` and `get_nearest_obstacle(point)` queries. |
| `PointCloudPlanner_new.py` | **`PointCloudPlanner` class (v2, preferred).** Adds input validation, bounds checking (`in_bounds`), and treats out-of-bounds points as collisions. |
| `RRTStarPlanner.py` | **`RRTStarPlanner` class.** RRT* planner in continuous 2D NED space. Samples random nodes, steers toward them, checks collision against a KDTree obstacle map, rewires the tree to minimize cost, and applies shortcut smoothing to the final path. Returns a smoothed NumPy path array of `[north, east]` waypoints. |
| `RRTExample.py` | Example script: builds a `GlobalMapper` map from 60 simulated depth frames, then plans a path from `[0,0]` to `[15,2]` using `RRTStarPlanner`, and visualizes the result. |
| `RRTExample_new.py` | Same as `RRTExample.py` but imports from the newer `GlobalMapper_new` module. |

---

### Object Detection

| File | Description |
|------|-------------|
| `Detector.py` | **`Detector` class.** Threaded YOLO inference pipeline using Ultralytics. Accepts images via `submit_image()`, runs inference in worker threads, saves annotated images to disk, pushes frames to a dedicated display thread (dropping stale frames), and fires a user callback on detection. Configurable model path, confidence threshold, number of workers, and device. |
| `UseDetectorExample.py` | **`VisionApp`** integrates `Detector` with the Gazebo camera feed. Subscribes to the RGB camera topic, converts frames from RGB to BGR, and submits them to the detector pipeline for real-time YOLO inference and display. |
| `Train_YOLO_Models.ipynb` | Jupyter notebook for training custom YOLO models. |
| `yolov10n.pt` | Pre-trained YOLOv10-nano model weights used by `UseDetectorExample.py`. |

---

### Camera & Vision Utilities

| File | Description |
|------|-------------|
| `get_video.py` | Subscribes to a Gazebo RGB camera topic via `gz.transport` and displays the live feed using OpenCV. |
| `get_video_old.py` | Older approach — captures video from PX4 SITL via a GStreamer UDP pipeline using `cv2.VideoCapture`. Runs drone MAVSDK commands concurrently in a thread. |
| `photo.py` | Uses MAVSDK camera API to set video mode and trigger a single photo capture. |
| `save_photo.py` | Subscribes to a Gazebo camera topic and saves every incoming frame to disk as a sequentially numbered JPEG under `captured_images/`. |

---

## Key Dependencies

- **MAVSDK** — drone connection, telemetry, and flight actions
- **gz.transport / gz.msgs** — Gazebo topic subscriptions (depth & RGB camera)
- **NumPy / SciPy** — depth processing, KDTree, path planning math
- **OpenCV** — image display, color conversion, video capture
- **Ultralytics (YOLO)** — object detection
- **Matplotlib** — occupancy map and path visualization
- **asyncio** — all drone control is async; most scripts use `asyncio.run(main())`

---

## Typical Usage Patterns

**Basic telemetry check:**
```bash
python3 drone_diagnostics.py
```

**Keyboard-controlled flight:**
```bash
python3 keyboardcontrol.py
```

**Autonomous avoidance (position-based):**
```bash
python3 avoid.py
```

**Autonomous avoidance (velocity-based):**
```bash
python3 vel_avoidance.py
```

**Build a map and plan a path:**
```bash
python3 RRTExample_new.py
```

**Live object detection:**
```bash
python3 UseDetectorExample.py
```

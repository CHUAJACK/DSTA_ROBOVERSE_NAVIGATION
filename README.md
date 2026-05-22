# Frontier Exploration Drone

Autonomous drone exploration using PX4 SITL + Gazebo + ROS 2 Humble. The drone builds a live 3D OctoMap, slices it to a 2D occupancy grid, and uses Wavefront Frontier Detection + A* to navigate unexplored space. The frontier selection is an weighted formula that balances between distance, size and angle to direction the drone is heading.

## Stack

| Component | Details |
|-----------|---------|
| Simulator | PX4 SITL + Gazebo |
| Flight control | MAVSDK Python |
| Mapping | OctoMap server + custom 2D slicer |
| ROS 2 | Humble |
| CV model | yolo8n |
| Drone model | `x500_vision`, world: `roboverse` |

---

## Dependencies

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# OctoMap packages
sudo apt install ros-humble-octomap ros-humble-octomap-msgs \
                 ros-humble-octomap-ros ros-humble-octomap-server \
                 liboctomap-dev

# Python
pip install mavsdk
pip install "numpy==1.26.4"

# Yolo
pip install ultralytics
```

---

## Installation

```bash
git clone https://github.com/CHUAJACK/DSTA_ROBOVERSE_NAVIGATION.git
git clone https://github.com/CHUAJACK/PointCloudRangeFilter.git
cd DSTA_ROBOVERSE_NAVIGATION
git checkout octoZD

mkdir src && cd src
git clone https://github.com/CHUAJACK/octomap_z_slicer.git
git clone https://github.com/CHUAJACK/PointCloudRangeFilter.git
cd ..

colcon build
```

---

## Running

### Terminal 1 — Gazebo / PX4 SITL

```bash
cd ~/
./start_px4.sh
# Select: x500_vision, roboverse, QGC
# Wait ~30s for Gazebo and PX4 to fully load
```

Once loaded, run these in the `pxh>` MAVLink shell — **required every fresh PX4 start**:

```
commander set_ekf_origin 47.397742 8.545594 488.0
```

Wait for `Drone Ready for Takeoff` to confirm drone ready.

### Terminal 2 — Navigation + Mapping

```bash
cd ~/DSTA_ROBOVERSE_NAVIGATION
source /opt/ros/humble/setup.bash
ros2 launch frontier_launch.py
```

### Terminal 3 — Run CV detection code

```bash
cd ~/DSTA_ROBOVERSE_NAVIGATION
source .venv/bin/activate
python3 can_detector_gps.py
```

### Terminal 4 — Frontier Detection and Mapping

```bash
cd ~/DSTA_ROBOVERSE_NAVIGATION/cv
source /opt/ros/humble/setup.bash
source venv/bin/activate
python3 frontier_explore_drone.py 
```

The drone will:
1. Connect to PX4 via MAVSDK
2. Arm and take off to 2.5 m
3. Wait for OctoMap pipeline health check (10 frames)
4. Do a **double 360° sweep** to seed the initial map
5. Score and select the best frontier (distance + size + heading)
6. Plan an A* path with wall-proximity cost gradient, densified and smoothed
7. Follow the path, replanning only if a waypoint becomes BLOCKED or a much closer frontier appears
8. Do a 360° yaw sweep at the frontier, waiting for OctoMap to catch up
9. Repeat until no frontiers remain, then land
10. Print total exploration time on completion

---

## What You See in RViz2

| Display | Topic | Description |
|---------|-------|-------------|
| 2D Slice | `/octomap_2d_slice` | Live occupancy grid at drone height |
| OctoMap 3D | `/occupied_cells_vis_array` | 3D obstacle cloud |
| Frontier Markers | `/frontier_markers` | Green = frontiers, orange = current goal |
| Exploration Path | `/exploration_path` | Blue A* path to current frontier |

---

## Tuning Parameters

All parameters are at the top of `frontier_explore_drone.py`:

```python
# Frontier scoring weights
W_DISTANCE = 2.0      # prefer closer frontiers
W_SIZE     = 0.5      # prefer larger frontier clusters
W_HEADING  = 0.3      # prefer frontiers ahead of drone

# Velocity controller
KP_XY          = 0.8   # position P gain — lower if oscillating
MAX_SPEED      = 1.6   # m/s horizontal
APPROACH_SPEED = 1.0   # m/s — speed cap when within APPROACH_DIST of final goal
APPROACH_DIST  = 3.0   # m — distance from final goal at which speed is capped
ARRIVAL_DIST   = 1.0   # m — intermediate waypoint reached
FINAL_DIST     = 1.2   # m — frontier centroid reached

# Yaw sweep
SWEEP_RATE_DPS   = 40.0  # deg/s rotation speed
SWEEP_TOTAL_DEG  = 360.0
SWEEP_POST_WAIT  = 1.0   # min seconds to wait after sweep for OctoMap
SWEEP_MAP_FRAMES = 3     # new OctoMap frames to wait for after sweep (adaptive)

# A* / planning
INFLATION           = 2                       # obstacle padding in grid cells (0.3 m per cell)
WALL_COSTS          = [3.0, 3.0, 1.5, 0.5]  # A* cost penalty at 1/2/3/4 cells from wall
MIN_CLUSTER         = 3                       # ignore frontier clusters smaller than this
VISITED_RADIUS      = 0.75                   # m — don't revisit frontiers within this radius
MIN_FRONTIER_DIST_M = 1.5                    # ignore frontiers closer than this

# Mid-flight replanning
REPLAN_INTERVAL_S    = 0.2   # seconds between replan checks during flight
REPLAN_MIN_SAVING_M  = 3.0   # abort path if a closer frontier saves this many metres
REPLAN_WALL_COST_THR = 3.5   # effectively disabled — inflation already guarantees wall clearance
```

OctoMap settings are in `frontier_launch.py`:

```python
'resolution': 0.35,              # m per voxel
'sensor_model.max_range': 15.0,  # m
'sensor_model/hit':  0.8,        # occupied update probability
'sensor_model/miss': 0.4,        # free-space update probability
'transform_tolerance': 1.0,      # TF timing headroom (seconds)
```

---

## Autonomous Behaviours

- **OctoMap health check**: waits for 10 consecutive map frames before starting — catches TF timing failures at startup before the drone moves
- **Path commitment**: the drone commits to its A* path and only replans if a waypoint cell becomes directly BLOCKED within 5 m, or a frontier at least 3 m closer appears — no replanning on minor map updates
- **Path densification + smoothing**: A* waypoints are densified at 1 m intervals then gradient-smoothed for curved, wall-avoiding flight paths
- **Unknown cell padding**: the occupancy grid is wrapped in a 1-cell UNKNOWN border so WFD always finds frontier cells at the map edges
- **Push away from obstacles**: if replanning due to wall proximity, the drone backs away from immediately adjacent obstacle cells (3×3 neighbourhood) before picking a new path
- **Blocked frontier handling**: if a frontier triggers two consecutive blocked results, it is skipped; skip list clears only on actual arrival at a frontier
- **Frontier pullback**: navigation goal is pulled 2 cells back toward the robot to avoid flying into unregistered walls at frontier edges
- **Adaptive post-sweep wait**: waits for a minimum number of new OctoMap frames after each sweep before moving on
- **Point cloud downsampling**: `pointcloud_downsample.py` applies stride-2 downsampling (4× fewer points) before OctoMap ingestion, reducing processing lag without affecting map quality

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Drone lands immediately | Old OctoMap loaded — no frontiers found | Restart Terminal 2 |
| Battery failsafe / crash | Params reset on PX4 restart | Re-run `param set` commands in `pxh>` |
| OctoMap not rendering | TF timing mismatch at startup | Watch for `[EXPLORE] WARNING: only N/10 frames` — restart Terminal 2 |
| Ghost points not clearing | `sensor_model/hit` too high | Lower toward 0.7 |
| Drone keeps replanning same path | Path near wall, no alternative route | Frontier will be skipped after 2 blocked attempts automatically |
| Drone deviates from path / clips walls | `ARRIVAL_DIST` too loose | Lower to 0.5–0.7 m |
| Drone oscillates at waypoints | `KP_XY` too high | Lower to 0.5–0.6 |
| A* skipping all frontiers | Inflation blocking all paths | Lower `INFLATION` to 1 |
| Gazebo / RViz2 close silently | OOM kill | Lower `sensor_model.max_range` or increase `STRIDE` in `pointcloud_downsample.py` |
| `/depth_camera/points_filtered` not publishing | Downsampler crashed | Check Terminal 2 output; ROS 2 must be sourced |
| `[EXPLORE] Still exploring` every minute | Normal heartbeat | Not an error |

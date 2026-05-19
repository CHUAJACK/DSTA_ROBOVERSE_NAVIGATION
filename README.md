# Frontier Exploration Drone

Autonomous drone exploration using PX4 SITL + Gazebo + ROS 2 Humble. The drone builds a live 3D OctoMap, slices it to a 2D occupancy grid, and uses Wavefront Frontier Detection + A* to navigate unexplored space.

## Stack

| Component | Details |
|-----------|---------|
| Simulator | PX4 SITL + Gazebo |
| Flight control | MAVSDK Python |
| Mapping | OctoMap server + custom 2D slicer |
| ROS 2 | Humble |
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
```

---

## Installation

```bash
git clone https://github.com/CHUAJACK/DSTA_ROBOVERSE_NAVIGATION.git
cd DSTA_ROBOVERSE_NAVIGATION
git checkout octoZD

mkdir src && cd src
git clone https://github.com/CHUAJACK/octomap_z_slicer.git
cd ..

colcon build --packages-select octomap_2d_slicer
```

---

## Running

### Terminal 1 — Gazebo / PX4 SITL

```bash
cd ~/
bash start_px4.sh
# Select: x500_vision, roboverse, No QGC
# Wait ~30s for Gazebo and PX4 to fully load
```

Once loaded, run these in the `pxh>` MAVLink shell — **required every fresh PX4 start**:

```
commander set_ekf_origin 47.397742 8.545594 488.0
param set COM_LOW_BAT_ACT 0
param set SIM_BAT_DRAIN 0
```

Wait ~10s for EKF to stabilise (`home set` confirms it).

### Terminal 2 — Mapping Stack + RViz2

```bash
source /opt/ros/humble/setup.bash
cd ~/octomapper
source install/setup.bash
ros2 launch frontier_launch.py
```

Wait for RViz2 to open. **Restart this terminal between runs** to clear the old OctoMap.

### Terminal 3 — Frontier Exploration

```bash
source /opt/ros/humble/setup.bash
cd ~/octomapper
python3 frontier_explore_drone.py
```

The drone will:
1. Connect to PX4 via MAVSDK
2. Arm and take off to 2.5 m
3. Do a **double 360° sweep** to seed the initial map
4. Score and select the best frontier (distance + size + heading)
5. Plan an A* path with wall-proximity cost gradient
6. Follow the path, replanning every second if obstacles appear or a closer frontier is found
7. Do a 360° yaw sweep at the frontier, waiting for OctoMap to catch up
8. Repeat until no frontiers remain, then land
9. Print total exploration time on completion

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
KP_XY        = 0.5   # position P gain — lower if oscillating
MAX_SPEED    = 1.8   # m/s horizontal
ARRIVAL_DIST = 0.7   # m — intermediate waypoint reached
FINAL_DIST   = 1.2   # m — frontier centroid reached

# Yaw sweep
SWEEP_RATE_DPS   = 40.0  # deg/s rotation speed
SWEEP_TOTAL_DEG  = 360.0
SWEEP_POST_WAIT  = 1.0   # min seconds to wait after sweep for OctoMap
SWEEP_MAP_FRAMES = 3     # new OctoMap frames to wait for after sweep (adaptive)

# A* / planning
INFLATION           = 1          # obstacle padding in grid cells (0.35 m per cell)
WALL_COSTS          = [3.0, 1.5, 0.5]  # A* cost penalty at 1/2/3 cells from wall
MIN_CLUSTER         = 3          # ignore frontier clusters smaller than this
VISITED_RADIUS      = 0.75       # m — don't revisit frontiers within this radius
MIN_FRONTIER_DIST_M = 1.5        # ignore frontiers closer than this (camera blind spot)

# Mid-flight replanning
REPLAN_INTERVAL_S    = 1.0  # seconds between replan checks during flight
REPLAN_MIN_SAVING_M  = 2.0  # abort path if a closer frontier saves this many metres
REPLAN_WALL_COST_THR = 2.5  # abort path if a waypoint is this close to a wall
```

OctoMap settings are in `frontier_launch.py`:

```python
'resolution': 0.35,             # m per voxel
'sensor_model.max_range': 10.0  # m
```

---

## Autonomous Behaviours

- **Mid-flight replanning**: every second, checks if an obstacle appeared on the path, if the path passes too close to a newly registered wall, or if a significantly closer frontier has appeared — replans immediately if so
- **Push away from obstacles**: if replanning due to wall proximity, the drone backs away from nearby obstacles before picking a new path
- **Frontier pullback**: if a frontier centroid falls in UNKNOWN space, the navigation goal is pulled 2 cells back toward the robot to avoid flying into unregistered walls
- **Depth camera blind spot filter**: frontiers closer than 1.5 m are ignored (camera can't map them reliably at close range)
- **Adaptive post-sweep wait**: waits for a minimum number of new OctoMap frames after each sweep before moving on, so map processing keeps up regardless of map size
- **Unreachable frontier handling**: after 3 failed A* attempts, a frontier is temporarily skipped; cleared automatically when any path succeeds

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Drone lands immediately | Old OctoMap loaded — no frontiers found | Restart Terminal 2 |
| Battery failsafe / crash | Params reset on PX4 restart | Re-run `param set` commands in `pxh>` |
| Drone keeps replanning same path | Path near wall, no alternative route | Raise `REPLAN_WALL_COST_THR` or lower `WALL_COSTS` |
| Drone oscillates at waypoints | `KP_XY` too high | Lower to 0.3–0.4 |
| A* skipping all frontiers | Inflation blocking all paths | Lower `INFLATION` to 0 |
| Gazebo / RViz2 close silently | OOM kill | Resolution and range already tuned to mitigate |
| `[EXPLORE] Still exploring` every minute | Normal heartbeat | Not an error |

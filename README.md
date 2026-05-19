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
# ROS 2 Humble (source before running)
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
git clone https://github.com/CHUAJACK/PointCloudRangeFilter.git
cd DSTA_ROBOVERSE_NAVIGATION
git checkout octoZD

mkdir src && cd src
git clone https://github.com/CHUAJACK/octomap_z_slicer.git
cd ..

colcon build
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

Once loaded, in the `pxh>` MAVLink shell run these **every fresh PX4 start**:

```
commander set_ekf_origin 47.397742 8.545594 488.0
param set COM_LOW_BAT_ACT 0
param set SIM_BAT_DRAIN 0
```

Wait ~10s for EKF to stabilise (`home set` message confirms it).

### Terminal 2 — Mapping Stack + RViz2

```bash
source /opt/ros/humble/setup.bash
cd ~/octomapper
source install/setup.bash
ros2 launch frontier_launch.py
```

Wait for the RViz2 window to open showing the 2D slice map. **Restart this terminal between script runs** to clear the old OctoMap.

### Terminal 3 — Frontier Exploration

```bash
source /opt/ros/humble/setup.bash
cd ~/octomapper
python3 frontier_explore_drone.py
```

The drone will:
1. Connect to PX4 via MAVSDK
2. Arm and take off to 2.5 m
3. Wait for the first map frame
4. Score and select the best frontier (distance + size + heading)
5. Plan an A* path and follow it
6. Do a 360° yaw sweep at the frontier
7. Repeat until no frontiers remain, then land

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
W_DISTANCE = 2.0      # prefer closer frontiers
W_SIZE     = 0.5      # prefer larger frontier clusters
W_HEADING  = 0.3      # prefer frontiers ahead of drone

KP_XY        = 0.8   # position P gain
MAX_SPEED    = 1.5   # m/s horizontal
ARRIVAL_DIST = 0.7   # m — intermediate waypoint reached
FINAL_DIST   = 1.2   # m — frontier centroid reached

SWEEP_RATE_DPS  = 40.0
SWEEP_TOTAL_DEG = 360.0

INFLATION      = 1    # obstacle padding in grid cells (0.35 m per cell)
MIN_CLUSTER    = 3    # ignore frontier clusters smaller than this
VISITED_RADIUS = 0.75 # m — don't revisit frontiers within this radius
```

OctoMap settings are in `frontier_launch.py`:

```python
'resolution': 0.35,           # m per voxel
'sensor_model.max_range': 10.0  # m
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Drone lands immediately after takeoff | Old OctoMap loaded — no frontiers found | Restart Terminal 2 |
| Battery failsafe / drone crashes mid-flight | Battery params reset on PX4 restart | Re-run `param set` commands in `pxh>` |
| A* skipping many frontiers | `INFLATION` too large | Lower `INFLATION` |
| Drone oscillates at waypoints | `KP_XY` too high | Lower to 0.4–0.6 |
| No frontier markers in RViz2 | `frontier_explore_drone.py` not running | Check Terminal 3 |
| TF errors in RViz2 | `odom → base_link` not broadcasting | `frontier_explore_drone.py` must be running (it starts the TF broadcaster) |
| Gazebo / RViz2 close silently | OOM kill — too much RAM usage | Already mitigated: resolution=0.35, max_range=10 |

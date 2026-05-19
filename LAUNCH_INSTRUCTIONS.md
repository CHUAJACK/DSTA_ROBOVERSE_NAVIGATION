# Frontier Exploration — Launch Instructions

## Overview

| File | Role |
|------|------|
| `frontier_launch.py` | ROS 2 launch: gz bridge + OctoMap + 2D slicer + RViz2 |
| `frontier_explore_drone.py` | Autonomous flight: WFD + A* + velocity controller |
| `frontier_explore.rviz` | RViz2 config with frontier markers + path overlay |

---

## Terminal 1 — Start Gazebo / PX4 SITL

```bash
cd ~/
bash start_px4.sh
# Select: x500_depth, roboverse, No QGC
# Wait for Gazebo + PX4 to fully load (~30s)
```

Once Gazebo is running, set the EKF origin in the MAVLink shell:
```
commander set_ekf_origin 47.397742 8.545594 488.0
# Should print: home set
```

---

## Terminal 2 — Mapping Stack + RViz2

```bash
source /opt/ros/humble/setup.bash
cd ~/octomapper
source install/setup.bash
ros2 launch frontier_launch.py
```

Wait until you see:
- `octomap_server` printing transform messages
- RViz2 window opens showing the 2D slice map

---

## Terminal 3 — Frontier Exploration

```bash
source /opt/ros/humble/setup.bash
cd ~/octomapper
python3 frontier_explore_drone.py
```

The drone will:
1. Connect to PX4
2. Arm and take off to **2.5 m**
3. Wait for the first map frame
4. Pick the highest-scoring frontier
5. Plan an A* path and follow it
6. Do a **360° yaw sweep** at the frontier
7. Repeat until no frontiers remain, then land

---

## What You See in RViz2

| Display | Topic | Description |
|---------|-------|-------------|
| **2D Slice** | `/octomap_2d_slice` | Live 2D occupancy grid at drone height |
| **OctoMap 3D** | `/occupied_cells_vis_array` | 3D obstacle cloud |
| **Frontier Markers** | `/frontier_markers` | Green spheres = frontiers, orange = current goal |
| **Exploration Path** | `/exploration_path` | Blue A* path to current frontier |
| **TF** | — | `map → odom → base_link → camera_link` tree |

---

## Tuning Parameters

All tunable parameters are at the top of `frontier_explore_drone.py`:

```python
# Frontier scoring weights
W_DISTANCE = 1.0    # prefer closer frontiers (increase to go nearest first)
W_SIZE     = 0.5    # prefer larger clusters (increase for more info gain)
W_HEADING  = 0.3    # prefer frontiers ahead of drone (increase for smoother flight)

# Controller
KP_XY        = 0.8   # position P gain — lower if oscillating, raise for faster response
MAX_SPEED    = 1.5   # m/s — max horizontal speed
ARRIVAL_DIST = 0.7   # m — waypoint reached threshold
FINAL_DIST   = 1.2   # m — frontier reached threshold

# Sweep
SWEEP_RATE_DPS  = 40.0   # deg/s — yaw sweep speed
SWEEP_TOTAL_DEG = 360.0  # always full sweep

# Planning
INFLATION      = 2    # grid cells inflated around obstacles (0 = no inflation)
MIN_CLUSTER    = 4    # min frontier cluster size to consider
VISITED_RADIUS = 1.5  # m — don't revisit frontiers within this radius
```

---

## Stopping Early

Press `Ctrl+C` in Terminal 3 to interrupt. The drone will not auto-land on interrupt — switch to Terminal 3 and run:

```bash
python3 -c "
import asyncio
from mavsdk import System
async def land():
    d = System()
    await d.connect('udp://:14540')
    await d.action.land()
asyncio.run(land())
"
```

Or use QGC / `keyboardcontrol.py` to land manually.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `No frontiers found` immediately | Map not received yet | Wait longer after launching Terminal 2 |
| Drone oscillates at waypoints | `KP_XY` too high | Lower to 0.4–0.6 |
| Drone stops short of frontier | `FINAL_DIST` too large | Lower to 0.8 |
| A* skipping many frontiers | `INFLATION` too large | Lower to 1 |
| RViz shows no frontier markers | `frontier_explore_drone.py` not running | Check Terminal 3 |
| TF errors in RViz | `odom → base_link` not broadcasting | Ensure `frontier_explore_drone.py` is running (it starts the TF broadcaster) |

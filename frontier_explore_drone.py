#!/usr/bin/env python3
"""
frontier_explore_drone.py — Autonomous frontier exploration.

Stack:
  WFD (frontier_detector.py) → frontier cells
  Tunable scoring             → best frontier selection
  A* on inflated 2D grid      → collision-free path
  P velocity controller       → waypoint following in NED
  360° yaw sweep              → full-coverage scan at each frontier

Run AFTER: ros2 launch frontier_launch.py
"""
import asyncio
import heapq
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

from position_tf import ODOMtoBASELINKTF, Telemetry
from frontier_detector import (
    wavefront_frontier_detection, cluster_frontiers,
    centroid as cluster_centroid, FREE, BLOCKED, UNKNOWN,
)

# ═══════════════════════════════════════════════════════════════════════════════
# TUNABLE PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════

MAVSDK_ADDRESS  = "udp://:14540"
TAKEOFF_ALT_M   = 2.5          # metres AGL

# Frontier scoring weights (set any weight to 0 to disable that term)
W_DISTANCE = 2.0    # 1/dist       — prefer closer frontiers
W_SIZE     = 0.5    # log(size)    — prefer information-rich clusters
W_HEADING  = 0.3    # cos(Δangle)  — prefer frontiers ahead of drone


# Velocity controller
KP_XY        = 0.8    # proportional gain (m/s per m error)
MAX_SPEED    = 1.5    # m/s horizontal
ARRIVAL_DIST = 0.7   # m — intermediate waypoint reached
FINAL_DIST   = 1.2   # m — frontier centroid reached

# 360° yaw sweep
SWEEP_RATE_DPS  = 40.0    # degrees per second
SWEEP_TOTAL_DEG = 360.0

# A* / planning
INFLATION      = 1    # grid cells to inflate around obstacles
MIN_CLUSTER    = 3    # ignore frontier clusters smaller than this
VISITED_RADIUS = 0.75  # m — skip re-visiting frontiers within this radius

CONTROL_HZ = 10       # velocity setpoint rate


# ═══════════════════════════════════════════════════════════════════════════════
# GRID UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════

def world_to_grid(wx, wy, meta):
    """ENU (x=East, y=North) → (row, col), clamped to grid bounds."""
    res = meta.resolution
    col = int((wx - meta.origin.position.x) / res)
    row = int((wy - meta.origin.position.y) / res)
    return max(0, min(meta.height - 1, row)), max(0, min(meta.width - 1, col))


def grid_to_world(row, col, meta):
    """(row, col) → ENU cell centre (x=East, y=North)."""
    res = meta.resolution
    x = meta.origin.position.x + (col + 0.5) * res
    y = meta.origin.position.y + (row + 0.5) * res
    return x, y   # (East, North)


def inflate_grid(grid, radius):
    rows, cols = len(grid), len(grid[0])
    out = [row[:] for row in grid]
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == BLOCKED:
                for dr in range(-radius, radius + 1):
                    for dc in range(-radius, radius + 1):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < rows and 0 <= nc < cols:
                            out[nr][nc] = BLOCKED
    return out


def nearest_free(grid, r, c, radius=5):
    """Return nearest non-BLOCKED cell to (r,c), or None."""
    rows, cols = len(grid), len(grid[0])
    if 0 <= r < rows and 0 <= c < cols and grid[r][c] != BLOCKED:
        return r, c
    for d in range(1, radius + 1):
        for dr in range(-d, d + 1):
            for dc in range(-d, d + 1):
                if abs(dr) == d or abs(dc) == d:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != BLOCKED:
                        return nr, nc
    return None


# ═══════════════════════════════════════════════════════════════════════════════
# A* PLANNER
# ═══════════════════════════════════════════════════════════════════════════════

def astar(grid, start, goal):
    """
    A* on 2D occupancy grid. Passes through FREE and UNKNOWN cells.
    Returns list of (row, col) from start to goal, or None if unreachable.
    """
    rows, cols = len(grid), len(grid[0])

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def passable(r, c):
        return 0 <= r < rows and 0 <= c < cols and grid[r][c] != BLOCKED

    DIRS  = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    COSTS = [1.0,   1.0,  1.0,   1.0,  1.414,   1.414,  1.414,  1.414]

    open_set = [(h(start, goal), 0.0, start)]
    g_score  = {start: 0.0}
    came_from = {}

    while open_set:
        _, g_cur, cur = heapq.heappop(open_set)
        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            return list(reversed(path))
        if g_cur > g_score.get(cur, float('inf')):
            continue
        for (dr, dc), cost in zip(DIRS, COSTS):
            nb = (cur[0] + dr, cur[1] + dc)
            if not passable(*nb):
                continue
            ng = g_cur + cost
            if ng < g_score.get(nb, float('inf')):
                g_score[nb] = ng
                came_from[nb] = cur
                heapq.heappush(open_set, (ng + h(nb, goal), ng, nb))
    return None


def _line_clear(grid, a, b):
    """Bresenham line: True if no BLOCKED cell between a and b."""
    rows, cols = len(grid), len(grid[0])
    r0, c0 = a; r1, c1 = b
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr, sc = (1 if r1 > r0 else -1), (1 if c1 > c0 else -1)
    err = dr - dc
    r, c = r0, c0
    while True:
        if not (0 <= r < rows and 0 <= c < cols):
            return False
        if grid[r][c] == BLOCKED:
            return False
        if r == r1 and c == c1:
            return True
        e2 = 2 * err
        if e2 > -dc: err -= dc; r += sr
        if e2 <  dr: err += dr; c += sc


def simplify_path(path, grid):
    """Remove collinear waypoints where direct line-of-sight is clear."""
    if len(path) <= 2:
        return path
    result = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if _line_clear(grid, path[i], path[j]):
                break
            j -= 1
        result.append(path[j])
        i = j
    return result


# ═══════════════════════════════════════════════════════════════════════════════
# ROS 2 MAP NODE
# ═══════════════════════════════════════════════════════════════════════════════

class MapNode(Node):
    """Subscribes to /octomap_2d_slice; publishes frontier + path markers."""

    def __init__(self):
        super().__init__('frontier_map_node')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self._lock = threading.Lock()
        self._grid = None
        self._meta = None

        self.create_subscription(OccupancyGrid, '/octomap_2d_slice', self._map_cb, 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        self._path_pub   = self.create_publisher(Path, '/exploration_path', 10)

    def _map_cb(self, msg: OccupancyGrid):
        rows, cols = msg.info.height, msg.info.width
        raw  = msg.data
        grid = []
        for r in range(rows):
            row = []
            for c in range(cols):
                v = raw[r * cols + c]
                row.append(BLOCKED if v >= 50 else (UNKNOWN if v == -1 else FREE))
            grid.append(row)
        with self._lock:
            self._grid = grid
            self._meta = msg.info

    def get_map(self):
        with self._lock:
            return self._grid, self._meta

    def publish_frontiers(self, clusters, goal_rc, meta):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        clear = Marker(); clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i, cl in enumerate(clusters):
            cr, cc = cluster_centroid(cl)
            ex, ey = grid_to_world(cr, cc, meta)
            is_goal = goal_rc is not None and (cr, cc) == goal_rc
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = ex
            m.pose.position.y = ey
            m.pose.position.z = TAKEOFF_ALT_M
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.a = 0.85
            m.color.r = 1.0 if is_goal else 0.0
            m.color.g = 0.5 if is_goal else 0.9
            m.color.b = 0.0
            ma.markers.append(m)
        self._marker_pub.publish(ma)

    def publish_path(self, waypoints_ne):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for n, e in waypoints_ne:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = e   # ENU x = East = NED east
            ps.pose.position.y = n   # ENU y = North = NED north
            ps.pose.position.z = TAKEOFF_ALT_M
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)


# ═══════════════════════════════════════════════════════════════════════════════
# FRONTIER EXPLORER
# ═══════════════════════════════════════════════════════════════════════════════

class FrontierExplorer:
    def __init__(self, drone: System, tel: Telemetry, map_node: MapNode):
        self.drone   = drone
        self.tel     = tel
        self.map     = map_node
        self.visited = []   # (north, east) of explored frontiers

    # ── internals ────────────────────────────────────────────────────────────

    def _enu_pos(self):
        """Drone ENU position: (x=East, y=North)."""
        return self.tel.east, self.tel.north

    def _ned_pos(self):
        return self.tel.north, self.tel.east

    def _yaw(self):
        return self.tel.yaw_deg or 0.0

    def _already_visited(self, north, east):
        return any(math.hypot(north - vn, east - ve) < VISITED_RADIUS
                   for vn, ve in self.visited)

    # ── frontier scoring ──────────────────────────────────────────────────────

    def _score(self, cluster, robot_r, robot_c, meta):
        cr, cc = cluster_centroid(cluster)
        ex, ey = grid_to_world(cr, cc, meta)   # ENU: x=E, y=N
        frontier_n, frontier_e = ey, ex         # NED

        dist_cells = math.hypot(cr - robot_r, cc - robot_c)
        dist_score = W_DISTANCE / (dist_cells + 1.0)

        size_score = W_SIZE * math.log1p(len(cluster))

        dn = frontier_n - (self.tel.north or 0.0)
        de = frontier_e - (self.tel.east  or 0.0)
        angle_to = math.degrees(math.atan2(de, dn))
        diff = (angle_to - self._yaw() + 180) % 360 - 180
        heading_score = W_HEADING * (1.0 + math.cos(math.radians(diff))) / 2.0

        return dist_score + size_score + heading_score

    def pick_frontier(self, grid, meta):
        """Run WFD + scoring. Returns (goal_rc, all_clusters, robot_rc) or (None,None,None)."""
        if self.tel.north is None:
            return None, None, None

        ex, ey = self._enu_pos()
        robot_r, robot_c = world_to_grid(ex, ey, meta)

        if grid[robot_r][robot_c] == BLOCKED:
            return None, None, None

        frontiers, _ = wavefront_frontier_detection(grid, robot_r, robot_c)
        clusters = cluster_frontiers(frontiers)
        clusters = [c for c in clusters if len(c) >= MIN_CLUSTER]

        # Remove already-visited
        def fresh(cl):
            cr, cc = cluster_centroid(cl)
            ex2, ey2 = grid_to_world(cr, cc, meta)
            return not self._already_visited(ey2, ex2)   # (north, east)

        clusters = [c for c in clusters if fresh(c)]
        if not clusters:
            return None, None, None

        best = max(clusters, key=lambda c: self._score(c, robot_r, robot_c, meta))
        return cluster_centroid(best), clusters, (robot_r, robot_c)

    # ── controller ───────────────────────────────────────────────────────────

    async def _vel(self, vn, ve, yaw):
        try:
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(north_m_s=vn, east_m_s=ve, down_m_s=0.0, yaw_deg=yaw))
        except Exception as e:
            print(f"[CTRL] {e}")

    async def follow_path(self, waypoints_ne):
        """P controller: follow list of (north, east) waypoints."""
        dt = 1.0 / CONTROL_HZ
        for i, (wn, we) in enumerate(waypoints_ne):
            tol = FINAL_DIST if i == len(waypoints_ne) - 1 else ARRIVAL_DIST
            while True:
                if self.tel.north is None:
                    await asyncio.sleep(dt)
                    continue
                cn, ce = self._ned_pos()
                dn, de = wn - cn, we - ce
                dist = math.hypot(dn, de)
                if dist < tol:
                    break
                vn = KP_XY * dn
                ve = KP_XY * de
                spd = math.hypot(vn, ve)
                if spd > MAX_SPEED:
                    vn, ve = vn / spd * MAX_SPEED, ve / spd * MAX_SPEED
                yaw = math.degrees(math.atan2(de, dn))   # bearing from North
                await self._vel(vn, ve, yaw)
                await asyncio.sleep(dt)
        await self._vel(0.0, 0.0, self._yaw())

    async def yaw_sweep(self):
        """360° in-place yaw sweep."""
        print("[EXPLORE] 360° sweep")
        dt = 0.1
        steps = int(SWEEP_TOTAL_DEG / SWEEP_RATE_DPS / dt)
        start = self._yaw()
        for i in range(steps):
            raw = start + SWEEP_RATE_DPS * dt * i
            target = (raw + 180) % 360 - 180   # normalise to -180..180
            await self._vel(0.0, 0.0, target)
            await asyncio.sleep(dt)
        await self._vel(0.0, 0.0, start)
        await asyncio.sleep(0.5)

    # ── main loop ─────────────────────────────────────────────────────────────

    async def run(self):
        print("[EXPLORE] Waiting for map + position...")
        while True:
            grid, meta = self.map.get_map()
            if grid is not None and self.tel.north is not None:
                break
            await asyncio.sleep(0.5)
        print("[EXPLORE] Exploration starting")

        while True:
            grid, meta = self.map.get_map()
            if grid is None:
                await asyncio.sleep(1.0)
                continue

            goal_rc, clusters, robot_rc = self.pick_frontier(grid, meta)

            if goal_rc is None:
                print("[EXPLORE] No frontiers remaining — exploration complete")
                break

            self.map.publish_frontiers(clusters, goal_rc, meta)

            # Inflate grid for A*
            inf_grid = inflate_grid(grid, INFLATION)

            # Snap both robot and goal to nearest free cell in inflated grid
            robot_snapped = nearest_free(inf_grid, *robot_rc)
            if robot_snapped is None:
                print("[EXPLORE] Robot inside inflation zone — waiting for map update")
                await asyncio.sleep(1.0)
                continue

            snapped = nearest_free(inf_grid, *goal_rc)
            if snapped is None:
                print(f"[EXPLORE] Goal {goal_rc} unreachable after inflation — skipping")
                await asyncio.sleep(0.2)
                continue

            path_rc = astar(inf_grid, robot_snapped, snapped)

            if path_rc is None:
                print(f"[EXPLORE] No A* path to {snapped} — skipping")
                await asyncio.sleep(0.2)
                continue

            simplified = simplify_path(path_rc, inf_grid)

            # Convert to NED waypoints
            waypoints_ne = []
            for r, c in simplified:
                ex, ey = grid_to_world(r, c, meta)
                waypoints_ne.append((ey, ex))   # (north, east)

            self.map.publish_path(waypoints_ne)
            print(f"[EXPLORE] → {goal_rc}  ({len(waypoints_ne)} waypoints)")

            await self.follow_path(waypoints_ne)
            await self.yaw_sweep()

            gx, gy = grid_to_world(*goal_rc, meta)
            self.visited.append((gy, gx))
            print(f"[EXPLORE] Frontier done ({len(self.visited)} visited)")
            await asyncio.sleep(0.2)

        print("[EXPLORE] Landing")
        await self.drone.offboard.stop()
        await self.drone.action.land()


# ═══════════════════════════════════════════════════════════════════════════════
# MAVSDK HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

async def connect_drone(drone):
    print(f"[MAVSDK] Connecting to {MAVSDK_ADDRESS}")
    await drone.connect(system_address=MAVSDK_ADDRESS)
    async for health in drone.telemetry.health():
        if health.is_home_position_ok:
            break
    print("[MAVSDK] Connected")


async def arm_and_takeoff(drone):
    print("[MAVSDK] Arming")
    await drone.action.arm()
    print(f"[MAVSDK] Takeoff → {TAKEOFF_ALT_M} m")
    await drone.action.takeoff()
    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m
        print(f"\r[MAVSDK] Alt: {alt:.2f}/{TAKEOFF_ALT_M:.2f} m   ", end='', flush=True)
        if alt >= TAKEOFF_ALT_M - 0.2:
            break
    print()


async def start_offboard(drone):
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    await drone.offboard.start()
    print("[MAVSDK] Offboard active")


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

async def main():
    rclpy.init()
    stop_event = asyncio.Event()

    drone = System()
    await connect_drone(drone)

    telemetry = Telemetry()
    map_node  = MapNode()
    tf_node   = ODOMtoBASELINKTF(Drone=drone, state=telemetry, stop_event=stop_event)

    executor = MultiThreadedExecutor()
    executor.add_node(tf_node)
    executor.add_node(map_node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    await arm_and_takeoff(drone)
    await start_offboard(drone)

    explorer = FrontierExplorer(drone, telemetry, map_node)

    try:
        await asyncio.gather(
            tf_node.position_monitor_task(),
            explorer.run(),
        )
    except asyncio.CancelledError:
        pass
    finally:
        stop_event.set()
        executor.shutdown(wait=False)
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted")

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
import time
import types
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
TAKEOFF_ALT_M   = 3.2         # metres AGL

# Frontier scoring weights (set any weight to 0 to disable that term)
W_DISTANCE = 2.0    
W_DIST_ORDER = 1  # 1/dist       — prefer closer frontiers
W_SIZE     = 0.5    # log(size)    — prefer information-rich clusters
W_HEADING  = 0.3    # cos(Δangle)  — prefer frontiers ahead of drone


# Velocity controller
KP_XY          = 1.0    # proportional gain (m/s per m error)
MAX_SPEED      = 3.5    # m/s horizontal
APPROACH_SPEED = 1.0    # m/s — speed cap when within APPROACH_DIST of final goal
APPROACH_DIST  = 3.0    # m — distance from final goal at which speed is capped
ARRIVAL_DIST   = 1.0    # m — intermediate waypoint reached
FINAL_DIST     = 1.2   # m — frontier centroid reached

# 360° yaw sweep
SWEEP_RATE_DPS  = 120.0    # degrees per second
SWEEP_TOTAL_DEG = 360.0
SWEEP_POST_WAIT = 1.0     # seconds to wait after sweep for OctoMap to catch up

# A* / planning
INFLATION      = 2    # grid cells to inflate around obstacles
MIN_CLUSTER    = 3    # ignore frontier clusters smaller than this
VISITED_RADIUS = 0.75  # m — skip re-visiting frontiers within this radius

CONTROL_HZ = 10       # velocity setpoint rate

REPLAN_INTERVAL_S   = 0.2   # seconds between frontier recheck during flight
REPLAN_MIN_SAVING_M = 3.0   # abort path if a new frontier is this much closer (metres)
REPLAN_WALL_COST_THR = 3.5  # replan if a remaining waypoint is this close to a wall (wall cost units, max possible = 3.0)

MIN_FRONTIER_DIST_M = 1.5   # ignore frontiers closer than this (depth camera blind spot)
SWEEP_MAP_FRAMES    = 3     # wait for this many new map frames after sweep before moving on
SWEEP_KNOWN_RADIUS_M = 4.0  # skip yaw sweep if no UNKNOWN cells within this radius (m)




# ═══════════════════════════════════════════════════════════════════════════════
# GRID UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════

def pad_grid(grid, meta, padding=1):
    """Wrap the grid in UNKNOWN cells and shift the metadata origin to match."""
    rows, cols = len(grid), len(grid[0])
    new_rows, new_cols = rows + 2 * padding, cols + 2 * padding
    padded = [[UNKNOWN] * new_cols for _ in range(new_rows)]
    for r in range(rows):
        for c in range(cols):
            padded[r + padding][c + padding] = grid[r][c]
    res = meta.resolution
    new_meta = types.SimpleNamespace(
        resolution=res,
        height=new_rows,
        width=new_cols,
        origin=types.SimpleNamespace(
            position=types.SimpleNamespace(
                x=meta.origin.position.x - padding * res,
                y=meta.origin.position.y - padding * res,
            )
        ),
    )
    return padded, new_meta


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


WALL_COSTS = [3.0, 3.0, 1.5, 0.5]   # cost penalty at distance 1, 2, 3, 4 cells from a wall

def surroundings_known(grid, robot_rc, radius_m, resolution):
    """Return True if every UNKNOWN cell within radius_m is occluded by a wall.
    UNKNOWN cells that are line-of-sight visible from robot_rc still count as unknown."""
    r0, c0 = robot_rc
    r_cells = int(math.ceil(radius_m / resolution))
    rows, cols = len(grid), len(grid[0])
    r_sq = r_cells * r_cells
    for dr in range(-r_cells, r_cells + 1):
        for dc in range(-r_cells, r_cells + 1):
            if dr * dr + dc * dc > r_sq:
                continue
            nr, nc = r0 + dr, c0 + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if grid[nr][nc] == UNKNOWN and _line_clear(grid, robot_rc, (nr, nc)):
                return False
    return True


def wall_cost_grid(grid):
    """BFS from all BLOCKED cells outward, assigning proximity penalties."""
    rows, cols = len(grid), len(grid[0])
    costs = [[0.0] * cols for _ in range(rows)]
    from collections import deque
    queue = deque()
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == BLOCKED:
                queue.append((r, c, 0))
    visited = [[False] * cols for _ in range(rows)]
    DIRS4 = [(-1,0),(1,0),(0,-1),(0,1)]
    while queue:
        r, c, d = queue.popleft()
        if visited[r][c]:
            continue
        visited[r][c] = True
        if 0 < d <= len(WALL_COSTS):
            costs[r][c] = WALL_COSTS[d - 1]
        if d < len(WALL_COSTS):
            for dr, dc in DIRS4:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                    queue.append((nr, nc, d + 1))
    return costs


# ═══════════════════════════════════════════════════════════════════════════════
# A* PLANNER
# ═══════════════════════════════════════════════════════════════════════════════

def astar(grid, start, goal, wcosts=None):
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
            # Prevent diagonal corner cutting — both cardinal neighbours must be free
            if dr != 0 and dc != 0:
                if not passable(cur[0] + dr, cur[1]) or not passable(cur[0], cur[1] + dc):
                    continue
            wall_pen = wcosts[nb[0]][nb[1]] if wcosts else 0.0
            ng = g_cur + cost + (8.0 if grid[nb[0]][nb[1]] == UNKNOWN else 0.0) + wall_pen
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


def densify_waypoints(waypoints_ne, step_m=0.5):
    """Insert intermediate points every step_m metres along each segment."""
    if len(waypoints_ne) < 2:
        return waypoints_ne
    result = [waypoints_ne[0]]
    for (n0, e0), (n1, e1) in zip(waypoints_ne, waypoints_ne[1:]):
        dist = math.hypot(n1 - n0, e1 - e0)
        n_steps = max(1, int(dist / step_m))
        for k in range(1, n_steps + 1):
            t = k / n_steps
            result.append((n0 + t * (n1 - n0), e0 + t * (e1 - e0)))
    return result


def smooth_waypoints_ned(waypoints_ne, grid, meta, iterations=15, weight=0.6):
    """Gradient smoother on densified NED waypoints with LOS safety checks."""
    if len(waypoints_ne) <= 2:
        return waypoints_ne
    rows, cols = len(grid), len(grid[0])
    p = [list(wp) for wp in waypoints_ne]
    for _ in range(iterations):
        for i in range(1, len(p) - 1):
            cn = p[i][0] * (1.0 - weight) + (p[i-1][0] + p[i+1][0]) / 2.0 * weight
            ce = p[i][1] * (1.0 - weight) + (p[i-1][1] + p[i+1][1]) / 2.0 * weight
            pr, pc = world_to_grid(ce, cn, meta)
            pr = max(0, min(rows - 1, pr))
            pc = max(0, min(cols - 1, pc))
            if grid[pr][pc] == BLOCKED:
                continue
            r_prev, c_prev = world_to_grid(p[i-1][1], p[i-1][0], meta)
            r_next, c_next = world_to_grid(p[i+1][1], p[i+1][0], meta)
            if _line_clear(grid, (r_prev, c_prev), (pr, pc)) and \
               _line_clear(grid, (pr, pc), (r_next, c_next)):
                p[i] = [cn, ce]
    return [tuple(wp) for wp in p]






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
        self._update_count = 0

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
        grid, meta = pad_grid(grid, msg.info)
        with self._lock:
            self._grid = grid
            self._meta = meta
            self._update_count += 1

    def get_map(self):
        with self._lock:
            return self._grid, self._meta

    def get_update_count(self):
        with self._lock:
            return self._update_count

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
        self.visited = []        # (north, east) of explored frontiers
        self._goal_fails = {}    # grid_rc -> consecutive failure count (A* + follow_path)
        self._skip = set()       # temporarily unreachable frontiers (cleared on success)
        self._wall_replan_cooldown = 0.0  # persists across follow_path calls

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
        dist_score = W_DISTANCE / (dist_cells + 1.0) ** W_DIST_ORDER

        size_score = W_SIZE * math.log1p(len(cluster))

        dn = frontier_n - (self.tel.north or 0.0)
        de = frontier_e - (self.tel.east  or 0.0)
        angle_to = math.degrees(math.atan2(de, dn))
        diff = (angle_to - self._yaw() + 180) % 360 - 180
        heading_score = W_HEADING * (1.0 + math.cos(math.radians(diff))) / 2.0

        return dist_score + size_score + heading_score

    def pick_frontier(self, grid, meta, skip=None):
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

        # Remove already-visited and too-close frontiers (depth camera blind spot)
        def fresh(cl):
            cr, cc = cluster_centroid(cl)
            ex2, ey2 = grid_to_world(cr, cc, meta)
            if self._already_visited(ey2, ex2):
                return False
            dist_m = math.hypot(cr - robot_r, cc - robot_c) * meta.resolution
            return dist_m >= MIN_FRONTIER_DIST_M

        clusters = [c for c in clusters if fresh(c)]
        if skip:
            clusters = [c for c in clusters if cluster_centroid(c) not in skip]
        if not clusters:
            return None, None, None

        best = max(clusters, key=lambda c: self._score(c, robot_r, robot_c, meta))
        return cluster_centroid(best), clusters, (robot_r, robot_c)

    # ── controller ───────────────────────────────────────────────────────────

    async def push_away_from_obstacles(self, inf_grid, meta):
        """Move 1 cell away from nearby obstacles before replanning."""
        cn, ce = self._ned_pos()
        ex, ey = self._enu_pos()
        rr, rc = world_to_grid(ex, ey, meta)
        rows, cols = len(inf_grid), len(inf_grid[0])
        rep_n, rep_e = 0.0, 0.0
        for dr in range(-1, 2):
            for dc in range(-1, 2):
                nr, nc = rr + dr, rc + dc
                if 0 <= nr < rows and 0 <= nc < cols and inf_grid[nr][nc] == BLOCKED:
                    bx, by = grid_to_world(nr, nc, meta)
                    dn, de = cn - by, ce - bx
                    dist = math.hypot(dn, de) + 1e-6
                    rep_n += dn / dist
                    rep_e += de / dist
        mag = math.hypot(rep_n, rep_e)
        if mag < 1e-6:
            return
        rep_n, rep_e = rep_n / mag, rep_e / mag
        spd = 0.5
        duration = meta.resolution / spd   # time to move 1 cell
        steps = max(1, int(duration / 0.1))
        yaw = self._yaw()
        print("[REPLAN] Pushing away from obstacle before replanning")
        for _ in range(steps):
            await self._vel(rep_n * spd, rep_e * spd, yaw)
            await asyncio.sleep(0.1)
        await self._vel(0.0, 0.0, yaw)
        await asyncio.sleep(0.3)

    async def _vel(self, vn, ve, yaw):
        try:
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(north_m_s=vn, east_m_s=ve, down_m_s=0.0, yaw_deg=yaw))
        except Exception as e:
            print(f"[CTRL] {e}")

    async def follow_path(self, waypoints_ne, goal_ne):
        """P controller: follow list of (north, east) waypoints.
        Returns 'done', 'blocked' (wall/obstacle replan), or 'closer' (closer frontier found)."""
        dt = 1.0 / CONTROL_HZ
        last_replan_check = 0.0
        now = 0.0
        goal_n, goal_e = goal_ne
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
                goal_dist = math.hypot(goal_n - cn, goal_e - ce)
                speed_limit = APPROACH_SPEED if goal_dist < APPROACH_DIST else MAX_SPEED
                yaw = math.degrees(math.atan2(de, dn))

                if spd > speed_limit:
                    vn, ve = vn / spd * speed_limit, ve / spd * speed_limit
                await self._vel(vn, ve, yaw)
                await asyncio.sleep(dt)

                now = time.monotonic()
                if now - last_replan_check >= REPLAN_INTERVAL_S:
                    last_replan_check = now
                    grid, meta = self.map.get_map()
                    if grid is not None:
                        inf_grid = inflate_grid(grid, INFLATION)
                        drone_rc = world_to_grid(ce, cn, meta)
                        dr, dc = drone_rc

                        rows_g, cols_g = len(inf_grid), len(inf_grid[0])
                        wcosts_live = wall_cost_grid(inf_grid)
                        remaining = waypoints_ne[i:]
                        replan_reason = None

                        for wn2, we2 in remaining:
                            wp_dist = math.hypot(wn2 - cn, we2 - ce)
                            pr, pc = world_to_grid(we2, wn2, meta)
                            pr = max(0, min(rows_g - 1, pr))
                            pc = max(0, min(cols_g - 1, pc))
                            # Skip checking the drone's own cell — may be transiently BLOCKED
                            if (pr, pc) == (dr, dc):
                                continue
                            if inf_grid[pr][pc] == BLOCKED and wp_dist < 5.0:
                                replan_reason = "Obstacle appeared on path"
                                break
                            if now > self._wall_replan_cooldown and wcosts_live[pr][pc] >= REPLAN_WALL_COST_THR:
                                replan_reason = "Path too close to wall"
                                break

                        if replan_reason:
                            print(f"[REPLAN] {replan_reason} — stopping and replanning")
                            await self._vel(0.0, 0.0, self._yaw())
                            stop_duration = 0.8 if replan_reason == "Obstacle appeared on path" else 0.5
                            await asyncio.sleep(stop_duration)
                            if replan_reason == "Path too close to wall":
                                await self.push_away_from_obstacles(inf_grid, meta)
                                self._wall_replan_cooldown = time.monotonic() + 5.0
                            return "blocked"

                        # Check if a significantly closer frontier has appeared
                        best_rc, _, _ = self.pick_frontier(grid, meta, skip=self._skip)
                        if best_rc is not None:
                            bx, by = grid_to_world(*best_rc, meta)
                            best_dist = math.hypot(by - cn, bx - ce)
                            goal_dist = math.hypot(goal_n - cn, goal_e - ce)
                            if goal_dist - best_dist > REPLAN_MIN_SAVING_M:
                                print(f"[REPLAN] Closer frontier found ({best_dist:.1f}m vs {goal_dist:.1f}m) — stopping and replanning")
                                await self._vel(0.0, 0.0, self._yaw())
                                await asyncio.sleep(0.5)
                                return "closer"
        await self._vel(0.0, 0.0, self._yaw())
        print("[FOLLOW] Path complete — arrived at frontier")
        return "done"

    async def yaw_sweep(self, rotations=1):
        """In-place yaw sweep. rotations=2 for double spin on takeoff."""
        total_deg = SWEEP_TOTAL_DEG * rotations
        print(f"[EXPLORE] {total_deg:.0f}° sweep")
        dt = 0.1
        steps = int(total_deg / SWEEP_RATE_DPS / dt)
        start = self._yaw()
        for i in range(steps):
            raw = start + SWEEP_RATE_DPS * dt * i
            target = (raw + 180) % 360 - 180
            await self._vel(0.0, 0.0, target)
            await asyncio.sleep(dt)
        await self._vel(0.0, 0.0, start)
        # Wait for OctoMap to process SWEEP_MAP_FRAMES new frames (adaptive to map size)
        count_before = self.map.get_update_count()
        deadline = time.monotonic() + SWEEP_POST_WAIT + 5.0   # max wait
        while self.map.get_update_count() - count_before < SWEEP_MAP_FRAMES:
            if time.monotonic() > deadline:
                break
            await asyncio.sleep(0.1)

    # ── main loop ─────────────────────────────────────────────────────────────

    async def run(self):
        print("[INIT] taking off")
        await self.drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
        await arm_and_takeoff(self.drone)
        await start_offboard(self.drone)
        print("[EXPLORE] Waiting for map + position...")
        while True:
            grid, meta = self.map.get_map()
            if grid is not None and self.tel.north is not None:
                break
            await asyncio.sleep(0.5)

        # Health check — confirm OctoMap pipeline is streaming before starting
        MAP_HEALTH_FRAMES = 10   # consecutive map updates required
        MAP_HEALTH_TIMEOUT = 30.0  # seconds before warning and proceeding anyway
        print("[EXPLORE] Checking OctoMap pipeline health...")
        count_start = self.map.get_update_count()
        deadline = time.monotonic() + MAP_HEALTH_TIMEOUT
        while True:
            frames = self.map.get_update_count() - count_start
            if frames >= MAP_HEALTH_FRAMES:
                print(f"[EXPLORE] OctoMap healthy — {frames} frames received, starting")
                break
            if time.monotonic() > deadline:
                print(f"[EXPLORE] WARNING: only {frames}/{MAP_HEALTH_FRAMES} frames in "
                      f"{MAP_HEALTH_TIMEOUT:.0f}s — TF may be misaligned, map quality uncertain")
                break
            await asyncio.sleep(0.5)
        print("[EXPLORE] Exploration starting")
        await self.yaw_sweep(rotations=2)

        explore_start = time.monotonic()
        last_heartbeat = explore_start

        while True:
            now = time.monotonic()
            if now - last_heartbeat >= 60.0:
                elapsed = now - explore_start
                m, s = divmod(int(elapsed), 60)
                print(f"[EXPLORE] Still exploring — {m}m {s:02d}s elapsed")
                last_heartbeat = now
            grid, meta = self.map.get_map()
            if grid is None:
                await asyncio.sleep(1.0)
                continue

            goal_rc, clusters, robot_rc = self.pick_frontier(grid, meta, skip=self._skip)

            if goal_rc is None:
                print("[EXPLORE] No frontiers remaining — exploration complete")
                break

            self.map.publish_frontiers(clusters, goal_rc, meta)

            # Inflate grid for A* and compute wall proximity costs
            inf_grid = inflate_grid(grid, INFLATION)
            wcosts = wall_cost_grid(inf_grid)

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

            # Pull goal 2 cells back toward robot to avoid navigating into unregistered walls
            gr, gc = snapped
            rr, rc2 = robot_snapped
            dr, dc = rr - gr, rc2 - gc
            length = math.hypot(dr, dc)
            if length > 2:
                rows_g, cols_g = len(inf_grid), len(inf_grid[0])
                pr = max(0, min(rows_g - 1, int(round(gr + 2 * dr / length))))
                pc = max(0, min(cols_g - 1, int(round(gc + 2 * dc / length))))
                if inf_grid[pr][pc] != BLOCKED:
                    snapped = (pr, pc)

            path_rc = astar(inf_grid, robot_snapped, snapped, wcosts=wcosts)

            if path_rc is None:
                fails = self._goal_fails.get(goal_rc, 0) + 1
                self._goal_fails[goal_rc] = fails
                print(f"[EXPLORE] No A* path to {goal_rc} — attempt {fails}, trying next frontier")
                if fails >= 4:
                    self._skip.add(goal_rc)
                    self._goal_fails.pop(goal_rc, None)
                    print(f"[EXPLORE] {goal_rc} temporarily skipped — will retry after a success")
                await asyncio.sleep(0.2)
                continue

            # Successful path — reset failure count for this goal only
            self._goal_fails.pop(goal_rc, None)

            # Greedy LOS shortcut → densify → gradient smooth in NED space
            path_rc = simplify_path(path_rc, inf_grid)
            waypoints_ne = [(grid_to_world(r, c, meta)[1], grid_to_world(r, c, meta)[0])
                            for r, c in path_rc]
            waypoints_ne = densify_waypoints(waypoints_ne, step_m=1.0)
            waypoints_ne = smooth_waypoints_ned(waypoints_ne, inf_grid, meta)

            self.map.publish_path(waypoints_ne)
            print(f"[EXPLORE] → {goal_rc}  ({len(waypoints_ne)} waypoints)")

            gx, gy = grid_to_world(*goal_rc, meta)
            goal_ne = (gy, gx)   # (north, east)

            result = await self.follow_path(waypoints_ne, goal_ne)
            if result == "closer":
                # Deprioritize the abandoned goal so we don't immediately snap back to it
                self._skip.add(goal_rc)
                print(f"[EXPLORE] {goal_rc} deprioritised — exploring closer frontier first")
                continue
            if result == "blocked":
                fails = self._goal_fails.get(goal_rc, 0) + 1
                self._goal_fails[goal_rc] = fails
                if fails >= 2:
                    self._skip.add(goal_rc)
                    self._goal_fails.pop(goal_rc, None)
                    print(f"[EXPLORE] {goal_rc} skipped after {fails} blocked attempts — trying different frontier")
                else:
                    print(f"[EXPLORE] {goal_rc} blocked (attempt {fails}/2) — retrying after settle")
                    await asyncio.sleep(1.0)  # let map update + drone settle before retry
                continue

            # Arrived — clear skipped frontiers so unreachable ones can be retried
            if self._skip:
                self._skip.clear()
                print("[EXPLORE] Frontier arrived — resetting skipped frontiers")
            if surroundings_known(grid, robot_rc, SWEEP_KNOWN_RADIUS_M, meta.resolution):
                print(f"[EXPLORE] Surroundings already mapped (r={SWEEP_KNOWN_RADIUS_M}m) — skipping sweep")
            else:
                await self.yaw_sweep()
            self.visited.append((gy, gx))
            print(f"[EXPLORE] Frontier done ({len(self.visited)} visited)")
            await asyncio.sleep(0.2)

        elapsed = time.monotonic() - explore_start
        m, s = divmod(int(elapsed), 60)
        print(f"[EXPLORE] Exploration complete — total time {m}m {s:02d}s")
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


    # await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
    # await arm_and_takeoff(drone)
    # await start_offboard(drone)  

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
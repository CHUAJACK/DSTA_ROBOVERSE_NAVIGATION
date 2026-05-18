from collections import deque


FREE = 0
BLOCKED = 100
UNKNOWN = -1


def get_neighbors4(grid, r, c):
    rows, cols = len(grid), len(grid[0])
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols:
            yield nr, nc


def is_frontier(grid, r, c):
    """A frontier cell is a FREE cell adjacent to at least one UNKNOWN cell."""
    if grid[r][c] != FREE:
        return False
    return any(grid[nr][nc] == UNKNOWN for nr, nc in get_neighbors4(grid, r, c))


def wavefront_frontier_detection(grid, robot_r, robot_c):
    """
    Wavefront Frontier Detector (WFD).

    Performs a BFS from the robot position through FREE cells.
    Any FREE cell adjacent to an UNKNOWN cell is marked as a frontier.

    Args:
        grid:    2D list of ints. FREE=0, BLOCKED=100, UNKNOWN=-1
        robot_r: Robot row index
        robot_c: Robot column index

    Returns:
        frontiers: list of (r, c) tuples that are frontier cells
        visited:   set of (r, c) tuples the BFS explored
    """
    if grid[robot_r][robot_c] == BLOCKED:
        raise ValueError("Robot cannot be placed on a blocked cell.")

    visited = set()
    queue = deque()
    queue.append((robot_r, robot_c))
    visited.add((robot_r, robot_c))

    frontiers = []

    while queue:
        r, c = queue.popleft()

        if is_frontier(grid, r, c):
            frontiers.append((r, c))

        for nr, nc in get_neighbors4(grid, r, c):
            if (nr, nc) not in visited and grid[nr][nc] == FREE:
                visited.add((nr, nc))
                queue.append((nr, nc))

    return frontiers, visited


def cluster_frontiers(frontiers):
    """
    Groups adjacent frontier cells into contiguous regions using BFS.

    Args:
        frontiers: list of (r, c) frontier cells

    Returns:
        list of clusters, where each cluster is a list of (r, c) cells
    """
    frontier_set = set(frontiers)
    visited = set()
    clusters = []

    for start in frontiers:
        if start in visited:
            continue
        cluster = []
        queue = deque([start])
        visited.add(start)
        while queue:
            cell = queue.popleft()
            cluster.append(cell)
            r, c = cell
            for nr, nc in [
                (r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1),
                (r - 1, c - 1), (r - 1, c + 1), (r + 1, c - 1), (r + 1, c + 1),
            ]:
                neighbor = (nr, nc)
                if neighbor in frontier_set and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        clusters.append(cluster)

    return clusters


def centroid(cluster):
    """Returns the (row, col) centroid of a cluster, rounded to nearest cell."""
    r = round(sum(c[0] for c in cluster) / len(cluster))
    c = round(sum(c[1] for c in cluster) / len(cluster))
    return r, c


def select_frontier(clusters, robot_r, robot_c):
    """
    Selects the closest frontier cluster by Euclidean distance to its centroid.

    Args:
        clusters:  list of frontier clusters
        robot_r:   robot row
        robot_c:   robot column

    Returns:
        best_cluster: the chosen cluster
        goal:         (r, c) centroid of the chosen cluster
    """
    if not clusters:
        return None, None

    def dist(cluster):
        cr, cc = centroid(cluster)
        return ((cr - robot_r) ** 2 + (cc - robot_c) ** 2) ** 0.5

    best = min(clusters, key=dist)
    return best, centroid(best)


def print_map(grid, robot_r, robot_c, frontiers=None, visited=None):
    """Prints a coloured ASCII map to the terminal."""
    frontier_set = set(frontiers) if frontiers else set()
    visited_set = set(visited) if visited else set()

    RESET  = "\033[0m"
    GREEN  = "\033[92m"
    BLUE   = "\033[94m"
    RED    = "\033[91m"
    GRAY   = "\033[90m"
    YELLOW = "\033[93m"

    for r, row in enumerate(grid):
        line = ""
        for c, val in enumerate(row):
            if r == robot_r and c == robot_c:
                line += f"{RED}R {RESET}"
            elif (r, c) in frontier_set:
                line += f"{GREEN}F {RESET}"
            elif (r, c) in visited_set:
                line += f"{BLUE}. {RESET}"
            elif val == BLOCKED:
                line += f"{GRAY}# {RESET}"
            elif val == UNKNOWN:
                line += f"{YELLOW}? {RESET}"
            else:
                line += "  "
        print(line)


# ---------------------------------------------------------------------------
# ROS 2 live test — subscribes to /octomap_2d_slice + tf2 for drone pose,
# runs WFD on every map update, and plots the result with matplotlib.
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import math
    import threading

    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid
    import tf2_ros

    import matplotlib
    matplotlib.use("TkAgg")          # change to "Qt5Agg" if TkAgg is unavailable
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    import numpy as np

    # ------------------------------------------------------------------
    # Colour map: unknown=grey(128,128,128), free=white, blocked=black
    # ------------------------------------------------------------------
    CMAP = mcolors.ListedColormap(["#808080", "#ffffff", "#000000"])
    BOUNDS = [-1.5, -0.5, 50.0, 100.5]   # boundaries between the 3 colours
    NORM   = mcolors.BoundaryNorm(BOUNDS, CMAP.N)

    class FrontierTestNode(Node):
        def __init__(self):
            super().__init__("frontier_detector_test")

            self._lock      = threading.Lock()
            self._grid      = None     # 2-D list[list[int]]
            self._map_meta  = None     # OccupancyGrid metadata (origin, resolution)
            self._robot_pos = None     # (world_x, world_y) in metres

            # tf2 listener — drone pose
            self._tf_buffer   = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

            # Occupancy-grid subscriber
            self._map_sub = self.create_subscription(
                OccupancyGrid,
                "/octomap_2d_slice",
                self._map_callback,
                10,
            )

            # Re-run WFD + redraw every second
            self._timer = self.create_timer(1.0, self._update)

            # Matplotlib figure (non-blocking)
            plt.ion()
            self._fig, self._ax = plt.subplots(figsize=(9, 7))
            self._fig.canvas.manager.set_window_title("Wavefront Frontier Detector")
            self._img_handle      = None
            self._robot_scatter   = None
            self._frontier_scatter = None
            self._goal_scatter    = None

            self.get_logger().info("FrontierTestNode ready — waiting for /octomap_2d_slice …")

        # ------------------------------------------------------------------
        def _map_callback(self, msg: OccupancyGrid):
            """Convert ROS OccupancyGrid to a 2-D list with FREE/BLOCKED/UNKNOWN."""
            rows = msg.info.height
            cols = msg.info.width
            raw  = msg.data                   # row-major, -1/0..100

            grid = []
            for r in range(rows):
                row = []
                for c in range(cols):
                    v = raw[r * cols + c]
                    if v == -1:
                        row.append(UNKNOWN)
                    elif v >= 50:             # treat ≥50 occupancy as blocked
                        row.append(BLOCKED)
                    else:
                        row.append(FREE)
                grid.append(row)

            with self._lock:
                self._grid     = grid
                self._map_meta = msg.info

        # ------------------------------------------------------------------
        def _get_drone_world_pos(self):
            """
            Look up the drone's position from tf2.
            Returns (world_x, world_y) or None on failure.
            Adjust the frame names to match your setup.
            """
            try:
                tf = self._tf_buffer.lookup_transform(
                    "map",          # target frame
                    "base_link",    # source frame (drone body)
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                tx = tf.transform.translation.x
                ty = tf.transform.translation.y
                return tx, ty
            except Exception as e:
                self.get_logger().warn(f"tf2 lookup failed: {e}", throttle_duration_sec=5.0)
                return None

        # ------------------------------------------------------------------
        def _world_to_grid(self, world_x, world_y, meta):
            """Convert world (x, y) metres to (row, col) grid indices."""
            ox = meta.origin.position.x
            oy = meta.origin.position.y
            res = meta.resolution
            col = int((world_x - ox) / res)
            row = int((world_y - oy) / res)
            rows = meta.height
            cols  = meta.width
            row = max(0, min(rows - 1, row))
            col = max(0, min(cols  - 1, col))
            return row, col

        # ------------------------------------------------------------------
        def _grid_to_world(self, grid_r, grid_c, meta):
            """Convert (row, col) grid indices to world (x, y) metres (cell centre)."""
            ox  = meta.origin.position.x
            oy  = meta.origin.position.y
            res = meta.resolution
            x = ox + (grid_c + 0.5) * res
            y = oy + (grid_r + 0.5) * res
            return x, y

        # ------------------------------------------------------------------
        def _update(self):
            """Called every second — run WFD and refresh the plot."""
            with self._lock:
                grid = self._grid
                meta = self._map_meta

            if grid is None:
                return

            world_pos = self._get_drone_world_pos()
            if world_pos is None:
                self.get_logger().warn("No drone pose — skipping WFD this cycle.")
                return

            world_x, world_y = world_pos
            robot_r, robot_c = self._world_to_grid(world_x, world_y, meta)

            # Guard: robot cell must be free (may land on UNKNOWN at edges)
            if grid[robot_r][robot_c] == BLOCKED:
                self.get_logger().warn("Robot cell is BLOCKED in the map — skipping.")
                return

            # Run WFD
            frontiers, _ = wavefront_frontier_detection(grid, robot_r, robot_c)
            clusters      = cluster_frontiers(frontiers)
            _, goal_rc    = select_frontier(clusters, robot_r, robot_c)

            self.get_logger().info(
                f"WFD: {len(frontiers)} frontier cells, "
                f"{len(clusters)} clusters, "
                f"goal={goal_rc}"
            )

            self._draw(grid, meta, robot_r, robot_c, frontiers, goal_rc)

        # ------------------------------------------------------------------
        def _draw(self, grid, meta, robot_r, robot_c, frontiers, goal_rc):
            """Render the occupancy grid with WFD overlays using matplotlib."""
            rows = len(grid)
            cols = len(grid[0]) if rows else 0

            # Build a numpy array for imshow (row 0 = bottom in world coords)
            arr = np.array(grid, dtype=np.float32)

            ax = self._ax
            ax.cla()

            # Extent maps pixel edges to world coordinates
            ox  = meta.origin.position.x
            oy  = meta.origin.position.y
            res = meta.resolution
            extent = [ox, ox + cols * res, oy, oy + rows * res]

            ax.imshow(
                arr,
                cmap=CMAP, norm=NORM,
                origin="lower",
                extent=extent,
                interpolation="nearest",
            )

            # --- frontier cells (all) as small green dots ---
            if frontiers:
                fx = [self._grid_to_world(r, c, meta)[0] for r, c in frontiers]
                fy = [self._grid_to_world(r, c, meta)[1] for r, c in frontiers]
                ax.scatter(fx, fy, s=6, c="#22c55e", zorder=3, label="Frontier cells")

            # --- selected goal as a larger green star ---
            if goal_rc is not None:
                gx, gy = self._grid_to_world(goal_rc[0], goal_rc[1], meta)
                ax.scatter(gx, gy, s=180, c="#16a34a", marker="*",
                           zorder=5, label=f"Goal {goal_rc}")

            # --- robot position as a red circle ---
            rx, ry = self._grid_to_world(robot_r, robot_c, meta)
            ax.scatter(rx, ry, s=120, c="#ef4444", marker="o",
                       zorder=6, label="Drone")

            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_title("Wavefront Frontier Detector — /octomap_2d_slice")
            ax.legend(loc="upper right", fontsize=8)

            self._fig.canvas.draw()
            self._fig.canvas.flush_events()

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------
    rclpy.init()
    node = FrontierTestNode()

    # Spin in a background thread so matplotlib's event loop runs on main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        plt.show(block=True)    # blocks until the window is closed
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
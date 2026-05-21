#!/usr/bin/env python3

import asyncio
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from scipy.ndimage import (
    distance_transform_edt,
    binary_dilation,
    generate_binary_structure,
    label,
)


class OctomapToNumpyGrid(Node):
    """
    Converts OctoMap marker outputs into live 3D numpy grids.

    Assumed ROS / RViz / OctoMap map frame:
        x = East
        y = North
        z = Up

    Numpy grid indexing:
        grid[ix, iy, iz]

    Meaning:
        ix = East index
        iy = North index
        iz = Up index

    State grid:
        -1 = unknown
         0 = known free
         1 = occupied

    Cost grid for pathfinding3D:
        0.0 = blocked
        1.0 = normal free space
        >1.0 = higher traversal cost near obstacles
    """

    def __init__(
        self,
        occupied_topic="/occupied_cells_vis_array",
        free_topic="/free_cells_vis_array",
        resolution=0.5,
        size_x=40.0,
        size_y=40.0,
        size_z=8.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_z=0.0,
        inflation_radius=1.0,
        soft_radius=2.0,
        unknown_cost=5.0,
        roof_height=8.0,
    ):
        super().__init__("octomap_to_numpy_grid")

        # Use Gazebo / ROS simulation time by default
        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])

        self.occupied_topic = occupied_topic
        self.free_topic = free_topic

        self.resolution = float(resolution)

        self.size_x = float(size_x)
        self.size_y = float(size_y)
        self.size_z = float(size_z)

        # Minimum corner of planning volume in map frame
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.origin_z = float(origin_z)

        self.nx = int(np.ceil(self.size_x / self.resolution))
        self.ny = int(np.ceil(self.size_y / self.resolution))
        self.nz = int(np.ceil(self.size_z / self.resolution))

        self.inflation_radius = float(inflation_radius)
        self.soft_radius = float(soft_radius)
        self.unknown_cost = float(unknown_cost)
        self.roof_height = float(roof_height)

        # Raw layers
        self.occupied_grid = np.zeros(
            (self.nx, self.ny, self.nz),
            dtype=bool
        )

        self.known_free_grid = np.zeros(
            (self.nx, self.ny, self.nz),
            dtype=bool
        )

        # -1 unknown, 0 free, 1 occupied
        self.state_grid = np.full(
            (self.nx, self.ny, self.nz),
            -1,
            dtype=np.int8
        )

        # Inflation and planning layers
        self.inflated_grid = np.zeros_like(self.occupied_grid, dtype=bool)

        self.cost_grid = np.ones(
            (self.nx, self.ny, self.nz),
            dtype=np.float32
        )

        self.distance_grid_m = np.full(
            (self.nx, self.ny, self.nz),
            np.inf,
            dtype=np.float32
        )

        # Frontier layer
        self.frontier_grid = np.zeros_like(self.occupied_grid, dtype=bool)

        self.has_occupied = False
        self.has_free = False
        self.has_map = False

        # Subscribers
        self.occupied_sub = self.create_subscription(
            MarkerArray,
            occupied_topic,
            self.occupied_callback,
            10
        )

        self.free_sub = self.create_subscription(
            MarkerArray,
            free_topic,
            self.free_callback,
            10
        )

        # RViz publishers
        self.frontier_pub = self.create_publisher(
            MarkerArray,
            "/frontier_cells_vis_array",
            10
        )

        self.frontier_cluster_pub = self.create_publisher(
            MarkerArray,
            "/frontier_clusters_vis_array",
            10
        )

        self.get_logger().info(f"Listening to occupied topic: {occupied_topic}")
        self.get_logger().info(f"Listening to free topic: {free_topic}")
        self.get_logger().info(
            f"Grid size: {self.nx} x {self.ny} x {self.nz}, "
            f"resolution={self.resolution:.2f} m"
        )
        self.get_logger().info(
            f"Inflation radius={self.inflation_radius:.2f} m, "
            f"soft radius={self.soft_radius:.2f} m"
        )

    # =========================================================
    # Coordinate conversion
    # =========================================================

    def world_to_grid(self, x, y, z):
        """
        Convert map-frame coordinates to numpy grid indices.

        map frame:
            x = East
            y = North
            z = Up
        """

        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        iz = int((z - self.origin_z) / self.resolution)

        return ix, iy, iz

    def grid_to_world(self, ix, iy, iz):
        """
        Convert numpy grid index to map-frame coordinates at cell centre.

        map frame:
            x = East
            y = North
            z = Up
        """

        x = self.origin_x + (float(ix) + 0.5) * self.resolution
        y = self.origin_y + (float(iy) + 0.5) * self.resolution
        z = self.origin_z + (float(iz) + 0.5) * self.resolution

        return x, y, z

    def ned_position_to_grid(self, north, east, down):
        """
        Convert MAVSDK NED to numpy grid index.

        MAVSDK:
            north, east, down

        map frame:
            x = east
            y = north
            z = up = -down
        """

        x = east
        y = north
        z = -down

        return self.world_to_grid(x, y, z)

    def grid_to_ned_position(self, ix, iy, iz):
        """
        Convert numpy grid index back to MAVSDK NED position.
        """

        x, y, z = self.grid_to_world(ix, iy, iz)

        east = x
        north = y
        down = -z

        return north, east, down

    def is_in_bounds(self, ix, iy, iz):
        return (
            0 <= ix < self.nx and
            0 <= iy < self.ny and
            0 <= iz < self.nz
        )

    # =========================================================
    # Marker conversion
    # =========================================================

    def marker_array_to_grid(self, msg):
        """
        Convert visualization_msgs/MarkerArray points into a boolean grid.
        """

        grid = np.zeros((self.nx, self.ny, self.nz), dtype=bool)
        count = 0

        for marker in msg.markers:
            for p in marker.points:
                ix, iy, iz = self.world_to_grid(p.x, p.y, p.z)

                if self.is_in_bounds(ix, iy, iz):
                    grid[ix, iy, iz] = True
                    count += 1

        return grid, count

    # =========================================================
    # ROS callbacks
    # =========================================================

    def occupied_callback(self, msg: MarkerArray):
        """
        Updates occupied cells from OctoMap.
        """

        new_occupied, count = self.marker_array_to_grid(msg)

        self.occupied_grid = new_occupied
        self.has_occupied = True

        self.rebuild_state_grid()
        self.update_cost_grid()
        self.update_frontiers()

        self.has_map = self.has_occupied and self.has_free

        self.get_logger().info(
            f"Updated occupied grid. Occupied voxels={count}",
            throttle_duration_sec=1.0
        )

    def free_callback(self, msg: MarkerArray):
        """
        Updates known free cells from OctoMap.
        """

        new_free, count = self.marker_array_to_grid(msg)

        self.known_free_grid = new_free
        self.has_free = True

        self.rebuild_state_grid()
        self.update_cost_grid()
        self.update_frontiers()

        self.has_map = self.has_occupied and self.has_free

        self.get_logger().info(
            f"Updated free grid. Free voxels={count}",
            throttle_duration_sec=1.0
        )

    # =========================================================
    # Grid construction
    # =========================================================

    def get_roof_grid(self):
        """
        Returns a boolean grid where True means artificial roof / blocked ceiling.

        map z = Up
        roof_height = maximum allowed height in metres
        """

        roof = np.zeros((self.nx, self.ny, self.nz), dtype=bool)

        for iz in range(self.nz):
            _, _, z = self.grid_to_world(0, 0, iz)

            if z >= self.roof_height:
                roof[:, :, iz] = True

        return roof

    def rebuild_state_grid(self):
        """
        Builds a 3-state grid:
            -1 = unknown
             0 = known free
             1 = occupied

        Occupied always overrides free.
        """

        state = np.full(
            (self.nx, self.ny, self.nz),
            -1,
            dtype=np.int8
        )

        free = self.known_free_grid & (~self.occupied_grid)

        state[free] = 0
        state[self.occupied_grid] = 1

        self.state_grid = state

    def update_cost_grid(self):
        """
        Builds a lightweight weighted 3D costmap.

        Meaning:
            0.0 = blocked
            1.0 = normal free space
            >1.0 = higher traversal cost near obstacles

        Drone safety:
            Occupied voxels are inflated by self.inflation_radius.
            Default is 1.0 m for a drone about 0.8 m wide.
        """

        occupied = self.occupied_grid

        if not np.any(occupied):
            roof_grid = self.get_roof_grid()

            self.inflated_grid = roof_grid
            self.distance_grid_m = np.full(
                occupied.shape,
                np.inf,
                dtype=np.float32
            )

            cost = np.ones(occupied.shape, dtype=np.float32)
            cost[roof_grid] = 0.0

            self.cost_grid = cost
            return

        # Distance from each cell to nearest occupied voxel.
        free_mask = ~occupied
        distance_cells = distance_transform_edt(free_mask)
        distance_m = distance_cells * self.resolution

        self.distance_grid_m = distance_m.astype(np.float32)

        # Hard obstacle inflation
        blocked = distance_m <= self.inflation_radius

        # Artificial roof: block all cells at or above roof_height
        roof_grid = self.get_roof_grid()
        blocked = blocked | roof_grid

        cost = np.ones(occupied.shape, dtype=np.float32)

        # pathfinding3D typically treats 0 as unwalkable
        cost[blocked] = 0.0

        # Soft weighted zone outside hard safety radius
        if self.soft_radius > self.inflation_radius:
            near_obstacle = (
                (distance_m > self.inflation_radius) &
                (distance_m < self.soft_radius)
            )

            cost[near_obstacle] = (
                1.0 +
                4.0 *
                (self.soft_radius - distance_m[near_obstacle]) /
                (self.soft_radius - self.inflation_radius)
            )

        # Unknown cells are walkable but discouraged
        unknown = self.state_grid == -1
        cost[unknown] = np.maximum(cost[unknown], self.unknown_cost)

        # Occupied and inflated cells must remain blocked
        cost[blocked] = 0.0

        self.inflated_grid = blocked
        self.cost_grid = cost

    # =========================================================
    # Frontier detection
    # =========================================================

    def update_frontiers(self):
        """
        Detect 3D frontiers.

        Frontier = known free cell adjacent to unknown cell.

        Uses 6-connected neighbourhood.
        """

        known_free = self.state_grid == 0
        unknown = self.state_grid == -1

        structure = generate_binary_structure(rank=3, connectivity=1)

        unknown_neighbourhood = binary_dilation(
            unknown,
            structure=structure
        )

        frontier = known_free & unknown_neighbourhood

        # Do not allow frontiers inside inflated obstacle region
        frontier = frontier & (~self.inflated_grid)

        self.frontier_grid = frontier

    # =========================================================
    # RViz visualisation
    # =========================================================

    def publish_frontiers(self, max_frontier_cells=5000):
        """
        Publish detected frontier cells as green cube markers in RViz.

        If there are too many frontiers, downsample for visualisation only.
        """

        frontier_indices = np.argwhere(self.frontier_grid)

        marker_array = MarkerArray()

        # Delete old markers first
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "frontiers"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        if frontier_indices.shape[0] == 0:
            self.frontier_pub.publish(marker_array)
            return

        # Downsample only for visualisation
        if frontier_indices.shape[0] > max_frontier_cells:
            selected = np.random.choice(
                frontier_indices.shape[0],
                max_frontier_cells,
                replace=False
            )
            frontier_indices = frontier_indices[selected]

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 1
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = self.resolution

        # Green frontier voxels
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.65

        for ix, iy, iz in frontier_indices:
            x, y, z = self.grid_to_world(ix, iy, iz)

            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)

            marker.points.append(p)

        marker_array.markers.append(marker)
        self.frontier_pub.publish(marker_array)

    def publish_frontier_clusters(self, min_cluster_size=20):
        """
        Publish frontier cluster centres as blue sphere markers in RViz.
        """

        clusters = self.get_frontier_clusters(
            min_cluster_size=min_cluster_size
        )

        marker_array = MarkerArray()

        # Delete old markers first
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "frontier_clusters"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        if clusters.shape[0] == 0:
            self.frontier_cluster_pub.publish(marker_array)
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontier_clusters"
        marker.id = 1
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Blue cluster centres
        marker.color.r = 0.0
        marker.color.g = 0.25
        marker.color.b = 1.0
        marker.color.a = 1.0

        for north, east, down in clusters:
            p = Point()
            p.x = float(east)
            p.y = float(north)
            p.z = float(-down)
            marker.points.append(p)

        marker_array.markers.append(marker)
        self.frontier_cluster_pub.publish(marker_array)

    # =========================================================
    # Accessors
    # =========================================================

    def get_state_grid(self):
        return self.state_grid.copy()

    def get_occupied_grid(self):
        return self.occupied_grid.copy()

    def get_known_free_grid(self):
        return self.known_free_grid.copy()

    def get_inflated_grid(self):
        return self.inflated_grid.copy()

    def get_cost_grid(self):
        """
        Returns weighted grid for pathfinding3D.

        Meaning:
            0.0 = blocked
            1.0 = normal free
            >1.0 = higher traversal cost
        """
        return self.cost_grid.copy()

    def get_pathfinding_grid(self):
        """
        Alias for pathfinding3D input.
        """
        return self.get_cost_grid()

    def get_frontier_grid(self):
        return self.frontier_grid.copy()

    def get_frontier_indices(self):
        return np.argwhere(self.frontier_grid)

    def get_frontier_world_points(self):
        """
        Returns frontier cells as map-frame points:
            [[east, north, up], ...]
        """

        indices = self.get_frontier_indices()

        if indices.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32)

        points = []

        for ix, iy, iz in indices:
            x, y, z = self.grid_to_world(ix, iy, iz)
            points.append([x, y, z])

        return np.array(points, dtype=np.float32)

    def get_frontier_ned_points(self):
        """
        Returns frontier cells as MAVSDK NED points:
            [[north, east, down], ...]
        """

        world_points = self.get_frontier_world_points()

        if world_points.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32)

        east = world_points[:, 0]
        north = world_points[:, 1]
        up = world_points[:, 2]
        down = -up

        return np.column_stack([north, east, down]).astype(np.float32)

    def get_frontier_clusters(self, min_cluster_size=20):
        """
        Groups frontier cells into 3D connected components.

        Returns representative cluster centre points in MAVSDK NED:
            [[north, east, down], ...]
        """

        structure = generate_binary_structure(rank=3, connectivity=1)
        labelled, num_features = label(self.frontier_grid, structure=structure)

        cluster_centres = []

        for cluster_id in range(1, num_features + 1):
            indices = np.argwhere(labelled == cluster_id)

            if indices.shape[0] < min_cluster_size:
                continue

            centre_index = np.mean(indices, axis=0)

            ix, iy, iz = centre_index
            x, y, z = self.grid_to_world(ix, iy, iz)

            east = x
            north = y
            down = -z

            cluster_centres.append([north, east, down])

        if len(cluster_centres) == 0:
            return np.empty((0, 3), dtype=np.float32)

        return np.array(cluster_centres, dtype=np.float32)

    def print_summary(self):
        occupied_count = int(np.sum(self.occupied_grid))
        free_count = int(np.sum(self.known_free_grid))
        unknown_count = int(np.sum(self.state_grid == -1))
        inflated_count = int(np.sum(self.inflated_grid))
        frontier_count = int(np.sum(self.frontier_grid))

        self.get_logger().info(
            f"Summary | occupied={occupied_count}, "
            f"free={free_count}, unknown={unknown_count}, "
            f"inflated={inflated_count}, frontiers={frontier_count}",
            throttle_duration_sec=1.0
        )

    def get_frontier_cluster_infos(self, min_cluster_size=5):
        """
        Groups frontier cells into 3D connected components.

        Returns a list of dictionaries:
            {
                "center_ned": np.array([north, east, down]),
                "center_grid": np.array([ix, iy, iz]),
                "size": number of frontier cells in cluster
            }
        """

        structure = generate_binary_structure(rank=3, connectivity=1)
        labelled, num_features = label(self.frontier_grid, structure=structure)

        cluster_infos = []

        for cluster_id in range(1, num_features + 1):
            indices = np.argwhere(labelled == cluster_id)

            cluster_size = indices.shape[0]

            if cluster_size < min_cluster_size:
                continue

            centre_index = np.mean(indices, axis=0)

            ix, iy, iz = centre_index
            x, y, z = self.grid_to_world(ix, iy, iz)

            east = x
            north = y
            down = -z

            cluster_infos.append({
                "center_ned": np.array([north, east, down], dtype=np.float32),
                "center_grid": np.array([ix, iy, iz], dtype=np.float32),
                "size": int(cluster_size),
            })

        return cluster_infos

async def ros_spin_task(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)

        # Publish RViz visualisations
        node.publish_frontiers(max_frontier_cells=5000)
        node.publish_frontier_clusters(min_cluster_size=20)

        node.print_summary()

        await asyncio.sleep(0.1)


async def main():
    rclpy.init()

    node = OctomapToNumpyGrid(
        occupied_topic="/occupied_cells_vis_array",
        free_topic="/free_cells_vis_array",

        # Planner grid resolution
        resolution=0.5,

        # Arena size
        size_x=45.0,
        size_y=45.0,
        size_z=8.5,

        # If your map starts at East=0, North=0:
        origin_x=0.0,
        origin_y=0.0,
        origin_z=0.0,

        # Drone is about 0.8 m wide.
        # 1.0 m gives conservative obstacle inflation.
        inflation_radius=1.0,

        # Soft cost zone around obstacles.
        soft_radius=2.0,

        # Unknown space is walkable but discouraged.
        unknown_cost=5.0,

        # Artificial roof height
        roof_height=8.0,
    )

    try:
        await ros_spin_task(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
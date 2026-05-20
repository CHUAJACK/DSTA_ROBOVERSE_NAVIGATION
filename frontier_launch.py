from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

_DIR = os.path.dirname(os.path.abspath(__file__))

def generate_launch_description():
    return LaunchDescription([

        # ── Gazebo → ROS 2 bridge ────────────────────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
             #   '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            output='screen',
        ),

        # ── PointCloud Republisher ───────────────────────────────────────────
        Node(
            package="depth_to_pointcloud",
            executable="depth_to_pointcloud_node",
            name="gz_depth_republisher",
            output="screen",
            parameters=[
                {
                    "gz_depth": "/depth_camera",
                    "gz_camera_info":  "/camera_info",
                    "output_topic":      "/depth_camera_bridged/points",
                    "null_range_min":      0.95,
                    "null_range_max":      1.0,
                    "downsample": 3,
                    "danger_threshold": 3.0
                },
            ],
        ),

        # ── Static TFs ───────────────────────────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_link',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--x', '0.12', '--y', '0.03', '--z', '0.242',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_map',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
        ),

        # ── Point cloud downsampler (stride-2 → 4× fewer points for OctoMap) ─
        ExecuteProcess(
            cmd=['bash', '-c',
                 'source /opt/ros/humble/setup.bash && python3 '
                 + os.path.join(_DIR, 'pointcloud_downsample.py')],
            shell=False,
            output='screen',
        ),

        # ── OctoMap server ───────────────────────────────────────────────────
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            remappings=[('cloud_in', '/depth_camera_bridged/points')],
            parameters=[{
                'resolution': 0.35,  
                'frame_id': 'map',
                'base_frame_id': 'base_link',
                'sensor_model.max_range': 15.0,
                'transform_tolerance': 1.0,
                'use_sim_time': True,
                'ground_filter': True,
                'occupancy_min_z': 0.8,
                'occupancy_max_z': 8.0,
                'sensor_model/hit': 0.95,
                'sensor_model/miss': 0.3,
                'latch':False,


   
            }],
        ),

        # ── 2D slicer ────────────────────────────────────────────────────────
        Node(
            package='octomap_2d_slicer',
            executable='octomap_2d_slicer_node',
            name='octomap_2d_slicer',
            output='screen',
            parameters=[{
                'drone_frame': 'base_link',
                'world_frame': 'map',
                'slice_thickness': 0.4,
                'publish_rate_ms': 50,
                'publish_rate_ms': 50,
                'use_sim_time': True,
            }],
        ),

        # ── RViz2 with frontier config ────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(_DIR, 'frontier_explore.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])

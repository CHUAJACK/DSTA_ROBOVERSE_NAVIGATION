from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge_clock',
            parameters=[{'use_sim_time': True}],      # ← add
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            output='screen'
        ),
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
                },
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_link',
            parameters=[{'use_sim_time': True}],      # ← add
            arguments=[
                '--x', '0.12',
                '--y', '0.03',
                '--z', '0.242',
                '--qx', '0',
                '--qy', '0',
                '--qz', '0',
                '--qw', '1',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_map',
            parameters=[{'use_sim_time': True}],      # ← add
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--qx', '0',
                '--qy', '0',
                '--qz', '0',
                '--qw', '1',
                '--frame-id', 'map',
                '--child-frame-id', 'odom'
            ]
        ),   
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            remappings=[
                ('cloud_in', '/depth_camera_bridged/points'),
            ],
            parameters=[{
                'resolution': 0.25,
                'frame_id': 'map',
                'base_frame_id': 'base_link',
                'sensor_model.max_range': 16.0,
                'transform_tolerance': 0.5,
                'use_sim_time': True,
                'ground_filter': True,
                'occupancy_min_z':0.3,
                'occupancy_max_z':8.0,
                'sensor_model/hit':       0.7,
                'sensor_model/miss':      0.4,
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.expanduser('./backup.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package="octomap_2d_slicer",
            executable="octomap_2d_slicer_node",
            name="octomap_2d_slicer",
            output="screen",
            parameters=[{
                "drone_frame":     'base_link',
                "world_frame":     'map',
                "slice_thickness": 0.4,
                "use_sim_time": True,
            }],
            remappings=[
                # remap if your topics differ
                # ("octomap_binary", "/octomap_binary"),
                # ("octomap_2d_slice", "/map_2d"),
            ],
        )
    ])
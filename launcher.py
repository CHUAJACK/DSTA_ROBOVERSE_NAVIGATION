from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge_clock',
            parameters=[{'use_sim_time': True}],      # ← add
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
            ],
            output='screen'
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
                ('cloud_in', '/depth_camera/points'),
            ],
            parameters=[{
                'resolution': 0.1,
                'frame_id': 'map',
                'base_frame_id': 'base_link',
                'sensor_model.max_range': 5.0,
                'transform_tolerance': 0.5,
                'use_sim_time': True,
            }]
        ),
    ])
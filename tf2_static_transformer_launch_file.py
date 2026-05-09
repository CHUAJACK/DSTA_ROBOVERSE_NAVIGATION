from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
            # args: x y z yaw pitch roll parent_frame child_frame
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_link',
            arguments=['0.12', '0.03', '0.242', '0', '0', '0', 'base_link', 'camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])
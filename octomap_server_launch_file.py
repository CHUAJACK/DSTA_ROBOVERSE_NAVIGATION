from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[{
                'resolution': 0.1,
                'frame_id': 'map',
                'base_frame_id': 'base_link',
                'sensor_model/max_range': 5.0,
                'occupancy_min_z': -10.0,
                'occupancy_max_z': 10.0,
            }],
            remappings=[
                ('cloud_in', '/camera/points')  # adjust to your topic
            ]
        )
    ])
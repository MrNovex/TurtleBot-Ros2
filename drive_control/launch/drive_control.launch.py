from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive_control',
            executable='drive_node',
            name='drive_node',
            output='screen',
            parameters=[{
                'linear_speed': 0.1,
                'turn_speed': 1.1,
                'safe_distance': 0.3,
                'use_heading_control': True,  # Kurs ueber Odometrie halten
            }],
        ),
    ])

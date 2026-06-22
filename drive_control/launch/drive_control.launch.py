"""Launch-Datei fuer den ``drive_control``-Node.

Startet den ``drive_node`` mit sinnvollen Standardparametern. Einzelne
Werte koennen hier direkt angepasst oder beim Aufruf ueberschrieben
werden, z. B.::

    ros2 launch drive_control drive_control.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Erzeugt die Launch-Beschreibung mit dem konfigurierten Node."""
    return LaunchDescription([
        Node(
            package='drive_control',
            executable='drive_node',
            name='drive_node',
            output='screen',
            parameters=[{
                # Fahrgeschwindigkeit in m/s
                'linear_speed': 0.1,
                # Drehgeschwindigkeit beim Ausweichen in rad/s
                'turn_speed': 1.1,
                # Sicherheitsabstand vorne in m
                'safe_distance': 0.3,
                # Odometrie-Kursregelung gegen schraeges Fahren (True/False)
                'use_heading_control': True,
            }],
        ),
    ])

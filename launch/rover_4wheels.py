import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """ 
    Launch für das Rover_4Wheels Projekt

    Automatischer Start folgender Nodes:

    1. `teleop_twist_joy` - Package für die Steuerung des Gamepads
    2. `rover_4wheels`-Node für die Ansteuerung des esp32 zur Steuerung der Antriebseinheiten
    """
    return LaunchDescription([
        # Start teleop_twist_joy node für die Joystick-Steuerung
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[{
                #'require_enable_button': False,
                #'enable_button': 5,  # Beispiel: Button 5 aktiviert Steuerung
                'axis_linear': {'x': 1},
                'scale_linear': {'x': 0.5},
                'axis_angular': {'yaw': 0},
                'scale_angular': {'yaw': 0.5},
            }]
        ),
        Node(
            package='rover_4wheels',
            executable='rover_node',
            name='rover_4wheels_node',
            output='screen',
            parameters=[{
                # Abstand zwischen den Rädern (mm)
                'wheel_separation': 120, 
                'wheel_track_width': 110,    # Spurweite der Räder
                'wheel_radius': 60,         # Radius der Räder (mm)
                'max_speed': 1.5,           # Maximale Geschwindigkeit (cm/s) 
            }]       
        )
    ])   # Abstand zwischen den Rädern (mm)
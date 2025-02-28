```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file für das Roverprojekt. 
    Startet automatisch:
    1. Das `teleop_twist_joy`-Package für die Steuerung per Gamepad.
    2. Den `rover_4wheels`-Node für die Ansteuerung der Motoren.
    """
    return LaunchDescription([
        # Start teleop_twist_joy node für die Joystick-Steuerung
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[{
                'require_enable_button': False,
                'enable_button': 5,  # Beispiel: Button 5 aktiviert Steuerung
                'axis_linear': {'x': 1},
                'scale_linear': {'x': 0.5},
                'axis_angular': {'yaw': 0},
                'scale_angular': {'yaw': 0.5},
            }]
        ),
        
        # Start rover_4wheels node zur Steuerung des Rovers
        Node(
            package='rover_4wheels',
            executable='rover_control',
            name='rover_4wheels',
            output='screen',
            parameters=[{
                'wheel_separation': 0.4,  # Abstand zwischen den Rädern (m)
                'wheel_radius': 0.1,      # Radius der Räder (m)
                'max_speed': 1.5,         # Maximale Geschwindigkeit (m/s)
            }]
        )
    ])
```

### Beschreibung der `launch.py` Datei

Diese Datei startet zwei ROS2 Nodes:
1. **teleop_twist_joy**: Wandelt Gamepad-Inputs in `Twist`-Nachrichten um, die zur Steuerung des Rovers genutzt werden.
2. **rover_4wheels**: Verantwortlich für die physische Bewegung des Rovers basierend auf empfangenen `Twist`-Befehlen.

### Parameter-Erklärung:
- **teleop_twist_joy**:
  - `require_enable_button`: Falls `True`, muss eine Taste gedrückt gehalten werden, um Eingaben zu senden.
  - `enable_button`: Button-ID für die Aktivierung (hier `5`).
  - `axis_linear` & `scale_linear`: Steuerung der Vorwärtsbewegung.
  - `axis_angular` & `scale_angular`: Steuerung der Drehbewegung.

- **rover_4wheels**:
  - `wheel_separation`: Abstand der Räder, beeinflusst die Steuerungsdynamik.
  - `wheel_radius`: Setzt die Größe der Räder.
  - `max_speed`: Definiert die Höchstgeschwindigkeit des Rovers.

### Verwendung:
Diese Datei wird genutzt, um das Rover-Steuerungssystem über eine Launch-Datei automatisch zu starten. Ausgeführt wird sie mit:
```bash
ros2 launch rover_4wheels rover_launch.py
```
Dabei wird der `teleop_twist_joy`-Node für die Steuerung per Joystick und der `rover_4wheels`-Node für die Motorsteuerung parallel gestartet.

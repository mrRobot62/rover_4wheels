# ROVER - Software

# Allgemein


# Rover-Python


# verwendete Packages

## ros2-teleop (prefered)

- Installation
    - `sudo snap install ros2-teleop`


## teleop_twist_jox (alternativ)
Mit diesem Package wird der F170 Gamepad-Controller abgefragt. Dieses Package bietet als Publisher folgende ausgaben

[teleop_twist_joy - docu](https://docs.ros.org/en/rolling/p/teleop_twist_joy/)
[CheetSheet ROS2](https://github.com/nfry321/ROS2_cheat_sheets/blob/master/package-tests/teleop.md)



### Arguments
- `joy_config (string, default: 'ps3')`
    - Welcher Typ genutzt wird `ps3`, `xbox`, ``
- `joy_dev (string, default: '0')`
    - Joystick device
- `config_filepath (string, default: '/opt/ros/<rosdistro>/share/teleop_twist_joy/config/' + LaunchConfig('joy_config') + '.config.yaml')`
    - Pfad zu einer Konfigurationsdatei (wenn notwendig)
- `publish_stamped_twist (bool, default: false)`
    - Wenn `geometry_msgs/msg/TwistStamped` für Kommandos für Geschwindigkeiten (velocity) veröffentlicht werden sollen

### Topic

### Test
- Start des Nodes (in Terminal 1) `ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'`
- Topic auslesen: (in Temrinal 2) `ros2 topic echo /joy` 
    - den Gamepad-Controller (Schalter Rückseite) auf D stellen
    - nun sollte man im Terminal ausgaben sehen, wenn man die Joysticks & Buttons betätigt. Wenn das funktioniert, können Nachrichten subscribed werden


`ros2 topic list`
Zeigt alle aktuell verfügbaren topics

### Test TurtleSim
**Terminal 1: **
`ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'`

**Terminal 2: **
XBOX Controller Einstellung prüfen: `ros2 topic echo /cmd_val`.
Jetzt den Button hinten oben rechts drücken und gedrückt halten. Anschließend den linken Joystick bewegen. Im Terminal sollten nun Bewegungen für die X/Z Achse angezeigt werden.
mit CTRL-C abbrechen.

Folgende Befehl eingeben: `ros2 run turtlesim turtlesim_no --ros-args --remap /turtle1/cmd_vel:=/cmd_vel`
(Das Topic /cmd_vel, wird nun umgeleitet an das Topic /turtle1/cmd_vel).
Taste hinten oben rechts drücken und nun den Joystick bewegen, die Schildkröte sollte nun den Befehlen am Bildschirm folgen. Die Geschwindigkeit der Schildkröte wird mit dem Joystickausschlag beeinflusst.

[YT-Video] (https://youtu.be/_MVA1fkzRKM?si=xtjr2p0QEK_OgA7I)

# Links, Tutorials, Videos

[Teleop_twist_joy](https://youtu.be/_MVA1fkzRKM?si=cW9Zs1woERQ3pFu9)
# DEMO Pi5 + PythonNode + GamePad + ESP32
Demo-Programm lauffähig auf einem PI5 unter Ubuntu 24.04 und ros2

# Folder-Struktur

```
.
├──./doc
       └──./doc/readme.md
       └──./doc/readme_topics.md
├──./launch
       └──./launch/ros2_esp32_dynamixel.launch.py
├──./package.xml
├──./resource
       └──./resource/ros2_esp32_dynamixel
├──./ros2_esp32_dynamixel
       └──./ros2_esp32_dynamixel/__init__.py
       └──./ros2_esp32_dynamixel/ros2_esp32_dynamixel_with_controller.py
├──./ros2_esp32_dynamixel.code-workspace
├──./setup.cfg
├──./setup.py
├──./test
       └──./test/test_copyright.py
       └──./test/test_flake8.py
       └──./test/test_pep257.py
├──./tree_2_markdown.sh
```

# PythonNode - ros2_esp32_dynamixel_with_controller
Dieser PythonNode importiert die PyGame Library über die der Controller abgefragt wird.
Steuerbewegungen der beiden JoySticks können dann über den ESP32 vier Dynamixel-Servos antreiben.

Es geht in dieser Demo nur darum auszuprobieren ob und wie man Dynamixel-Servos nutzen kann.


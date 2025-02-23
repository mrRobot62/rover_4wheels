import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bernd/ros2_ws/rover_4wheels/install/rover_4wheels'

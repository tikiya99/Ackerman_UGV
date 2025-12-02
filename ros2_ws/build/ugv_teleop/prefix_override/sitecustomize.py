import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/thasinduwickrama/Documents/PlatformIO/Projects/Ackerman_UGV/ros2_ws/install/ugv_teleop'

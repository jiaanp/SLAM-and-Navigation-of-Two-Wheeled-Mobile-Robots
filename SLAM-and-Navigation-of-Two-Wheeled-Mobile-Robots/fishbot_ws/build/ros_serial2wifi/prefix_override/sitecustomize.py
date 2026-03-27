import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hw/SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/fishbot_ws/install/ros_serial2wifi'

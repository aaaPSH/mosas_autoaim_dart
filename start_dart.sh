#!/bin/bash

sleep 5


# 加载 ROS 2 Humble 环境
source /opt/ros/humble/setup.bash

# 加载工作空间环境
source /home/nvidia/mosas_autoaim_dart/install/setup.bash

# 启动
ros2 launch mosas_bringup start.launch.py



#!/usr/bin/env python3
import sys
import draccus

from lerobot_gazebo_robot import ROS2GazeboRobot, ROS2GazeboRobotConfig
from keyboard_teleop import KeyboardTeleop, KeyboardTeleopConfig

from lerobot.robots.config import RobotConfig
from lerobot.teleoperators.config import TeleoperatorConfig

RobotConfig.register_subclass("ros2_gazebo_robot")(ROS2GazeboRobotConfig)
TeleoperatorConfig.register_subclass("keyboard_gazebo")(KeyboardTeleopConfig)

from lerobot.scripts.lerobot_record import main

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nRecording stopped by user (Ctrl+C). Exiting cleanly...")
        sys.exit(0)
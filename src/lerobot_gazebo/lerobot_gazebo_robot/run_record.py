#!/usr/bin/env python3
import sys
import draccus

from lerobot_gazebo_robot import ROS2GazeboRobot, ROS2GazeboRobotConfig

from lerobot.robots.config import RobotConfig
RobotConfig.register_subclass("ros2_gazebo_robot")(ROS2GazeboRobotConfig)

from lerobot.scripts.lerobot_record import main

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nRecording stopped by user (Ctrl+C). Exiting cleanly...")
        sys.exit(0)
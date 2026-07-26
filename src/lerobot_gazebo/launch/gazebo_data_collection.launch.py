import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    lerobot_gazebo_share = get_package_share_directory("lerobot_gazebo")
    ros2_control_share = get_package_share_directory("lerobot_controller")
    
    ros2_control_launch_path = os.path.join(
        ros2_control_share, "launch", "so101_controller.launch.py"
    )
    
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_control_launch_path)
    )

    gazebo_launch_path = os.path.join(
        lerobot_gazebo_share, "launch", "so101_gazebo.launch.py"
    )
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )
    
    lerobot_bridge = Node(
        package="lerobot_gazebo",
        executable="lerobot_ros2_bridge",
        name="lerobot_ros2_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
        
    return LaunchDescription([
        gazebo_launch,
        lerobot_bridge,
        ros2_control_launch,
    ])
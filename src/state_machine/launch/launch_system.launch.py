import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare the 'is_sim' launch argument (defaults to 'true')
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="Whether running in simulation mode (Gazebo)"
    )
    
    is_sim = LaunchConfiguration("is_sim")

    lerobot_gazebo_share = get_package_share_directory("lerobot_gazebo")
    ros2_control_share = get_package_share_directory("lerobot_controller")
    lerobot_moveit_share = get_package_share_directory("lerobot_moveit")
    
    # Map 'is_sim' to both 'is_sim' and standard ROS 'use_sim_time'
    common_launch_args = {
        "is_sim": is_sim,
        "use_sim_time": is_sim,
    }.items()

    ros2_control_launch_path = os.path.join(
        ros2_control_share, "launch", "so101_controller.launch.py"
    )
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_control_launch_path),
        launch_arguments=common_launch_args
    )

    gazebo_launch_path = os.path.join(
        lerobot_gazebo_share, "launch", "so101_gazebo.launch.py",
    )
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments=common_launch_args, condition=IfCondition(is_sim)
        
    )
    
    ros2_moveit_move_group_path = os.path.join(
        lerobot_moveit_share, "launch", "move_group.launch.py"
    )
    ros2_moveit_move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_moveit_move_group_path),
        launch_arguments=common_launch_args
    )
    
    ros2_moveit_rviz_path = os.path.join(
        lerobot_moveit_share, "launch", "moveit_rviz.launch.py"
    )
    ros2_moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_moveit_rviz_path),
        launch_arguments=common_launch_args
    )
        
    return LaunchDescription([
        is_sim_arg,
        gazebo_launch,
        ros2_control_launch,
        ros2_moveit_move_group_launch,
        ros2_moveit_rviz_launch
    ])
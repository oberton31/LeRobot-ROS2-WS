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
    lerobot_description = get_package_share_directory("lerobot_description") # where package installs
    lerobot_gazebo = get_package_share_directory("lerobot_gazebo")
    
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        lerobot_description, "urdf", "so101_w_cameras.urdf.xacro"
                                        ),
                                      description="Absolute path to robot/scene urdf file"
    )

    world_path = os.path.join(lerobot_gazebo, "worlds", "table_scene.sdf")

    # put models in place where Gazebo can access them
    gazebo_resource_path = SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=[
                str(Path(lerobot_description).parent.resolve()),
            ]
        )
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " use_sim:=True"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                        "gz_args": f"-v 4 -r {world_path}"
                    }.items()
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "so101", '-x', '0.0', '-y', '0.0', '-z', '1.015'],
    )

    # bridge Ignition transport topics over to ROS 2
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=[
            # simulation Clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            
            # RGB Color Stream & Info
            "/wrist_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            
            "/overhead_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/world/robot/control@ros_gz_interfaces/srv/ControlWorld",
            # # depth Stream
            # "/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            
            # # 3D Point Cloud Stream. Note that 
            # "/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        # remappings=[
        #     # standardize topic names to match typical ROS RealSense layout
        #     ("/camera/image", "/camera/camera/color/image_raw"),
        #     ("/camera/camera_info", "/camera/camera/color/camera_info"),
        #     ("/camera/depth_image", "/camera/camera/depth/image_rect_raw"),
        #     ("/camera/points", "/camera/camera/depth/color/points"),
        # ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])
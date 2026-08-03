# Gazebo

## Overview
Gazebo is an open source robotics simulator maintained by Open Robotics. As Open Robotics also maintains ROS, there is strong ROS support while using the Gazebo platform. Namely, the ROS-Gazebo bridge maps internal simulation topics to standard ROS2 topics, emulating sensors. Additionally, the ROS2 control Gazebo plugin allows you to emulate control inputs, exposing them in an identical fashion as the actual sensors would. For the full Gazebo simulation, see the link [here](https://gazebosim.org/home).

## Scene Files
Denoted with the .sdf filename, these files contain information on the underlying gazebo scene. Using these scene files, you can spawn in a world scene, as well as a robot (note that .urdf files cannot be spawned in this way)

## Launching Gazebo with ROS2
The package *ros_gz_sim*, which contains tools for using Gazebo Sim with ROS, contains a standalone launch file for launching gazebo sim

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf"
```

If you have your own scene file (note that the easiest way to build this is by launching an empty sim, modifying it, and then saving the file), you can launch this by changing the gazebo argument to be the filepath of this scene.

## Spawning URDF Files with ROS2
URDF files are often used within the ROS ecosystem to define robots. However, URDF files are not natively supported by Gazebo. The *ros_gz_sim* package provides a node called *create*, which essentially takes your URDF file, parses it, and dynamically injects it into world. 

The steps to making this happen are a bit more involved. First, a *robot_state_publisher* node needs to be spun, which takes your URDF file and publishes it as live text. The aformentioned *create* node subscribes to this topic, reads the raw URDF file, converts it to a format Gazebo can understand, and then spawns it in.

## Using Sensors
To begin, include the sensor links and joint connections as part of your URDF. Afterward, include the sensor gazebo plugin under a gazebo tag. Finally, specify the sensor parameters under a gazebo/sensor tag. Note that for cameras, the realsense and ROS convention is that the camera points forward along the +Z axis of the camera optical frame, but gazebo renders the +X axis as being forward from the camera.

## Updating Robot State Based on Simulation Using ROS2
We have discussed how to inject the URDF file into Gazebo. However, to update the robot based on the simulation and control input, another node is needed. This node needs to publish on the *\joint_states* topic subscribed to by *robot_state_publisher*, which then references the original URDF to calculate the live coordinate transforms between joints. There are two methods of doing this, the first, and most common, is by using the ros2 control framework, and the associated gazebo plugin, details of which can be found [here](ros2_control.md). This spins up a *joint_state_broadcaster* node which reads the simulation angles directly. If there are passive joints, you can use the *ros_gz_bridge* library and its *parameter_bridge* node, specifying it to publish joint states. You can setup a combination of these two methods if you have both actuated and un-actuated joints, but you need to be careful about overlap, as by default they both publish to the same topic. While this information is used by robot state publisher to update transforms, note that Gazebo does not actually use these topics as it keeps track of things internally.

Finally, you want to ensure that the simulation and ROS2 nodes are all using the same clock, so they stay in sync and mimic real execution, especially for algorithms like state estimation that require timing. For example, say you issue a command for a robot to move forward for 3 seconds. If the simulation is running 50% slower than wallclock time, without the sim time being used the robot would only move for 1.5 simulated seconds, causing errors.

To achieve this synchronization, first set a flag  for your ROS nodes to use simulation time. Then, use the previously mentioned *parameter_bridge* node to publish the simulation time.

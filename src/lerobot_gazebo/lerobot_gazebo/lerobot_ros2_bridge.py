#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from cv_bridge import CvBridge
from multiprocessing.shared_memory import SharedMemory

class ROS2GazeboBridgeNode(Node):
    def __init__(self):
        super().__init__("lerobot_gazebo_shm_bridge")
        self.bridge = CvBridge()

        self.joint_names = [
            "1",  # Rotation
            "2",  # Pitch
            "3",  # Elbow
            "4",  # Wrist Pitch
            "5",  # Wrist Roll
            "6",  # Gripper
        ]
        self.num_joints = 6

        # buffer specs
        self.img_shape = (480, 640, 3)
        self.img_bytes = int(np.prod(self.img_shape) * np.uint8().itemsize)
        self.joint_shape = (self.num_joints,)
        self.joint_bytes = int(self.num_joints * np.float32().itemsize)

        # allocate Shared Memory blocks safely
        self.shm_cam_overhead = self._init_shm("shm_cam_overhead", self.img_bytes)
        self.shm_cam_wrist = self._init_shm("shm_cam_wrist", self.img_bytes)
        self.shm_joint_state = self._init_shm("shm_joint_state", self.joint_bytes)
        self.shm_action = self._init_shm("shm_action", self.joint_bytes)

        # create NumPy array views onto shared RAM
        self.arr_overhead = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.shm_cam_overhead.buf)
        self.arr_wrist = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.shm_cam_wrist.buf)
        self.arr_joint_state = np.ndarray(self.joint_shape, dtype=np.float32, buffer=self.shm_joint_state.buf)
        self.arr_action = np.ndarray(self.joint_shape, dtype=np.float32, buffer=self.shm_action.buf)

        self.arr_action.fill(0.0)

        # subscriptions
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(Image, "/overhead_camera/image_raw", self._cam_overhead_cb, 10)
        self.create_subscription(Image, "/wrist_camera/image_raw", self._cam_wrist_cb, 10)

        # Publishers
        self.arm_action_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.gripper_action_pub = self.create_publisher(JointTrajectory, "/gripper_controller/gripper_command", 10)

        # Timer to publish actions (~30 Hz)
        self.create_timer(1.0 / 30.0, self._publish_action_cb)
        self.get_logger().info("ROS 2 Shared Memory Bridge initialized and ready (6 Total Joints).")

    def _init_shm(self, name: str, size: int) -> SharedMemory:
        """Create SHM or attach if it already exists."""
        try:
            return SharedMemory(name=name, create=True, size=size)
        except FileExistsError:
            return SharedMemory(name=name, create=False, size=size)

    def _js_cb(self, msg: JointState):
        # no guarantee of joint ordering in /joint_states, so map by name specified in URDF/joint_names
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.arr_joint_state[idx] = pos

    def _cam_overhead_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        np.copyto(self.arr_overhead, cv_img)

    def _cam_wrist_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        np.copyto(self.arr_wrist, cv_img)

    def _publish_action_cb(self):
        action = self.arr_action.copy()
        if np.all(action == 0.0):
            return  # No action set yet

        arm_msg = JointTrajectory()
        arm_point = JointTrajectoryPoint()
        arm_point.positions = action[:5].tolist()  # First 5 elements -> Arm
        arm_point.time_from_start.nanosec = int(1e8 / 3)
        arm_msg.points.append(arm_point)
        self.arm_action_pub.publish(arm_msg)

        gripper_msg = JointTrajectory()
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [action[5].item()]  # Index 5 -> Gripper
        gripper_point.time_from_start.nanosec = int(1e8 / 3)
        gripper_msg.points.append(gripper_point)
        self.gripper_action_pub.publish(gripper_msg)

    def destroy_node(self):
        for shm in [self.shm_cam_overhead, self.shm_cam_wrist, self.shm_joint_state, self.shm_action]:
            shm.close()
            try:
                shm.unlink()
            except FileNotFoundError:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ROS2GazeboBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
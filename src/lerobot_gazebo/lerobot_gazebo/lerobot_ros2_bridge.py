#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray
import numpy as np
from cv_bridge import CvBridge
from multiprocessing.shared_memory import SharedMemory
from ros_gz_interfaces.srv import ControlWorld

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

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

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
        self.shm_reset = self._init_shm("shm_reset", 1)
        
        # create NumPy array views onto shared RAM
        self.arr_overhead = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.shm_cam_overhead.buf)
        self.arr_wrist = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.shm_cam_wrist.buf)
        self.arr_joint_state = np.ndarray(self.joint_shape, dtype=np.float32, buffer=self.shm_joint_state.buf)
        self.arr_action = np.ndarray(self.joint_shape, dtype=np.float32, buffer=self.shm_action.buf)
        self.arr_reset = np.ndarray((1,), dtype=np.uint8, buffer=self.shm_reset.buf)
        
        self.arr_reset.fill(0)
        self.arr_action.fill(0.0)

        # subscriptions
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(Image, "/overhead_camera/image_raw", self._cam_overhead_cb, 10)
        self.create_subscription(Image, "/wrist_camera/image_raw", self._cam_wrist_cb, 10)

        # publishers for ForwardCommandControllers (streaming mode)
        self.arm_action_pub = self.create_publisher(
            Float64MultiArray, "/arm_streaming_controller/commands", 10
        )
        self.gripper_action_pub = self.create_publisher(
            Float64MultiArray, "/gripper_streaming_controller/commands", 10
        )
        
        self.home_arm_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.home_gripper_pose = np.array([0.0], dtype=np.float32)


        # timer to publish actions (~30 Hz). Need to pick this rate to match the Hz at which I think I can run my VLA or faster
        self.create_timer(1.0 / 30.0, self._publish_action_cb)
        self.get_logger().info("ROS 2 Shared Memory Bridge initialized and ready (Streaming Controller Mode).")
        self.prev_action = np.zeros(self.joint_shape, dtype=np.float32)
        
        self.reset_client = self.create_client(ControlWorld, "/world/robot/control")
        self.is_resetting = False

    def _init_shm(self, name: str, size: int) -> SharedMemory:
        """Always unlink stale RAM blocks first to guarantee a fresh, clean segment."""
        try:
            shm = SharedMemory(name=name, create=False)
            shm.close()
            shm.unlink()
        except FileNotFoundError:
            pass

        return SharedMemory(name=name, create=True, size=size)

    def _js_cb(self, msg: JointState):
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
        if self.is_resetting:
            return

        if self.arr_reset[0] == 1:
            self.get_logger().info("Reset signal received from shared memory.")
            self._reset_sim()
            return

        action = self.arr_action.copy()

        if np.isnan(action).any():
            return

        arm_msg = Float64MultiArray()
        arm_msg.data = [float(x) for x in action[:5]]
        self.arm_action_pub.publish(arm_msg)

        gripper_msg = Float64MultiArray()
        gripper_msg.data = [float(action[5])]
        self.gripper_action_pub.publish(gripper_msg)
        
        self.prev_action = action.copy()
        
        
    def _reset_sim(self):
        # resets floating items in sim, not the arm pose
        self.get_logger().info("Resetting Gazebo simulation...")
        if not self.reset_client.service_is_ready():
            self.get_logger().warn("Reset service not ready yet. Skipping reset.")
            self.arr_reset[0] = 0
            return

        self.get_logger().info("Resetting Gazebo simulation...")
        self.is_resetting = True
        
        request = ControlWorld.Request()
        request.world_control.reset.model_only = True

        future = self.reset_client.call_async(request)
        future.add_done_callback(self._on_reset_complete)

    def _on_reset_complete(self, future):
        try:
            _ = future.result()
            self.get_logger().info("Gazebo reset complete.")            
        except Exception as e:
            self.get_logger().error(f"Gazebo reset failed: {e}")

        # clear shared memory & reset local flags on completion
        self.arr_reset[0] = 0
        self.is_resetting = False
        
    def destroy_node(self):
        for shm in [self.shm_cam_overhead, self.shm_cam_wrist, self.shm_joint_state, self.shm_action, self.shm_reset]:
            shm.close()
            shm.unlink()
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
import time
import torch
import numpy as np
from dataclasses import dataclass, field
from multiprocessing.shared_memory import SharedMemory

from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot


@RobotConfig.register_subclass("ros2_gazebo_robot")
@dataclass
class ROS2GazeboRobotConfig(RobotConfig):
    type: str = "ros2_gazebo_robot"

    features: dict = field(
        default_factory=lambda: {
            "observation.state": {"dtype": "float32", "shape": (7,)},
            "observation.images.cam_overhead": {"dtype": "video", "shape": (3, 480, 640)},
            "observation.images.cam_wrist": {"dtype": "video", "shape": (3, 480, 640)},
            "action": {"dtype": "float32", "shape": (7,)},
        }
    )

class ROS2GazeboRobot(Robot):
    name = "ros2_gazebo_robot"
    config_class = ROS2GazeboRobotConfig

    def __init__(self, config: ROS2GazeboRobotConfig):
        self.config = config
        super().__init__(config)

        self.img_shape = (480, 640, 3)
        self.joint_shape = (7,)

        self.shm_cam_overhead = None
        self.shm_cam_wrist = None
        self.shm_joint_state = None
        self.shm_action = None

    def connect(self):
        print("Connecting to ROS 2 Shared Memory buffers...")
        while True:
            try:
                self.shm_cam_overhead = SharedMemory(name="shm_cam_overhead", create=False)
                self.shm_cam_wrist = SharedMemory(name="shm_cam_wrist", create=False)
                self.shm_joint_state = SharedMemory(name="shm_joint_state", create=False)
                self.shm_action = SharedMemory(name="shm_action", create=False)
                break
            except FileNotFoundError:
                print("Waiting for ros2_shm_bridge.py node to start...")
                time.sleep(1.0)

        # Wrap raw RAM in NumPy views
        self.arr_overhead_camera = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.shm_cam_overhead.buf)
        self.arr_wrist_camera = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.shm_cam_wrist.buf)
        self.arr_joint_state = np.ndarray(self.joint_shape, dtype=np.float32, buffer=self.shm_joint_state.buf)
        self.arr_action = np.ndarray(self.joint_shape, dtype=np.float32, buffer=self.shm_action.buf)

        print("Connected to Gazebo simulation successfully via Shared Memory.")

    def get_observation(self) -> dict:
        # Convert NumPy views directly to torch.Tensors matching your original structure
        state_tensor = torch.from_numpy(self.arr_joint_state.copy()).float()
        overhead_tensor = torch.from_numpy(self.arr_overhead_camera.copy()).permute(2, 0, 1)
        wrist_tensor = torch.from_numpy(self.arr_wrist_camera.copy()).permute(2, 0, 1)

        return {
            "observation.state": state_tensor,
            "observation.images.cam_overhead": overhead_tensor,
            "observation.images.cam_wrist": wrist_tensor,
        }

    def send_action(self, action: torch.Tensor):
        # Write action tensor straight to shared RAM for the ROS node to publish
        action_np = action.detach().cpu().numpy().astype(np.float32)
        np.copyto(self.arr_action, action_np)

    def disconnect(self):
        for shm in [self.shm_cam_overhead, self.shm_cam_wrist, self.shm_joint_state, self.shm_action]:
            if shm:
                shm.close()
        print("Disconnected from Shared Memory.")
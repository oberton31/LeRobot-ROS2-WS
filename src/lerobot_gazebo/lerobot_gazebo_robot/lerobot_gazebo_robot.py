import time
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
            "observation.state": {"dtype": "float32", "shape": (6,)},
            "observation.images.cam_overhead": {"dtype": "video", "shape": (3, 480, 640)},
            "observation.images.cam_wrist": {"dtype": "video", "shape": (3, 480, 640)},
            "action": {"dtype": "float32", "shape": (6,)},
        }
    )


class ROS2GazeboRobot(Robot):
    name = "ros2_gazebo_robot"
    config_class = ROS2GazeboRobotConfig

    def __init__(self, config: ROS2GazeboRobotConfig):
        super().__init__(config)
        self.config = config

        self.img_shape = (480, 640, 3)
        self.joint_shape = (6,)

        self.shm_cam_overhead = None
        self.shm_cam_wrist = None
        self.shm_joint_state = None
        self.shm_action = None
        
        self.prev_action = np.zeros(self.joint_shape, dtype=np.float32)
    
    # required abstract properties
    @property
    def observation_features(self) -> dict:
        """Returns observation feature descriptions matching channel-first layout (C, H, W)."""
        return {
            "observation.state": {"dtype": "float32", "shape": (6,)},
            "observation.images.cam_overhead": {"dtype": "video", "shape": (3, 480, 640)},
            "observation.images.cam_wrist": {"dtype": "video", "shape": (3, 480, 640)},
        }

    @property
    def action_features(self) -> dict:
        """Returns action feature descriptions expected by send_action()."""
        return {
            "action": {"dtype": "float32", "shape": (6,)},
        }

    @property
    def is_connected(self) -> bool:
        """Returns True if all shared memory segments are attached."""
        return all(
            shm is not None
            for shm in [self.shm_cam_overhead, self.shm_cam_wrist, self.shm_joint_state, self.shm_action]
        )

    @property
    def is_calibrated(self) -> bool:
        """Gazebo simulation requires no physical calibration."""
        return True

    @property
    def cameras(self) -> dict:
        """Returns camera name keys so LeRobot's recorder counts camera streams correctly."""
        return {
            "cam_overhead": "shm_cam_overhead",
            "cam_wrist": "shm_cam_wrist",
        }

    @property
    def features(self) -> dict:
        """Returns the features dict from the config."""
        return self.config.features

    # required abstract methods
    def calibrate(self) -> None:
        """No-op for simulation."""
        pass

    def configure(self) -> None:
        """No-op runtime configuration."""
        pass

    def connect(self, calibrate: bool = True):
        print("Connecting to ROS 2 Shared Memory buffers...")
        while True:
            try:
                self.shm_cam_overhead = SharedMemory(name="shm_cam_overhead", create=False)
                self.shm_cam_wrist = SharedMemory(name="shm_cam_wrist", create=False)
                self.shm_joint_state = SharedMemory(name="shm_joint_state", create=False)
                self.shm_action = SharedMemory(name="shm_action", create=False)
                break
            except FileNotFoundError:
                print("Waiting for lerobot_ros2_bridge.py node to start...")
                time.sleep(1.0)

        # wrap raw RAM in NumPy views
        self.arr_overhead_camera = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.shm_cam_overhead.buf)
        self.arr_wrist_camera = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.shm_cam_wrist.buf)
        self.arr_joint_state = np.ndarray(self.joint_shape, dtype=np.float32, buffer=self.shm_joint_state.buf)
        self.arr_action = np.ndarray(self.joint_shape, dtype=np.float32, buffer=self.shm_action.buf)

        print("Connected to Gazebo simulation successfully via Shared Memory.")

    def get_observation(self) -> dict[str, np.ndarray]:
        """Returns latest sensory observation array."""
        return {
            "observation.state": self.arr_joint_state.copy(),
            "cam_overhead": np.transpose(self.arr_overhead_camera.copy(), (2, 0, 1)),
            "cam_wrist": np.transpose(self.arr_wrist_camera.copy(), (2, 0, 1)),
        }

    capture_observation = get_observation

    def send_action(self, action: np.ndarray | dict) -> np.ndarray:
            """Writes command straight to shared RAM for ROS 2 node."""
            # extract array if action is passed as a dictionary
            if isinstance(action, dict):
                if not action:
                    return self.arr_action.copy()

                if "action" in action:
                    action = action["action"]
                else:
                    action = next(iter(action.values()))

            if hasattr(action, "detach"):
                action = action.detach().cpu().numpy()

            # TODO: have actual logical way to send actions via teleop or controller
            action_np = np.asarray(action, dtype=np.float32).flatten()

            # 4. Copy directly to shared memory buffer
            np.copyto(self.arr_action, action_np)
            self.prev_action = action_np.copy()
            return action_np

    def disconnect(self):
            try:
                from multiprocessing.resource_tracker import unregister
            except ImportError:
                unregister = None

            shms = [self.shm_cam_overhead, self.shm_cam_wrist, self.shm_joint_state, self.shm_action]
            for shm in shms:
                if shm is not None:
                    if unregister is not None:
                        unregister(shm._name, "shared_memory")
                    try:
                        shm.close()
                    except Exception:
                        pass

            self.shm_cam_overhead = None
            self.shm_cam_wrist = None
            self.shm_joint_state = None
            self.shm_action = None
            print("Disconnected cleanly from Shared Memory.")

    def __del__(self):
        self.disconnect()
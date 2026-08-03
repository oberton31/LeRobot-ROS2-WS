from dataclasses import dataclass
import time
import numpy as np
from pynput import keyboard

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from multiprocessing.shared_memory import SharedMemory

from lerobot.types import RobotAction


@dataclass
class KeyboardTeleopConfig(TeleoperatorConfig):
    type: str = "keyboard_gazebo"

    joint_velocity: float = 0.5
    gripper_velocity: float = 0.1
    hz: float = 30.0
    fine_control_scale: float = 0.2


class KeyboardTeleop(Teleoperator):
    name = "keyboard_gazebo"
    config_class = KeyboardTeleopConfig

    def __init__(self, config: KeyboardTeleopConfig):
        super().__init__(config)
        self.config = config

        self.dt = 1.0 / config.hz

        # currently pressed keys
        self.pressed = set()
        
        # u/j -> joint 1
        # i/k -> joint 2
        # o/l -> joint 3
        # p/; -> joint 4
        # [/'] -> joint 5
        # ./, -> joint 6
        self.mapping = {
            "u": (0, +1),
            "j": (0, -1),

            "i": (1, +1),
            "k": (1, -1),

            "o": (2, +1),
            "l": (2, -1),

            "p": (3, +1),
            ";": (3, -1),

            "[": (4, +1),
            "'": (4, -1),

            "m": (5, +1),
            ",": (5, -1),
        }

        # replace these with your robot's actual joint limits
        self.lower_limits = np.array([
            -3.14,
            -1.57,
            -2.50,
            -3.14,
            -3.14,
             0.00,
        ], dtype=np.float32)

        self.upper_limits = np.array([
             3.14,
             1.57,
             2.50,
             3.14,
             3.14,
             0.04,
        ], dtype=np.float32)

        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        
        self.shm_reset = SharedMemory(name="shm_reset", create=False)
        self.arr_reset = np.ndarray((1,), dtype=np.uint8, buffer=self.shm_reset.buf)
        
        self.shm_joint_state = SharedMemory(name="shm_joint_state", create=False)
        self.joint_shape = (6,)
        self.arr_joint_state = np.ndarray(self.joint_shape, dtype=np.float32, buffer=self.shm_joint_state.buf)
        self.target = self.arr_joint_state.copy()
        
        self._last_print = self.target.copy()


    # keyboard callbacks
    def _normalize_key(self, key):
        # Helper to convert both char and special Key objects into standard string representation.
        if hasattr(key, "char") and key.char is not None:
            return key.char.lower()
        if hasattr(key, "name"):
            return key.name.lower()
        # Fallback for raw Key enum values or uncommon modifiers
        return str(key).replace("Key.", "").lower()

    def _on_press(self, key):
        normalized = self._normalize_key(key)
        self.pressed.add(normalized)

    def _on_release(self, key):
        normalized = self._normalize_key(key)
        self.pressed.discard(normalized)

    # required properties
    @property
    def is_connected(self):
        return True

    @property
    def action_features(self):
        return {
            "action": {
                "dtype": "float32",
                "shape": (6,),
            }
        }

    @property
    def feedback_features(self):
        return {}

    @property
    def is_calibrated(self):
        return True

    # required methods
    def connect(self):
        self.listener.start()

        print(
    """
    ============================================================
    Keyboard Teleoperation Enabled

    Joint Controls
    --------------
    Joint 1 : u / j
    Joint 2 : i / k
    Joint 3 : o / l
    Joint 4 : p / ;
    Joint 5 : [ / '
    Gripper : m / ,

    Other Controls
    --------------
    SHIFT  : Fine control (20% speed)
    SPACE  : Reset (IMPORTANT)

    LeRobot Recording Controls
    --------------------------
    RIGHT / LEFT : select episode action
    n            : accept and move to next episode
    r            : re-record current episode
    q            : quit recording
    ESC          : stop

    Robot holds its current commanded position whenever no keys
    are pressed.
    ============================================================
    """
        )

    def disconnect(self):
        if self.listener.is_alive():
            self.listener.stop()

    def calibrate(self):
        pass

    def configure(self):
        pass

    def send_feedback(self, feedback):
        pass

    # stateful teleoperation
    def get_action(self):

        # reset to home
        if "space" in self.pressed:
            self.pressed.discard("space")
            self._reset()

        joint_velocity = self.config.joint_velocity
        gripper_velocity = self.config.gripper_velocity

        # fine control
        if (
            keyboard.Key.shift in self.pressed
            or keyboard.Key.shift_l in self.pressed
            or keyboard.Key.shift_r in self.pressed
        ):
            joint_velocity *= self.config.fine_control_scale
            gripper_velocity *= self.config.fine_control_scale

        # integrate joint velocities
        for key, (joint, direction) in self.mapping.items():
            if key in self.pressed:
                if joint == 5:
                    self.target[joint] += direction * gripper_velocity * self.dt
                else:
                    self.target[joint] += direction * joint_velocity * self.dt

        # enforce joint limits
        np.clip(
            self.target,
            self.lower_limits,
            self.upper_limits,
            out=self.target,
        )

        # print only when changed
        if not np.allclose(self.target, self._last_print):
            print(
                f"\rTarget: {np.round(self.target, 3)}",
                end="",
                flush=True,
            )
            self._last_print[:] = self.target

        return RobotAction(
            {
                "action": self.target.copy()
            }
        )

    def _reset(self):
        print("resetting gazebo scene")

        self.arr_reset[0] = 1

        # wait until bridge consumes request
        while self.arr_reset[0] != 0:
            time.sleep(0.01)

#!/usr/bin/env python3
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
import yasmin
from yasmin import State, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
import time

from .utils import RobotController


class StateA(State):
    def __init__(self, node):
        super().__init__(outcomes=["outcome1", "outcome99"])
        self.node = node
        self.robot = RobotController(node)
        self.prev_success = False

    def execute(self, blackboard):
        if not self.prev_success:
            self.prev_success = self.robot.goto([0.20, 0.0, 0.15], [0.0, 0.78, 0.0])
            print(
                f"Position: {self.robot.get_current_position()}, Orientation: {self.robot.get_current_orientation()}"
            )
        if self.prev_success:
            return "outcome99"
        return "outcome1"


def main():
    rclpy.init()

    node = rclpy.create_node("lerobot_state_machine_node")
    set_ros_loggers()

    # Spin ROS2 node in other thread
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    sm = StateMachine(outcomes=["outcome99"])
    sm.add_state("STATE_A", StateA(node), transitions={"outcome1": "STATE_A"})

    viewer_pub = YasminViewerPub("lerobot_state_machine", sm, 1.0, node)

    try:
        outcome = sm()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down state machine...")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)
    finally:
        viewer_pub.shutdown()
        executor.shutdown()
        node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

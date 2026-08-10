#!/usr/bin/env python3
import rclpy
from yasmin import State, StateMachine
import yasmin
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

class StateA(State):
    def __init__(self, node):
        # Must pass a list or set of strings
        super().__init__(outcomes=["outcome1"])
        self.node = node

    def execute(self, blackboard):
        # Your state logic here
        return "outcome1"

def main():
    rclpy.init()

    node = rclpy.create_node("lerobot_state_machine_node") # make a default ROS2 node. Later in the states, can add subscriptions, publishers, etc. just like a normal node
    set_ros_loggers()
    sm = StateMachine(outcomes=["outcome99"]) # no outcome as loop

    sm.add_state("STATE_A", StateA(node), transitions={"outcome1": "STATE_A"})

    viewer_pub = YasminViewerPub("lerobot_state_machine", sm)

    # Execute State Machine
    try:
        outcome = sm()
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)
    finally:
        viewer_pub.shutdown()
        
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
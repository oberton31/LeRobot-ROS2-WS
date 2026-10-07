import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController


def switch_controllers(
    node: Node,
    controller_type: str,
) -> bool:
    """Switch active and inactive controllers via /controller_manager/switch_controller. """
    
    client = node.create_client(SwitchController, "/controller_manager/switch_controller")

    strictness = SwitchController.Request.BEST_EFFORT
    timeout_sec = 5.0

    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().error("Controller manager switch service not available.")
        return False
    
    if controller_type == "streaming":
        deactivate_controllers = ["arm_trajectory_controller", "gripper_trajectory_controller"]
        activate_controllers = ["arm_streaming_controller", "gripper_streaming_controller"]
    elif controller_type == "trajectory":
        deactivate_controllers = ["arm_streaming_controller", "gripper_streaming_controller"]
        activate_controllers = ["arm_trajectory_controller", "gripper_trajectory_controller"]
    else:
        node.get_logger().error(f"Unknown controller type: {controller_type}")
        return False

    req = SwitchController.Request()
    req.activate_controllers = activate_controllers
    req.deactivate_controllers = deactivate_controllers
    req.strictness = strictness

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if future.done():
        response = future.result()
        if response is not None and response.ok:
            node.get_logger().info(
                f"Switched: activated {activate_controllers}, deactivated {deactivate_controllers}"
            )
            return True
        else:
            node.get_logger().error("Failed to switch controllers.")
            return False
    else:
        node.get_logger().error("Service call timed out.")
        return False
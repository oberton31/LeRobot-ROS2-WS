from platform import node

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    OrientationConstraint,
    PositionConstraint,
    BoundingVolume,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene

from shape_msgs.msg import SolidPrimitive, Plane
import tf_transformations

# NOTE TO SELF: NO BLOCKING CALLS INSIDE CALLBACKS, AND NO CALLING RCLPY SPIN INSIDE CALLBACKS


class RobotController:
    """Class to manage a LeRobot"""

    def __init__(self, node: Node):
        self.node = node
        self.moveit_action_client = ActionClient(self.node, MoveGroup, "/move_action")
        self.collision_scene_client = self.node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._wait_for_clients()

        self._add_collision_objects(
            boxes=[], planes=[("table", [0.0, 0.0, -0.1], [0.0, 0.0, 1.0])]
        )

        self.current_controller_type = None

    def _wait_for_clients(self):
        """Wait for the action and service clients to be available."""
        while not self.moveit_action_client.wait_for_server(
            timeout_sec=5.0
        ) and not self.collision_scene_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().warn("Waiting for clients to become available...")

    # TODO: getting MoveIt error when running this. Al
    def goto(self, position: list, orientation: list) -> bool:
        """Send a position command to the robot. Does not handle the end effector gripper"""
        if len(position) != 3 and len(orientation) != 3:
            self.node.get_logger().error(
                "Position and orientation must be lists of 3 values each. Expect [x, y, z] and [roll, pitch, yaw]"
            )
            return False

        if self.current_controller_type != "trajectory":
            if not self._switch_controllers("trajectory"):
                return False
            self.current_controller_type = "trajectory"

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        target_pose = PoseStamped()
        target_pose.header.frame_id = "base"
        target_pose.pose.position.x = position[0]
        target_pose.pose.position.y = position[1]
        target_pose.pose.position.z = position[2]

        # Convert roll, pitch, yaw to quaternion
        quaternion = tf_transformations.quaternion_from_euler(
            orientation[0], orientation[1], orientation[2]
        )
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]

        # Set the goal constraint (i.e. tolerance for target pose)
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base"
        position_constraint.link_name = "gripper"
        position_constraint.weight = 1.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]  # Tolerance in m

        volume = BoundingVolume()
        volume.primitives.append(primitive)
        volume.primitive_poses.append(target_pose.pose)
        position_constraint.constraint_region = volume

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base"
        orientation_constraint.link_name = "gripper"
        orientation_constraint.orientation = target_pose.pose.orientation

        # Tolerances in rad
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        self.node.get_logger().info("Sending goal...")
        send_goal_future = self.moveit_action_client.send_goal_async(goal_msg)

        timeout_sec = 5.0
        rclpy.spin_until_future_complete(
            self.node, send_goal_future, timeout_sec=timeout_sec
        )

        try:
            goal_handle = send_goal_future.result()
        except Exception as e:
            self.node.get_logger().error(
                f"Failed to communicate with action server: {e}"
            )
            return False

        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("Goal rejected by server or timed out.")
            return False

        self.node.get_logger().info("Goal accepted, waiting for result...")

        future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        try:
            result_response = future.result()
        except Exception as e:
            self.node.get_logger().error(f"Error while waiting for result: {e}")
            return False

        if result_response is None:
            self.node.get_logger().error(
                f"Goal execution timed out after {timeout_sec}s."
            )
            return False

        result = result_response.result
        if result.error_code.val == result.error_code.SUCCESS:
            self.node.get_logger().info("Goal succeeded!")
        else:
            self.node.get_logger().error(
                f"Goal failed with error code: {result.error_code.val}"
            )
            return False

        return True

    def goto_gripper(self, position: float) -> bool:
        pass

    def open_gripper(self) -> bool:
        pass

    def close_gripper(self) -> bool:
        pass

    def goto_streaming(self, position: list) -> bool:
        """Streaming including gripper. Position is a list of 7 values: [x, y, z, roll, pitch, yaw, gripper_position]"""
        pass

    def _add_collision_objects(self, boxes: list, planes: list) -> bool:
        """Add collision objects to the planning scene."""

        if len(boxes) == 0 and len(planes) == 0:
            self.node.get_logger().warn("No collision objects to add.")
            return True

        planning_scene = PlanningScene()

        for box in boxes:
            name, position, size = box
            box_shape, box_pose = self._create_collision_box(name, position, size)
            if not box_shape or not box_pose:
                return False

            collision = CollisionObject()
            collision.id = name
            collision.header.frame_id = "base"
            collision.primitives.append(box_shape)
            collision.primitive_poses.append(box_pose)
            collision.operation = CollisionObject.ADD
            planning_scene.world.collision_objects.append(collision)

        for plane in planes:
            name, position, normal = plane
            plane_shape, plane_pose = self._create_collision_plane(
                name, position, normal
            )
            if not plane_shape or not plane_pose:
                return False

            collision = CollisionObject()
            collision.id = name
            collision.header.frame_id = "base"
            collision.planes.append(plane_shape)
            collision.plane_poses.append(plane_pose)
            collision.operation = CollisionObject.ADD
            planning_scene.world.collision_objects.append(collision)

        planning_scene.is_diff = True  # Only apply changes
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene

        future = self.collision_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(f"Error while waiting for result: {e}")
            return False

        if not result.success:
            self.node.get_logger().error("Failed to add collision objects.")
            return False
        return True

    def _create_collision_box(self, name: str, position: list, size: list) -> bool:
        """Create a collision box in the planning scene."""
        if len(position) != 3 or len(size) != 3:
            self.node.get_logger().error(
                "Position and size must be lists of 3 values. Expect [x, y, z]"
            )
            return False

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size  # [x_size, y_size, z_size]

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0

        return (box, pose)

    def _create_collision_plane(self, name: str, position: list, normal: list) -> bool:
        """Create a collision plane in the planning scene."""
        if len(position) != 3 or len(normal) != 3:
            self.node.get_logger().error(
                "Position and normal must be lists of 3 values."
            )
            return False

        plane = Plane()

        plane.coef = [
            normal[0],
            normal[1],
            normal[2],
            -sum(a * b for a, b in zip(position, normal)),
        ]

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0

        return (plane, pose)

    def _switch_controllers(
        self,
        controller_type: str,
    ) -> bool:
        """Switch active and inactive controllers via /controller_manager/switch_controller."""

        client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        strictness = SwitchController.Request.BEST_EFFORT
        timeout_sec = 5.0

        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error(
                "Controller manager switch service not available."
            )
            return False

        if controller_type == "streaming":
            deactivate_controllers = [
                "arm_trajectory_controller",
                "gripper_trajectory_controller",
            ]
            activate_controllers = [
                "arm_streaming_controller",
                "gripper_streaming_controller",
            ]
        elif controller_type == "trajectory":
            deactivate_controllers = [
                "arm_streaming_controller",
                "gripper_streaming_controller",
            ]
            activate_controllers = [
                "arm_trajectory_controller",
                "gripper_trajectory_controller",
            ]
        else:
            self.node.get_logger().error(f"Unknown controller type: {controller_type}")
            return False

        req = SwitchController.Request()
        req.activate_controllers = activate_controllers
        req.deactivate_controllers = deactivate_controllers
        req.strictness = strictness

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        if future.done():
            response = future.result()
            if response is not None and response.ok:
                self.node.get_logger().info(
                    f"Switched: activated {activate_controllers}, deactivated {deactivate_controllers}"
                )
                return True
            else:
                self.node.get_logger().error("Failed to switch controllers.")
                return False
        else:
            self.node.get_logger().error("Service call timed out.")
            return False

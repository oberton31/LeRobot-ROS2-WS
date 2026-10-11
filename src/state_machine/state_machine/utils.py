import time
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
)

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from shape_msgs.msg import SolidPrimitive, Plane
import tf_transformations


class RobotController:
    """Class to manage a LeRobot"""

    def __init__(self, node: Node):
        self.node = node
        self.moveit_action_client = ActionClient(self.node, MoveGroup, "/move_action")
        self.collision_object_publisher = self.node.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self._wait_for_clients()

        self._add_collision_objects(
            boxes=[], planes=[("table", [0.0, 0.0, -0.1], [0.0, 0.0, 1.0])]
        )

        self.current_controller_type = None

    def _wait_for_future(self, future, timeout_sec: float = 10.0):
        """Block the caller thread until the future is done, leaving the ROS executor free."""
        start_time = time.monotonic()
        while not future.done():
            if time.monotonic() - start_time > timeout_sec:
                return None
            time.sleep(0.01)
        return future.result()

    def _wait_for_clients(self):
        """Wait for the action and service clients to be available."""
        while not (self.moveit_action_client.wait_for_server(timeout_sec=1.0)):
            self.node.get_logger().warn("Waiting for clients to become available...")

    def goto(self, position: list, orientation: list) -> bool:
        """Send a position command to the robot. Blocks until motion completes."""
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

        quaternion = tf_transformations.quaternion_from_euler(
            orientation[0], orientation[1], orientation[2]
        )
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]

        # Set goal position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base"
        position_constraint.link_name = "gripper"
        position_constraint.weight = 1.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.02, 0.02, 0.02]  # Tolerance: 2 cm

        volume = BoundingVolume()
        volume.primitives.append(primitive)
        volume.primitive_poses.append(target_pose.pose)
        position_constraint.constraint_region = volume

        # Set goal orientation constraint -> TODO So1010 is 5DOF so cannot reach all orientations: keep this off for now
        # TODO: looks like there is a way to set orientation constraints in a way that allows for 5DOF, but needs more careful engineering
        # orientation_constraint = OrientationConstraint()
        # orientation_constraint.header.frame_id = "base"
        # orientation_constraint.link_name = "gripper"
        # orientation_constraint.orientation = target_pose.pose.orientation
        # orientation_constraint.absolute_x_axis_tolerance = 0.05  # ~2.8 deg
        # orientation_constraint.absolute_y_axis_tolerance = 0.05
        # orientation_constraint.absolute_z_axis_tolerance = 0.05
        # orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        # constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        self.node.get_logger().info("Sending goal...")
        send_goal_future = self.moveit_action_client.send_goal_async(goal_msg)

        # Block until goal is accepted/rejected
        goal_handle = self._wait_for_future(send_goal_future, timeout_sec=5.0)

        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("Goal rejected by server or timed out.")
            return False

        self.node.get_logger().info("Goal accepted, waiting for result...")

        # Block until trajectory execution finishes (30s allows for planning + motion)
        result_future = goal_handle.get_result_async()
        result_response = self._wait_for_future(result_future, timeout_sec=30.0)

        if result_response is None:
            self.node.get_logger().error("Goal execution timed out.")
            return False

        result = result_response.result
        if result.error_code.val == result.error_code.SUCCESS:
            self.node.get_logger().info("Goal succeeded!")
            return True
        else:
            self.node.get_logger().error(
                f"Goal failed with error code: {result.error_code.val}"
            )
            return False

    def goto_gripper(self, position: float) -> bool:
        pass

    def open_gripper(self) -> bool:
        pass

    def close_gripper(self) -> bool:
        pass

    def goto_streaming(self, position: list) -> bool:
        """Streaming including gripper. Position is a list of 7 values: [x, y, z, roll, pitch, yaw, gripper_position]"""
        pass

    def get_current_pose(
        self, target_frame: str = "base", source_frame: str = "gripper"
    ) -> Pose:
        """Get the current pose of the gripper relative to the base frame."""
        try:
            # Time() with zero time value requests the latest available transform
            transform_stamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            pose = Pose()
            pose.position.x = transform_stamped.transform.translation.x
            pose.position.y = transform_stamped.transform.translation.y
            pose.position.z = transform_stamped.transform.translation.z
            pose.orientation = transform_stamped.transform.rotation
            return pose

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().error(
                f"Failed to lookup transform from {target_frame} to {source_frame}: {e}"
            )
            return None

    def get_current_position(self) -> list:
        """Helper to get just [x, y, z] of the gripper in meters."""
        pose = self.get_current_pose()
        if pose is not None:
            return [pose.position.x, pose.position.y, pose.position.z]
        return None

    def get_current_orientation(self) -> list:
        """Helper to get just [roll, pitch, yaw] of the gripper in radians."""
        pose = self.get_current_pose()
        if pose is not None:
            quaternion = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            euler = tf_transformations.euler_from_quaternion(quaternion)
            return list(euler)
        return None

    def get_current_position(self) -> list:
        """Helper to get just [x, y, z] of the gripper in meters."""
        pose = self.get_current_pose()
        if pose is not None:
            return [pose.position.x, pose.position.y, pose.position.z]
        return None

    def _add_collision_objects(self, boxes: list, planes: list) -> bool:
        """Add collision objects to the planning scene."""
        if len(boxes) == 0 and len(planes) == 0:
            self.node.get_logger().warn("No collision objects to add.")
            return True

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
            self.collision_object_publisher.publish(collision)

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
            self.collision_object_publisher.publish(collision)

    def _create_collision_box(self, name: str, position: list, size: list):
        if len(position) != 3 or len(size) != 3:
            self.node.get_logger().error(
                "Position and size must be lists of 3 values. Expect [x, y, z]"
            )
            return None, None

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0

        return box, pose

    def _create_collision_plane(self, name: str, position: list, normal: list):
        if len(position) != 3 or len(normal) != 3:
            self.node.get_logger().error(
                "Position and normal must be lists of 3 values."
            )
            return None, None

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

        return plane, pose

    def _switch_controllers(self, controller_type: str) -> bool:
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
        response = self._wait_for_future(future, timeout_sec=timeout_sec)

        if response is not None and response.ok:
            self.node.get_logger().info(
                f"Switched: activated {activate_controllers}, deactivated {deactivate_controllers}"
            )
            return True
        else:
            self.node.get_logger().error("Failed to switch controllers.")
            return False

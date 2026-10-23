#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler


class TablePosition(str, Enum):
    CC = "cc"
    PLAYER_1 = "player1"
    PLAYER_2 = "player2"


@dataclass
class TableCoordinates:
    x: float
    y: float
    yaw_degrees: float


class RX200DealerClientNode(Node):
    """
    Bounded RX200 blackjack pick-and-place sequence.

    IMPORTANT:
    - Workspace values below are examples and must be calibrated/validated
      against the real robot, table, base frame, gripper geometry, and obstacles.
    - Joint/controller limits must also be verified in the installed robot config.
    - Software checks are not a substitute for the lab-approved emergency stop.
    """

    def __init__(self):
        super().__init__("rx200_dealer")
        self.get_logger().info("Dealer started")

        # ----- Motion safety parameters -----
        self.declare_parameter("x_min", -0.35)
        self.declare_parameter("x_max", 0.35)
        self.declare_parameter("y_min", -0.35)
        self.declare_parameter("y_max", 0.35)
        self.declare_parameter("z_min", 0.05)
        self.declare_parameter("z_max", 0.65)

        # Allowed Euler-angle ranges (degrees). These limit requested poses,
        self.declare_parameter("roll_min_deg", -10.0)
        self.declare_parameter("roll_max_deg", 10.0)
        self.declare_parameter("pitch_min_deg", -90.0)
        self.declare_parameter("pitch_max_deg", 90.0)
        self.declare_parameter("yaw_min_deg", -180.0)
        self.declare_parameter("yaw_max_deg", 180.0)

        self.declare_parameter("step_delay", 0.5)
        self.declare_parameter("goal_timeout_sec", 30.0)

        self.arm_group = "interbotix_arm"
        self.gripper_group = "interbotix_gripper"
        self.ee_link = "rx200/ee_gripper_link"
        self.base_link = "rx200/base_link"
        self.gripper_joint = "left_finger"

        self.step_delay = float(self.get_parameter("step_delay").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)

        if self.step_delay < 0.0 or self.goal_timeout_sec <= 0.0:
            raise ValueError("step_delay must be >= 0 and goal_timeout_sec must be > 0")

        self.table_coords = {
            TablePosition.CC: TableCoordinates(0.3, 0.0, 0.0),
            TablePosition.PLAYER_1: TableCoordinates(0.0, 0.3, 90.0),
            TablePosition.PLAYER_2: TableCoordinates(-0.3, 0.0, 180.0),
        }

        self.declare_parameter("start_position", value=TablePosition.CC.value)
        self.declare_parameter("end_position", value=TablePosition.PLAYER_1.value)

        self._busy = False
        self._faulted = False
        self._active_goal_handle = None
        self._goal_timer = None
        self._wait_timer = None
        self._goal_started_at = None
        self._pending_sequence = []

        try:
            start = TablePosition(self.get_parameter("start_position").value)
            end = TablePosition(self.get_parameter("end_position").value)
            self.sequence = self._start_and_end_positions_to_sequence(start, end)
        except (ValueError, KeyError) as exc:
            self.get_logger().fatal(
                f"Invalid start/end position. No motion will be sent: {exc}"
            )
            self._faulted = True
            self.sequence = []

        self.step = 0

        if not self._faulted:
            try:
                self._validate_sequence_positions()
            except ValueError as exc:
                self.get_logger().fatal(
                    f"Configured sequence is outside the allowed workspace. "
                    f"No motion will be sent: {exc}"
                )
                self._faulted = True

        if not self._faulted:
            self._client = ActionClient(self, MoveGroup, "/move_action")
            self._server_wait_timer = self.create_timer(
                1.0, self._check_action_server
            )
            self.get_logger().warning(
                "Workspace checks are enabled, but configured bounds and poses "
                "must be physically validated before use."
            )
        else:
            self._client = None

    def _check_action_server(self):
        if self._faulted:
            self._server_wait_timer.cancel()
            self.destroy_timer(self._server_wait_timer)
            return

        if self._client.server_is_ready():
            self._server_wait_timer.cancel()
            self.destroy_timer(self._server_wait_timer)
            self.get_logger().info("MoveIt action server available.")
            self._send_next()
        else:
            self.get_logger().warning("Waiting for /move_action server...")

    # ---------- Workspace / input validation ----------

    def _limit(self, name):
        return float(self.get_parameter(name).value)

    def validate_position(self, x, y, z):
        values = (x, y, z)
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError("Position contains NaN or infinity")

        bounds = (
            ("x", float(x), self._limit("x_min"), self._limit("x_max")),
            ("y", float(y), self._limit("y_min"), self._limit("y_max")),
            ("z", float(z), self._limit("z_min"), self._limit("z_max")),
        )

        for axis, value, lower, upper in bounds:
            if lower > upper:
                raise ValueError(f"Invalid configuration: {axis}_min > {axis}_max")
            if not lower <= value <= upper:
                raise ValueError(
                    f"{axis.upper()}={value:.3f} m is outside "
                    f"[{lower:.3f}, {upper:.3f}] m"
                )

    def validate_orientation(self, roll_degrees, pitch_degrees, yaw_degrees):
        angles = (
            ("roll", float(roll_degrees),
             self._limit("roll_min_deg"), self._limit("roll_max_deg")),
            ("pitch", float(pitch_degrees),
             self._limit("pitch_min_deg"), self._limit("pitch_max_deg")),
            ("yaw", float(yaw_degrees),
             self._limit("yaw_min_deg"), self._limit("yaw_max_deg")),
        )

        for name, value, lower, upper in angles:
            if not math.isfinite(value):
                raise ValueError(f"{name} angle is NaN or infinity")
            if lower > upper:
                raise ValueError(
                    f"Invalid configuration: {name}_min_deg > {name}_max_deg"
                )
            if not lower <= value <= upper:
                raise ValueError(
                    f"{name}={value:.1f} degrees is outside "
                    f"[{lower:.1f}, {upper:.1f}] degrees"
                )

    def _validate_sequence_positions(self):
        for start_or_end in (
            TablePosition(self.get_parameter("start_position").value),
            TablePosition(self.get_parameter("end_position").value),
        ):
            coords = self.table_coords[start_or_end]
            # Above-table pose
            self.validate_position(coords.x, coords.y, 0.3)
            self.validate_orientation(0.0, 45.0, coords.yaw_degrees)
            # On-table pose
            self.validate_position(coords.x, coords.y, 0.07)
            self.validate_orientation(0.0, 90.0, coords.yaw_degrees)

        # Validate home pose.
        self.validate_position(0.0, 0.0, 0.6)
        self.validate_orientation(0.0, -45.0, 0.0)

    # ---------- Sequence construction ----------

    def _start_and_end_positions_to_sequence(self, start, end):
        if start not in self.table_coords or end not in self.table_coords:
            raise ValueError("Start/end must be one of: cc, player1, player2")

        return [
            ("home", self.home_pose),
            ("open gripper", lambda: self.send_gr_pose(True)),
            ("above pickup", self._table_coordinates_to_above_table_pose(
                self.table_coords[start])),
            ("descend to pickup", self._table_coordinates_to_on_table_pose(
                self.table_coords[start])),
            ("close gripper", lambda: self.send_gr_pose(False)),
            ("lift from pickup", self._table_coordinates_to_above_table_pose(
                self.table_coords[start])),
            ("above destination", self._table_coordinates_to_above_table_pose(
                self.table_coords[end])),
            ("descend to destination", self._table_coordinates_to_on_table_pose(
                self.table_coords[end])),
            ("release block", lambda: self.send_gr_pose(True)),
            ("retreat from destination", self._table_coordinates_to_above_table_pose(
                self.table_coords[end])),
        ]

    def home_pose(self):
        return self.send_ee_readable_pose(
            x=0.0, y=0.0, z=0.6, pitch_degrees=-45.0
        )

    def _table_coordinates_to_on_table_pose(self, coords):
        return lambda: self.send_ee_readable_pose(
            x=coords.x,
            y=coords.y,
            z=0.07,
            yaw_degrees=coords.yaw_degrees,
            pitch_degrees=90.0,
        )

    def _table_coordinates_to_above_table_pose(self, coords):
        return lambda: self.send_ee_readable_pose(
            x=coords.x,
            y=coords.y,
            z=0.3,
            yaw_degrees=coords.yaw_degrees,
            pitch_degrees=45.0,
        )

    # ---------- Sequence execution ----------

    def _send_next(self):
        if self._faulted:
            self.get_logger().error("FAULT state: sequence is stopped.")
            return
        if self._busy:
            self.get_logger().warning("A goal is already active; ignoring next step.")
            return
        if self.step >= len(self.sequence):
            self.get_logger().info("Sequence complete.")
            return

        step_name, callback = self.sequence[self.step]
        self.get_logger().info(
            f"Sending step {self.step + 1}/{len(self.sequence)}: {step_name}"
        )

        try:
            submitted = callback()
        except Exception as exc:
            self._enter_fault(f"Step '{step_name}' raised an exception: {exc}")
            return

        if submitted is False:
            self._enter_fault(f"Step '{step_name}' was rejected by local validation.")
            return

        self._busy = True
        self._current_step_name = step_name
        self._goal_started_at = time.monotonic()
        self._goal_timer = self.create_timer(0.25, self._check_goal_timeout)
        self.step += 1

    def _check_goal_timeout(self):
        if not self._busy or self._goal_started_at is None:
            self._cancel_timer(self._goal_timer)
            return

        elapsed = time.monotonic() - self._goal_started_at
        if elapsed > self.goal_timeout_sec:
            self._enter_fault(
                f"Timeout during '{self._current_step_name}' after "
                f"{self.goal_timeout_sec:.1f} seconds. No next step will be sent."
            )

    def _enter_fault(self, reason):
        if self._faulted:
            return

        self._faulted = True
        self._busy = False
        self.get_logger().fatal(f"FAULT: {reason}")
        self.get_logger().fatal(
            "Automatic sequence stopped. Assess the robot and use the lab-approved "
            "stop/recovery procedure. This software fault handler is not an E-stop."
        )
        self._cancel_timer(self._goal_timer)
        self._cancel_timer(self._wait_timer)

        if self._active_goal_handle is not None:
            try:
                self._active_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(f"Could not request goal cancellation: {exc}")

    def _cancel_timer(self, timer):
        if timer is not None:
            try:
                timer.cancel()
                self.destroy_timer(timer)
            except Exception:
                pass

    def _goal_response_cb(self, future):
        if self._faulted:
            return

        try:
            handle_goal = future.result()
        except Exception as exc:
            self._enter_fault(f"Could not receive goal response: {exc}")
            return

        if not handle_goal.accepted:
            self._enter_fault(f"MoveIt rejected step '{self._current_step_name}'.")
            return

        self._active_goal_handle = handle_goal
        self.get_logger().info(f"Goal accepted: {self._current_step_name}")
        handle_goal.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        if self._faulted:
            return
        state = feedback_msg.feedback.state
        self.get_logger().info(f"MoveIt state: {state}")

    def _result_cb(self, future):
        if self._faulted:
            return

        self._cancel_timer(self._goal_timer)
        self._goal_timer = None
        self._goal_started_at = None

        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            status = wrapped_result.status
        except Exception as exc:
            self._enter_fault(f"Could not read action result: {exc}")
            return

        code = result.error_code.val
        self.get_logger().info(
            f"Step '{self._current_step_name}' finished with "
            f"action status={status}, MoveIt error code={code}"
        )

        if status != 4 or code != MoveItErrorCodes.SUCCESS:
            self._enter_fault(
                f"Step '{self._current_step_name}' did not complete successfully."
            )
            return

        self._active_goal_handle = None
        self._busy = False
        self._wait_timer = self.create_timer(
            self.step_delay, self._delayed_next
        )

    def _delayed_next(self):
        self._cancel_timer(self._wait_timer)
        self._wait_timer = None
        if not self._faulted:
            self._send_next()

    # ---------- Motion requests ----------

    def send_ee_readable_pose(
        self, x, y, z,
        roll_degrees=0.0,
        pitch_degrees=0.0,
        yaw_degrees=0.0,
    ):
        """Convert validated human-readable Euler angles to a quaternion."""
        try:
            self.validate_position(x, y, z)
            self.validate_orientation(
                roll_degrees, pitch_degrees, yaw_degrees
            )
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"Motion rejected locally: {exc}")
            return False

        q = quaternion_from_euler(
            math.radians(roll_degrees),
            math.radians(pitch_degrees),
            math.radians(yaw_degrees),
        )
        return self.send_ee_pose(
            x=x, y=y, z=z, qx=q[0], qy=q[1], qz=q[2], qw=q[3]
        )

    def send_ee_pose(
        self, x, y, z,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0,
    ):
        if self._faulted:
            return False

        p = PoseStamped()
        p.header.frame_id = self.base_link
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        p.pose.orientation = Quaternion(
            x=float(qx), y=float(qy), z=float(qz), w=float(qw)
        )

        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.allowed_planning_time = 10.0
        req.num_planning_attempts = 3

        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [p.pose]

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = p.pose.orientation
        oc.absolute_x_axis_tolerance = 0.005
        oc.absolute_y_axis_tolerance = 0.005
        oc.absolute_z_axis_tolerance = 0.005
        oc.weight = 1.0

        gc = Constraints()
        gc.position_constraints = [pc]
        gc.orientation_constraints = [oc]
        req.goal_constraints = [gc]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.look_around = False

        future = self._client.send_goal_async(goal, self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)
        return True

    def send_gr_pose(self, open=True):
        if self._faulted:
            return False

        req = MotionPlanRequest()
        req.group_name = self.gripper_group
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint  # left_finger
        jc.position = 0.035 if open else 0.0238
        jc.tolerance_above = 0.0
        jc.tolerance_below = 0.005
        jc.weight = 1.0

        gc = Constraints()
        gc.joint_constraints = [jc]
        req.goal_constraints = [gc]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RX200DealerClientNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"RX200 dealer stopped during startup: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

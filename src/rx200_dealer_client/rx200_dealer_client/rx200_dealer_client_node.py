#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from enum import Enum
from rx200_dealer_interfaces.srv import PickAndPlaceCard, PointToPosition

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

PICK_AND_PLACE_SRV_NAME: str = 'pick_and_place_card'
POINT_TO_POSITION_SRV_NAME: str = 'point_to_position'

@dataclass  
class TableCoordinates:
    # Container for an (x, y) position on the table (in meters), in the robot's base_link frame, which come from the user input
    x: float
    y: float

    def get_yaw_degrees(self):
        # The direction from the robot's base_link to this point on the table. The yaw must face straight down at the target
        return math.degrees(math.atan2(self.y, self.x))


class RX200DealerClientNode(Node):
    def __init__(self):
        super().__init__("rx200_dealer") # Registers this node with ROS2 as "rx200_dealer"
        self._client = ActionClient(self, MoveGroup, "/move_action") # Client for MoveIt's planning and execution actions
        self.get_logger().info("Dealer started")
        while not self._client.wait_for_server(1.0): # Blocks until MoveIt is running
            self.get_logger().warning("Action server not up")

        # Motion safety and timing parameters: x, y, and z limits define the safe workspace box (in meters), roll, pitch and yaw limits defne the allowed orientations (in degrees) and the stop_delay is the pause between steps
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

        # MoveIt's uses including planning groups (arm and gripper), the gripper's tip link, the base frame and the left finger joint (right finger mirrors left finger automatically)
        self.arm_group = "interbotix_arm"
        self.gripper_group = "interbotix_gripper"
        self.ee_link = "rx200/ee_gripper_link"
        self.base_link = "rx200/base_link"
        self.gripper_joint = "left_finger"

        # Reads the timing parameters and refuses to start if they are invalid
        self.step_delay = float(self.get_parameter("step_delay").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)

        if self.step_delay < 0.0 or self.goal_timeout_sec <= 0.0:
            raise ValueError("step_delay must be >= 0 and goal_timeout_sec must be > 0")

        self.arm_group = 'interbotix_arm'
        self.gripper_group = 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
        self.gripper_joint = 'left_finger'
        self.waist_joint = 'waist'

        # Upright home position as exact angles (in radians), recorded from simulated robot in 'Upright' pose from RViz in /joint_states
        self.home_joints = {
            'waist': 0.0,
            'shoulder': 0.0,
            'elbow': -1.57,
            'wrist_angle': 0.0,
            'wrist_rotate': 0.0,
        }
        self.home_pose = lambda: self.send_joint_pose(self.home_joints) # Updates for upright home position according to RViz coordinates
        self.step_delay = 0.5 # seconds to wait between steps

        # Registers both services with their relevant handler
        self.create_service(PickAndPlaceCard, PICK_AND_PLACE_SRV_NAME, self.handle_pick_and_place_card)
        self.create_service(PointToPosition, POINT_TO_POSITION_SRV_NAME, self.handle_point_to_position)

        # State variables
        self.step = 0 # Index of the next step in self.sequence
        self.sequence = [] # List of steps for the current request
        self._busy = False # True unitl the whole sequence finishes (including pauses between steps)
        self._goal_active = False # True while one single MoveIt goal is still in progress
        self._active_goal_handle = None # Hangle to the running goal (so it can be cancelled)
        self._goal_timer = None # Check the current step for timeout
        self._wait_timer = None # Pause between steps
        self._goal_started_at = None # When the current step was sent (checks for the timeout and updated when the step finishes)
        self._pending_sequence = []
        self._server_wait_timer = self.create_timer(
            1.0, self._check_action_server
        )

    # Called once a second by the timer, when MoveIt is ready, it stops the timer and logs
    def _check_action_server(self):
        if self._client.server_is_ready():
            self._server_wait_timer.cancel()
            self.destroy_timer(self._server_wait_timer)
            self.get_logger().info("MoveIt action server available.")
        else:
            self.get_logger().warning("Waiting for /move_action server...")

    def _limit(self, name):
        return float(self.get_parameter(name).value)

    def _validate_position(self, x, y, z):
        values = (x, y, z)
        if not all(math.isfinite(float(v)) for v in values): # Raise ValueError if (x, y, z) is NaN/infinite or outside the robot's workspace limit
            raise ValueError("Position contains NaN or infinity")

        # A list of limits for the robot's workspace, any coordinates entered by the user that are outside these limits will be rejected before the arm moves
        bounds = (
            ("x", float(x), self._limit("x_min"), self._limit("x_max")),
            ("y", float(y), self._limit("y_min"), self._limit("y_max")),
            ("z", float(z), self._limit("z_min"), self._limit("z_max")),
        )

        for axis, value, lower, upper in bounds:
            if lower > upper: # Catches a misconfiguration (min > max)
                raise ValueError(f"Invalid configuration: {axis}_min > {axis}_max") 
            if not lower <= value <= upper: # Catches a value outside it's limits
                # Stops the function and passes the error back so the handlers know to reject a request
                raise ValueError(
                    f"{axis.upper()}={value:.3f} m is outside "
                    f"[{lower:.3f}, {upper:.3f}] m"
                )

    # Follows the same pattern as _validate_position but instead for roll, pitch and yaw values 
    def _validate_orientation(self, roll_degrees, pitch_degrees, yaw_degrees):
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

    # Steps for pointing: Goes home, opens the gripper, lowers to grasp heigh over (x , y) position as entered by the user. The user then places the block between the open gripper
    def _point_sequence(self, coordinates: TableCoordinates):
        return [
            ("home", self.home_pose),
            ("open gripper", lambda: self.send_gr_pose(True)),
            ("point at position", self._table_coordinates_to_on_table_pose(coordinates)),
        ]

    # Service callback: rejects if busy or invalid, otherwise builds and starts the pick_and_place sequence
    def handle_pick_and_place_card(self, request, response):
        # Reject requests while another operation is running.
        if self._busy:
            response.accepted = False
            response.error = (
                "Robot is busy executing another pick-and-place operation. "
                "Please wait until it finishes."
            )
            self.get_logger().warning(response.error)
            return response

        start_coords = TableCoordinates(x=request.start_x, y=request.start_y)
        end_coords = TableCoordinates(x=request.end_x, y=request.end_y)

        # Prepare and reserve the operation before starting movement.
        try:
            self.sequence = self._start_and_end_positions_to_sequence(
                start_coords, end_coords
            )
        except ValueError as exc:
            response.accepted = False
            response.error = str(exc)
            self.get_logger().fatal(
                f"Configured sequence is outside the allowed workspace. "
                f"No motion will be sent: {exc}"
            )
            self._enter_fault(str(exc))
            return response
        self.step = 0
        self._busy = True

        self.get_logger().info(
            "New pick-and-place request accepted and being serviced."
        )

        response.accepted = True
        response.error = ""

        self._send_next()
        return response

    def handle_point_to_position(self, request, response):
        coords = TableCoordinates(x=request.position_x, y=request.position_y)

        if self._busy:
            response.accepted = False
            response.error = "Robot is busy executing another operation. Please wait until it finishes." 
            return response

        self.get_logger().info(f"Pointing at {coords}")
        try:
            self.sequence = self._point_sequence(coords)
        except ValueError as exc:
            response.accepted = False
            response.error = str(exc)
            self.get_logger().fatal(
                f"Configured sequence is outside the allowed workspace. "
                f"No motion will be sent: {exc}"
            )
            self._enter_fault(str(exec))
            return response
        self.step = 0
        self._busy = True
        self._send_next()

        response.accepted = True
        return response

    # Steps for moving a card. Assumes the arm is already at 'start' with the gripper open 
    def _start_and_end_positions_to_sequence(self, start: TableCoordinates, end: TableCoordinates):
        # Doesn't check whether the arm is actually at start as the CLI always points first
        return [
            ("close gripper", lambda: self.send_gr_pose(False)),
            ("lift from pickup", self._table_coordinates_to_above_table_pose(start)),
            ("above destination", self._table_coordinates_to_above_table_pose(end)),
            ("descend to destination", self._table_coordinates_to_on_table_pose(end)),
            ("release gripper", lambda: self.send_gr_pose(True)),
            ("retreat from destination", self._table_coordinates_to_above_table_pose(end)),
        ]

    def home_pose(self):
        return self.send_ee_readable_pose(
            x=0.0, y=0.0, z=0.6, pitch_degrees=-45.0
        )

    # Checks the pose, works out the yaw from the x and y. If anything is out of bounds it raises a ValueError and the function stops
    def _table_coordinates_to_on_table_pose(self, coords):
        self.validate_ee_readable_pose(
            x=coords.x,
            y=coords.y,
            z=0.07,
            pitch_degrees=90.0,
        )
        # Returns a move to be carried out later, the lambda goes into the sequence list and _send_next calls it later
        return lambda: self.send_ee_readable_pose(
            x=coords.x,
            y=coords.y,
            z=0.07,
            pitch_degrees=90.0,
        )

    # Going from _on_table to _above_table changes the height and tilts the wrist because the two poses have different pitches
    def _table_coordinates_to_above_table_pose(self, coords):
        self.validate_ee_readable_pose(
            x=coords.x,
            y=coords.y,
            z=0.3,
            pitch_degrees=45.0,
        )
        return lambda: self.send_ee_readable_pose(
            x=coords.x,
            y=coords.y,
            z=0.3,
            pitch_degrees=45.0,
        )

    # Send the next step of self.sequence or finish if there are none left
    def _send_next(self): 
        if self._goal_active: # Safety guard: If a goal is still running, don't send another one
            self.get_logger().warning(
                "A movement goal is already active; ignoring next step."
            )
            return

        if self.step >= len(self.sequence): # No steps left means the sequence is finished and it then frees the robot
            self._busy = False
            self.get_logger().info("Sequence complete.")
            return

        step_name, callback = self.sequence[self.step] # Unpacks the (name, function) pair
        self.get_logger().info(
            f"Sending step {self.step + 1}/{len(self.sequence)}: {step_name}"
        )

        # Runs teh step, if it raises an error or returns False, it faults
        try:
            submitted = callback()
        except Exception as exc:
            self._enter_fault(
                f"Step '{step_name}' raised an exception: {exc}"
            )
            return

        if submitted is False:
            self._enter_fault(
                f"Step '{step_name}' was rejected by local validation."
            )
            return

        # Marks the step as running, records the start time, start the 0.25 second timer and moves 'step' on 
        self._busy = True
        self._goal_active = True
        self._current_step_name = step_name
        self._goal_started_at = time.monotonic()
        self._goal_timer = self.create_timer(
            0.25, self._check_goal_timeout
        )
        self.step += 1

    # Runs every 0.25 seconds while a step is in progress, if the step has taken longer than goal_timeout_sec it faults
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

    # Stops the current sequence: logs the reason, cancels timers and tells MoveIt to cancel the active goal
    def _enter_fault(self, reason):
        self._busy = False
        self._goal_active = False
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

    # Stops and destroys a timer if it exists, ignores timers that are already gone 
    def _cancel_timer(self, timer):
        if timer is not None:
            try:
                timer.cancel()
                self.destroy_timer(timer)
            except Exception:
                pass

    # MoveIt has either accepted or rejected the goal, if accepted, the results will be """". future.result() is the goal handle, get_result_async() asks for the final result and _result_cb runs when it arrives
    def _goal_response_cb(self, future):
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

    # Logs MoveIt's progress
    def _feedback_cb(self, feedback_msg):
        state = feedback_msg.feedback.state
        self.get_logger().info(f"MoveIt state: {state}")

    # When a step is completed, it checks whether it succeeded (if status is 4). If there are steps left, it starts a one-off step_delay timer that calls _delayed_next. The delay gives /joint_states time to settle so the next plan starts from the arm's real position
    def _result_cb(self, future):
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
        self._goal_active = False

        if self.step >= len(self.sequence):
            self._busy = False
            self.get_logger().info(
                "Pick-and-place sequence completed." 
                "Robot is ready for another request."
            )
            return

        self._wait_timer = self.create_timer(
            self.step_delay, self._delayed_next
        )

    # Cancels the one-off timer and sends the next step 
    def _delayed_next(self):
        self._cancel_timer(self._wait_timer)
        self._wait_timer = None
        self._send_next()

    # Raise ValueError if this pose (yaw calculated from a and y) is outside the limits
    def validate_ee_readable_pose(
            self, x, y, z,
            roll_degrees=0.0,
            pitch_degrees=0.0,
        ):
            yaw_degrees = TableCoordinates(x=x, y=y).get_yaw_degrees()

            self._validate_position(x, y, z)
            self._validate_orientation(
                roll_degrees, pitch_degrees, yaw_degrees
            )

    # Validate and send the gripper pose given in degrees
    def send_ee_readable_pose(
        self, x, y, z,
        roll_degrees=0.0,
        pitch_degrees=0.0,
    ):
        """
        Define the end effector pose in human readable values. We use degrees instead
        of radians and Euler (roll, pitch, and yaw) instead of quaternions
        Yaw is always atan2(y, x) so the gripper faces outwards from the base
        """
        yaw_degrees = TableCoordinates(x=x, y=y).get_yaw_degrees()

        self._validate_position(x, y, z)
        self._validate_orientation(
            roll_degrees, pitch_degrees, yaw_degrees
        )

        q = quaternion_from_euler(
            math.radians(roll_degrees),
            math.radians(pitch_degrees),
            math.radians(yaw_degrees),
        )
        return self._send_ee_pose(
            x=x, y=y, z=z, qx=q[0], qy=q[1], qz=q[2], qw=q[3]
        )

    # Builds and sends a MoveIt goal for the gripper tip
    def _send_ee_pose(
        self, x, y, z,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0,
    ):
        """
        The arguments accepted by this method are assumed to have been validated already
        """
        # A pose and the frame it's measured in
        p = PoseStamped() 
        p.header.frame_id = self.base_link
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        p.pose.orientation = Quaternion(
            x=float(qx), y=float(qy), z=float(qz), w=float(qw)
        )

        # The plan request (group, time, attempts)
        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.allowed_planning_time = 10.0
        req.num_planning_attempts = 3

        # The position constraint, the tip 'ee_link' must end inside a 1cm sphere at the target
        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [p.pose]

        # The target orientations with a tolerance of 0.05 radians per axis
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = p.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0

        # Waist lock to ensure efficient trajectory i.e., not travelling the long way around 
        wc = JointConstraint()
        wc.joint_name = self.waist_joint
        wc.position = math.atan2(y , x)
        wc.tolerance_above = 0.05
        wc.tolerance_below = 0.05
        wc.weight = 1.0

        # All three constraints must hold
        gc = Constraints()
        gc.position_constraints = [pc]
        gc.orientation_constraints = [oc]
        gc.joint_constraints = [wc]
        req.goal_constraints = [gc]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False # plan and execute, replan, look_around
        goal.planning_options.replan = True
        goal.planning_options.look_around = False

        future = self._client.send_goal_async(goal, self._feedback_cb) # sends without waiting, the callbacks handle the reply
        future.add_done_callback(self._goal_response_cb)
        return True

    def send_gr_pose(self, open=True):
        req = MotionPlanRequest()
        req.group_name = self.gripper_group
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        # Up to 5mm more closed than the target, never more open
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

    def send_joint_pose(self, joint_positions):
        """
        Moves the arm to the exact joint angles. 
        'joint_positions' is a dictionary of joint names and their angles in raians. 
        Each joint gets its own constraint and the arm moves at 20% speed.
        """
        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 3
        req.max_velocity_scaling_factor = 0.2

        gc = Constraints()
        for name, angle in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = angle
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            gc.joint_constraints.append(jc)
        req.goal_constraints = [gc]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        future = self._client.send_goal_async(goal, self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)    

# When the node is run, it starts ROS2 and the dealer node
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RX200DealerClientNode()
        rclpy.spin(node) # Keeps the node running and handling callbacks until user runs Ctrl+C
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

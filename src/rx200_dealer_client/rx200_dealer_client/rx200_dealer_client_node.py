import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler
from dataclasses import dataclass
from enum import Enum

class TablePosition(str, Enum):
    CC = 'cc' # Community Cards
    PLAYER_1 = 'player1'
    PLAYER_2 = 'player2'

@dataclass
class TableCoordinates:
    x: float
    y: float
    yaw_degrees: float


class RX200DealerClientNode(Node):
    def __init__(self):
        super().__init__('rx200_dealer')
        self.get_logger().info('Dealer started')
        self._client = ActionClient(self, MoveGroup, '/move_action')
        while not self._client.wait_for_server(1.0):
            self.get_logger().warning("Action server not up")

        self.arm_group = 'interbotix_arm'
        self.gripper_group = 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
        self.gripper_joint = 'left_finger'
        self.waist_joint = 'waist'
        self.home_joints = {
            'waist': 0.0,
            'shoulder': 0.0,
            'elbow': -1.57,
            'wrist_angle': 0.0,
            'wrist_rotate': 0.0,
        }
        self.home_pose = lambda: self.send_joint_pose(self.home_joints) # Updates for upright home position according to RViz coordinates
        self.table_coords = {
            TablePosition.CC: TableCoordinates(x = 0.3, y = 0.0, yaw_degrees = 0.0),
            TablePosition.PLAYER_1: TableCoordinates(x = 0.0, y = 0.3, yaw_degrees = 90.0),
            TablePosition.PLAYER_2: TableCoordinates(x = -0.3, y = 0.0, yaw_degrees = 180.0),
        }

        # parameters to be removed once you can specify the start and end positions from the CLI
        self.declare_parameter('start_position', value = TablePosition.CC.value)
        self.declare_parameter('end_position', value = TablePosition.PLAYER_1.value)

        try:
            start = TablePosition(self.get_parameter('start_position').value)
            end = TablePosition(self.get_parameter('end_position').value)
            self.sequence = self._start_and_end_positions_to_sequence(start, end)
        except ValueError as e:
            self.get_logger().error(f"Wrong value for start position or end position provided. Cannot initialize robot: {e}")

        self.step = 0
        self.step_delay = 0.5 # seconds to wait between steps
        self._send_next()

        self.get_logger().info('Robot initialized')

    def _start_and_end_positions_to_sequence(self, start: TablePosition, end: TablePosition):
        return [
            self.home_pose,
            lambda: self.send_gr_pose(True),
            self._table_coordinates_to_on_table_pose(self.table_coords[start]),
            lambda: self.send_gr_pose(False),
            self._table_coordinates_to_above_table_pose(self.table_coords[end]),
            self._table_coordinates_to_on_table_pose(self.table_coords[end]),
            lambda: self.send_gr_pose(True),
            self._table_coordinates_to_above_table_pose(self.table_coords[end]),
        ]

    def _table_coordinates_to_on_table_pose(self, coords: TableCoordinates):
        return lambda: self.send_ee_readable_pose(x = coords.x, y = coords.y, z = 0.07, yaw_degrees = coords.yaw_degrees, pitch_degrees = 90.0)

    def _table_coordinates_to_above_table_pose(self, coords: TableCoordinates):
        return lambda: self.send_ee_readable_pose(x = coords.x, y = coords.y, z = 0.3, yaw_degrees = coords.yaw_degrees, pitch_degrees = 45)

    def _send_next(self):
        if self.step >= len(self.sequence):
            self.get_logger().info('Sequence complete')
            return
        self.get_logger().info(f'Sending step {self.step + 1}/{len(self.sequence)}')
        self.sequence[self.step]()
        self.step += 1

    def send_ee_readable_pose(self, x, y, z, roll_degrees = 0.0, pitch_degrees = 0.0, yaw_degrees = 0.0):
        """
        Define the end effector pose in human readable values. We use degrees instead
        of radians and Euler (roll, pitch, and yaw) instead of quaterions
        """

        q = quaternion_from_euler(math.radians(roll_degrees), math.radians(pitch_degrees), math.radians(yaw_degrees))

        return self.send_ee_pose(x=x, y=y, z=z, qx=q[0], qy=q[1], qz=q[2], qw=q[3])

    def send_ee_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        # From documentation, the default quaternion is x=0,y=0,z=0,w=1
        # For orientation set by human, documentation suggests to first
        # think of them in terms of Euler (roll, pitch, and yaw) then
        # convert to quaterion (hard for humans to think in terms of quaterions)
        p = PoseStamped()
        p.header.frame_id = self.base_link
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.allowed_planning_time = 10.0
        req.num_planning_attempts = 3

        # position tolerance
        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01] # 1CM
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
        goal.planning_options.look_around = False # True would be a waste of compute for a 5 dof arm

        future = self._client.send_goal_async(goal, self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def send_gr_pose(self, open = True):
        req = MotionPlanRequest()
        req.group_name = self.gripper_group
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.035 if open else 0.0238 # open is 3.5cm (+0 above and -0.5cm below) and close is 2.5cm
        jc.tolerance_above = 0.0
        jc.tolerance_below = 0.005
        jc.weight = 1.0

        gc = Constraints()
        gc.joint_constraints = [jc]
        req.goal_constraints = [gc]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        future = self._client.send_goal_async(goal) # No need for feedback but it can be provided
        future.add_done_callback(self._goal_response_cb)

    def send_joint_pose(self, joint_positions):
        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 3
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

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

    def _goal_response_cb(self, future):
        handle_goal = future.result()
        if not handle_goal.accepted:
            self.get_logger().warning("Goal rejected by Moveit Server!")
            return

        self.get_logger().info("Goal accepted by Moveit Server!")
        handle_goal.get_result_async().add_done_callback(self._result_cb)
    
    def _feedback_cb(self, feedback_msg):
        state = feedback_msg.feedback.state
        self.get_logger().info(f'Current state: {state}')

        # TODO: Potential place to control the flow of the program

    def _result_cb(self, future):
        result = future.result().result
        code = result.error_code.val
        self.get_logger().info(f'Result code {code}')

        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'Step {self.step} failed, stopping sequence')
            return

        # Give joint states time to settle, otherwise the next plan's start state can be stale
        # TODO: What would be the advantage of calling the next pose from _feedback_cb instead of here?
        self._wait_timer = self.create_timer(self.step_delay, self._delayed_next)

    def _delayed_next(self):
        self._wait_timer.cancel()
        self.destroy_timer(self._wait_timer)
        self._send_next()


def main(args = None):
    rclpy.init(args=args)
    node = RX200DealerClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

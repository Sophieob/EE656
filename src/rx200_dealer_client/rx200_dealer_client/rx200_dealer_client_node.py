import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion


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

        self.declare_parameter('gripper_start_state', value = True) # Can be set in arguments in launch files and set when you run ros2 run
        self.sequence = [
            lambda: self.send_gr_pose(True),
            lambda: self.send_ee_pose(x = 0.3, y = 0.0, z = 0.08, qy = 0.7071068, qw = 0.7071068), # Pitch 90deg about y, gripper pointing down
            lambda: self.send_gr_pose(False),
        ]
        self.step = 0
        self.step_delay = 0.5 # seconds to wait between steps
        self._send_next()

        self.get_logger().info('Robot initialized')

    def _send_next(self):
        if self.step >= len(self.sequence):
            self.get_logger().info('Sequence complete')
            return
        self.get_logger().info(f'Sending step {self.step + 1}/{len(self.sequence)}')
        self.sequence[self.step]()
        self.step += 1


    def send_ee_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        p = PoseStamped()
        p.header.frame_id = self.base_link
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw) # Must be a unit quaternion

        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.allowed_planning_time = 5.0 # WTF is this?
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
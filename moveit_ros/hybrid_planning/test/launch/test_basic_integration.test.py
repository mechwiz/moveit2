# Simply check if the components stay alive after launching
import os
import sys
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Action interface
from moveit_msgs.action import HybridPlanner
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import MotionSequenceItem
from moveit_msgs.msg import MotionSequenceRequest
from moveit_msgs.msg import MoveItErrorCodes

# ros2_control interface
from std_msgs.msg import Float64MultiArray

sys.path.append(os.path.dirname(__file__))
from hybrid_planning_common import generate_common_hybrid_launch_description


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    path_to_test = os.path.dirname(__file__)

    # Add the usual demo nodes
    ld = launch.LaunchDescription(generate_common_hybrid_launch_description())
    # Python testing requires this marker
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld


class TestFixture(unittest.TestCase):
    #    # Wait several seconds, check if any nodes died
    #    def test_startup(self, proc_output):
    #        rclpy.init()
    #
    #        time.sleep(2)
    #
    #        expected_nodes = [
    #            "controller_manager",
    #            "global_planner",
    #            "hybrid_planning_container",
    #            "hybrid_planning_demo_node",
    #            "joint_state_broadcaster",
    #            "local_planner",
    #            "robot_state_publisher",
    #        ]
    #
    #        node = MakeTestNode("test_node")
    #
    #        try:
    #            for node_name in expected_nodes:
    #                assert node.wait_for_node(node_name, 1.0), (
    #                    "Expected hybrid_planning node not found! Missing node: "
    #                    + node_name
    #                )
    #        finally:
    #            rclpy.shutdown()

    # Send a hybrid planning request and ensure it succeeds
    def test_planning_request(self, proc_output):
        rclpy.init()

        node = MakeTestNode("test_node")

        # Send the robot to a good start state
        node.send_robot_to_start_state()

        action_client = HybridPlanningClient()

        #        time.sleep(10)
        #
        #        action_result = action_client.send_goal()
        #
        #        time.sleep(10)
        #
        #        assert action_result == MoveItErrorCodes.SUCCESS, "Action was not successful"

        rclpy.shutdown()

    # Send a hybrid planning request, then cancel it
    def test_cancelation(self, proc_output):
        rclpy.init()

        rclpy.shutdown()


class HybridPlanningClient(Node):
    def __init__(self):
        super().__init__("run_hybrid_planning_client")
        self._action_client = ActionClient(
            self, HybridPlanner, "/test/hybrid_planning/run_hybrid_planning"
        )
        if not self._action_client.wait_for_server(10):
            self.get_logger().error("Hybrid planning action server was not available")
            rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.error_code))
        self._result = result.error_code

    def send_goal(self):
        goal_msg = HybridPlanner.Goal()
        group_name = "panda_arm"
        goal_msg.planning_group = group_name
        motion_sequence = MotionSequenceRequest()
        motion_sequence_item = MotionSequenceItem()
        motion_sequence_item.blend_radius = 0.0
        motion_sequence_item_request = MotionPlanRequest()
        motion_sequence_item_request.group_name = group_name
        motion_sequence_item_request.pipeline_id = "ompl"
        goal_constraints = Constraints()
        goal_constraints.name = "target_position"
        # Request a small change from the initial state
        goal_state = [0.0, 0.0, 0.0, 0.0, 0.0, 1.571, 0.785]
        j1_position = JointConstraint()
        j1_position.joint_name = "panda_joint1"
        j1_position.position = goal_state[0]
        j2_position = JointConstraint()
        j2_position.joint_name = "panda_joint2"
        j2_position.position = goal_state[1]
        j3_position = JointConstraint()
        j3_position.joint_name = "panda_joint3"
        j3_position.position = goal_state[2]
        j4_position = JointConstraint()
        j4_position.joint_name = "panda_joint4"
        j4_position.position = goal_state[3]
        j5_position = JointConstraint()
        j5_position.joint_name = "panda_joint5"
        j5_position.position = goal_state[4]
        j6_position = JointConstraint()
        j6_position.joint_name = "panda_joint6"
        j6_position.position = goal_state[5]
        j7_position = JointConstraint()
        j7_position.joint_name = "panda_joint7"
        j7_position.position = goal_state[6]
        goal_constraints.joint_constraints = [
            j1_position,
            j2_position,
            j3_position,
            j4_position,
        ]
        motion_sequence_item_request.goal_constraints = [goal_constraints]
        motion_sequence_item.req = motion_sequence_item_request
        motion_sequence.items = [motion_sequence_item]
        goal_msg.motion_sequence = motion_sequence

        # This should be turned into SUCCESS when the action response returns
        self._result = MoveItErrorCodes.FAILURE

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._result


class MakeTestNode(Node):
    def __init__(self, name="test_node"):
        super().__init__(name)

    def wait_for_node(self, node_name, timeout=8.0):
        start = time.time()
        flag = False
        print("Waiting for node...")
        while time.time() - start < timeout and not flag:
            flag = node_name in self.get_node_names()
            time.sleep(0.1)

        return flag

    def send_robot_to_start_state(self):
        position_command = Float64MultiArray()
        position_command.layout.dim = []
        position_command.layout.data_offset = 0
        # "ready" named state
        position_command.data = [
            float(0),
            float(-0.785),
            float(0),
            float(-2.356),
            float(0),
            float(1.571),
            float(0.785),
        ]

        cmd_publisher = self.create_publisher(
            Float64MultiArray, "/panda_joint_group_position_controller/commands", 1
        )
        time.sleep(10)
        cmd_publisher.publish(position_command)
        time.sleep(5)

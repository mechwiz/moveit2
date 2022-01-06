import launch_testing
import os
import sys
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from hybrid_planning_common import (
    generate_common_hybrid_launch_description,
    get_robot_description,
    get_robot_description_semantic,
    load_file,
    load_yaml,
)


def generate_test_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    common_launch = generate_common_hybrid_launch_description()
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()

    hybrid_planning_gtest = Node(
        executable=PathJoinSubstitution(
            [LaunchConfiguration("test_binary_dir"), "test_basic_integration"]
        ),
        parameters=[],
        output="screen",
    )
    return LaunchDescription(
        common_launch
        + [
            TimerAction(period=2.0, actions=[hybrid_planning_gtest]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, hybrid_planning_gtest):
        self.proc_info.assertWaitForShutdown(hybrid_planning_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_gtest_pass(self, proc_info, hybrid_planning_gtest):
        launch_testing.asserts.assertExitCodes(proc_info, process=hybrid_planning_gtest)

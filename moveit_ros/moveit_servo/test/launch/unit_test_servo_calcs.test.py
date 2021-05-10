import launch
import launch_ros
import launch_testing
import os
import sys
import unittest
from launch.actions import ExecuteProcess, TimerAction

sys.path.append(os.path.dirname(__file__))
from servo_launch_test_common import load_yaml


def generate_test_description():
    # Get parameters using the demo config file
    servo_yaml = load_yaml("moveit_servo", "config/panda_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    test_binary_dir_arg = launch.actions.DeclareLaunchArgument(
        name="test_binary_dir",
        description="Binary directory of package " "containing test executables",
    )

    servo_gtest = launch_ros.actions.Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration("test_binary_dir"),
                "unit_test_servo_calcs",
            ]
        ),
        parameters=[servo_params],
        output="screen",
        # prefix="kitty gdb -e run --args"
    )

    return (
        launch.LaunchDescription(
            [
                test_binary_dir_arg,
                servo_gtest,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"servo_gtest": servo_gtest},
    )


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, servo_gtest):
        self.proc_info.assertWaitForShutdown(servo_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_gtest_pass(self, proc_info, servo_gtest):
        launch_testing.asserts.assertExitCodes(proc_info, process=servo_gtest)

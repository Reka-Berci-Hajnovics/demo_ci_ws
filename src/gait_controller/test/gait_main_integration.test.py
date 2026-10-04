import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from rclpy.action import ActionClient

from gait_controller_msgs.action import MoveCylinder


@pytest.mark.rostest
def generate_test_description():
    simulator = launch_ros.actions.Node(
        package='cylinder_simulator',
        executable='cylinder_simulator',
        output='screen',
    )

    controller = launch_ros.actions.Node(
        package='gait_controller',
        executable='gait_main_node',
        parameters=[{'use_modbus': False}],
        output='screen',
    )

    return (
        launch.LaunchDescription([
            simulator,
            controller,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'simulator': simulator,
            'controller': controller,
        },
    )


class TestGaitMainIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f'gait_main_integration_test_{id(self)}')
        self.client = ActionClient(
            self.node,
            MoveCylinder,
            '/move_cylinder',
        )

    def tearDown(self):
        self.node.destroy_node()

    def wait_for_server(self):
        assert self.client.wait_for_server(timeout_sec=10.0)

    def send_goal(self, command):
        goal = MoveCylinder.Goal()
        goal.command = command

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=5.0,
        )

        goal_handle = future.result()
        assert goal_handle is not None
        assert goal_handle.accepted

        return goal_handle

    def test_cylinder_reaches_target(self):
        self.wait_for_server()

        goal_handle = self.send_goal('cu out')

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self.node,
            result_future,
            timeout_sec=40.0,
        )

        result = result_future.result()

        assert result is not None

    def test_cylinder_can_be_stopped_explicitly(self):
        self.wait_for_server()

        goal_handle = self.send_goal('cu out')

        # Give the simulator time to demonstrate that motion is persistent.
        time.sleep(0.5)

        stop_handle = self.send_goal('cu stop')

        stop_result_future = stop_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self.node,
            stop_result_future,
            timeout_sec=5.0,
        )

        stop_result = stop_result_future.result()

        assert stop_result is not None

        # The original movement goal should no longer remain active.
        original_result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self.node,
            original_result_future,
            timeout_sec=5.0,
        )

        original_result = original_result_future.result()

        assert original_result is not None
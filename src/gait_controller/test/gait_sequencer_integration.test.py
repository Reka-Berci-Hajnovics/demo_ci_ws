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
        package="cylinder_simulator",
        executable="cylinder_simulator",
        name="cylinder_simulator",
        output="screen",
    )

    gait_main = launch_ros.actions.Node(
        package="gait_controller",
        executable="gait_main_node",
        name="gait_main_node",
        parameters=[
            {"use_modbus": False},
        ],
        output="screen",
    )

    sequencer = launch_ros.actions.Node(
        package="gait_controller",
        executable="gait_sequencer_time_action_node",
        name="gait_sequencer_action",
        parameters=[
            {"frequency": 10},
        ],
        output="screen",
    )

    return (
        launch.LaunchDescription(
            [
                simulator,
                gait_main,
                sequencer,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "simulator": simulator,
            "gait_main": gait_main,
            "sequencer": sequencer,
        },
    )


class TestGaitSequencerIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(
            f"gait_sequencer_integration_test_{id(self)}"
        )

        self.client = ActionClient(
            self.node,
            MoveCylinder,
            "/gait_sequence",
        )

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, condition, timeout=10.0):
        end_time = self.node.get_clock().now().nanoseconds / 1e9 + timeout

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if condition():
                return True

            now = self.node.get_clock().now().nanoseconds / 1e9
            if now >= end_time:
                return False

        return False

    def test_start_sequence(self):
        self.assertTrue(
            self.client.wait_for_server(timeout_sec=10.0),
            "Gait sequencer action server did not become available",
        )

        goal = MoveCylinder.Goal()
        goal.command = "start"

        send_future = self.client.send_goal_async(goal)

        self.assertTrue(
            self.spin_until(
                lambda: send_future.done(),
                timeout=10.0,
            ),
            "Timed out waiting for gait sequence goal response",
        )

        goal_handle = send_future.result()

        self.assertIsNotNone(goal_handle)
        self.assertTrue(
            goal_handle.accepted,
            "Gait sequence goal was rejected",
        )

        result_future = goal_handle.get_result_async()

        # The start sequence contains six actuator commands.
        # Allow enough time for the configured actuator durations.
        self.assertTrue(
            self.spin_until(
                lambda: result_future.done(),
                timeout=90.0,
            ),
            "Timed out waiting for gait sequence result",
        )

        result = result_future.result().result

        self.assertEqual(result.result, "start done")
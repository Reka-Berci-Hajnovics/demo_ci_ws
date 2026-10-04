#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class CylinderSimulator(Node):
    def __init__(self):
        super().__init__("cylinder_simulator")

        # Simulation configuration
        self.step = 10
        self.min_pos = 10
        self.max_pos = 4000

        self.splits = {
            "cu": 2600,
            "cl": 3000,
        }

        # Simulated cylinder positions
        self.positions = {
            "cu": 250,
            "cp": 1500,
            "cl": 250,
        }

        # Current actuator commands
        self.states = {
            "cu": "stop",
            "cp": "stop",
            "cl": "stop",
        }

        # Position feedback
        self.axis_publishers = {
            "cu": self.create_publisher(Int32, "/cu_pos", 10),
            "cp": self.create_publisher(Int32, "/cp_pos", 10),
            "cl": self.create_publisher(Int32, "/cl_pos", 10),
        }

        # Actuator command input
        self.create_subscription(
            String,
            "/arduino_command",
            self.command_callback,
            10,
        )

        # Simulate actuator movement at 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.update_positions,
        )

        self.get_logger().info(
            "Cylinder simulator started."
        )

    def command_callback(self, msg):
        parts = msg.data.split()

        if len(parts) != 2:
            self.get_logger().warning(
                f"Invalid command: {msg.data}"
            )
            return

        axis = parts[0].lower()
        state = parts[1].lower()

        valid_states = {
            "cu": ["in", "out", "stop", "door_open", "door_close"],
            "cp": ["in", "out", "stop"],
            "cl": ["in", "out", "stop", "door_open", "door_close"],
        }

        if axis not in valid_states:
            self.get_logger().warning(
                f"Unknown axis: {axis}"
            )
            return

        if state not in valid_states[axis]:
            self.get_logger().warning(
                f"Invalid state '{state}' for '{axis}'"
            )
            return

        self.states[axis] = state

        self.get_logger().info(
            f"{axis.upper()} -> {state}"
        )

    def move_towards(self, position, target):
        if position < target:
            return min(position + self.step, target)

        if position > target:
            return max(position - self.step, target)

        return position

    def update_positions(self):
        for axis in ["cu", "cp", "cl"]:
            position = self.positions[axis]
            state = self.states[axis]

            split = self.splits.get(axis)

            if axis in ["cu", "cl"]:
                if state == "in":
                    position = self.move_towards(
                        position,
                        self.min_pos,
                    )

                elif state == "out":
                    position = self.move_towards(
                        position,
                        split,
                    )

                elif state == "door_open":
                    position = self.move_towards(
                        position,
                        self.max_pos,
                    )

                elif state == "door_close":
                    position = self.move_towards(
                        position,
                        split,
                    )

            elif axis == "cp":
                if state == "in":
                    position = self.move_towards(
                        position,
                        1300,
                    )

                elif state == "out":
                    position = self.move_towards(
                        position,
                        3000,
                    )

            position = max(
                self.min_pos,
                min(self.max_pos, position),
            )

            self.positions[axis] = position

            msg = Int32()
            msg.data = position
            self.axis_publishers[axis].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = CylinderSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class CylPosPubNode(Node):
    def __init__(self):
        super().__init__("cylinder_pos_pub_node")

        # -----------------------
        # Config
        # -----------------------
        self.step = 10  # movement per tick

        self.min_pos = 10
        self.max_pos = 4000

        # Split values (IMPORTANT)
        self.splits = {
            "cu": 2600,
            "cl": 3000
        }

        # -----------------------
        # State
        # -----------------------
        self.positions = {
            "cu": 250,
            "cp": 1500,
            "cl": 250
        }

        self.states = {
            "cu": "stop",
            "cp": "stop",
            "cl": "stop"
        }

        # -----------------------
        # ROS interfaces
        # -----------------------
        self.axis_publishers = {
            "cu": self.create_publisher(Int32, "/cu_pos", 10),
            "cp": self.create_publisher(Int32, "/cp_pos", 10),
            "cl": self.create_publisher(Int32, "/cl_pos", 10)
        }

        self.create_subscription(
            String, "/arduino_command", self.command_callback, 10
        )

        self.timer = self.create_timer(0.1, self.update_positions)

        self.get_logger().info("Cylinder position fallback node running.")

    # -----------------------
    # Command handling
    # -----------------------
    def command_callback(self, msg: String):
        parts = msg.data.split()
        if len(parts) != 2:
            self.get_logger().warn(f"Invalid command: {msg.data}")
            return

        axis, state = parts[0].lower(), parts[1].lower()

        valid_states = {
            "cu": ["in", "out", "stop", "door_open", "door_close"],
            "cp": ["in", "out", "stop"],
            "cl": ["in", "out", "stop", "door_open", "door_close"]
        }

        if axis not in valid_states:
            self.get_logger().warn(f"Unknown axis: {axis}")
            return

        if state not in valid_states[axis]:
            self.get_logger().warn(f"Invalid state '{state}' for '{axis}'")
            return

        self.states[axis] = state
        self.get_logger().info(f"{axis.upper()} → {state}")

    # -----------------------
    # Motion helper (NO overshoot)
    # -----------------------
    def move_towards(self, pos, target):
        if pos < target:
            return min(pos + self.step, target)
        elif pos > target:
            return max(pos - self.step, target)
        return pos

    # -----------------------
    # Main update loop
    # -----------------------
    def update_positions(self):
        for axis in ["cu", "cp", "cl"]:
            pos = self.positions[axis]
            state = self.states[axis]

            split = self.splits.get(axis, None)

            # -----------------------
            # CU & CL (with split logic)
            # -----------------------
            if axis in ["cu", "cl"]:
                if state == "in":
                    pos = self.move_towards(pos, self.min_pos)

                elif state == "out":
                    pos = self.move_towards(pos, split)

                elif state == "door_open":
                    pos = self.move_towards(pos, self.max_pos)

                elif state == "door_close":
                    pos = self.move_towards(pos, split)

                # stop → no change

            # -----------------------
            # CP (no split)
            # -----------------------
            elif axis == "cp":
                if state == "in":
                    pos = self.move_towards(pos, 1300)

                elif state == "out":
                    pos = self.move_towards(pos, 3000)

            # -----------------------
            # Safety clamp
            # -----------------------
            pos = max(self.min_pos, min(self.max_pos, pos))

            self.positions[axis] = pos

            msg = Int32()
            msg.data = pos
            self.axis_publishers[axis].publish(msg)


# -----------------------
# Main
# -----------------------
def main(args=None):
    rclpy.init(args=args)
    node = CylPosPubNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
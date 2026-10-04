import subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from gait_controller_msgs.action import MoveCylinder


class FoxgloveActionBridge(Node):

    def __init__(self):
        super().__init__('foxglove_button_action_bridge_node')

        # Subscriber
        self.subscription = self.create_subscription(String, '/ui_command', self.command_callback, 10)

        # Action clients
        self.gait_client = ActionClient(self, MoveCylinder,  '/gait_sequence')
        self.cylinder_client = ActionClient(self, MoveCylinder, '/move_cylinder')

        #self.sweeper_process = None
        #self.sweeper_ch12 = None
        #self.sweeper_ch9 = None

        # Command mapping
        self.command_map = {
            # --- Gait sequence ---
            "start": ("gait", "start"),
            "move_up": ("gait", "up"),
            "move_down": ("gait", "down"),
            "rings_out": ("gait", "rings_out"),
            "stop": ("gait", "stop"),
            "upper_door_open": ("gait", "upper_door_open"),
            "lower_door_open": ("gait", "lower_door_open"),
            "upper_door_close": ("gait", "upper_door_close"),
            "lower_door_close": ("gait", "lower_door_close"),

            # --- Cylinders ---
            "cl_out": ("cylinder", "cl out"),
            "cl_in": ("cylinder", "cl in"),
            "cl_stop": ("cylinder", "cl stop"),

            "cp_out": ("cylinder", "cp out"),
            "cp_in": ("cylinder", "cp in"),
            "cp_stop": ("cylinder", "cp stop"),

            "cu_out": ("cylinder", "cu out"),
            "cu_in": ("cylinder", "cu in"),
            "cu_stop": ("cylinder", "cu stop"),

            # --- Servos ---
            #"sweeper_start": ("sweeper", "start"),
            #"sweeper_stop": ("sweeper", "stop"),
        }

        self.get_logger().info("Foxglove Action Bridge Ready")

    def command_callback(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f"Received: {cmd}")

        if cmd not in self.command_map:
            self.get_logger().warn(f"Unknown command: {cmd}")
            return

        target, action_cmd = self.command_map[cmd]

        if target == "gait":
            self.send_goal(self.gait_client, action_cmd, "gait_sequence")

        elif target == "cylinder":
            self.send_goal(self.cylinder_client, action_cmd, "move_cylinder")


        # elif target == "sweeper":
        #     if action_cmd == "start":
        #         if self.sweeper_ch12 is None and self.sweeper_ch9 is None:
        #             self.sweeper_ch12 = subprocess.Popen([
        #                 'ros2', 'run', 'low_level_controller', 'servo_sweeper',
        #                 '--ros-args', '-p', 'servo_channel:=12', '-p', 'move_duration:=18.0'
        #             ])
        #             self.sweeper_ch9 = subprocess.Popen([
        #                 'ros2', 'run', 'low_level_controller', 'servo_sweeper',
        #                 '--ros-args', '-p', 'servo_channel:=9', '-p', 'move_duration:=18.0'
        #             ])
        #             self.get_logger().info("servo_sweeper started")
        #         else:
        #             self.get_logger().warn("servo_sweeper already running")
        #
        #
        #     elif action_cmd == "stop":
        #         if self.sweeper_ch12 or self.sweeper_ch9:
        #             for proc in [self.sweeper_ch12, self.sweeper_ch9]:
        #                 if proc and proc.poll() is None:
        #                     proc.terminate()
        #             for proc in [self.sweeper_ch12, self.sweeper_ch9]:
        #                 if proc:
        #                     try:
        #                         proc.wait(timeout=40)  # stop_servo() needs move_duration to finish
        #                     except subprocess.TimeoutExpired:
        #                         proc.kill()
        #             self.sweeper_ch12 = None
        #             self.sweeper_ch9 = None
        #             self.get_logger().info("servo_sweeper stopped")
        #         else:
        #             self.get_logger().warn("servo_sweeper not running")


    def send_goal(self, client, command_str, name):
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f"{name} action server not available!")
            return

        goal_msg = MoveCylinder.Goal()
        goal_msg.command = command_str

        self.get_logger().info(f"Sending to {name}: {command_str}")
        client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveActionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import time
import threading
from typing import NamedTuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ConnectionException, ModbusIOException
from gait_controller_msgs.action import MoveCylinder
from gait_controller.gait_targets import TARGETS


OPTA_IP = "192.168.100.50"
OPTA_PORT = 502
MODBUS_TIMEOUT_S = 0.3          # bounded worst case for a single write

# Relay coil mapping per axis: (relay_in, relay_out)
RELAY_MAP = {
    "cu": (0, 1),
    "cp": (2, 3),
    "cl": (4, 5),
}

# ROS2 Node
class GaitMainActionServer(Node):
    def __init__(self):
        super().__init__('gait_main_action_server')

        self.declare_parameter("use_modbus", True)
        self.use_modbus = self.get_parameter("use_modbus").value

        self.declare_parameter("frequency", 10)
        self.frequency = self.get_parameter("frequency").value
        if self.frequency not in TARGETS:
            available = ", ".join(str(f) for f in sorted(TARGETS))
            raise ValueError(f"Unsupported frequency: {self.frequency} Hz. Available frequencies: {available} Hz")
        self.get_logger().info(f"Using target configuration for {self.frequency} Hz")
        # This is the configuration used by this node.
        self.targets = TARGETS[self.frequency]

        # ROS publisher (kept for logging/telemetry compatibility)
        self.arduino_pub = self.create_publisher(String, '/arduino_command', 10)

        # Position tracking (updated continuously, independent of goal execution)
        self.positions = {"cu": 250, "cp": 250, "cl": 250}
        self.create_subscription(Int32, "/cu_pos", self.create_pos_cb("cu"), 10)
        self.create_subscription(Int32, "/cp_pos", self.create_pos_cb("cp"), 10)
        self.create_subscription(Int32, "/cl_pos", self.create_pos_cb("cl"), 10)

        # Modbus TCP client
        self.client = None

        if self.use_modbus:
            self.client = ModbusTcpClient(OPTA_IP, port=OPTA_PORT, timeout=MODBUS_TIMEOUT_S)
            if not self.client.connect():
                self.get_logger().error("Failed to connect to Modbus TCP server")
            else:
                self.get_logger().info("Connected to Modbus TCP server")
        else:
            self.get_logger().info("Modbus disable - running with ROS actuator interface only")

        # Exclusivity + safety locks
        self._exec_lock = threading.Lock()       # only one axis may move at a time
        self._preempt_event = threading.Event()  # ask the running goal to yield ASAP
        self._modbus_lock = threading.Lock()      # protects the shared TCP socket

        # Last relay states to avoid redundant writes
        self._last_relay_state = {}

        # Action server
        self._action_server = ActionServer(
            self,
            MoveCylinder,
            'move_cylinder',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("GaitMainActionServer with persistent Modbus relays running.")

    # -------------------------
    # Position Callbacks
    # -------------------------
    def create_pos_cb(self, axis):
        def callback(msg: Int32):
            self.positions[axis] = int(msg.data)
        return callback

    # -------------------------
    # Modbus reconnect helper
    # -------------------------
    def _reconnect_modbus(self):
        try:
            self.client.close()
        except Exception:
            pass
        ok = self.client.connect()
        if ok:
            self.get_logger().warn("Modbus reconnected after comms error")
        else:
            self.get_logger().error("Modbus reconnect failed")
        return ok

    # -------------------------
    # Map commands to coil relays
    # -------------------------
    def send_modbus_relay(self, axis, state):
        if not self.use_modbus:
            self.get_logger().info(
                f"Modbus disabled: skipping hardware command "
                f"axis={axis}, state={state}"
            )
            return True

        relay_in, relay_out = RELAY_MAP[axis]

        # Normalize door commands
        if state == "door_open":
            state = "out"
        elif state == "door_close" or state == "door_intermediate_close":
            state = "in"

        # Normalize pusher steps
        if state == "cp_step1" or state == "cp_step2" or state == "cp_step3" or state == "cp_step4":
            state = "out"

        # Determine coil states
        if state == "in":
            in_state, out_state = True, False
        elif state == "out":
            in_state, out_state = False, True
        elif state == "stop":
            in_state, out_state = False, False
        else:
            self.get_logger().warn(f"Unknown state '{state}' for axis '{axis}'")
            return False

        last_state = self._last_relay_state.get(axis, (None, None))
        new_state = (in_state, out_state)
        if last_state == new_state:
            return True  # nothing to do

        try:
            with self._modbus_lock:
                r1 = self.client.write_coil(relay_in, in_state)
                r2 = self.client.write_coil(relay_out, out_state)

        except (ConnectionException, ModbusIOException) as e:
            self.get_logger().error(f"Modbus comms lost writing axis '{axis}': {e}")
            self._reconnect_modbus()
            return False

        if r1.isError() or r2.isError():
            self.get_logger().error(f"Modbus write error for axis '{axis}': {r1} {r2}")
            return False

        self._last_relay_state[axis] = new_state
        self.get_logger().info(
            f"Modbus: axis={axis}, state={state}, relays=({relay_in},{relay_out})"
        )
        return True

    # -------------------------
    # Action Server callbacks
    # -------------------------
    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request: '{goal_request.command}'")
        # Ask whichever goal currently owns the actuators to yield as soon as possible.
        # This does NOT grant this goal access - it still has to wait for _exec_lock.
        self._preempt_event.set()
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received for a goal.")
        return CancelResponse.ACCEPT

    def _stop_and_finish(self, goal_handle, axis, result_msg, text, succeed=True):
        self.send_modbus_relay(axis, "stop")
        if succeed:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result_msg.result = text
        return result_msg

    # -------------------------
    # Main execution
    # -------------------------
    def execute_callback(self, goal_handle):
        cmd_msg = goal_handle.request.command.strip()
        self.get_logger().info(f"Execute command: '{cmd_msg}'")

        result_msg = MoveCylinder.Result()

        parts = cmd_msg.split()
        if len(parts) != 2:
            goal_handle.abort()
            result_msg.result = f"Invalid command format: {cmd_msg}"
            return result_msg

        axis, state = parts[0].lower(), parts[1].lower()

        if axis not in self.targets:
            goal_handle.abort()
            result_msg.result = f"Invalid axis '{axis}' for frequency {self.frequency} Hz"
            return result_msg

        if state != "stop" and state not in self.targets[axis]:
            goal_handle.abort()
            available_states = ", ".join(self.targets[axis].keys())
            result_msg.result = (f"Invalid state '{state}' for axis '{axis}'. Available states: {available_states}, stop")
            return result_msg

        # Resolve target/direction up front
        if state == "stop":
            target, direction = self.positions.get(axis, 0), "hold"
        else:
            target, direction = self.targets[axis][state]

        self.get_logger().info(
            f"Frequency={self.frequency} Hz, "
            f"axis={axis}, "
            f"state={state}, "
            f"target={target}, "
            f"direction={direction}"
        )

        # Block here until whichever axis is currently moving has actually
        # stopped and released the lock - guarantees true exclusivity, no
        # window where two axes' relay writes can interleave on the wire.
        with self._exec_lock:
            self._preempt_event.clear()  # we now own execution
            self.arduino_pub.publish(String(data=cmd_msg))

            if not self.send_modbus_relay(axis, state):
                goal_handle.abort()
                result_msg.result = f"Failed to start: {cmd_msg}"
                return result_msg

            feedback_msg = MoveCylinder.Feedback()
            try:
                while rclpy.ok():
                    try:
                        canceled = goal_handle.is_cancel_requested()
                    except TypeError:
                        canceled = goal_handle.is_cancel_requested

                    if canceled:
                        return self._stop_and_finish(
                            goal_handle, axis, result_msg,
                            f"Canceled: {cmd_msg}", succeed=False
                        )

                    if self._preempt_event.is_set():
                        return self._stop_and_finish(
                            goal_handle, axis, result_msg,
                            f"Preempted: {cmd_msg}", succeed=False
                        )

                    current = int(self.positions.get(axis, 0))
                    feedback_msg.current_position = current
                    goal_handle.publish_feedback(feedback_msg)

                    if direction == "hold":
                        return self._stop_and_finish(
                            goal_handle, axis, result_msg, f"{cmd_msg} completed (hold)"
                        )
                    if direction == "decrease" and current <= target:
                        return self._stop_and_finish(
                            goal_handle, axis, result_msg, f"{cmd_msg} completed"
                        )
                    if direction == "increase" and current >= target:
                        return self._stop_and_finish(
                            goal_handle, axis, result_msg, f"{cmd_msg} completed"
                        )

                    time.sleep(0.05)

                # rclpy shutting down mid-goal
                return self._stop_and_finish(
                    goal_handle, axis, result_msg, f"Shutdown: {cmd_msg}", succeed=False
                )

            except Exception as e:
                self.get_logger().error(f"Unexpected error executing '{cmd_msg}': {e}")
                return self._stop_and_finish(
                    goal_handle, axis, result_msg, f"Error: {cmd_msg}: {e}", succeed=False
                )


def main(args=None):
    rclpy.init(args=args)
    node = GaitMainActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        #try:
        #    node.client.close()
        #except Exception:
        #    pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import (
    ActionServer,
    ActionClient,
    GoalResponse,
    CancelResponse,
)
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from gait_controller_msgs.action import MoveCylinder
from functools import partial
from std_msgs.msg import String, Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# Frequency-dependent duration configuration
# Each tuple contains:
#   0: duration_cu_in
#   1: duration_cu_out
#   2: duration_cp_in
#   3: duration_cp_out
#   4: duration_cl_in
#   5: duration_cl_out
#   6: duration_door_open
#   7: duration_door_close_mid
#   8: duration_door_close

RAW_DURATIONS = {
    #hz:  cu_in cu_out cp_in cp_out cl_in cl_out door_u_open door_u_close_mid door_u_close door_l_open door_l_close_mid door_l_close cp1, cp2, cp3 cp4
    #5:  (40,     40,    40,  40,    40,   40,        25,           25,            25,           25,         25,            25,       5,   5,   5,   5),
    5:   (21,     33,    19,  31,    19,   30,        19,           5,             7,            22,         5,             8,        36,   36,   36,   36),
    10:  (11,     17,    10,  16,    20,   15,        10,           3,             3,            11,         3,             4,        36,   36,   36,   36),
    15:  (7,      11,    7,   11,    6,    10,        7,            2,             3,            8,          2,             3,        36,   36,   36,   36),
    20:  (5,      9,     5,   9,     5,    9,         5,            2,             2,            6,          2,             2,        36,   36,   36,   36),
    25:  (4,      7,     4,   7,     4,    6,         4,            1,             2,            5,          1,             2,        36,   36,   36,   36),
    #25: (40, 40, 40, 40, 40, 40, 25, 25, 25),
}

SAFETY_MARGIN = 5

# Names corresponding to the values in RAW_DURATIONS.
DURATION_NAMES = (
    "cu_in",
    "cu_out",
    "cp_in",
    "cp_out",
    "cl_in",
    "cl_out",
    "door_u_open",
    "door_u_close_mid",
    "door_u_close",
    "door_l_open",
    "door_l_close_mid",
    "door_l_close",
    "cp_step1",
    "cp_step2",
    "cp_step3",
    "cp_step4",
)

def build_durations(raw_durations):
    """
    Convert the compact duration table into a lookup dictionary.
    Example:
        RAW_DURATIONS[10]
    becomes:
        {
            "cu_in": 40,
            "cu_out": 40,
            "cp_in": 40,
            "cp_out": 40,
            "cl_in": 40,
            "cl_out": 40,
            "door_u_open": 25,
            "door_u_close_mid": 25,
            "door_u_close": 25,
            "door_l_open": 25,
            "door_l_close_mid": 25,
            "door_l_close": 25,
            "cp_step1":5,
            "cp_step2":5,
            "cp_step3":5,
            "cp_step4":5,
        }
    """
    return {
        frequency: {
            name: duration + SAFETY_MARGIN
            for name, duration in zip(DURATION_NAMES, durations)
        }
        for frequency, durations in raw_durations.items()
    }
# Build the complete lookup table once.
DURATIONS = build_durations(RAW_DURATIONS)

class GaitSequencerActionServer(Node):

    def __init__(self):
        super().__init__("gait_sequencer_action")

        #Frequency parameters
        self.declare_parameter("frequency", 10)
        self.frequency = self.get_parameter("frequency").value
        if self.frequency not in DURATIONS:
            available = ", ".join(str(f) for f in sorted(DURATIONS))
            raise ValueError(f"Unsupported frequency: {self.frequency} Hz. Available frequencies: {available} Hz")
        # Select the duration configuration for this frequency.
        self.durations = DURATIONS[self.frequency]
        self.get_logger().info(f"Using duration configuration for {self.frequency} Hz")

        # Timing parameters
        self.duration_stop = 0.5
        self.declare_parameter("duration_stop", self.duration_stop)

        # State publisher
        self.state_pub = self.create_publisher(String, "/gait_state", 10)
        self.create_timer(0.2, self.publish_state)

        # Action client
        self.client = ActionClient(self, MoveCylinder, "move_cylinder")

        # Servo sync
        self.declare_parameter("servo_namespaces", ["servo_sweeper_ch12", "servo_sweeper_ch9"])
        self.declare_parameter("servo_status_timeout", 1.0)  # sec, freshness window
        self.declare_parameter("servo_toggle_timeout", 30.0)  # see helper below

        self.servo_namespaces = self.get_parameter("servo_namespaces").value
        self.servo_status_timeout = self.get_parameter("servo_status_timeout").value

        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,)

        self.servo_status = {}        # ns -> last state string
        self.servo_status_stamp = {}  # ns -> rclpy Time
        self.servo_toggle_event = {}  # ns -> threading.Event, set on each toggle

        for ns in self.servo_namespaces:
            self.servo_status[ns] = None
            self.servo_status_stamp[ns] = None
            self.servo_toggle_event[ns] = threading.Event()

            self.create_subscription(String, f'/{ns}/status', partial(self.servo_status_cb, ns=ns), status_qos)
            self.create_subscription(Empty, f'/{ns}/toggle', partial(self.servo_toggle_cb, ns=ns), 10)

        # State machine
        self._lock = threading.Lock()
        self.state = "IDLE"
        self.active_command = ""
        self.current_goal = None
        # Multiple persistent actuator goals
        self.current_subgoals = []
        self.stop_requested = False
        self.active_sync_namespaces = []

        # Action server
        self._action_server = ActionServer(
            self,
            MoveCylinder,
            "gait_sequence",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Gait Sequencer ready.")

    def servo_status_cb(self, msg, ns):
        self.servo_status[ns] = msg.data
        self.servo_status_stamp[ns] = self.get_clock().now()

    def servo_toggle_cb(self, msg, ns):
        self.servo_toggle_event[ns].set()

    def servo_status_fresh(self, ns):
        stamp = self.servo_status_stamp.get(ns)
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age < self.servo_status_timeout

    def servo_sync_available(self):
        """True if at least one watched servo is present, alive, and sweeping."""
        for ns in self.servo_namespaces:
            if self.servo_status_fresh(ns) and self.servo_status.get(ns) == 'SWEEPING':
                return True
        return False

    def wait_for_servo_toggle(self, goal_handle, timeout):
        """Block until every servo in active_sync_namespaces has toggled, stop is requested, or timeout elapses."""
        events = [self.servo_toggle_event[ns] for ns in self.active_sync_namespaces]
        if not events:
            # Nothing confirmed sweeping for this motion - don't block.
            return True

        for ev in events:
            ev.clear()

        deadline = time.time() + timeout
        while rclpy.ok():
            if self.should_stop(goal_handle):
                self.cancel_subgoals()
                self.send_global_stop()
                return False
            if all(ev.is_set() for ev in events):
                return True
            if time.time() >= deadline:
                self.get_logger().warn("servo_sync: toggle timeout, advancing anyway")
                return True
            time.sleep(0.02)
        return False


    # Publish state
    def publish_state(self):
        msg = String()

        with self._lock:
            msg.data = self.state
        self.state_pub.publish(msg)

    # Goal policy
    def goal_callback(self, goal_request):
        cmd = goal_request.command.strip().lower()
        with self._lock:
            # STOP always accepted
            if cmd == "stop":
                return GoalResponse.ACCEPT

            # Reject new motions while busy
            if self.state in ("RUNNING", "STOPPING"):
                self.get_logger().warn(f"Rejecting '{cmd}' because machine busy ({self.state})")
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        with self._lock:
            self.stop_requested = True
        return CancelResponse.ACCEPT

    # Execute callback
    def execute_callback(self, goal_handle):
        cmd = goal_handle.request.command.strip().lower()
        if cmd == "stop":
            return self.execute_stop(goal_handle)
        return self.execute_motion(goal_handle, cmd)

    # STOP
    def execute_stop(self, goal_handle):
        result = MoveCylinder.Result()

        with self._lock:
            self.state = "STOPPING"
            self.stop_requested = True

        self.cancel_subgoals()
        self.send_global_stop()

        with self._lock:
            self.state = "IDLE"
            self.active_command = ""
            self.current_goal = None
            self.current_subgoals.clear()
            self.stop_requested = False

        goal_handle.succeed()
        result.result = "Stopped"
        return result

    # Execute motion sequence
    def execute_motion(self, goal_handle, command):
        result = MoveCylinder.Result()
        feedback = MoveCylinder.Feedback()
        steps = self.build_steps(command)

        if steps is None:
            goal_handle.abort()
            result.result = "Unknown command"
            return result

        #Servo sync mode check
        sync_mode = (command == "down") and self.servo_sync_available()
        if sync_mode:
            self.get_logger().info(f"'{command}': servo sweeping detected — using SERVO_SYNC timing")

        with self._lock:
            self.state = "RUNNING"
            self.active_command = command
            self.current_goal = goal_handle
            self.stop_requested = False

            # snapshot which servos are actually sweeping right now, for the whole motion
            self.active_sync_namespaces = [
                ns for ns in self.servo_namespaces
                if self.servo_status_fresh(ns) and self.servo_status.get(ns) == 'SWEEPING'
            ] if sync_mode else []

        if sync_mode:
            self.get_logger().info(f"servo_sync: waiting on {self.active_sync_namespaces}")

        try:
            #For loop for each step from the steps sequence
            for cmd, axis, wait_for_toggle in steps:
                if self.should_stop(goal_handle):
                    goal_handle.abort()
                    result.result = "Interrupted"
                    return result

                duration = self.get_step_duration(cmd)
                ok = self.run_step(cmd, axis, duration, goal_handle, feedback, sync_mode=sync_mode, wait_for_toggle=wait_for_toggle)

                if not ok:
                    goal_handle.abort()
                    result.result = "Interrupted"
                    return result

            goal_handle.succeed()
            result.result = f"{command} done"
            return result

        finally:
            with self._lock:
                self.state = "IDLE"
                self.active_command = ""
                self.current_goal = None
                self.stop_requested = False
                self.active_sync_namespaces = []

    # Gait sequences
    def build_steps(self, command):

        if command == "start":
            return [
                ("cu in", "cu", False),
                ("cu stop", "cu", False),

                ("cp in", "cp", False),
                ("cp stop", "cp", False),

                ("cl in", "cl", False),
                ("cl stop", "cl", False),
            ]

        elif command == "up":
            return [
                ("cu out", "cu", False),
                ("cu stop", "cu", False),

                ("cp out", "cp", False),
                ("cp stop", "cp", False),

                ("cu in", "cu", False),
                ("cu stop", "cu", False),

                ("cl out", "cl", False),
                ("cl stop", "cl", False),

                ("cp in", "cp", False),
                ("cp stop", "cp", False),

                ("cl in", "cl", False),
                ("cl stop", "cl", False),
            ]

        elif command == "down":
            return [
                ("cl out", "cl", False),
                ("cl stop", "cl", False),

                ("cp cp_step1", "cp", True),
                ("cp cp_step2", "cp", True),
                ("cp cp_step3", "cp", True),
                ("cp cp_step4", "cp", True),
                ("cp out", "cp", False),
                ("cp stop", "cp", False),

                ("cl in", "cl", False),
                ("cl stop", "cl", False),

                ("cu out", "cu", False),
                ("cu stop", "cu", False),

                ("cp in", "cp", False),
                ("cp stop", "cp", False),

                ("cu in", "cu", False),
                ("cu stop", "cu", False),
            ]

        elif command == "rings_out":
            return [
                ("cu out", "cu", False),
                ("cu stop", "cu", False),

                ("cl out", "cl", False),
                ("cl stop", "cl", False),
            ]

        elif command == "upper_door_open":
            return [
                ("cu door_open", "cu", False),
                ("cu stop", "cu", False),
            ]

        elif command == "lower_door_open":
            return [
                ("cl door_open", "cl", False),
                ("cl stop", "cl", False),
            ]

        elif command == "upper_door_close":
            return [
                ("cu door_intermediate_close", "cu", False),
                ("cu stop", "cu", False),
                ("cu door_close", "cu", False),
                ("cu stop", "cu", False),
            ]

        elif command == "lower_door_close":
            return [
                ("cl door_intermediate_close", "cl", False),
                ("cl stop", "cl", False),
                ("cl door_close", "cl", False),
                ("cl stop", "cl", False),
            ]

        return None

    # Duration lookup
    def get_step_duration(self, cmd):

        duration_map = {
            "cu in": self.durations["cu_in"],
            "cu out": self.durations["cu_out"],
            "cu stop": self.duration_stop,

            "cp in": self.durations["cp_in"],
            "cp cp_step1": self.durations["cp_step1"],
            "cp cp_step2": self.durations["cp_step2"],
            "cp cp_step3": self.durations["cp_step3"],
            "cp cp_step4": self.durations["cp_step4"],
            "cp out": self.durations["cp_out"],
            "cp stop": self.duration_stop,

            "cl in": self.durations["cl_in"],
            "cl out": self.durations["cl_out"],
            "cl stop": self.duration_stop,

            "cu door_open": self.durations["door_u_open"],
            "cl door_open": self.durations["door_l_open"],

            "cu door_intermediate_close": self.durations["door_u_close_mid"],
            "cl door_intermediate_close": self.durations["door_l_close_mid"],

            "cu door_close": self.durations["door_u_close"],
            "cl door_close": self.durations["door_l_close"],


        }

        duration = duration_map.get(cmd)
        if duration is None:
            self.get_logger().warn(
                f"No duration configured for '{cmd}'"
            )
            return 1.0

        return float(duration)

    # Stop helper
    def should_stop(self, goal_handle):
        if goal_handle.is_cancel_requested:
            with self._lock:
                self.stop_requested = True
        with self._lock:
            return self.stop_requested

    # Action client helpers
    def send_subgoal(self, command):
        goal_msg = MoveCylinder.Goal()
        goal_msg.command = command

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("move_cylinder unavailable")

            return None
        return self.client.send_goal_async(goal_msg)

    def wait_goal_handle(self, future):
        if future is None:
            return None

        while rclpy.ok():
            if future.done():
                gh = future.result()
                if not gh.accepted:
                    return None
                with self._lock:
                    self.current_subgoals.append(gh)
                return gh
            time.sleep(0.01)
        return None

    # Cancel all active actuator goals
    def cancel_subgoals(self):

        with self._lock:
            goals = list(self.current_subgoals)

        for gh in goals:
            gh.cancel_goal_async()

    # Send STOP to all actuators
    def send_global_stop(self):
        for cmd in [
            "cu stop",
            "cp stop",
            "cl stop",
        ]:

            future = self.send_subgoal(cmd)
            self.wait_goal_handle(future)

    # Execute one step
    def run_step(self,cmd,axis,duration,goal_handle,feedback, sync_mode=False, wait_for_toggle=False):
        self.get_logger().info(f"STEP: {cmd} ({duration:.1f} sec)")
        feedback.current_position = 0
        goal_handle.publish_feedback(feedback)

        # Send persistent actuator command
        future = self.send_subgoal(cmd)
        gh = self.wait_goal_handle(future)

        if gh is None:
            return False

        if sync_mode and wait_for_toggle:
            servo_timeout = self.get_parameter("servo_toggle_timeout").value
            self.get_logger().info(f"STEP: {cmd} -> waiting for servo toggle (timeout {servo_timeout:.1f}s)")
            return self.wait_for_servo_toggle(goal_handle, servo_timeout)

        # Hold force for fixed duration
        return self.wait_step_time(duration,goal_handle)

    # Fixed-time step wait
    def wait_step_time(self,duration,goal_handle,):
        start = time.time()

        while rclpy.ok():
            if self.should_stop(goal_handle):
                self.cancel_subgoals()
                self.send_global_stop()
                return False

            if time.time() - start >= duration:
                return True
            time.sleep(0.05)

def main(args=None):

    rclpy.init(args=args)
    node = GaitSequencerActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

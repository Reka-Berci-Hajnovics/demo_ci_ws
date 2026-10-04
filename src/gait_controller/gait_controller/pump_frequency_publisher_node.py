#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64


class FrequencyPublisher(Node):
    def __init__(self):
        super().__init__('frequency_publisher')

        # Declare and read the parameter
        self.declare_parameter('frequency', 10)
        frequency = self.get_parameter('frequency').value

        # QoS for a static value:
        # - Keep the last message
        # - Deliver it reliably
        # - Keep it for late-joining subscribers
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(Float64, '/pump_frequency', qos)

        # Publish once
        msg = Float64()
        msg.data = float(frequency)
        self.publisher.publish(msg)
        self.get_logger().info(f'Pump frequency is set to: {frequency} Hz')


def main(args=None):
    rclpy.init(args=args)
    node = FrequencyPublisher()
    # Keep the node alive so the publisher and its transient-local
    # history remain available to late-joining subscribers.
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
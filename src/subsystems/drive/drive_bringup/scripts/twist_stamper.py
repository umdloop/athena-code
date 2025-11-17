#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')
        self.declare_parameter('in_topic', '/cmd_vel')
        self.declare_parameter('out_topic', '/ackermann_steering_controller/reference')

        in_topic  = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        self.pub = self.create_publisher(TwistStamped, out_topic, 10)
        self.sub = self.create_subscription(TwistStamped, in_topic, self.cb, 10)

        self.get_logger().info(
            f"Relaying TwistStamped '{in_topic}' → '{out_topic}'"
        )

    def cb(self, msg: TwistStamped):
        # Just forward the message as-is
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TwistRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

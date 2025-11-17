#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.declare_parameter('in_topic', '/robot/cmd_vel')
        self.declare_parameter('out_topic', '/ackermann_steering_controller/reference')
        in_topic  = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.pub = self.create_publisher(TwistStamped, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self.cb, 10)
        self.get_logger().info(f"Stamping Twist '{in_topic}' → TwistStamped '{out_topic}'")

    def cb(self, msg: Twist):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist = msg
        self.pub.publish(ts)

def main():
    rclpy.init()
    rclpy.spin(TwistStamper())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

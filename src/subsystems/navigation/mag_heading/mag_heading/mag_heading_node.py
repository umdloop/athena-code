import math
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import MagneticField
from msgs.msg import Heading


class MagHeadingNode(Node):
    def __init__(self):
        super().__init__('mag_heading_node')

        self.declare_parameter('mag_topic', '/zed/zed_node/imu/mag')
        self.declare_parameter('frame_id', 'base_link')
        # hard_iron: [x, y, z] bias in Tesla to subtract from raw reading
        self.declare_parameter('hard_iron', [0.0, 0.0, 0.0])
        # soft_iron: row-major 3x3 matrix as 9-element list applied after hard-iron subtraction
        self.declare_parameter('soft_iron', [1.0, 0.0, 0.0,
                                             0.0, 1.0, 0.0,
                                             0.0, 0.0, 1.0])

        mag_topic = self.get_parameter('mag_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        hi = self.get_parameter('hard_iron').get_parameter_value().double_array_value
        si = self.get_parameter('soft_iron').get_parameter_value().double_array_value
        self.hard_iron = np.array(hi, dtype=float)
        self.soft_iron = np.array(si, dtype=float).reshape(3, 3)

        self.pub = self.create_publisher(Heading, 'heading', 10)
        self.sub = self.create_subscription(MagneticField, mag_topic, self._mag_cb, 10)
        self.get_logger().info(f'Subscribed to {mag_topic}')

    def _mag_cb(self, msg: MagneticField):
        raw = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ])

        # Hard-iron then soft-iron correction
        corrected = self.soft_iron @ (raw - self.hard_iron)

        # heading_rad: ROS convention — 0=East, CCW, radians
        heading_rad = math.atan2(corrected[1], corrected[0])

        # compass_bearing_deg: magnetic north convention — 0=North, CW, degrees [0, 360)
        compass_bearing_deg = (90.0 - math.degrees(heading_rad)) % 360.0

        # heading_acc from covariance diagonal (Bx, By components), in radians
        # Propagate through atan2: sigma_theta ≈ sigma_B / |B_xy|
        cov = msg.magnetic_field_covariance
        b_xy_mag = math.hypot(corrected[0], corrected[1])
        if b_xy_mag > 1e-9 and cov[0] > 0.0:
            sigma_b = math.sqrt((cov[0] + cov[4]) / 2.0)
            heading_acc = sigma_b / b_xy_mag
        else:
            heading_acc = 0.0

        out = Heading()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.heading = heading_rad
        out.heading_acc = heading_acc
        out.compass_bearing = compass_bearing_deg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MagHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

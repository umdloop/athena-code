import rclpy
from rclpy.node import Node
from .can_interface import CANInterface
from msgs.msg import CANA

class CANNode(Node):
    def __init__(self):
        super().__init__('can_node')

        self.declare_parameter('can_interface', 'can0')
        can_channel = self.get_parameter('can_interface').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(CANA, 'can_rx', 10)

        self.subscription = self.create_subscription(
            CANA,
            'can_tx',
            self.can_send_callback,
            10
        )

        self.can_interface = CANInterface(callback=self.can_rx_callback, channel=can_channel)
        
        self.get_logger().info(f'CAN Node Initialized on {can_channel}')

        # # Check for new CAN messages at 100Hz
        # self.timer = self.create_timer(0.01, self.check_can_messages)

    def can_send_callback(self, msg):
        arbitration_id = msg.id & 0x7FF
        data = msg.data[:8] if len(msg.data) > 8 else msg.data
        self.can_interface.send(arbitration_id, data)

    def check_can_messages(self):
        self.get_logger().info('Checking for CAN messages')
        msg = self.can_interface.receive()
        if msg:
            self.get_logger().info(f"Received message. ID: {msg.arbitration_id}, Data: {msg.data}")
            can_msg = CANA()
            can_msg.id = msg.arbitration_id
            can_msg.data = [int(byte) for byte in msg.data]
            self.publisher_.publish(can_msg)

    def can_rx_callback(self, msg):
        if msg:
            can_msg = CANA()
            can_msg.id = msg.arbitration_id & 0xFFFF
            # can_msg.data = [int(byte) for byte in msg.data]
            can_msg.data = list(msg.data)
            self.get_logger().info(f"Received message. ID: {can_msg.id}, Data: {can_msg.data}")
            self.publisher_.publish(can_msg)

def main(args=None):
    rclpy.init(args=args)
    can_node = CANNode()
    rclpy.spin(can_node)
    can_node.can_interface.stop_listening()
    can_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

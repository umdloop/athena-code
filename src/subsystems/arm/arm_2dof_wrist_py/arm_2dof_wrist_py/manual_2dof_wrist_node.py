"""Minimal Python replacement for the Manual2DOFWristJointByJointController.

This node listens to /joy directly, computes per-joint velocities, and writes
SMC SPEED_CONTROL frames to socketcan. It bypasses ros2_control entirely, so
the SMC hardware interface should NOT be running on the same joints while this
node is active.

CAN format reference: src/hardware_interfaces/smc_ros2_control/src/smc_hardware_interface.cpp
"""

import math
import threading
from typing import List, Optional

import can
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool


# SMC CAN command IDs (see smc_hardware_interface.hpp).
SPEED_CONTROL_CMD = 0xA2
MOTOR_RUNNING_CMD = 0x88
MOTOR_STOP_CMD = 0x81
MOTOR_SHUTDOWN_CMD = 0x80


def joint_velocity_to_motor_counts(joint_velocity: float, gear_ratio: int) -> int:
    """Mirror SMCHardwareInterface::calculate_motor_velocity_from_desired_joint_velocity.

    Converts joint rad/s -> motor centi-degrees/s, scaled by gear ratio.
    """
    return int(round(joint_velocity * (180.0 / math.pi) * 100.0 * gear_ratio))


class Manual2DOFWristNode(Node):

    def __init__(self) -> None:
        super().__init__('manual_2dof_wrist_node')

        # -- Parameters -- #
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 1000000)
        self.declare_parameter('joint_names', ['wrist_pitch', 'wrist_roll'])
        self.declare_parameter('node_ids', [0x147, 0x145])
        self.declare_parameter('gear_ratios', [40, 40])
        self.declare_parameter('joint_orientations', [1, 1])
        self.declare_parameter('joint_max_velocities', [0.785398, 0.785398])
        self.declare_parameter('pitch_axis_index', 1)
        self.declare_parameter('roll_axis_index', 0)
        self.declare_parameter('gate_button_index', 1)
        self.declare_parameter('slow_factor', 2.0)
        self.declare_parameter('joy_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.joint_names: List[str] = list(
            self.get_parameter('joint_names').value)
        self.node_ids: List[int] = [int(x) for x in self.get_parameter('node_ids').value]
        self.gear_ratios: List[int] = [int(x) for x in self.get_parameter('gear_ratios').value]
        self.orientations: List[int] = [
            int(x) for x in self.get_parameter('joint_orientations').value]
        self.max_velocities: List[float] = [
            float(x) for x in self.get_parameter('joint_max_velocities').value]

        if not (len(self.joint_names) == len(self.node_ids) == len(self.gear_ratios)
                == len(self.orientations) == len(self.max_velocities) == 2):
            raise ValueError(
                'joint_names, node_ids, gear_ratios, joint_orientations, and '
                'joint_max_velocities must all be length 2 (pitch, roll).')

        self.pitch_axis = int(self.get_parameter('pitch_axis_index').value)
        self.roll_axis = int(self.get_parameter('roll_axis_index').value)
        self.gate_button = int(self.get_parameter('gate_button_index').value)
        self.slow_factor = float(self.get_parameter('slow_factor').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout_sec').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)

        # -- State -- #
        self._lock = threading.Lock()
        self._latest_joy: Optional[Joy] = None
        self._latest_joy_stamp: float = 0.0
        self._slow_mode = False
        self._last_sent_counts: List[Optional[int]] = [None, None]

        # -- CAN bus -- #
        can_iface = self.get_parameter('can_interface').value
        bitrate = int(self.get_parameter('can_bitrate').value)
        try:
            self._bus = can.interface.Bus(
                channel=can_iface, bustype='socketcan', bitrate=bitrate)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to open socketcan interface '{can_iface}': {exc}")
            raise

        self.get_logger().info(
            f"Opened socketcan '{can_iface}' for joints "
            f"{self.joint_names} (node ids "
            f"{[hex(n) for n in self.node_ids]}).")

        # Bring motors up.
        self._send_simple_to_all(MOTOR_RUNNING_CMD)

        # -- ROS interfaces -- #
        self.create_subscription(Joy, 'joy', self._joy_callback, 10)
        self.create_service(
            SetBool, '~/set_slow_control_mode', self._set_slow_mode_callback)

        period = 1.0 / max(self.publish_rate, 1.0)
        self.create_timer(period, self._timer_callback)

    # -- Callbacks -- #

    def _joy_callback(self, msg: Joy) -> None:
        with self._lock:
            self._latest_joy = msg
            self._latest_joy_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _set_slow_mode_callback(
            self, request: SetBool.Request,
            response: SetBool.Response) -> SetBool.Response:
        with self._lock:
            self._slow_mode = bool(request.data)
        self.get_logger().info(
            f"Slow control mode {'enabled' if self._slow_mode else 'disabled'}.")
        response.success = True
        response.message = 'slow' if self._slow_mode else 'fast'
        return response

    def _timer_callback(self) -> None:
        with self._lock:
            joy = self._latest_joy
            stamp = self._latest_joy_stamp
            slow = self._slow_mode

        now = self.get_clock().now().nanoseconds * 1e-9
        stale = (joy is None) or ((now - stamp) > self.joy_timeout)

        if stale:
            pitch_vel = 0.0
            roll_vel = 0.0
        else:
            pitch_vel, roll_vel = self._compute_velocities(joy)
            if slow and self.slow_factor > 0.0:
                pitch_vel /= self.slow_factor
                roll_vel /= self.slow_factor

        self._send_velocity(0, pitch_vel)
        self._send_velocity(1, roll_vel)

    # -- Helpers -- #

    def _compute_velocities(self, joy: Joy) -> tuple:
        # Replicates the C++ controller: pitch <- axes[1]*buttons[gate]*max[0],
        # roll <- axes[0]*buttons[gate]*max[0/1].
        try:
            gate = float(joy.buttons[self.gate_button])
            pitch_axis_val = float(joy.axes[self.pitch_axis])
            roll_axis_val = float(joy.axes[self.roll_axis])
        except IndexError:
            self.get_logger().warn(
                'Joy message missing expected axes/buttons; commanding zero.',
                throttle_duration_sec=2.0)
            return 0.0, 0.0

        pitch = pitch_axis_val * gate * self.max_velocities[0]
        roll = roll_axis_val * gate * self.max_velocities[1]
        return pitch, roll

    def _send_velocity(self, joint_idx: int, joint_velocity: float) -> None:
        counts = (self.orientations[joint_idx]
                  * joint_velocity_to_motor_counts(
                      joint_velocity, self.gear_ratios[joint_idx]))

        # Skip resending an unchanged command, mirroring the HWI's prev-vs-new check.
        if self._last_sent_counts[joint_idx] == counts:
            return
        self._last_sent_counts[joint_idx] = counts

        data = bytearray(8)
        data[0] = SPEED_CONTROL_CMD
        # int32 little-endian into bytes 4..7 (matches format_control_command).
        data[4] = counts & 0xFF
        data[5] = (counts >> 8) & 0xFF
        data[6] = (counts >> 16) & 0xFF
        data[7] = (counts >> 24) & 0xFF

        self._send_frame(self.node_ids[joint_idx], bytes(data))

    def _send_simple_to_all(self, cmd_id: int) -> None:
        for node_id in self.node_ids:
            data = bytearray(8)
            data[0] = cmd_id
            self._send_frame(node_id, bytes(data))

    def _send_frame(self, arbitration_id: int, data: bytes) -> None:
        msg = can.Message(
            arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            self._bus.send(msg)
        except can.CanError as exc:
            self.get_logger().warn(
                f'Failed to send CAN frame to 0x{arbitration_id:X}: {exc}',
                throttle_duration_sec=2.0)

    # -- Shutdown -- #

    def shutdown(self) -> None:
        # Stop motors and close the bus.
        try:
            self._send_simple_to_all(MOTOR_STOP_CMD)
            self._send_simple_to_all(MOTOR_SHUTDOWN_CMD)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Error sending stop/shutdown frames: {exc}')
        try:
            self._bus.shutdown()
        except Exception:  # noqa: BLE001
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Manual2DOFWristNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

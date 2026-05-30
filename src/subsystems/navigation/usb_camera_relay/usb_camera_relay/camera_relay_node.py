#!/usr/bin/env python3

import os
import time
import threading
from typing import List, Optional

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def _discover_devices(max_index: int = 10) -> List[str]:
    """Scan /dev/video0..N and return paths that produce frames."""
    found = []
    for i in range(max_index):
        path = f"/dev/video{i}"
        if not os.path.exists(path):
            continue
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                found.append(path)
        cap.release()
    return found


class UsbCameraRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("usb_camera_relay")

        self.declare_parameter("devices", [""])
        self.declare_parameter("fps", 10.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("topic_prefix", "/cameras")

        raw_devices: List[str] = list(self.get_parameter("devices").value)
        self._fps: float = float(self.get_parameter("fps").value)
        self._width: int = int(self.get_parameter("width").value)
        self._height: int = int(self.get_parameter("height").value)
        prefix: str = str(self.get_parameter("topic_prefix").value)

        self._bridge = CvBridge()
        self._running = True
        self._threads: List[threading.Thread] = []

        if not raw_devices or raw_devices == [""]:
            self.get_logger().info("No devices specified — auto-discovering cameras...")
            devices = _discover_devices()
            if not devices:
                self.get_logger().error("No USB cameras found. Check /dev/video* devices.")
                return
            self.get_logger().info(f"Discovered: {devices}")
        else:
            devices = raw_devices

        for idx, device in enumerate(devices):
            topic = f"{prefix}/cam{idx}/image_raw"
            pub = self.create_publisher(Image, topic, 5)
            t = threading.Thread(
                target=self._capture_loop,
                args=(device, idx, pub),
                daemon=True,
            )
            self._threads.append(t)
            t.start()

    def _capture_loop(self, device: str, idx: int, pub) -> None:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f"[cam {idx}] Cannot open {device}")
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)

        self.get_logger().info(f"[cam {idx}] {device} → {pub.topic_name}")

        interval = 1.0 / self._fps

        while self._running and rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f"[cam {idx}] Frame read failed, retrying...")
                time.sleep(0.5)
                continue

            try:
                msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f"camera_{idx}"
                pub.publish(msg)
            except Exception as exc:
                self.get_logger().warn(f"[cam {idx}] Publish error: {exc}")

            time.sleep(interval)

        cap.release()
        self.get_logger().info(f"[cam {idx}] Stopped.")

    def destroy_node(self) -> None:
        self._running = False
        for t in self._threads:
            t.join(timeout=2.0)
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = UsbCameraRelayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
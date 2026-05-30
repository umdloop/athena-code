#!/usr/bin/env python3

from typing import Optional, List, NamedTuple
import os

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class _CamPubs(NamedTuple):
    detections: object
    target_found: object
    target_label: object
    target_center: object
    annotated_image: object


class YoloRosNode(Node):
    def __init__(self) -> None:
        super().__init__('yolo_node')

        pkg_share = get_package_share_directory('yolo_ros_bt')
        default_model_path = os.path.join(pkg_share, 'models', 'bestNoPreProcessingStep.pt')

        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('image_topics', ['/camera/image_raw'])
        self.declare_parameter(
            'target_classes',
            ['Bottle', 'Mallet', 'Rock-Pick-Hammer']
        )
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('queue_size', 5)

        self.conf_thres: float = float(
            self.get_parameter('conf_thres').get_parameter_value().double_value
        )
        self.model_path: str = (
            self.get_parameter('model_path').get_parameter_value().string_value
        )
        image_topics: List[str] = list(self.get_parameter('image_topics').value)
        self.target_classes = self.get_parameter('target_classes').value
        self.publish_annotated_image: bool = (
            self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        )
        self.queue_size: int = int(
            self.get_parameter('queue_size').get_parameter_value().integer_value
        )

        self.bridge = CvBridge()

        if YOLO is None:
            self.get_logger().error(
                'Ultralytics is not installed. Install with: pip install ultralytics'
            )
            raise RuntimeError('Missing ultralytics package')

        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'Loaded YOLO model from: {self.model_path}')
            self.get_logger().info(f'YOLO classes: {self.model.names}')
        except Exception as exc:
            self.get_logger().error(f'Failed to load YOLO model: {exc}')
            raise

        for idx, topic in enumerate(image_topics):
            ns = f'/yolo/cam{idx}'
            pubs = _CamPubs(
                detections=self.create_publisher(
                    Detection2DArray, f'{ns}/detections', self.queue_size),
                target_found=self.create_publisher(
                    Bool, f'{ns}/target_found', self.queue_size),
                target_label=self.create_publisher(
                    String, f'{ns}/target_label', self.queue_size),
                target_center=self.create_publisher(
                    PointStamped, f'{ns}/target_center', self.queue_size),
                annotated_image=self.create_publisher(
                    Image, f'{ns}/annotated_image', self.queue_size),
            )
            self.create_subscription(
                Image,
                topic,
                lambda msg, p=pubs: self._image_callback(msg, p),
                self.queue_size,
            )
            self.get_logger().info(f'Camera {idx}: {topic} → {ns}/')

        self.get_logger().info('YOLO ROS node initialized')
        self.get_logger().info(f'Target classes: {self.target_classes}')
        self.get_logger().info(f'Confidence threshold: {self.conf_thres:.2f}')

    def _image_callback(self, msg: Image, pubs: _CamPubs) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        try:
            results = self.model(frame, verbose=False, conf=self.conf_thres)
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        detection_array = Detection2DArray()
        detection_array.header = msg.header
        annotated_frame = frame.copy()

        img_h, img_w = frame.shape[:2]
        norm_w = float(img_w) if img_w else 1.0
        norm_h = float(img_h) if img_h else 1.0

        if not results:
            self._publish_empty(msg.header, pubs)
            return

        result = results[0]
        boxes = getattr(result, 'boxes', None)
        names = getattr(result, 'names', {})

        if boxes is None or len(boxes) == 0:
            pubs.detections.publish(detection_array)
            self._publish_target(msg.header, pubs, found=False, center=None, label='')
            if self.publish_annotated_image:
                self._publish_annotated(annotated_frame, msg.header, pubs)
            return

        target_found = False
        best_conf = -1.0
        best_center = None
        best_label = ''

        for box in boxes:
            conf = float(box.conf[0].item())
            class_id = int(box.cls[0].item())

            if conf < self.conf_thres:
                continue

            class_name = names[class_id] if class_id in names else str(class_id)

            x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
            center_x = (x_min + x_max) / 2.0
            center_y = (y_min + y_max) / 2.0

            # Publish bbox in normalized [0, 1] image coordinates so the GUI
            # can overlay it on any WebRTC stream regardless of encode size.
            detection_msg = Detection2D()
            detection_msg.header = msg.header
            detection_msg.bbox.center.position.x = center_x / norm_w
            detection_msg.bbox.center.position.y = center_y / norm_h
            detection_msg.bbox.size_x = (x_max - x_min) / norm_w
            detection_msg.bbox.size_y = (y_max - y_min) / norm_h

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = conf
            detection_msg.results.append(hypothesis)

            detection_array.detections.append(detection_msg)

            if class_name in self.target_classes and conf > best_conf:
                target_found = True
                best_conf = conf
                best_center = (center_x / norm_w, center_y / norm_h)
                best_label = class_name

            cv2.rectangle(
                annotated_frame,
                (int(x_min), int(y_min)),
                (int(x_max), int(y_max)),
                (0, 255, 0),
                2,
            )
            cv2.putText(
                annotated_frame,
                f'{class_name} {conf:.2f}',
                (int(x_min), max(0, int(y_min) - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        pubs.detections.publish(detection_array)
        self._publish_target(
            msg.header, pubs,
            found=target_found,
            center=best_center,
            label=best_label if target_found else '',
        )
        if self.publish_annotated_image:
            self._publish_annotated(annotated_frame, msg.header, pubs)

    def _publish_empty(self, header, pubs: _CamPubs) -> None:
        empty = Detection2DArray()
        empty.header = header
        pubs.detections.publish(empty)
        self._publish_target(header, pubs, found=False, center=None, label='')

    def _publish_target(
        self,
        header,
        pubs: _CamPubs,
        found: bool,
        center: Optional[tuple],
        label: str,
    ) -> None:
        found_msg = Bool()
        found_msg.data = found
        pubs.target_found.publish(found_msg)

        label_msg = String()
        label_msg.data = label
        pubs.target_label.publish(label_msg)

        if found and center is not None:
            point_msg = PointStamped()
            point_msg.header = header
            point_msg.point.x = float(center[0])
            point_msg.point.y = float(center[1])
            point_msg.point.z = 0.0
            pubs.target_center.publish(point_msg)

    def _publish_annotated(self, frame, header, pubs: _CamPubs) -> None:
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header = header
            pubs.annotated_image.publish(img_msg)
        except Exception as exc:
            self.get_logger().warn(f'Failed to publish annotated image: {exc}')


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = YoloRosNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Fatal error in yolo_node: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

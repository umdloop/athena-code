#!/usr/bin/env python3
import math
import os
import time
from pathlib import Path

import cv2 as cv
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import TakePanorama
from msgs.msg import Heading

class PanoramaNode(Node):
    def __init__(self):
        super().__init__(node_name="panorama_node")

        self.heading = None
        self.camera_index = 0
        self.camera = cv.VideoCapture(self.camera_index)

        if not self.camera.isOpened():
            raise RuntimeError(f"Unable to open camera index {self.camera_index} with OpenCV")

        
        # -- Services/Publisher/Subscriptions -- #
        self.panorama_service = self.create_service(
            TakePanorama,
            'take_panorama',
            self.handlePanorama
        )

        self.servo_command = self.create_publisher(
            Float64MultiArray,
            '/zed_servo_controller/commands',
            10
        )

        self.subscription = self.create_subscription(
            Heading,
            "heading",
            self.set_heading,
            10
        )

    def handlePanorama(self, request, response):
        if(request.take_panorama):
            self.get_logger().info("Panoramic Service called")
            try:
                self.max_angle = request.max_angle 
                frames = request.frames

                total_angle = self.max_angle
                if(frames<3):
                    self.get_logger().info("More than 2 photos required")
                    response.message = "More than 2 photos required"
                    response.success = False
                    return response

                i = 0
                current_angle = 0.0
                dir_path = "temp_photos"
                os.makedirs(dir_path, exist_ok=True)
                rotate_amount = total_angle/(frames-1)
                self.get_logger().info(f"Rotate_amount:{rotate_amount}")
                
                mid_point = math.ceil(frames/2)
                while i < frames:
                    write_path = f"{dir_path}/img_"+str(i+1)+".png"
                    if i > 0:
                        current_angle += rotate_amount
                        self.servo_command.publish(Float64MultiArray(data=[current_angle]))
                        time.sleep(1.0)
                    if i == mid_point:
                        self.get_logger().info(f"Mid Point Reached at image {i+1}, current angle: {current_angle} degrees")

                    ret, frame = self.camera.read()
                    if not ret or frame is None:
                        raise RuntimeError(f"Failed to capture image {i + 1} from camera index {self.camera_index}")

                    height, width = frame.shape[:2]
                    self.get_logger().info(
                        f"Image resolution: {width} x {height} || Image index: {i + 1}"
                    )
                    cv.imwrite(write_path, frame)
                    self.get_logger().info(f"Image: {i+1} captured")
                    i+=1

                self.get_logger().info(f"Stitching photos from {dir_path}")
                self.stitch_photos(dir_path)
                self.get_logger().info(f"Service call: {self.max_angle} degrees and {frames}")
                response.message = "Panoramic Shots successfully captured"
                response.success = True
            except Exception as e:
                self.get_logger().info(f"Panoramic service Failed: \n{e}")
                response.message = f"Panoramic service failed: {e}"
                response.success = False
        else:
            self.get_logger().info("Deactivating Panoramic Shots")
            response.message = "Deactivated Panoramic photos"
            response.success = False
        return response

    def set_heading(self, msg):
        self.heading = msg.heading_acc
        self.get_logger().info("Heading received")

    def stitch_photos(self, dir_path="temp_photos", out_image_name="panoramic_image"):
        folder_path = Path(dir_path)
        photo_list = []
        for item in folder_path.iterdir():
            if item.is_file() and item.name.startswith("img_") and item.name.endswith(".png"):
                photo_list.append(item.name)

        photo_list.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))
        images = [cv.imread(str(folder_path / photo)) for photo in photo_list]
        images = [image for image in images if image is not None]

        if len(images) < 2:
            raise RuntimeError("Need at least two readable images to stitch a panorama")

        stitcher = cv.Stitcher_create()
        status, stitched_image = stitcher.stitch(images)

        if status != cv.Stitcher_OK:
            raise RuntimeError(f"Stitching failed with status code: {status}")

        out_path = folder_path / f"{out_image_name}.png"
        cv.imwrite(str(out_path), stitched_image)
        self.get_logger().info(f"Stitching completed successfully. Image saved as '{out_path}'.")

def main(args=None):
    rclpy.init(args=args)
    node = PanoramaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.camera.release()
    node.destroy_node()
    rclpy.shutdown()

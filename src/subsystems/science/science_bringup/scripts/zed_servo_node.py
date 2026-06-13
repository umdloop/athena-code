#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32, Float64MultiArray,Float32
from science_bringup.srv import TakePanorama
import pyzed.sl as sl
import cv2 as cv
import math
import numpy as np
from pathlib import Path
import time
import os
#To run the following node: 
#ros2 run drive_bringup zed_servo_node.py (I kinda forgot where the node was supposed to be)
#ros2 run science_bringup zed_servo_node.py
#
#Panorama Service sits in science
#ros2 service call /take_panorama science_bringup/TakePanorama     "{take_panorama: True, frames: 31, max_angle: 270.0}"

#I think I was implied to make this but I never made this
#Zed Servo rotate left/right
#ros2 service call /servo_turn science_bringup/Servo_Turn          "data:[]" (degree to turn to)/270

#Mulitple nodes within the same porgram...

cam_dir = [sl.VIEW.LEFT,sl.VIEW.SIDE_BY_SIDE,sl.VIEW.RIGHT]
SUCCESS_CODE = sl.ERROR_CODE.SUCCESS

#MAT object to CV2 object
def slMat2cvMat(sl_mat:sl.Mat) -> cv.Mat:
   return cv.cvtColor(crop_black_borders(sl_mat.get_data()),cv.COLOR_BGRA2BGR)
def crop_black_borders(image):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(gray, 1, 255, cv.THRESH_BINARY)
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    x, y, w, h = cv.boundingRect(contours[0])
    return image[y:y+h, x:x+w]

class Zed_Servo_Node(Node):
    #Node Parameters: take_panorama:bool <-- Do we take the panorama? 
    #angle: 0.0 - 360.0 | frames = 100 - 2000
    def __init__(self):
        #Ros2 service call
        super().__init__(node_name="zed_servo_node")
        #Node needs to be turned on/off
        # Then in your camera init:

        self.init_param = sl.InitParameters()
        self.runtime_param = sl.RuntimeParameters()
        self.tracking_parameter = sl.PositionalTrackingParameters()
        self.cv_image_list = []
        self.camera = sl.Camera()
        self.pose = sl.Pose()
        self.get_logger().info(f"Called Panorama Node")

        self.service = self.create_service(
            TakePanorama,
            'take_panorama',
            self.handlePanorama
        )

        self.servo_publisher = self.create_publisher(
            Float32,
            'zed_servo_publisher',
            10
        )
        #Make this a service
        #frames to take
        #take_panorama.srv
        # take_panorama = take pictures or not 
        # max_angle = angle to turn to,
        # frames = Frames to capture 
        
        #creates the GPS Subscription here
        
        ##Subscribe to the heading
        ##heading_topic
        self.subscription = self.create_subscription(#can you check if the subscription is correct? 
            String,
            "sensor_msgs/msg/NavSatFix",
            self.set_header,
            10
        )

        #########Initializes all Camera Parameters##############
        self.init_param.camera_resolution=sl.RESOLUTION.HD2K#Test with 1080|2K #We have to use 720, because images aren't wide enough
        self.init_param.camera_fps = 15
        
        # # Refer here: https://www.stereolabs.com/docs/positional-tracking/coordinate-frames
        # RIGHT_HANDED_Z_UP_X_FWD is apparently ROS2 standard
        
        self.init_param.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD 
        self.init_param.coordinate_units = sl.UNIT.METER
        
        #May be changed from depth mode into an active depth mode
        self.init_param.depth_mode = sl.DEPTH_MODE.PERFORMANCE #PERFORMANCE is fastest 

        cam_status = self.camera.open(self.init_param)
        if(cam_status!=SUCCESS_CODE):
            self.get_logger().info("Camera unable to be fetched")
            exit(1)

        #Gets the camera information to confirm serial number called
        cam_info = self.camera.get_camera_information()
        self.get_logger().info(f"Confirm Camera serial Number: {cam_info.serial_number}")

        ###Camera status to check if positional tracking status has been opened successfully
        positional_status = self.camera.enable_positional_tracking(self.tracking_parameter)
        if(positional_status!=SUCCESS_CODE):
            self.get_logger().info("Error: Positional Tracking not working")
            exit(1)
        
    def handlePanorama(self,request,response):
        if(request.take_panorama):
            self.get_logger().info("Panormic Service called")
            try:
                self.max_angle = request.max_angle 
                frames = request.frames
                #And then we pass this off into the take_panorama for it's initial position 
                #self.take_panorama()
                self.get_logger().info(f"Service call: {self.max_angle} degrees and {frames}")
                response.message = "Panoramic Shots successfully captured"
                response.success = True
            except Exception as e:
                self.get_logger().info(f"Panoramic service Failed: \n{e}")
                response.success = False
        else:
            #Deactivate the panoramic shots
            self.get_logger().info("Deactivating Panoramic Shots")
            response.message = "Deactivated Panoramic photos"
            response.success = False
        return response

    def take_panorama(self):
        total_angle = self.max_angle
        frames = self.frames

        #If Current object at the current run time parameter is active 
        if(frames%2==0):
            self.get_logger().info("Requires odd amount of images")
            return -1
        if(frames<3):
            self.get_logger().info("More than 2 photos required")
            return -1
        #Initialize Matrix here
        img = sl.Mat()
        i = 0
        cv_image_list = []
        current_angle = 0.0
        #Makes directory called temp_photos if one doesn't exist
        dir_path = "temp_photos"#Please Help me name this something official (or whatever is good with base station)
        os.makedirs(dir_path, exist_ok=True)
        #Iterate through X Amount of images + Rotations
        rotate_amount = total_angle/(frames-1)
        self.get_logger().info(f"Rotate_amount:{rotate_amount}")
        
        #31/2 = 15.5 -> 16 - 1 = 15 
        mid_point = math.ceil(frames/2)
        while i < frames:
            #Move servo first before taking picture, except for the first picture
            write_path = f"{dir_path}/img_"+str(i+1)+".png"
            if i > 0:
                current_angle += rotate_amount
                #Actuate servo here
                self.servo_publisher.publish(current_angle)
                time.sleep(1.0)  # Wait for servo to fully stop
            if i == mid_point:
                self.get_logger().info(f"Mid Point Reached at image {i+1}, current angle: {current_angle} degrees")
                #GPS node can get "Header information first"
                #Request or 
            #Takes picture
            self.camera.grab(self.runtime_param)
            self.camera.retrieve_image(img,sl.VIEW.LEFT)

            #Gets current time stamp
            timestamp = self.camera.get_timestamp(sl.TIME_REFERENCE.IMAGE)
            self.get_logger().info("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(img.get_width(), img.get_height(), timestamp.get_milliseconds()))
            #Converts image from sl object into cv2 object
            cv_img = slMat2cvMat(img)
            cv.imwrite(write_path, cv_img)
            cv_image_list.append(write_path)
            self.get_logger().info(f"Image: {i+1} captured")
            i+=1
        #Read every photo from cv to convert into JPEG
        self.get_logger().info(f"Stitching photos from {dir_path}")
        self.stitch_photos(dir_path)

    def set_header(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
    
    #Helper Method to sstitch photos together
    def stitch_photos(self,dir_path="pano_photos",out_image_name="panoramic_image"):
        # Read the images from this path
        folder_path = Path(dir_path)
        photo_list = []
        # Iterate over all items in the directory and only adds the ok images
        for item in folder_path.iterdir():
            if item.is_file() and item.name.startswith("img_") and item.name.endswith(".png"):
                photo_list.append(item.name)
        #Ensures images are read in order
        photo_list.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))
        images = [cv.imread(str(folder_path / photo)) for photo in photo_list]
        
        # Create a stitcher object
        stitcher = cv.Stitcher_create()
        
        # Stitch the images together
        status, stitched_image = stitcher.stitch(images)

        if status == cv.Stitcher_OK:
            # Save the stitched image
            cv.imwrite(f"{dir_path}/{out_image_name}.png", stitched_image)
            self.get_logger().info(f"Stitching completed successfully. Image saved as '{dir_path}/{out_image_name}.png'.")
        else:
            self.get_logger().info("Stitching failed with status code:", status)
    
def main(args=None):
    #Checks if Cython is being run
    
    rclpy.init(args=args) # Initializes the ros2 communication
    node = Zed_Servo_Node()
    rclpy.spin(node=node) # Node spin = Keep this Node alive until we kill it
    #Maybe spin it up once then kill the subscription to get the location
    rclpy.shutdown() #Shutsdown a Node

if __name__ == "__main__":
    main()
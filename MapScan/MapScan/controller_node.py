###used this to store the threads version of doing this code 
###used threads so that it could work all together 


import rclpy
from rclpy.node import Node
import tf_transformations


from MapScan.pid import PID


from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Image

from copy import deepcopy
from enum import Enum
from math import sin, cos, inf
import random
import sys

import math


from cv_bridge import CvBridge, CvBridgeError
import cv2
import pandas as pd
from threading import Thread

from imutils.video import VideoStream
import argparse
import imutils
import time

import matplotlib
from matplotlib import pyplot as plt
matplotlib.use('Agg') ##opens the image windows, but produces errors
from PIL import Image as PILImage
import torch
from transformers import GLPNImageProcessor, GLPNForDepthEstimation
import numpy as np
import open3d as o3d
import cv2




class ThymioState(Enum):
    # Initially, move straight until the robot reaches an obstacle
    FORWARD = 1
    # Check if the robot didn't stop fast enough and hit the obstacle
    BACKUP = 2
    #  Rotate in a random direction until the robot is clear from obstacles
    ROTATING = 3




class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
        self.rightProximity = None
        self.cv_image = None

        self.stop_robot_two = False
        self.stop_robot_one = False ##forgot to add

        self.odom_subscriber_two = None 
        self.odom_valocity_two = None
        self.thymio_one_pose = None
        self.thymio_two_pose = None
        self.distance = 0
        self.angular_diff = 0
        self.store_image = None

        # Period of the update timer, set based on update frequencies of proximity sensors (10Hz) and odom (20Hz)
        self.UPDATE_STEP = 1/20

        # Max range of the Thymio's proximity sensors
        self.OUT_OF_RANGE = 0.12

        # Target distance of the robot from the wall at the end of FORWARD
        self.TARGET_DISTANCE = self.OUT_OF_RANGE - 0.04

        # Minimum distance from the wall to be able to rotate in place
        self.TOO_CLOSE = 0.05

        # Target difference between the distance measured by the two distance sensors
        self.TARGET_ERROR = 0.001
        

        self.update_step=self.UPDATE_STEP

        # Create a publisher for the topic 'cmd_vel'x
        self.vel_publisher = self.create_publisher(Twist, '/thymio_0/cmd_vel', 10)


        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, '/thymio_0/odom', self.odom_callback, 10)

        ##this is the subscriber for the camera for the robot
        self.camera_subscriber = self.create_subscription(Image, '/thymio_0/camera', self.image_callback, 10)
        self.bridge = CvBridge()

        ##for the second mighty thymio
        self.vel_publisher_two = self.create_publisher(Twist, '/thymio_1/cmd_vel', 10)
        self.odom_subscriber_two = self.create_subscription(Odometry, '/thymio_1/odom', self.odom_callback_two, 10)
        self.camera_subscriber_two = self.create_subscription(Image, '/thymio_1/camera', self.store_data_img_callback_two, 10)
        self.bridge_two = CvBridge()
    
   

        # Initialize the state machine
        self.current_state = None
        self.next_state = ThymioState.FORWARD
        
        # Subscribe to all proximity sensors at the same time
        self.front_sensors = ["center_left", "center", "center_right"]
        self.lateral_sensors = ["left", "right"]
        self.rear_sensors = ["rear_left", "rear_right"]
        self.proximity_sensors = self.front_sensors + self.lateral_sensors + self.rear_sensors
        self.proximity_distances = dict()
        self.proximity_subscribers = [
            self.create_subscription(Range, f'proximity/{sensor}', self.create_proximity_callback(sensor), 10)
            for sensor in self.proximity_sensors
        ]



        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        }

    def image_callback(self, data):
    
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        for (arucoName, arucoDict) in self.ARUCO_DICT.items():
            arucoDict = cv2.aruco.getPredefinedDictionary(arucoDict)
            arucoParams = cv2.aruco.DetectorParameters()


            detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

            # Detect markers
            corners, ids, rejected = detector.detectMarkers(image)

            # verify *at least* one ArUco marker was detected
            if len(corners) > 0:
                # flatten the ArUco IDs list
                ids = ids.flatten()

                # loop over the detected ArUCo corners
                for (markerCorner, markerID) in zip(corners, ids):
                    if markerID in range(1, 24): ##checks that the marker id is there #23, 1, 18, 10
                        ##stop the robot
                        cmd_vel = Twist()
                        self.vel_publisher.publish(cmd_vel)
                        self.get_logger().info("[INFO] ArUco marker ID: {}".format(markerID))

                        self.stop_robot_two = True
                        self.stop_robot_one = True
                        ##stop both robots
                        cmd_vel = Twist() 
                        self.vel_publisher.publish(cmd_vel)
                        cmd_vel = Twist() 
                        self.vel_publisher_two.publish(cmd_vel)

                        ##start the reconstruction of object
                        self.image_callback_two()




    def create_proximity_callback(self, sensor):
        # Create a callback function that has access to both the message and the name of the sensor that sent it
        def proximity_callback(msg):
            self.proximity_distances[sensor] = msg.range if msg.range >= 0.0 else inf
            
            self.get_logger().debug(
                f"proximity: {self.proximity_distances}",
                throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
            )
            
        return proximity_callback  
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(self.update_step, self.update_callback)
    
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

      

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        self.thymio_one_pose = pose2d
        
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
    
    def moving_robot(self):

        # Wait until the first update is received from odometry and each proximity sensor
        if self.odom_pose is None or \
           len(self.proximity_distances) < len(self.proximity_sensors):
            return
        
        # Check whether the state machine was asked to transition to a new 
        # state in the previous timestep. In that case, call the initialization
        # code for the new state.
        if self.next_state != self.current_state:
            self.get_logger().info(f"state_machine: transitioning from {self.current_state} to {self.next_state}")
            
            if self.next_state == ThymioState.FORWARD:
                self.init_forward()
            elif self.next_state == ThymioState.BACKUP:
                self.init_backup()
            elif self.next_state == ThymioState.ROTATING:
                self.init_rotating()
            
            self.current_state = self.next_state
        
        # Call update code for the current state
        if self.current_state == ThymioState.FORWARD:
            self.update_forward()
        elif self.current_state == ThymioState.BACKUP:
            self.update_backup()
        elif self.current_state == ThymioState.ROTATING:
            self.update_rotating()



    def update_callback(self):
        if self.stop_robot_two == True and self.stop_robot_one == True: ##added the stop_robot_one
            cmd_vel = Twist() 
            self.vel_publisher.publish(cmd_vel)
            cmd_vel = Twist() 
            self.vel_publisher_two.publish(cmd_vel)
            self.get_logger().info("setting 3D reconstruction of object")
        else:
            self.moving_robot()
            self.get_logger().info("moving the robot one")
            self.moving_robot_two()
            self.get_logger().info("moving the robot two")
      

    def init_forward(self):
        self.stop()
    
    def update_forward(self):
        # Check if the robot reached an obstacle it cannot pass through.        
        if any(self.proximity_distances[sensor] < self.TARGET_DISTANCE for sensor in self.front_sensors):
            self.next_state = ThymioState.BACKUP
            return
            
        # Just move forward with constant velocity
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 2.3 # [m/s]
        cmd_vel.angular.z = 0.0 # [rad/s]
        self.vel_publisher.publish(cmd_vel)
    
    def init_backup(self):
        self.stop()


        
    def update_backup(self):
        # Check if the robot didn't stop fast enough and hit the wall
        if all(self.proximity_distances[sensor] > self.TOO_CLOSE for sensor in self.front_sensors):
            self.next_state = ThymioState.ROTATING
            return
            
        # Slowly back up to clear the obstacle
        cmd_vel = Twist() 
        cmd_vel.linear.x  = -1.1 # [m/s]
        cmd_vel.angular.z =  0.0 # [rad/s]
        self.vel_publisher.publish(cmd_vel)
        
    def init_rotating(self):
        self.stop()
        
        # Choose a random rotation direction to clear the obstacle
        self.turn_direction = random.sample([-1, 1], 1)[0]
    
    def update_rotating(self):
        if all(self.proximity_distances[sensor] == inf for sensor in self.front_sensors):
            self.next_state = ThymioState.FORWARD
            return
            
        # Just rotate in place with constant velocity
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.0 # [m/s]
        cmd_vel.angular.z = self.turn_direction * 3.0 # [rad/s]
        self.vel_publisher.publish(cmd_vel)
    

    ####################################################################
                         #THYMIO TWO
    ####################################################################

    #get the pose of the thymio One
    def odom_callback_two(self, msg):
        self.odom_subscriber_two = msg.pose.pose 
        self.odom_valocity_two = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_subscriber_two)
        self.thymio_two_pose = pose2d 
        
        self.get_logger().info(
            "odometry for thymio two: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )

    
    def moving_robot_two(self):

        # Wait until the first update is received from odometry 
        if self.odom_pose is None or  self.thymio_two_pose is None or self.thymio_one_pose is None or \
            len(self.proximity_distances) < len(self.proximity_sensors):
            return

        """Compute shortest rotation from orientation thymio2_theta to orientation thymio1 theta"""
        self.angular_diff = math.atan2(sin(self.thymio_one_pose[2] - self.thymio_two_pose[2]), cos(self.thymio_one_pose[2] - self.thymio_two_pose[2]))


        """Euclidean distance between thymio one and thymio two."""
        self.distance_bet_thymios = math.sqrt(pow((self.thymio_two_pose[0] - self.thymio_one_pose[0]), 2) + pow((self.thymio_two_pose[1] - self.thymio_one_pose[1]), 2))

        cmd_vel = Twist() 
        cmd_vel.linear.x  =  0.5 * self.distance_bet_thymios
        cmd_vel.angular.z =  0.6* self.angular_diff ##kept reducing cause it was too big from 6 
        self.vel_publisher_two.publish(cmd_vel)

        # if self.distance_bet_thymios <= 0.1: ## to close
        #     self.stop_robot_two = True
        #     cmd_vel = Twist()
        #     self.vel_publisher_two.publish(cmd_vel)
        #     cmd_vel = Twist()
        #     self.vel_publisher.publish(cmd_vel)


    def create_point_cloud_from_depth_image(self, depth_image, intrinsic, scale=1.0):
        """
        Create a point cloud from a depth image using Open3D.
        Parameters:
            - depth_image: A numpy array containing the depth image.
            - intrinsic: An Open3D camera intrinsic object.
            - scale: A scaling factor to adjust the depth values.
        Returns:
            - A point cloud object.
            """

        height, width = depth_image.shape
        depth_image = o3d.geometry.Image((depth_image / scale).astype(np.float32))    # Create an RGBD image
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color=o3d.geometry.Image(np.zeros((height, width, 3), dtype=np.uint8)),
        depth=depth_image,convert_rgb_to_intensity=False)

        # Create the point cloud from the RGBD image
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
        return pcd


    ##to store the data, so that it image_callback_two is not auto used before needed
    def store_data_img_callback_two(self, data):
        self.store_image = data


    def image_callback_two(self):
        if self.store_image is None:
            self.get_logger().info("image is none")
            return 
        self.get_logger().info("the image is not none, reconstruction starting")

        cmd_vel = Twist() 
        self.vel_publisher.publish(cmd_vel)
        cmd_vel = Twist() 
        self.vel_publisher_two.publish(cmd_vel)
        
            
        self.cv_image = self.bridge_two.imgmsg_to_cv2(self.store_image, "bgr8")
        #self.get_logger().info(f"{type(self.cv_image), self.cv_image.shape}") #to confirm image type as there was some mismatch
        rgb_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        rgb_image = PILImage.fromarray(rgb_image) ##had to add this or data will be empty when resizing
        #image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        self.get_logger().info("starting image block")

        feature_extractor = GLPNImageProcessor.from_pretrained("vinvino02/glpn-nyu")
        model = GLPNForDepthEstimation.from_pretrained("vinvino02/glpn-nyu")

        self.get_logger().info("finsished model")
        new_height = 480 if rgb_image.height > 480 else rgb_image.height
        new_height -= (new_height % 32)
        new_width = int(new_height * rgb_image.width / rgb_image.height)
        diff = new_width % 32
        new_width = new_width - diff if diff < 16 else new_width + 32 - diff
        new_size = (new_height, new_width)
        image = rgb_image.resize(new_size)


        image_rgb = image.convert("RGB")
        image_np = np.array(image_rgb).astype(np.float32)
        self.get_logger().info("img going to model")

        inputs = feature_extractor(images=image_np, return_tensors='pt')
        with torch.no_grad():
            outputs = model(**inputs)
            predicted_depth = outputs.predicted_depth


        pad = 16
        output = predicted_depth.squeeze().cpu().numpy() * 1000.0
        output = output[pad:-pad, pad:-pad]
        image = image.crop((pad, pad, image.width- pad, image.height - pad))

        self.get_logger().info("showing img")


        fig, ax = plt.subplots(1, 2)
        ax[0].imshow(image)
        ax[0].tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
        plt.savefig("scanRoom/input.png")
        ax[1].imshow(output, cmap='plasma')
        ax[1].tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
        #plt.tight_layout()
        #plt.pause(5)
        plt.savefig("scanRoom/output.png")
        


        fx = 300  # Focal length in the x axis
        fy = 300  # Focal length in the y axis
        cx = output.shape[1] / 2  # Principal point in the x axis
        cy = output.shape[0] / 2  # Principal point in the y axis
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=output.shape[1],
            height=output.shape[0],
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy
        )


        cmd_vel = Twist() 
        self.vel_publisher.publish(cmd_vel)
        cmd_vel = Twist() 
        self.vel_publisher_two.publish(cmd_vel)


        # Create the point cloud from the depth image
        pcd = self.create_point_cloud_from_depth_image(output, intrinsic)
        # Visualize the point cloud
        o3d.visualization.draw_geometries([pcd])

        self.get_logger().info("saving depth cloud")


        o3d.io.write_point_cloud("scanRoom/output_point_cloud.ply", pcd)

        self.stop_robot_two = False ##continue moving the robots
        self.stop_robot_one = False
    



def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
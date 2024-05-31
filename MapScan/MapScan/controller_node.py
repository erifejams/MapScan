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
import cv2



class ThymioState(Enum):
    # Initially, move straight until the robot reaches an obstacle
    FORWARD = 1
    # Check if the robot didn't stop fast enough and hit the obstacle
    BACKUP = 2
    #  Rotate in a random direction until the robot is clear from obstacles
    ROTATING = 3


##sensor
from sensor_msgs.msg import Range #use to get one range reading of the distance measured 

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
        self.rightProximity = None
        self.cv_image = None

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
        self.vel_publisher = self.create_publisher(Twist, '/thymio0/cmd_vel', 10)


        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, '/thymio0/odom', self.odom_callback, 10)

        ##this is the subscriber for the camera for the robot
        self.camera_subscriber = self.create_subscription(Image, '/thymio0/camera', self.image_callback, 10)
        self.bridge = CvBridge()
        #self.vs = VideoStream(src=0).start()
   

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


    def image_callback(self, data):
        #got the video code from here https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

        #readVid = cv2.VideoCapture(src=data).start()
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Thymio Camera", self.cv_image)
        cv2.waitKey(1)
        time.sleep(10.0)

        # initialize the video stream and allow the camera sensor to warm up
        self.get_logger().info("[INFO] starting video stream...")
        #readVid = cv2.VideoCapture(src=data).start()
        if self.cv_image is None:
            return

        cv2.namedWindow('Image window')

        cv2.imshow("Image window", self.cv_image)
        self.get_logger().info("showing the robot camera")

        # dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        # parameters =  cv2.aruco.DetectorParameters()
        # detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        #self.get_logger().info("after dictionary")


        #frame = self.vs.read() - a mistake was using this
        # loop over the frames from the video stream
        while True:


            # grab the frame from the threaded video stream and resize it
            # to have a maximum width of 1000 pixels
            self.get_logger().info("going to the frame detection")
            #self.publisher_.publish(self.br.cv2_to_imgmsg(self.cv_image, "bgr8"))
            #ret, frame = self.vs.read()
            cv2.imshow("Image window", self.cv_image)
            cv2.waitKey(1)
            time.sleep(5.0)
            #frame = cv2.imread(self.cv_image)
            #frame = self.bridge.imgmsg_to_cv2(self.cv_image, "bgr8")
            #frame = cv2.resize(self.cv_image, width=1000)
            cv2.imshow("Thymio Camera", self.cv_image)
            cv2.waitKey(1)
            time.sleep(5.0)

            if self.cv_image: #- casused problems so remove it
                self.get_logger().info("detecting aruco markers")
                (corners, ids, rejected) = cv2.aruco.detectMarkers(self.cv_image,
                    cv2.aruco.DICT_ARUCO_ORIGINAL)
                
                # verify *at least* one ArUco marker was detected
                if len(corners) > 0:
                    # flatten the ArUco IDs list
                    ids = ids.flatten()
                    # loop over the detected ArUCo corners
                    for (markerCorner, markerID) in zip(corners, ids):
                        self.get_logger().info("adjusting the marker")
                        # extract the marker corners (which are always returned
                        # in top-left, top-right, bottom-right, and bottom-left
                        # order)
                        corners = markerCorner.reshape((4, 2))
                        (topLeft, topRight, bottomRight, bottomLeft) = corners
                        # convert each of the (x, y)-coordinate pairs to integers
                        topRight = (int(topRight[0]), int(topRight[1]))
                        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                        topLeft = (int(topLeft[0]), int(topLeft[1]))
                        cv2.imshow("Camera", self.cv_image)
                        cv2.waitKey(1)
                        time.sleep(10.0) 


            # Break the loop when user hits 'esc' key
            if cv2.waitKey(20) & 0xFF == 27:
                break

        cv2.destroyAllWindows()



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
        t1 = Thread(target= self.moving_robot)
        t1.start()
        self.get_logger().info("moving the robot")
      

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
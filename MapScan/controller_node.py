
import rclpy
from rclpy.node import Node
import tf_transformations


from MapScan.pid import PID


from geometry_msgs.msg import Twist, Pose, Pose2D
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


import argparse
import imutils
import time




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

        self.odom_subscriber_two = None 
        self.odom_valocity_two = None
        self.thymio_one_pose = None
        self.thymio_two_pose = None
        self.distance = 0
        self.angular_diff = 0


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


        ##for the second mighty thymio
        self.vel_publisher_two = self.create_publisher(Twist, '/thymio_1/cmd_vel', 10)
        self.odom_subscriber_two = self.create_subscription(Odometry, '/thymio_1/odom', self.odom_callback_two, 10)


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

        ##storing thymio1 pose
        self.thymio_one_pose = pose2d 
        
        self.get_logger().info(
            "odometry for thymio one: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
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
        # t1 = Thread(target= self.moving_robot)
        # t2 = Thread(target= self.moving_robot_two)
        # t1.start()
        # t2.start()
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
        cmd_vel.linear.x  =  1.5 * self.distance_bet_thymios
        cmd_vel.angular.z =  self.angular_diff ##kept reducing cause it was too big from 6 
        self.vel_publisher_two.publish(cmd_vel)

        if self.distance_bet_thymios <= 0.05:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.vel_publisher_two.publish(cmd_vel)


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
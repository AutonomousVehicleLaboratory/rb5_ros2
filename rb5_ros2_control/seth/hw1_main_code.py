#!/usr/bin/env python3
import sys
import time
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class Homework1(Node):
    def __init__(self):
        super().__init__('rb5_waypoint_feeder')
        self.timestep = 0.1
        self.kp = 1.0
        self.kp_theta = 1.0
        self.publisher_ = self.create_publisher(Twist, '/twist', 10)

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg
    


if __name__ == "__main__":
    rclpy.init()



    waypoint = np.array([[0.0,0.0,0.0], 
                         [1.0,0.0,0.0],
                         [1.0,1.0,np.pi/2.0],])
                        #  [-2.0,1.0,0.0],
                        #  [-2.0,2.0,-np.pi/2.0],
                        #  [-1.0,1.0,-np.pi/4.0],
                        #  [0.0,0.0,0.0]]) 

    # init robot controller
    robot = Homework1()

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print("move to way point", wp)

        
        x_error, y_error = np.linalg.norm(current_state[:2], wp[:2])
        orientation_error = np.atan(y_error, x_error) - current_state[2]
        print("errors: ", (x_error, y_error, orientation_error))

        vx, vy = robot.kp * x_error, robot.kp * y_error
        w = robot.kp_theta * orientation_error

        print("vx, vy, w", (vx, vy, w))

        robot.publisher_.publish(genTwistMsg(np.array([vx, vy, w])))


        while(np.linalg.norm(robot.getError(current_state, wp)) > 0.05): # check the error between current state and current way point


    # stop the car and exit
    robot.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))


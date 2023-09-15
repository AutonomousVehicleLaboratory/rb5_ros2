#!/usr/bin/env python3
""" MegaPi Controller ROS2 Wrapper"""
#import rospy
import rclpy # replaces rospy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mpi_control import MegaPiController
import numpy as np


###########################
# In this file, we will have a node which represents the control of the robot
# It should subscribe to the '/twist' topic and when a messaged is received execute a callback to a function
# This function can be called 'twist_callback' and will take the twist message, unpack it and translate
# it into commands suitable for the megapicontroller
#
# Make sure that you can run the python script 'mpi_control.py' first and that the robot moves in the correct
# directions, otherwise you may need to specify the correct motor ports (MFR, MBL, MBR, MFL) listed in the file
###########################
class RB5RobotControl(Node):
    def __init__(self, verbose=False, debug=False):
        print("Initializing RB5 Robot Node")
        super().__init__('rb5_robot_node')
        self.motor_controller = MegaPiController(port='/dev/ttyUSB0', verbose=verbose) # This is the megapi connected to the motors
        self.subscription = self.create_subscription(Twist, '/twist', self.twist_callback, 10)
        self.subscription

        self.r = 0.025 # radius of the wheel
        self.lx = 0.055 # half of the distance between front wheel and back wheel
        self.ly = 0.07 # half of the distance between left wheel and right wheel

        def run_diagnostics(self):
            self.motor_controller.printConfiguration()

        def get_motor_values_from_twist(twist_msg)
            jacobian_matrix = np.array([[1, -1, -(self.lx + self.ly)],
                                    [1, 1, (self.lx + self.ly)],
                                    [1, 1, -(self.lx + self.ly)],
                                    [1, -1, (self.lx + self.ly)]]) / self.r

            velocities = np.array([vx, vy, w])
            # vfl, vfr, vbl, vbr
            wheel_velocities = np.dot(jacobian_matrix, velocities)
            
            return wheel_velocities

        def twist_callback(self, msg):
            linear, angular = msg.linear, msg.angular
            print("Linear: ", linear)
            print("Angular: ", angular)

            vfl, vfr, vbl, vbr = get_motor_values_from_twist(msg)


        

if __name__ == "__main__":
    rclpy.init()
    rb5_robot_ctrl = RB5RobotControl()
    #rospy.init_node('megapi_controller')
    #rospy.Subscriber('/twist', Twist, mpi_ctrl_node.twist_callback, queue_size=1) 

    
    rclpy.spin(rb5_robot_ctrl) # Spin for until shutdown

    # Destroy node and shutdown when done. (Optional, as node would be cleaned up by garbage collection)
    rb5_robot_ctrl.destroy_node()
    rclpy.shutdown()

#!/usr/bin/env python3
import sys
import time
#import roslib
#import rospy
import rclpy
from rclpy.node import Node
# import geometry_msgs.msg
from geometry_msgs.msg import Twist
# from geometry_msgs.msg import Twist
import numpy as np
import math
# import tf
import tf2_ros
#from tf2 import quaternion_matrix
# from tf.transformations import quaternion_matrix
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener

"""
The class of the pid controller.
"""
class PIDcontroller(Node):
    def __init__(self, Kp, Ki, Kd):
        super().__init__('hw2_solutions')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.02
        self.publisher_ = self.create_publisher(Twist, '/twist', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result 

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

    def getCurrentPos(self):
        """
        Given the tf listener, we consider the camera's z-axis is the header of the car
        """
        #br = tf2_ros.TransformBroadcaster()
        result = None
        foundSolution = False

    # for i in range(0, 9):
        #camera_name = "camera_" + str(i)
        # if self.tf_listener.frameExists(camera_name):
        try:
            #now = rospy.Time()
            now = rclpy.time.Time()
            # wait for the transform ready from the map to the camera for 1 second.
            #self.tf_listener.waitForTransform("map", camera_name, now, rospy.Duration(1.0))
            # extract the transform camera pose in the map coordinate.
            (trans, rot) = self.tf_buffer.lookup_transform("map", "camera", now)
            print("Trans and rot: ", (trans, rot))
            # convert the rotate matrix to theta angle in 2d
            matrix = self.quaternion_rotation_matrix(rot) #quaternion_matrix(rot)
            angle = math.atan2(matrix[1][2], matrix[0][2])
            # this is not required, I just used this for debug in RVIZ
            br.sendTransform((trans[0], trans[1], 0), tf2_ros.transformations.quaternion_from_euler(0,0,angle), self.now(), "base_link", "map")
            result = np.array([trans[0], trans[1], angle])
            foundSolution = True
        #     break
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException):
        except TransformException as ex:
            print("meet error: ", ex)

        #listener.clear()
        return foundSolution, result

    def quaternion_rotation_matrix(self, Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix


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

def coord(twist, current_state):
    """
    Convert the twist into the car coordinate
    """
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)
    


if __name__ == "__main__":
    rclpy.init()
    #rospy.init_node("hw2")
    #pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    #listener = tf2_ros.TransformListener()

    waypoint = np.array([[0.0,0.0,0.0], 
                         [1.0,0.5,0.0],
                        #  [1.0,2.0,np.pi],
                         [0.0,0.0,0.0]])

    # init pid controller
    pid = PIDcontroller(0.1,0.005,0.005)

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        motor_command_msg = genTwistMsg(coord(update_value, current_state))
        #print("Sending new motor messsage: ", motor_command_msg)
        pid.publisher_.publish(motor_command_msg)
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        current_state += update_value
        found_state, estimated_state = pid.getCurrentPos()
        if found_state: # if the tag is detected, we can use it to update current state.
            current_state = estimated_state
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            #pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            motor_command_msg = genTwistMsg(coord(update_value, current_state))
            #print("Sending new motor messsage: ", motor_command_msg)
            pid.publisher_.publish(motor_command_msg)
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += update_value
            found_state, estimated_state = pid.getCurrentPos()
            if found_state:
                current_state = estimated_state
    # stop the car and exit
    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))


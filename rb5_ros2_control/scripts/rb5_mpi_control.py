#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy

from megapi import *

class MegaPiController(Node):
    def __init__(self):
        super().__init__('rb5_control')
        self.bot = MegaPi()
        self.bot.start('/dev/ttyUSB0')

        self.joy_sub = self.create_subscription(
                        Joy,
                        '/joy',
                        self.move_joy, 1)

        self.joy_sub

    def move(self, direction, speed):
        # port1: front right (wheel 1)
        # port2: rear left (wheel 0)
        # port9: rear right (wheel 3)
        # port10: front left (wheel 2)

        if direction == "left":
            self.get_logger().info("Moving left")
            # fl wheel
            self.bot.motorRun(10, speed)

            # fr wheel
            self.bot.motorRun(1, speed)

            # rl wheel
            self.bot.motorRun(2, -speed)

            # rr wheel
            self.bot.motorRun(9, -speed)
            return
        elif direction == "right":
            self.get_logger().info("Moving right")
            # fl wheel
            self.bot.motorRun(10, -speed)

            # fr wheel
            self.bot.motorRun(1, -speed)

            # rl wheel
            self.bot.motorRun(2, speed)

            # rr wheel
            self.bot.motorRun(9, speed)
            return
        elif direction == "forward":
            self.get_logger().info("Moving forward")
            # fl wheel
            self.bot.motorRun(10, -speed)

            # fr wheel
            self.bot.motorRun(1, speed)

            # rl wheel
            self.bot.motorRun(2, -speed)

            # rr wheel
            self.bot.motorRun(9, speed)


            return
        elif direction == "reverse":
            self.get_logger().info("Moving in reverse")
            # fl wheel
            self.bot.motorRun(10, speed)

            # fr wheel
            self.bot.motorRun(1, -speed)

            # rl wheel
            self.bot.motorRun(2, speed)

            # rr wheel
            self.bot.motorRun(9, -speed)

            return
        elif direction == "ccwise":
            # Counter clockwise
            self.get_logger().info("Moving counter clockwise")
            # fl wheel
            self.bot.motorRun(10, speed)

            # fr wheel
            self.bot.motorRun(1, speed)

            # rl wheel
            self.bot.motorRun(2, speed)

            # rr wheel
            self.bot.motorRun(9, speed)
        elif direction == "cwise":
            # Clockwise
            self.get_logger().info("Moving clockwise")
            # fl wheel
            self.bot.motorRun(10, -speed)

            # fr wheel
            self.bot.motorRun(1, -speed)

            # rl wheel
            self.bot.motorRun(2, -speed)

            # rr wheel
            self.bot.motorRun(9, -speed)

        else:
            self.get_logger().info("Stopping")
            # fl wheel
            self.bot.motorRun(10, 0)

            # fr wheel
            self.bot.motorRun(1, 0)

            # rl wheel
            self.bot.motorRun(2, 0)

            # rr wheel
            self.bot.motorRun(9, 0)
            return




    def move_joy(self, joy_cmd):

        if joy_cmd.buttons[5]:
            if joy_cmd.axes[0] > 0.0:
                # left
                self.move("left", 30)
            elif joy_cmd.axes[0] < 0.0:
                # right
                self.move("right", 30)
            elif joy_cmd.axes[1] > 0.0:
                # forward
                self.move("forward", 30)
            elif joy_cmd.axes[1] < 0.0:
                # reverse
                self.move("reverse", 30)
            elif joy_cmd.axes[3] < 0.0:
                # turn clock-wise 
                self.move("cwise", 30)
            elif joy_cmd.axes[3] > 0.0:
                # turn counter clock-wise 
                self.move("ccwise", 30)
            elif abs(joy_cmd.axes[0]) <= 0.1 and abs(joy_cmd.axes[1]) <= 0.0:
                self.move("stop", 0)
        else:
            self.move("stop", 0)


    def move_dir(self, str_cmd):
        
        #if direction == "left":
        #elif direction == "right":
        #elif direction == 
        return    

def main(args=None):
    rclpy.init(args=args)
    ctrl_node = MegaPiController()
    #rclpy.Subscriber('/rb5/ctrl_cmd_str', String, ctrl.move_dir) 
    #rclpy.Subscriber('/joy', Joy, ctrl.move_joy, queue_size=1) 
    
    rclpy.spin(ctrl_node)
    ctrl_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

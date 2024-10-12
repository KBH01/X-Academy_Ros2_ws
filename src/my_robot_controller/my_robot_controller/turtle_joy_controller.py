#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
from sensor_msgs.msg import Joy

class TurtleJoyControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_joy_controller")
        self.previous_x_ = 0
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel",10)
        self.joy_subscriber_ = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10 #queue size
            )
        self.get_logger().info("Turtle controller has been started.")

    def joy_callback(self, msg: Joy):
        cmd = Twist()
        cmd.linear.x = msg._axes[1]*(msg._axes[3]*2.5+2.5) #roll
        cmd.linear.y = msg._axes[0]*(msg._axes[3]*2.5+2.5) #pitch
        cmd.angular.z = msg._axes[2]*(msg._axes[3]*2.5+2.5) #yaw

        self.cmd_vel_publisher_.publish(cmd)

def main(args = None):
    rclpy.init(args = args)
    node  = TurtleJoyControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoySubscriberNode(Node):

    def __init__(self):
        super().__init__("joy_subscriber")
        self.joy_subscriber_ = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10 #queue size
            )
        
    def joy_callback(self, msg: Joy): #will be called when a message is received
        #self.get_logger().info("(" + str(msg.x) + "," + str(msg.y) + ")")
        self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
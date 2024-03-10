#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UltraDataSubscriber(Node):

    def __init__(self):
        super().__init__("ultra_subsriber_node")
        self.ultra_sub = self.create_subscription(String,'distance_topic',self.ultra_callback, 10)

    def ultra_callback(self, msg: String):
        self.get_logger().info(str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = UltraDataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class QrDataSubscriber(Node):

    def __init__(self):
        super().__init__("qr_subsriber_node")
        self.qr_sub = self.create_subscription(String,'/qr_data',self.qr_callback, 10)

    def qr_callback(self, msg: String):
        self.get_logger().info(str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = QrDataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
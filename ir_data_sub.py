#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class IrDataSubscriber(Node):

    def __init__(self):
        super().__init__("ir_subsriber_node")
        self.ir_sub = self.create_subscription(String,'/ir_data_topic',self.ir_callback, 10)

    def ir_callback(self, msg: String):
        self.get_logger().info(str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = IrDataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
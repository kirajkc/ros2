#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class trafficSubscriber(Node):

    def __init__(self):
        super().__init__("traffic_subsriber_node")
        self.ir_sub = self.create_subscription(String,'/traffic_light_topic',self.traffic_callback, 10)

    def traffic_callback(self, msg: String):
        self.get_logger().info(str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = trafficSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
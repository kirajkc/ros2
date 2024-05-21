#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class FieldNode(Node):
    def __init__(self):
        super().__init__("field_node")
        self.field_sub = self.create_subscription(String, '/traffic_light_topic', self.field_callback, 10)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.OUT)
        GPIO.setup(27, GPIO.OUT)
        GPIO.setup(22, GPIO.OUT)
        GPIO.setup(26, GPIO.OUT)
        self.color = ""
        self.received_msg = ""

    def field_callback(self, msg: String):
        self.get_logger().info("Received message: " + msg.data)
        self.received_msg = msg.data

    def check_received_message(self):
        if self.received_msg == self.color:
            self.get_logger().info("Received message matches current color: " + self.received_msg)
            self.received_msg = ""  # Clear the received message after processing

    def traffic(self):
        while True:
            GPIO.output(26, GPIO.LOW)

            GPIO.output(17, GPIO.LOW)
            GPIO.output(27, GPIO.LOW)
            GPIO.output(22, GPIO.LOW)
            self.color = "red"
            self.check_received_message()
            time.sleep(15)

            GPIO.output(17, GPIO.HIGH)
            GPIO.output(27, GPIO.HIGH)
            GPIO.output(22, GPIO.LOW)
            self.color = "yellow"
            self.check_received_message()
            time.sleep(15)

            GPIO.output(17, GPIO.LOW)
            GPIO.output(27, GPIO.HIGH)
            GPIO.output(22, GPIO.HIGH)
            self.color = "green"
            self.check_received_message()
            time.sleep(15)

def main(args=None):
    rclpy.init(args=args)
    node = FieldNode()
    try:
        node.traffic()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

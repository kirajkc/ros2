#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class IrNode(Node):
    def __init__(self):
        super().__init__('ir_sensor_node')
        self.ir_data_pub = self.create_publisher(String, '/ir_data_topic', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #declaring BCM pin 11 for left sensor
        self.sensorLeft = 11
        # declaring BCM pin 16 for right sensor
        self.sensorRight = 16

        # declaring the BCM mode of pins
        GPIO.setmode(GPIO.BOARD)

        # set the behavior of left sensor as input
        GPIO.setup(self.sensorLeft, GPIO.IN)
        # set the behavior of right sensor as input
        GPIO.setup(self.sensorRight, GPIO.IN)
        

    def timer_callback(self):
        # checking input on left sensor
        left_data = "1" if GPIO.input(self.sensorLeft) else "0"
        # checking input on right sensor
        right_data = "1" if GPIO.input(self.sensorRight) else "0"
        self.ir_data_pub.publish(String(data=f"({left_data}) | ({right_data})"))

def main():
    rclpy.init()
    ir_sensor_node = IrNode()
    try:
        rclpy.spin(ir_sensor_node)
    except KeyboardInterrupt:
        GPIO.cleanup()
        ir_sensor_node.destroy_node()


if __name__ == '__main__':
    main()

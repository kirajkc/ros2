#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rclpy
from std_msgs.msg import String
import time

def main():
    # declaring BCM pin 11 for left sensor
    sensorLeft = 11
    # declaring BCM pin 16 for right sensor
    sensorRight = 16

    # declaring the BCM mode of pins
    GPIO.setmode(GPIO.BOARD)

    # set the behavior of left sensor as input
    GPIO.setup(sensorLeft, GPIO.IN)
    # set the behavior of right sensor as input
    GPIO.setup(sensorRight, GPIO.IN)

    rclpy.init()

    # initialize the node
    node = rclpy.create_node('ir_sensor_node')

    # initialize the publisher
    ir_data_pub = node.create_publisher(String, '/ir_data_topic', 10)

    try:
        while rclpy.ok():
            # checking input on left sensor
            left_data = "1" if GPIO.input(sensorLeft) else "0"
            # checking input on right sensor
            right_data = "1" if GPIO.input(sensorRight) else "0"
            
            # publish data in the format: (ir left data 0,1) | (ir right data 0,1)
            ir_data_pub.publish(String(data=f"({left_data}) | ({right_data})"))
            
            # generate time delay of 0.2 seconds
            time.sleep(0.2)

    except KeyboardInterrupt:
        # cleanup the GPIO pins for any other program use
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

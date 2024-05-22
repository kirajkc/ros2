#!/usr/bin/env python3

import serial
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo
from time import sleep

class TrafficSubscriber(Node):

    def __init__(self):
        super().__init__("traffic_subscriber_node")
        self.ir_sub = self.create_subscription(String, '/traffic_light_topic', self.traffic_callback, 10)
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.last_traffic_state = None
        self.last_serial_data = None
        self.c_r = 0
        self.c_y = 0
        self.c_g = 0

        # Setup GPIO
        self.servo = AngularServo(17, min_pulse_width=0.0006, max_pulse_width=0.0023)

        # Start serial reading thread
        self.start_serial_reading_thread()

    def traffic_callback(self, msg: String):
        self.get_logger().info("Received traffic state: %s" % msg.data)
        self.last_traffic_state = msg.data
        self.check_and_move_servo()

    def check_and_move_servo(self):
        if self.last_traffic_state is not None and self.last_serial_data is not None:
            if self.last_traffic_state == self.last_serial_data:
                if self.last_traffic_state == 'red':
                    self.c_r = self.c_r + 1
                    self.c_y = 0
                    self.c_g = 0
                    if self.c_r == 1:
                        self.move_servo(90)
                elif self.last_traffic_state == 'yellow':
                    self.c_y = self.c_y + 1
                    self.c_r = 0
                    self.c_g = 0
                    if self.c_y == 1: 
                        self.move_servo(45)
                elif self.last_traffic_state == 'green':
                    self.c_g = self.c_g + 1
                    self.c_r = 0
                    self.c_y = 0
                    if self.c_g == 1:   
                        self.move_servo(0)

    def move_servo(self, angle):
        self.get_logger().info("Servo state: %d" % angle)
    
        self.servo.angle = angle
        sleep(2)
        self.servo.detach()

    def start_serial_reading_thread(self):
        threading.Thread(target=self.serial_reading_thread, daemon=True).start()

    def serial_reading_thread(self):
        while True:
            read_serial = self.ser.readline().decode('utf-8').rstrip()
            self.get_logger().info("Received from serial: %s" % read_serial)
            self.last_serial_data = read_serial

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()  # Clean up GPIO

if __name__ == '__main__':
    main()

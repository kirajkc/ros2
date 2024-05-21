#!/usr/bin/env python3

import serial
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class TrafficSubscriber(Node):

    def __init__(self):
        super().__init__("traffic_subscriber_node")
        self.ir_sub = self.create_subscription(String, '/traffic_light_topic', self.traffic_callback, 10)
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.last_traffic_state = None

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        self.servo_pin = 16  # GPIO pin 16
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.servo_pin, 50)  # 50 Hz PWM frequency
        self.servo_pwm.start(0)

    def traffic_callback(self, msg: String):
        self.get_logger().info("Received traffic state: %s" % msg.data)
        self.last_traffic_state = msg.data
        self.check_and_move_servo()

    def check_and_move_servo(self):
        if self.last_traffic_state is not None:
            read_serial = self.ser.readline().decode('utf-8').rstrip()
            self.get_logger().info("Received from serial: %s" % read_serial)
            print("Received from serial: %s" % read_serial)
            if self.last_traffic_state == read_serial:
                if self.last_traffic_state == 'red':
                    self.move_servo(90)
                elif self.last_traffic_state == 'yellow':
                    self.move_servo(45)
                elif self.last_traffic_state == 'green':
                    self.move_servo(0)

    def move_servo(self, angle):
        duty_cycle = angle / 18 + 2  # Map angle to duty cycle (2 to 12)
        self.servo_pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(1)  # Adjust this delay as needed for your servo
        self.servo_pwm.ChangeDutyCycle(0)

    def start_serial_reading_thread(self):
        threading.Thread(target=self.serial_reading_thread, daemon=True).start()

    def serial_reading_thread(self):
        while True:
            read_serial = self.ser.readline().decode('utf-8').rstrip()
            self.get_logger().info("Received from serial: %s" % read_serial)
            print("Received from serial: %s" % read_serial)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSubscriber()
    node.start_serial_reading_thread()
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

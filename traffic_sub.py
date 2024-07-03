#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo
from time import sleep

# Pin numbers for your LEDs
led_pins = [23, 27, 22]

class TrafficSubscriber(Node):

    def __init__(self):
        super().__init__("traffic_subscriber_node")
        self.ir_sub = self.create_subscription(String, '/traffic_light_topic', self.traffic_callback, 10)
        self.last_traffic_state = None
        self.light_on = None
        self.c_r = 0
        self.c_y = 0
        self.c_g = 0
        GPIO.setmode(GPIO.BCM)
        for pin in led_pins:
            GPIO.setup(pin, GPIO.OUT)
        # Setup GPIO
        self.servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0025)
        self.move_servo(-10)
        # Start thread
        threading.Thread(target=self.traffic_lights, daemon=True).start()

    def traffic_callback(self, msg: String):
        self.get_logger().info("Received traffic state: %s" % msg.data)
        self.last_traffic_state = msg.data
        self.check_and_move_servo()

    def check_and_move_servo(self):
        if self.last_traffic_state is not None and self.light_on is not None:
            if self.last_traffic_state == self.light_on:
                if self.last_traffic_state == 'yellow':
                    self.c_y += 1
                    self.c_g = 0
                    if self.c_y == 1: 
                        self.move_servo(-40)
                elif self.last_traffic_state == 'green':
                    self.c_g += 1
                    self.c_y = 0
                    if self.c_g == 1:
                        self.move_servo(-90)
                        time.sleep(40)
                        self.move_servo(-10)

    def move_servo(self, angle):
        self.get_logger().info("Servo state: %d" % angle)
        self.servo.angle = angle
        sleep(2)
        self.servo.detach()

    def traffic_lights(self):
        while True:
            # Turn on LEDs in alternate order
            GPIO.output(led_pins[0], GPIO.HIGH)
            self.light_on = 'red'
            time.sleep(10)
            GPIO.output(led_pins[0], GPIO.LOW)

            GPIO.output(led_pins[1], GPIO.HIGH)
            self.light_on = 'yellow'
            time.sleep(10)
            GPIO.output(led_pins[1], GPIO.LOW)

            GPIO.output(led_pins[2], GPIO.HIGH)
            self.light_on = 'green'
            time.sleep(10)
            GPIO.output(led_pins[2], GPIO.LOW)

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

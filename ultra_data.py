import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DistanceSensorNode(Node):
    def __init__(self):
        super().__init__('distance_sensor_node')
        self.publisher_ = self.create_publisher(String, 'distance_topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.GPIO_TRIGGER = 18
        self.GPIO_ECHO = 24
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

    def timer_callback(self):
        dist = self.distance()
        self.get_logger().info("Measured Distance = {} cm".format(round(dist, 4)))
        msg = String()
        msg.data = str(dist)
        self.publisher_.publish(msg)

    def distance(self):
        GPIO.output(self.GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
        StartTime = time.time()
        StopTime = time.time()

        while GPIO.input(self.GPIO_ECHO) == 0:
            StartTime = time.time()

        while GPIO.input(self.GPIO_ECHO) == 1:
            StopTime = time.time()

        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * 34300) / 2
        return distance

def main(args=None):
    rclpy.init(args=args)
    distance_sensor_node = DistanceSensorNode()
    try:
        rclpy.spin(distance_sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        distance_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

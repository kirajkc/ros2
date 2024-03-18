#!/usr/bin/env python3

import time
import board
import adafruit_bno055
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
class ImuPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')
        self.imu_publisher = self.create_publisher(Imu, 'imu_data', 10)
        self.get_logger().info('IMU Publisher Node Initialized')

    def run(self):
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)

        i = 0

        while rclpy.ok():
            try:
                quaternion = sensor.quaternion
                if all(isinstance(x, float) for x in quaternion):
                    orientation_x = quaternion[0]
                    orientation_y = quaternion[1]
                    orientation_z = quaternion[2]
                    orientation_w = quaternion[3]

                gyro = sensor.gyro
                if all(isinstance(x, float) for x in gyro):
                    angular_velocity_x = gyro[0]
                    angular_velocity_y = gyro[1]
                    angular_velocity_z = gyro[2]

                linear_acceleration = sensor.linear_acceleration
                if all(isinstance(x, float) for x in linear_acceleration):
                    linear_acceleration_x = linear_acceleration[0]
                    linear_acceleration_y = linear_acceleration[1]
                    linear_acceleration_z = linear_acceleration[2]

                heading,pitch,roll = sensor.euler

                # Publish data
                imu_msg = Imu()
                imu_msg.orientation.x = orientation_x
                imu_msg.orientation.y = orientation_y
                imu_msg.orientation.z = orientation_z
                imu_msg.orientation.w = orientation_w

                imu_msg.angular_velocity.x = angular_velocity_x
                imu_msg.angular_velocity.y = angular_velocity_y
                imu_msg.angular_velocity.z = angular_velocity_z

                imu_msg.linear_acceleration.x = linear_acceleration_x
                imu_msg.linear_acceleration.y = linear_acceleration_y
                imu_msg.linear_acceleration.z = linear_acceleration_z

                # Add pitch and roll to the message
                imu_msg.orientation_covariance[0] = pitch
                imu_msg.orientation_covariance[1] = roll

                # Set timestamp in the Header
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                imu_msg.header = header

                self.imu_publisher.publish(imu_msg)

                # Print in the terminal in the same format
                # self.get_logger().info(f"\n\
                #     Header:{i}\n\
                #     Orientation:\n\
                #         x: {orientation_x}\n\
                #         y: {orientation_y}\n\
                #         z: {orientation_z}\n\
                #         w: {orientation_w}\n\
                #     Angular Velocity:\n\
                #         x: {angular_velocity_x}\n\
                #         y: {angular_velocity_y}\n\
                #         z: {angular_velocity_z}\n\
                #     Linear Acceleration:\n\
                #         x: {linear_acceleration_x}\n\
                #         y: {linear_acceleration_y}\n\
                #         z: {linear_acceleration_z}\n")

                i += 1
                time.sleep(0.1)

            except Exception as e:
                self.get_logger().error(f"Error: {str(e)}")   

def main():
    rclpy.init()
    imu_publisher_node = ImuPublisherNode()
    try:
        imu_publisher_node.run()
    finally:
        imu_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

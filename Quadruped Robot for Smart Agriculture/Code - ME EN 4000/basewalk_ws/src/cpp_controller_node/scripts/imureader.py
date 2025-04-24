#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_interfaces.msg import Imuinterface as imu # CHANGE

import board
import busio
import adafruit_lsm6ds
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

i2c = busio.I2C(board.I2C5, board.I2C5)
sox = LSM6DSOX(i2c)
timer_period = 0.01 # Every defined seconds, run the timer

class IMUReader(Node):

    def __init__(self):
        super().__init__('imu_reader')
        # Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.
        # "self" is a sort of meta object that refers to the class itself
        self.publisher_ = self.create_publisher(imu, 'raw_imu_topic', 10)
          
        # A timer is created with a callback to execute every # seconds. self.i is a counter used in the callback
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # timer_callback creates a message with the counter value appended, and publishes it to the console with get_logger().info.
        # From msg file import imu.msg, this should allow different nodes to communicate through these variables
        imu_data = imu()

        # Sends the time
        imu_data.timeInc = timer_period

        # Retrieves accelerometer data
        imu_data.accel_x = sox.acceleration[0]
        imu_data.accel_y = sox.acceleration[1]
        imu_data.accel_z = sox.acceleration[2]

        # Retrieves gyroscope data
        imu_data.gyro_x = sox.gyro[0]
        imu_data.gyro_y = sox.gyro[1]
        imu_data.gyro_z = sox.gyro[2]

        # Send everything this node changed to imu.msg
        self.publisher_.publish(imu_data)
        self.get_logger().info('Publishing IMU Data')


def main(args=None):
    rclpy.init(args=args)
    
    imu_reader = IMUReader()
    
    rclpy.spin(imu_reader)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # imu_reader.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()

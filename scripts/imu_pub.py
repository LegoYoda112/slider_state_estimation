#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import Imu

import serial
import re
import numpy as np

class IMUPub(Node):

    def __init__(self):
        super().__init__('imu_pub')

        self.get_logger().info("Starting imu publisher")

        self.orientation_publisher = self.create_publisher(Quaternion, '/slider/sensors/imu/orientation', 10)
        self.imu_publisher = self.create_publisher(Imu, '/slider/sensors/imu/imu', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/slider/sensors/imu/pose', 10)

        self.ser = serial.Serial('/dev/ttyACM0',
                    baudrate=115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1,
                    xonxoff=0,
                    rtscts=0)
    
        self.ser.flush()
        
        self.get_logger().info("Connected to" + self.ser.name)
    
    def run_loop(self):
        while(rclpy.ok):
            # Read line and parse
            line = self.ser.readline()
            # print(line)
            try:
                data = np.array(re.findall(r"[-0-9.]+", str(line)), float)
            except ValueError:
                self.get_logger().warn("IMU ValueError")
                data = []

            # If we've got rpy data, convert to quaternion
            if(len(data) == 10):
                quat_msg = Quaternion()
                imu_msg = Imu()

                w, x, y, z, gx, gy, gz, ax, ay, az = data

                # Fill in quaternion message
                quat_msg.w = w
                quat_msg.x = x
                quat_msg.y = y
                quat_msg.z = z

                # Set imu orientation
                imu_msg.orientation = quat_msg
                imu_msg.orientation_covariance[0] = -1 # TODO: fill in

                # Set angular velocity and linear acceleration
                imu_msg.angular_velocity.x = gx
                imu_msg.angular_velocity.y = gy
                imu_msg.angular_velocity.z = gz
                imu_msg.angular_velocity_covariance[0] = -1 # TODO: fill in

                imu_msg.linear_acceleration.x = ax
                imu_msg.linear_acceleration.y = ay
                imu_msg.linear_acceleration.z = az
                imu_msg.linear_acceleration_covariance[0] = -1 # TODO: fill in

                # Make pose message (for rviz)
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "world"

                pose_msg.pose.orientation = quat_msg

                # Publish messages
                self.orientation_publisher.publish(quat_msg)
                self.imu_publisher.publish(imu_msg)
                self.pose_publisher.publish(pose_msg)

            rclpy.spin_once(self, timeout_sec = 0)

def main(args=None):
    rclpy.init(args=args)

    imu_pub = IMUPub()

    imu_pub.run_loop()
    imu_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped

import serial
import re
import numpy as np

# From https://math.stackexchange.com/questions/2975109/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
def euler_to_quaternion(r):
    (roll, pitch, yaw) = (r[0], -r[1], r[2])
    yaw = 0.0
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class IMUPub(Node):

    def __init__(self):
        super().__init__('imu_pub')

        self.get_logger().info("Starting imu publisher")

        self.orientation_publisher = self.create_publisher(Quaternion, '/slider/sensors/imu/orientation', 10)
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
            # print(str(line))
            rpy = np.array(re.findall(r"[-0-9.]+", str(line)), float)

            # If we've got rpy data, convert to quaternion
            if(len(rpy) == 3):

                quat_msg = Quaternion()

                quat = euler_to_quaternion(rpy)
                print(quat)

                # Fill in quaternion message
                quat_msg.x = quat[0]
                quat_msg.y = quat[1]
                quat_msg.z = quat[2]
                quat_msg.w = quat[3]

                # Publish message
                self.orientation_publisher.publish(quat_msg)

                # Make pose message (for rviz)
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "world"

                pose_msg.pose.orientation = quat_msg

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

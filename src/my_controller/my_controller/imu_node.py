#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from icm20948 import ICM20948

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Subscribe to wheel odometry to detect standstill
        self.create_subscription(Odometry, 'wheel/odom', self.odom_callback, 10)
        self.wheels_moving = False

        try:
            self.imu = ICM20948()
            self.get_logger().info("ICM-20948 IMU Node Started")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize IMU: {e}")
            self.imu = None

    def odom_callback(self, msg):
        """Track whether the wheels are moving based on wheel odometry."""
        lin = abs(msg.twist.twist.linear.x)
        ang = abs(msg.twist.twist.angular.z)
        # Consider wheels moving if either linear or angular velocity exceeds threshold
        self.wheels_moving = lin > 0.005 or ang > 0.01

    def timer_callback(self):
        if self.imu is None:
            return
        
        try:
            ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
        except Exception as e:
            self.get_logger().error(f"Failed to read from IMU: {e}")
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Zero out gyro when wheels are not moving to prevent standstill drift
        if self.wheels_moving:
            msg.angular_velocity.x = math.radians(gx)
            msg.angular_velocity.y = math.radians(gy)
            msg.angular_velocity.z = math.radians(gz)
        else:
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0

        msg.linear_acceleration.x = ax * 9.81
        msg.linear_acceleration.y = ay * 9.81
        msg.linear_acceleration.z = az * 9.81

        msg.orientation_covariance[0] = -1.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

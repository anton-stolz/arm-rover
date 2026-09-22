#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import struct
import math
import numpy as np

class BumpMapper(Node):
    def __init__(self):
        super().__init__("bump_mapper")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers:
        # 1. PointCloud2 for Nav2 Costmap (in odom or map frame)
        self.cloud_pub = self.create_publisher(PointCloud2, "/bumper_obstacles", 10)
        # 2. OccupancyGrid for visualization in Lichtblick/Foxglove or mapping
        self.grid_pub = self.create_publisher(OccupancyGrid, "/bumper_map", 10)

        # Subscribers to bumper hits
        self.create_subscription(Bool, "bumper/front_hit", self.front_cb, 10)
        self.create_subscription(Bool, "bumper/back_hit", self.back_cb, 10)

        # List of bumper hit points:
        # Each entry: {'x_odom': float, 'y_odom': float, 'stamp': float, 'weight': 100}
        self.points = []

        # Dedup debounce timer
        self.last_front_hit_time = 0.0
        self.last_back_hit_time = 0.0

        # Occupancy grid settings
        self.resolution = 0.05  # 5cm
        self.grid_size = 120    # 6m x 6m around robot
        
        # Periodic timer (2 Hz) to reapply TF drift and publish
        self.create_timer(0.5, self.update_and_publish)
        self.get_logger().info("BumpMapper node initialized. Tracking bumper hits in odom & re-projecting to map.")

    def front_cb(self, msg: Bool):
        if not msg.data:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_front_hit_time < 0.8:
            return
        self.last_front_hit_time = now
        # Spread 5 points along the front 30cm bumper edge (x=+0.15m, y in [-0.14, +0.14])
        self.record_bumper_segment(offset_x=0.15, y_min=-0.14, y_max=0.14, num_pts=5, now=now)

    def back_cb(self, msg: Bool):
        if not msg.data:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_back_hit_time < 0.8:
            return
        self.last_back_hit_time = now
        # Spread 5 points along the back 30cm bumper edge (x=-0.15m, y in [-0.14, +0.14])
        self.record_bumper_segment(offset_x=-0.15, y_min=-0.14, y_max=0.14, num_pts=5, now=now)

    def record_bumper_segment(self, offset_x, y_min, y_max, num_pts, now):
        try:
            # Transform from base_link to odom
            t = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            # Compute yaw from quaternion
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            y_vals = np.linspace(y_min, y_max, num_pts)
            for y_loc in y_vals:
                # Rotate (offset_x, y_loc) by yaw and translate
                gx = tx + (offset_x * cos_yaw - y_loc * sin_yaw)
                gy = ty + (offset_x * sin_yaw + y_loc * cos_yaw)
                self.points.append({
                    'x_odom': gx,
                    'y_odom': gy,
                    'stamp': now,
                    'intensity': 100.0
                })
            self.get_logger().warn(f"Recorded bumper collision! Total persistent obstacle points: {len(self.points)}")
        except Exception as e:
            self.get_logger().error(f"Failed to lookup transform for bumper event: {e}")

    def update_and_publish(self):
        now = self.get_clock().now().nanoseconds / 1e9
        
        # 1. Age points: gradually decay from 100 to 0 over 300 seconds (~5 minutes)
        # Initially (first 60s) stays full lethal 100, then fades down
        alive_points = []
        for p in self.points:
            age = now - p['stamp']
            if age < 60.0:
                p['intensity'] = 100.0
                alive_points.append(p)
            elif age < 300.0:
                # Linearly decrease from 100 down to 25
                p['intensity'] = 100.0 - (age - 60.0) / (300.0 - 60.0) * 75.0
                alive_points.append(p)
            # Beyond 300 seconds, discard
        self.points = alive_points

        if not self.points:
            return

        # 2. Transform points from odom -> map using latest SLAM loop-closure transform!
        target_frame = "map"
        try:
            t_map_odom = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            mx = t_map_odom.transform.translation.x
            my = t_map_odom.transform.translation.y
            q = t_map_odom.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw_m = math.atan2(siny_cosp, cosy_cosp)
            cos_m = math.cos(yaw_m)
            sin_m = math.sin(yaw_m)

            pts_transformed = []
            for p in self.points:
                px = mx + (p['x_odom'] * cos_m - p['y_odom'] * sin_m)
                py = my + (p['x_odom'] * sin_m + p['y_odom'] * cos_m)
                pts_transformed.append((px, py, p['intensity']))
        except Exception:
            # Fallback to odom frame if SLAM map frame is not yet available
            target_frame = "odom"
            pts_transformed = [(p['x_odom'], p['y_odom'], p['intensity']) for p in self.points]

        # 3. Publish PointCloud2 (for Nav2 Costmap ObstacleLayer)
        self.publish_cloud(pts_transformed, target_frame)

    def publish_cloud(self, pts, frame_id):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * len(pts)
        msg.is_dense = True

        buffer = bytearray()
        for px, py, intensity in pts:
            buffer.extend(struct.pack('ffff', px, py, 0.05, float(intensity)))
        msg.data = bytes(buffer)

        self.cloud_pub.publish(msg)

def main():
    rclpy.init()
    node = BumpMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

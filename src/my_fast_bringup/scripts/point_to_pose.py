#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class PointToPose(Node):
    def __init__(self):
        super().__init__("point_to_pose_translator")
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Topic publisher
        self.pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        
        # Action client directly to bt_navigator
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # TRANSIENT_LOCAL + RELIABLE QoS matching foxglove_bridge publisher
        qos_transient = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        # Also standard volatile QoS
        qos_volatile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.sub_tl = self.create_subscription(PointStamped, "/goal_point", self.cb_point_stamped, qos_transient)
        self.sub_vol = self.create_subscription(PointStamped, "/goal_point", self.cb_point_stamped, qos_volatile)

        self.get_logger().info("Point-to-Pose translator active! Subscribed to /goal_point with TRANSIENT_LOCAL & VOLATILE.")

    def cb_point_stamped(self, msg: PointStamped):
        self.get_logger().info(f">>> [TRANSLATOR] Got PointStamped on /goal_point: frame={msg.header.frame_id}, x={msg.point.x:.2f}, y={msg.point.y:.2f}")

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        target_frame = msg.header.frame_id if msg.header.frame_id else "map"
        pose.header.frame_id = target_frame
        pose.pose.position = msg.point

        yaw = 0.0
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            dx = msg.point.x - rx
            dy = msg.point.y - ry
            dist = math.hypot(dx, dy)
            if dist > 0.05:
                yaw = math.atan2(dy, dx)
                self.get_logger().info(
                    f"Point ({msg.point.x:.2f}, {msg.point.y:.2f}) from robot ({rx:.2f}, {ry:.2f}) -> "
                    f"heading {math.degrees(yaw):.1f} deg"
                )
        except Exception as e:
            self.get_logger().warn(f"TF lookup fallback: {e}")

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 1. Publish to /goal_pose topic
        self.pub.publish(pose)
        self.get_logger().info(f">>> Published PoseStamped to /goal_pose: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")

        # 2. Also dispatch directly via Nav2 action client if available
        if self.action_client.server_is_ready():
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            self.action_client.send_goal_async(goal_msg)
            self.get_logger().info("Dispatched goal directly via NavigateToPose ActionClient!")
        else:
            self.get_logger().warn("NavigateToPose action server not ready yet; relies on /goal_pose topic.")

def main():
    rclpy.init()
    node = PointToPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass

if __name__ == "__main__":
    main()

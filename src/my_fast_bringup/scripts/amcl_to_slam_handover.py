#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from slam_toolbox.srv import DeserializePoseGraph
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import math
import time

class AmclToSlamHandover(Node):
    def __init__(self):
        super().__init__('amcl_to_slam_handover')

        self.declare_parameter('cov_threshold', 0.15)
        self.declare_parameter('consecutive_hits', 5)
        self.declare_parameter('posegraph_filename', '/home/anton/ros2_ws/src/my_fast_bringup/maps/complete_map')

        self.cov_threshold = self.get_parameter('cov_threshold').value
        self.required_hits = self.get_parameter('consecutive_hits').value
        self.filename = self.get_parameter('posegraph_filename').value

        self.hit_count = 0
        self.handover_done = False

        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.slam_client = self.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
        self.amcl_lifecycle = self.create_client(ChangeState, '/amcl/change_state')

        self.get_logger().info(
            f"AMCL->SLAM Handover Node active. Waiting for covariance < {self.cov_threshold} "
            f"for {self.required_hits} consecutive readings..."
        )

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        if self.handover_done:
            return

        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]
        cov_yaw = msg.pose.covariance[35]
        max_pos_cov = max(cov_x, cov_y)

        if max_pos_cov < self.cov_threshold:
            self.hit_count += 1
            self.get_logger().info(
                f"[HANDOVER PROGRESS] Stable reading {self.hit_count}/{self.required_hits}: "
                f"cov_pos={max_pos_cov:.3f}, cov_yaw={cov_yaw:.3f}"
            )
        else:
            if self.hit_count > 0:
                self.get_logger().info(f"[HANDOVER RESET] Covariance rose to {max_pos_cov:.3f}. Resetting count.")
            self.hit_count = 0

        if self.hit_count >= self.required_hits:
            self.handover_done = True
            self.trigger_handover(msg)

    def trigger_handover(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)

        self.get_logger().info(
            f"*** STABLE AMCL CONVERGENCE ACHIEVED! ***\n"
            f"Pose: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°\n"
            f"Deactivating AMCL and seeding SLAM Toolbox Ceres Optimizer..."
        )

        # 1. Hand over to SLAM Toolbox with LOCALIZE_AT_POSE (match_type=3)
        if self.slam_client.wait_for_service(timeout_sec=5.0):
            req = DeserializePoseGraph.Request()
            req.filename = self.filename
            req.match_type = 3 # LOCALIZE_AT_POSE
            req.initial_pose.x = x
            req.initial_pose.y = y
            req.initial_pose.theta = yaw

            future = self.slam_client.call_async(req)
            self.get_logger().info("Dispatched seed pose to SLAM Toolbox!")
        else:
            self.get_logger().error("SLAM Toolbox deserialize service unavailable!")

        # 2. Deactivate AMCL so it yields map->odom TF to SLAM Toolbox
        if self.amcl_lifecycle.wait_for_service(timeout_sec=3.0):
            req_lc = ChangeState.Request()
            req_lc.transition.id = Transition.TRANSITION_DEACTIVATE
            self.amcl_lifecycle.call_async(req_lc)
            self.get_logger().info("Deactivated AMCL. SLAM Toolbox now owns map->odom TF.")

        self.get_logger().info("Handover complete! Shutting down handover supervisor.")
        # Node can exit cleanly after transition
        self.create_timer(3.0, lambda: rclpy.shutdown())

def main(args=None):
    rclpy.init(args=args)
    node = AmclToSlamHandover()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import subprocess
import os
import signal
import math
import time

class SequentialHandoverManager(Node):
    def __init__(self):
        super().__init__('sequential_handover_manager')

        self.declare_parameter('cov_threshold', 0.15)
        self.declare_parameter('consecutive_hits', 6)
        self.declare_parameter('params_path', '/home/anton/ros2_ws/src/my_fast_bringup/config/mapper_params_localization.yaml')

        self.cov_threshold = self.get_parameter('cov_threshold').value
        self.required_hits = self.get_parameter('consecutive_hits').value
        self.params_path = self.get_parameter('params_path').value

        self.scatter_triggered = False
        self.seen_high_cov = False
        self.hit_count = 0
        self.converged = False
        self.best_pose = None

        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.cb_amcl,
            10
        )

        # Trigger global localization directly as soon as service is ready
        self.timer_init = self.create_timer(1.0, self.check_and_trigger_global_loc)

        self.get_logger().info(
            f"=== Sequential Handover Supervisor Ready ===\n"
            f"Phase 1: Waiting to trigger Global Scatter across map..."
        )

    def check_and_trigger_global_loc(self):
        if not self.scatter_triggered:
            if self.global_loc_client.service_is_ready():
                self.global_loc_client.call_async(Empty.Request())
                self.scatter_triggered = True
                self.get_logger().info(">>> GLOBAL SCATTER TRIGGERED! Particles dispersed across entire map.")
                self.timer_init.cancel()

    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        if self.converged:
            return

        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]
        max_cov = max(cov_x, cov_y)

        # Step 1: Wait until we actually see the high covariance from the global scatter
        if not self.seen_high_cov:
            if max_cov > 1.0:
                self.seen_high_cov = True
                self.get_logger().info(f"[SEARCH ACTIVE] Confirmed global scatter (cov={max_cov:.2f}). Waiting for convergence...")
            return

        # Step 2: Now wait for covariance to drop below threshold
        if max_cov < self.cov_threshold:
            self.hit_count += 1
            self.best_pose = msg
            self.get_logger().info(f"[CONVERGING] Stable reading {self.hit_count}/{self.required_hits} (cov={max_cov:.3f})")
        else:
            if self.hit_count > 0:
                self.get_logger().info(f"[STILL MOVING] Covariance {max_cov:.3f} > {self.cov_threshold}. Continuing search...")
            self.hit_count = 0

        if self.hit_count >= self.required_hits:
            self.converged = True
            self.execute_handover()

    def execute_handover(self):
        x = self.best_pose.pose.pose.position.x
        y = self.best_pose.pose.pose.position.y
        qz = self.best_pose.pose.pose.orientation.z
        qw = self.best_pose.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)

        self.get_logger().info(
            f"\n************************************************\n"
            f"*** AMCL GLOBAL LOCALIZATION CONVERGED! ***\n"
            f"True Position: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°\n"
            f"Terminating AMCL and handing over to SLAM Toolbox Ceres Scan Matcher...\n"
            f"************************************************\n"
        )

        # 1. Kill AMCL and map_server completely
        subprocess.run(["pkill", "-9", "-f", "nav2_amcl/amcl"], check=False)
        subprocess.run(["pkill", "-9", "-f", "nav2_map_server/map_server"], check=False)
        subprocess.run(["pkill", "-9", "-f", "lifecycle_manager_localization"], check=False)
        time.sleep(1.0)

        # 2. Seed SLAM Toolbox with exact AMCL pose
        cmd_update = f"""python3 -c "
import yaml
with open('{self.params_path}', 'r') as f:
    cfg = yaml.safe_load(f)
cfg['slam_toolbox']['ros__parameters']['map_start_pose'] = [{x:.3f}, {y:.3f}, {yaw:.3f}]
with open('{self.params_path}', 'w') as f:
    yaml.dump(cfg, f, default_flow_style=False)
" """
        subprocess.run(cmd_update, shell=True, check=True)

        # 3. Launch SLAM Toolbox localization
        launch_cmd = [
            "ros2", "launch", "slam_toolbox", "localization_launch.py",
            f"slam_params_file:={self.params_path}",
            "use_sim_time:=false"
        ]
        subprocess.Popen(launch_cmd)

        self.get_logger().info("SLAM Toolbox active and locked! Supervisor exiting.")
        time.sleep(2.0)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SequentialHandoverManager()
    try:
        rclpy.spin(node)
    except:
        pass

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, UInt16, Bool


class BumperNode(Node):
    def __init__(self):
        super().__init__('bumper_node')

        # ── Exponential-moving-average state ──
        self.decay_rate = 0.94
        self.current_left = None
        self.current_right = None

        # ── Collision thresholds (from calibration) ──
        # Front hit: left goes HIGH (~465), right goes LOW (~320)
        # Back hit:  left goes LOW  (~260), right goes HIGH (~490)
        self.thresh_left_front = 465.0
        self.thresh_left_back = 260.0
        self.thresh_right_front = 320.0
        self.thresh_right_back = 490.0

        # ── Recovery parameters ──
        self.recovery_speed = 0.10        # m/s reverse speed
        self.recovery_drive_sec = 0.6     # seconds to drive back
        self.recovery_pause_sec = 0.5     # seconds to pause after reversing

        # ── Recovery state ──
        # None | 'driving' | 'pausing'
        self.recovery_phase = None
        self.recovery_direction = None    # 'front' or 'back'
        self.recovery_start_time = None

        # ── Subscribers ──
        self.create_subscription(
            UInt16, '/bumper_left_raw', self.listener_left, 10)
        self.create_subscription(
            UInt16, '/bumper_right_raw', self.listener_right, 10)
        self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        # ── Publishers ──
        self.av_bumper_l = self.create_publisher(Float32, 'bumper_left_averaged', 10)
        self.av_bumper_r = self.create_publisher(Float32, 'bumper_right_averaged', 10)
        self.front_hit_pub = self.create_publisher(Bool, 'bumper/front_hit', 10)
        self.back_hit_pub = self.create_publisher(Bool, 'bumper/back_hit', 10)
        self.safe_vel_pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)

        # ── Passthrough: latest user cmd_vel ──
        self.last_user_twist = Twist()
        self.last_user_twist_time = self.get_clock().now().nanoseconds / 1e9

        # ── Main loop at 10 Hz (plenty for bumper reaction) ──
        self.create_timer(0.1, self.tick)

        self.get_logger().info('Bumper node started (collision + safe_vel)')

    # ────────────────────────────────────────────
    #  Sensor callbacks (EMA filtering only, no publishing)
    # ────────────────────────────────────────────

    def listener_left(self, msg):
        v = float(msg.data)
        if self.current_left is None:
            self.current_left = v
        else:
            self.current_left = self.decay_rate * self.current_left + v * (1.0 - self.decay_rate)

    def listener_right(self, msg):
        v = float(msg.data)
        if self.current_right is None:
            self.current_right = v
        else:
            self.current_right = self.decay_rate * self.current_right + v * (1.0 - self.decay_rate)

    # ────────────────────────────────────────────
    #  User cmd_vel passthrough
    # ────────────────────────────────────────────

    def cmd_vel_cb(self, msg):
        self.last_user_twist = msg
        self.last_user_twist_time = self.get_clock().now().nanoseconds / 1e9

    # ────────────────────────────────────────────
    #  Collision detection
    # ────────────────────────────────────────────

    def check_collision(self):
        """Return 'front', 'back', or None."""
        if self.current_left is None or self.current_right is None:
            return None

        front = (self.current_left >= self.thresh_left_front or
                 self.current_right <= self.thresh_right_front)
        back = (self.current_left <= self.thresh_left_back or
                self.current_right >= self.thresh_right_back)

        if front:
            return 'front'
        if back:
            return 'back'
        return None

    # ────────────────────────────────────────────
    #  Main tick (10 Hz)
    # ────────────────────────────────────────────

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # Publish averaged bumper values (at tick rate, not per-sample)
        if self.current_left is not None:
            self.av_bumper_l.publish(Float32(data=self.current_left))
        if self.current_right is not None:
            self.av_bumper_r.publish(Float32(data=self.current_right))

        # ── Currently in recovery ──
        if self.recovery_phase is not None:
            elapsed = now - self.recovery_start_time

            if self.recovery_phase == 'driving':
                if elapsed < self.recovery_drive_sec:
                    twist = Twist()
                    if self.recovery_direction == 'front':
                        twist.linear.x = -self.recovery_speed
                    else:
                        twist.linear.x = self.recovery_speed
                    self.safe_vel_pub.publish(twist)
                    return
                else:
                    self.recovery_phase = 'pausing'
                    self.recovery_start_time = now

            if self.recovery_phase == 'pausing':
                if elapsed < self.recovery_pause_sec:
                    self.safe_vel_pub.publish(Twist())
                    return
                else:
                    self.get_logger().info(
                        f'Recovery from {self.recovery_direction} collision complete')
                    self.recovery_phase = None
                    self.recovery_direction = None

        # ── Check for new collisions ──
        collision = self.check_collision()

        self.front_hit_pub.publish(Bool(data=(collision == 'front')))
        self.back_hit_pub.publish(Bool(data=(collision == 'back')))

        if collision is not None:
            self.get_logger().warn(f'Collision detected: {collision}')

            self.recovery_phase = 'driving'
            self.recovery_direction = collision
            self.recovery_start_time = now

            self.safe_vel_pub.publish(Twist())
            return

        # ── No collision, pass through user command ──
        if (now - self.last_user_twist_time) > 0.5:
            self.last_user_twist = Twist()
        self.safe_vel_pub.publish(self.last_user_twist)


def main():
    rclpy.init()
    node = BumperNode()

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

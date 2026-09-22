import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16
from tf2_ros import TransformBroadcaster
import serial


# ============================================================
# Protocol constants
# ============================================================

CMD_MAGIC1 = 0xAA
CMD_MAGIC2 = 0x55

TEL_MAGIC = bytes([0xAB, 0xCD])

# Matches final Arduino TelemetryPacket:
#
# uint8_t   magic1
# uint8_t   magic2
# uint32_t  time_ms
# float     left_angle_deg
# float     right_angle_deg
# float     left_vel_deg_s
# float     right_vel_deg_s
# int16_t   left_target_dps10
# int16_t   right_target_dps10
# uint8_t   left_pwm
# uint8_t   right_pwm
# uint8_t   state_left
# uint8_t   state_right
# uint16_t  raw_bumper_left
# uint16_t  raw_bumper_right
# uint8_t   checksum
#
TEL_FORMAT = '<BBIffffhhBBBBHHB'
TEL_SIZE = struct.calcsize(TEL_FORMAT)

TEL_IDX_TIME_MS          = 2
TEL_IDX_LEFT_ANGLE_DEG   = 3
TEL_IDX_RIGHT_ANGLE_DEG  = 4
TEL_IDX_LEFT_VEL_DEG_S   = 5
TEL_IDX_RIGHT_VEL_DEG_S  = 6
TEL_IDX_LEFT_TARGET_DPS  = 7
TEL_IDX_RIGHT_TARGET_DPS = 8
TEL_IDX_LEFT_PWM         = 9
TEL_IDX_RIGHT_PWM        = 10
TEL_IDX_STATE_LEFT       = 11
TEL_IDX_STATE_RIGHT      = 12
TEL_IDX_BUMPER_LEFT_RAW  = 13
TEL_IDX_BUMPER_RIGHT_RAW = 14


class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.031)
        self.declare_parameter('wheel_separation', 0.190)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('max_wheel_deg_s', 360.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('connect_delay_s', 0.5)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.max_wheel_deg_s = float(self.get_parameter('max_wheel_deg_s').value)
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.connect_delay_s = float(self.get_parameter('connect_delay_s').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        if self.publish_rate <= 0.0:
            self.publish_rate = 20.0

        # ---------------- State ----------------
        self.ser = None
        self.ser_lock = threading.Lock()
        self.tel_lock = threading.Lock()

        self.latest_tel = None
        self.last_telemetry_mono = 0.0

        self.stop_reader = False
        self.pkt_count = 0
        self.last_hz_log = time.monotonic()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_angles = None
        self.last_odom_time = None

        self.last_lin = 0.0
        self.last_ang = 0.0
        self.last_cmd_time = 0.0

        # ---------------- ROS interfaces ----------------
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)

        self.bumper_left_pub = self.create_publisher(UInt16, 'bumper_left_raw', 10)
        self.bumper_right_pub = self.create_publisher(UInt16, 'bumper_right_raw', 10)

        self.create_subscription(Twist, 'cmd_vel_safe', self.cmd_vel_cb, 10)

        self.tf_br = TransformBroadcaster(self)

        self.create_timer(1.0 / self.publish_rate, self.timer_cb)

        threading.Thread(target=self.serial_reader, daemon=True).start()

        self.get_logger().info(
            f'diff_drive_node ready, telemetry size: {TEL_SIZE} bytes'
        )

    # ============================================================
    # Serial helpers
    # ============================================================

    def ensure_serial(self):
        if self.ser is not None and self.ser.is_open:
            return True

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.01
            )

            # Arduino Uno / clones often reset when the serial port opens.
            # Give it a short moment so the bootloader/sketch can settle.
            if self.connect_delay_s > 0.0:
                time.sleep(self.connect_delay_s)

            self.ser.reset_input_buffer()
            self.get_logger().info(f'Serial connected: {self.port}')
            return True

        except Exception as e:
            self.get_logger().warn(f'Serial not reachable ({self.port}): {e}')
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            return False

    def send_command(self, left_dps, right_dps):
        if not self.ensure_serial():
            return

        l10 = int(max(-32000, min(32000, left_dps * 10.0)))
        r10 = int(max(-32000, min(32000, right_dps * 10.0)))

        payload = struct.pack(
            '<BBhh',
            CMD_MAGIC1,
            CMD_MAGIC2,
            l10,
            r10
        )

        checksum = sum(payload) & 0xFF
        pkt = payload + bytes([checksum])

        with self.ser_lock:
            try:
                self.ser.write(pkt)
            except Exception as e:
                self.get_logger().warn(f'Serial write failed: {e}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def serial_reader(self):
        buf = bytearray()

        while not self.stop_reader:
            ser = self.ser

            if ser is None or not ser.is_open:
                time.sleep(0.1)
                continue

            try:
                chunk = ser.read(ser.in_waiting or 1)
            except Exception:
                time.sleep(0.1)
                continue

            if not chunk:
                time.sleep(0.001)
                continue

            buf.extend(chunk)

            while len(buf) >= TEL_SIZE:
                idx = buf.find(TEL_MAGIC)

                if idx == -1:
                    # Prevent unbounded memory growth if we are receiving garbage.
                    if len(buf) > TEL_SIZE * 4:
                        buf.clear()
                    break

                if idx > 0:
                    del buf[:idx]

                if len(buf) < TEL_SIZE:
                    break

                pkt = bytes(buf[:TEL_SIZE])

                # Checksum is the last byte and covers all previous bytes.
                if (sum(pkt[:-1]) & 0xFF) == pkt[-1]:
                    try:
                        d = struct.unpack(TEL_FORMAT, pkt)
                    except Exception as e:
                        self.get_logger().warn(f'Telemetry unpack failed: {e}')
                        del buf[:1]
                        continue

                    with self.tel_lock:
                        self.latest_tel = d

                    self.last_telemetry_mono = time.monotonic()
                    self.pkt_count += 1

                    del buf[:TEL_SIZE]
                else:
                    # Bad checksum: resync slowly.
                    del buf[:1]

    # ============================================================
    # cmd_vel -> wheel speeds
    # ============================================================

    def cmd_vel_cb(self, msg):
        self.last_lin = msg.linear.x
        self.last_ang = msg.angular.z
        self.last_cmd_time = self.get_clock().now().nanoseconds / 1e9

    def twist_to_wheels(self, lin, ang):
        v_l = lin - ang * self.wheel_separation / 2.0
        v_r = lin + ang * self.wheel_separation / 2.0

        dps_l = math.degrees(v_l / self.wheel_radius) * self.gear_ratio
        dps_r = math.degrees(v_r / self.wheel_radius) * self.gear_ratio

        dps_l = max(-self.max_wheel_deg_s, min(self.max_wheel_deg_s, dps_l))
        dps_r = max(-self.max_wheel_deg_s, min(self.max_wheel_deg_s, dps_r))

        return dps_r, dps_l #reversed to fix reverse connected motors meaning

    # ============================================================
    # Main periodic callback
    # ============================================================

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # Telemetry rate logging
        t_mono = time.monotonic()
        if t_mono - self.last_hz_log >= 2.0:
            hz = self.pkt_count / max(1e-6, t_mono - self.last_hz_log)
            self.get_logger().debug(f'Telemetry rate: {hz:.1f} Hz')
            self.pkt_count = 0
            self.last_hz_log = t_mono

        # Send command, or stop if cmd_vel timed out
        if (now - self.last_cmd_time) > self.cmd_timeout_s:
            lin, ang = 0.0, 0.0
        else:
            lin, ang = self.last_lin, self.last_ang

        left_dps, right_dps = self.twist_to_wheels(lin, ang)
        self.send_command(left_dps, right_dps)

        # Get latest telemetry
        with self.tel_lock:
            tel = self.latest_tel

        if tel is None:
            return

        # If telemetry is too old, reset odometry state to avoid jumps.
        if time.monotonic() - self.last_telemetry_mono > 1.0:
            self.last_angles = None
            self.last_odom_time = None
            return

        # ---------------- Publish bumper raw values ----------------
        left_raw = int(tel[TEL_IDX_BUMPER_LEFT_RAW])
        right_raw = int(tel[TEL_IDX_BUMPER_RIGHT_RAW])

        self.bumper_left_pub.publish(UInt16(data=left_raw))
        self.bumper_right_pub.publish(UInt16(data=right_raw))

        # ---------------- Odometry ----------------
        angles = (
            float(tel[TEL_IDX_LEFT_ANGLE_DEG]),
            float(tel[TEL_IDX_RIGHT_ANGLE_DEG])
        )

        if self.last_angles is None or self.last_odom_time is None:
            self.last_angles = angles
            self.last_odom_time = now
            return

        dt = now - self.last_odom_time
        if dt <= 0.0:
            return

        # Keep your original signs from the old node.
        dl = -math.radians(angles[0] - self.last_angles[0]) / self.gear_ratio * self.wheel_radius
        dr = -math.radians(angles[1] - self.last_angles[1]) / self.gear_ratio * self.wheel_radius

        self.last_angles = angles
        self.last_odom_time = now

        d_center = (dl + dr) / 2.0
        d_theta = (dl - dr) / self.wheel_separation * 0.6

        self.x -= d_center * math.cos(self.yaw + d_theta / 2.0)
        self.y -= d_center * math.sin(self.yaw + d_theta / 2.0)
        self.yaw += d_theta

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)

        odom.pose.covariance[0] = 0.8
        odom.pose.covariance[7] = 0.8
        odom.pose.covariance[35] = 3.0

        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_theta / dt

        odom.twist.covariance[0] = 0.8
        odom.twist.covariance[7] = 0.8
        odom.twist.covariance[35] = 3.0

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame

            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.w = math.cos(self.yaw / 2.0)
            tf.transform.rotation.z = math.sin(self.yaw / 2.0)

            self.tf_br.sendTransform(tf)

    # ============================================================
    # Shutdown
    # ============================================================

    def stop(self):
        self.stop_reader = True

        # Try to send one last stop command if serial is still open.
        try:
            if self.ser is not None and self.ser.is_open:
                payload = struct.pack(
                    '<BBhh',
                    CMD_MAGIC1,
                    CMD_MAGIC2,
                    0,
                    0
                )
                pkt = payload + bytes([sum(payload) & 0xFF])

                with self.ser_lock:
                    self.ser.write(pkt)
        except Exception:
            pass

        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = DiffDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


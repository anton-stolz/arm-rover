import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import pigpio
import math

class DiffController(Node):
    def __init__(self):
        super().__init__('diff_controller')
        # declare params
        self.declare_parameter('wheel_separation', 0.32)
        self.declare_parameter('max_wheel_speed', 0.5)
        self.declare_parameter('lf_pin', 12)
        self.declare_parameter('lb_pin', 13)
        self.declare_parameter('rf_pin', 18)
        self.declare_parameter('rb_pin', 19)
        self.declare_parameter('lf_stop', 1500)
        self.declare_parameter('lb_stop', 1500)
        self.declare_parameter('rf_stop', 1500)
        self.declare_parameter('rb_stop', 1500)
        self.declare_parameter('left_reversed', True)
        self.declare_parameter('right_reversed', False)
        self.declare_parameter('pwm_range', 400)

        # READ them once
        self.L = self.get_parameter('wheel_separation').value
        self.max_speed = self.get_parameter('max_wheel_speed').value
        self.lf_pin = self.get_parameter('lf_pin').value
        self.lb_pin = self.get_parameter('lb_pin').value
        self.rf_pin = self.get_parameter('rf_pin').value
        self.rb_pin = self.get_parameter('rb_pin').value
        self.lf_stop = self.get_parameter('lf_stop').value
        self.lb_stop = self.get_parameter('lb_stop').value
        self.rf_stop = self.get_parameter('rf_stop').value
        self.rb_stop = self.get_parameter('rb_stop').value
        self.left_reversed = self.get_parameter('left_reversed').value
        self.right_reversed = self.get_parameter('right_reversed').value
        self.pwm_range = self.get_parameter('pwm_range').value


        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('pigpiod not running :(')
        for pin in [self.lf_pin, self.lb_pin, self.rf_pin, self.rb_pin]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
        self.set_motor('lf',0)
        self.set_motor('rf',0)
        self.set_motor('lb',0)
        self.set_motor('rb',0)


       
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        self.tf_br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(0.05, self.odom_loop)

 


    def set_motor(self, motor, speed_norm):
        range = self.get_parameter('pwm_range').value
        if motor == 'lf':
            reversed = self.left_reversed
            pin = self.lf_pin
            stop = self.lf_stop
        if motor == 'lb':
            reversed = self.left_reversed
            pin = self.lb_pin
            stop = self.lb_stop
        if motor == 'rf':
            reversed = self.right_reversed
            pin = self.rf_pin
            stop = self.rf_stop
        if motor == 'rb':
            reversed = self.right_reversed
            pin = self.rb_pin
            stop = self.rb_stop
        
        if reversed:
            speed_norm *= -1
        self.pi.set_servo_pulsewidth(pin, stop + int(speed_norm*range))

    def cmd_cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        L = self.L
        left_mps = v - w*L/2.0
        right_mps = v + w*L/2.0
        left_norm = max(-1.0, min(1.0, left_mps/self.max_speed))
        right_norm = max(-1.0, min(1.0, right_mps/self.max_speed))
        self.set_motor('lf', left_norm)
        self.set_motor('lb', left_norm)
        self.set_motor('rf', right_norm)
        self.set_motor('rb', right_norm)
        self.v = v
        self.w = w
        self.last_cmd_time = self.get_clock().now()
        
    def odom_loop(self):
        now = self.get_clock().now()
        dt = (now-self.last_time).nanoseconds/1e9
        self.last_time = now
        
        if (now - self.last_cmd_time).nanoseconds/1e9 > 0.5:
            self.set_motor('lf',0)
            self.set_motor('rf',0)
            self.set_motor('lb',0)
            self.set_motor('rb',0)
            self.v = 0.0
            self.w = 0.0

        self.y += self.v * math.cos(self.theta) * dt
        self.x += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt
        
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id ='odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta/2.0)
        t.transform.rotation.w = math.cos(self.theta/2.0)
        self.tf_br.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.2
        self.odom_pub.publish(odom)


def main():
        rclpy.init()
        node = DiffController()
        try:
            rclpy.spin(node)
        finally:
            for p in [node.lf_pin, node.lb_pin, node.rf_pin, node.rb_pin]:
                node.pi.set_servo_pulsewidth(p,0)
            node.pi.stop()
            node.destroy_node()
            rclpy.shutdown()
            
            
                    


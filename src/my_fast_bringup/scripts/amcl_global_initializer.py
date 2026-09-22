#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from lifecycle_msgs.srv import GetState
import time

class AMCLGlobalInitializer(Node):
    def __init__(self):
        super().__init__('amcl_global_initializer')
        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')
        self.amcl_get_state = self.create_client(GetState, '/amcl/get_state')
        self.scattered = False
        self.timer = self.create_timer(0.5, self.check_and_trigger)
        self.get_logger().info("AMCL Global Initializer ready. Waiting for AMCL active state...")

    def check_and_trigger(self):
        if self.scattered:
            return
        if not self.amcl_get_state.service_is_ready() or not self.global_loc_client.service_is_ready():
            return
        req = GetState.Request()
        fut = self.amcl_get_state.call_async(req)
        fut.add_done_callback(self.cb_state)

    def cb_state(self, future):
        if self.scattered:
            return
        try:
            res = future.result()
            if res.current_state.label == 'active' or res.current_state.id == 3:
                time.sleep(0.5)
                self.global_loc_client.call_async(Empty.Request())
                self.scattered = True
                self.timer.cancel()
                self.get_logger().info("AMCL ACTIVE: Global localization reinitialized across downsampled map!")
                time.sleep(1.0)
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().warn(f"State query error: {e}")

def main():
    rclpy.init()
    node = AMCLGlobalInitializer()
    try:
        rclpy.spin(node)
    except:
        pass

if __name__ == '__main__':
    main()

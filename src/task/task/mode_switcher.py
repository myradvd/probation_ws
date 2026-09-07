#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode


class ModeSwitcherNode(Node):
    def __init__(self):
        super().__init__('mode_switcher_node')
        self.declare_parameter('target_mode', 'GUIDED')
        self.declare_parameter('max_attempts', 5)
        self.declare_parameter('retry_delay_seconds', 1.0)

        self.target_mode = self.get_parameter('target_mode').value
        self.max_attempts = self.get_parameter('max_attempts').value
        self.retry_delay_seconds = self.get_parameter('retry_delay_seconds').value

        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

    # Repeatedly attempts to call the set_mode service until it succeeds or attempts run out
    def switch_mode(self):
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('set_mode service not available.')
            return False

        for attempt in range(1, self.max_attempts + 1):
            req = SetMode.Request()
            req.custom_mode = self.target_mode
            future = self.mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            if future.result() is not None and future.result().mode_sent:
                self.get_logger().info(f'Mode switched to {self.target_mode} on attempt {attempt}.')
                return True

            self.get_logger().warn(f'Mode switch attempt {attempt} failed, retrying...')
            time.sleep(self.retry_delay_seconds)

        self.get_logger().error(f'Failed to switch to {self.target_mode} after {self.max_attempts} attempts.')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcherNode()
    node.switch_mode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
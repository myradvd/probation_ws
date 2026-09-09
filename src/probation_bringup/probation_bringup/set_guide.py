#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('set_guide')
        self.cli=self.create_client(SetMode,'/mavros/set_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /mavros/set_mode not available, waiting again...')
        self.req=SetMode.Request()

    def send_request(self,custom_mode):
        self.req.custom_mode=custom_mode
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client=MinimalClientAsync()
    future=minimal_client.send_request('GUIDED')
    rclpy.spin_until_future_complete(minimal_client,future)
    response=future.result()

    if response.mode_sent:
        minimal_client.get_logger().info('Successfully switched to GUIDED mode!')
    else:
        minimal_client.get_logger().warn('Failed to switch mode.')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
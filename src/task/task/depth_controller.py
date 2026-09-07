#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import TransformException


class DepthControllerNode(Node):
    # Sets up the tf listener, publisher, and parameters used to hold at the target depth
    def __init__(self):
        super().__init__('depth_controller_node')

        self.declare_parameter('flare_frames', [
            'blue_flare/base_link', 'orange_flare/base_link',
            'red_flare/base_link', 'yellow_flare/base_link',
        ])
        self.declare_parameter('depth_tolerance', 0.4)
        self.declare_parameter('depth_gain', 0.5)
        self.declare_parameter('max_heave_speed', 0.4)
        self.declare_parameter('control_rate_hz', 20.0)

        self.flare_frames = self.get_parameter('flare_frames').value
        self.depth_tolerance = self.get_parameter('depth_tolerance').value
        self.depth_gain = self.get_parameter('depth_gain').value
        self.max_heave_speed = self.get_parameter('max_heave_speed').value
        control_rate_hz = self.get_parameter('control_rate_hz').value

        # tf2 buffer stores recent transforms; the listener fills it from the /tf topic automatically
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.target_depth = None
        self.reached = False

        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_loop)

    # Looks up a frame's z coordinate in the map frame, returns None if the transform isn't available yet
    def lookup_z(self, frame_name):
        try:
            tf = self.tf_buffer.lookup_transform('map', frame_name, rclpy.time.Time())
        except TransformException:
            return None
        return tf.transform.translation.z

    # Computes the average z of all flare frames, used as the "middle depth of the flare gate"
    def compute_target_depth(self):
        depths = [self.lookup_z(f) for f in self.flare_frames]
        depths = [d for d in depths if d is not None]
        if not depths:
            return None
        return sum(depths) / len(depths)

    # Runs a simple P-controller on depth error every tick, until within tolerance
    def control_loop(self):
        if self.reached:
            return

        if self.target_depth is None:
            self.target_depth = self.compute_target_depth()
            if self.target_depth is None:
                return  # flare tf not published yet, wait for next tick

        auv_z = self.lookup_z('auv/base_link')
        if auv_z is None:
            self.get_logger().warn('auv/base_link tf not available yet.')
            return

        depth_error = self.target_depth - auv_z

        # TEMPORARY DEBUG LINE -- remove once depth-hold is confirmed working.
        # Logs at most once per second (throttle_duration_sec=1.0) so it doesn't spam at 20Hz.
        self.get_logger().info(
            f'target_depth={self.target_depth:.3f} auv_z={auv_z:.3f} error={depth_error:.3f}',
            throttle_duration_sec=1.0,
        )

        if abs(depth_error) < self.depth_tolerance:
            self.get_logger().info('Target depth reached.')
            self.cmd_pub.publish(Twist())  # zero velocity before finishing
            self.reached = True
            return

        msg = Twist()
        heave = depth_error * self.depth_gain
        msg.linear.z = max(min(heave, self.max_heave_speed), -self.max_heave_speed)
        self.cmd_pub.publish(msg)

    # Blocks, spinning the node, until control_loop reports the target depth has been reached
    def wait_until_reached(self):
        while rclpy.ok() and not self.reached:
            rclpy.spin_once(self, timeout_sec=0.1)


# Creates the node, waits for target depth to be reached, then exits (meant to run before nav_controller)
def main(args=None):
    rclpy.init(args=args)
    node = DepthControllerNode()
    node.wait_until_reached()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
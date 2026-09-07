#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import TransformException

from .graph_planner import GraphPlanner, yaw_from_quaternion


class NavControllerNode(Node):
    # Sets up the tf listener, publisher, planner, and parameters used for path tracking
    def __init__(self):
        super().__init__('nav_controller_node')

        self.declare_parameter('obstacle_frames', [
            'blue_flare/base_link', 'orange_flare/base_link',
            'red_flare/base_link', 'yellow_flare/base_link',
        ])
        self.declare_parameter('safety_radius', 0.75)  # 1.5x AUV width -- SET THIS TO YOUR REAL AUV WIDTH
        self.declare_parameter('approach_distance', 2.0)
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.declare_parameter('position_gain', 0.5)
        self.declare_parameter('yaw_gain', 0.8)
        self.declare_parameter('max_linear_speed', 0.5)   # safety buffer: caps commanded speed
        self.declare_parameter('max_yaw_rate', 0.4)
        self.declare_parameter('control_rate_hz', 20.0)

        self.obstacle_frames = self.get_parameter('obstacle_frames').value
        self.safety_radius = self.get_parameter('safety_radius').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.position_gain = self.get_parameter('position_gain').value
        self.yaw_gain = self.get_parameter('yaw_gain').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        control_rate_hz = self.get_parameter('control_rate_hz').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.planner = GraphPlanner(self.safety_radius, self.approach_distance)
        self.path = None
        self.current_waypoint_index = 0

        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_loop)

    # Looks up a frame's (x, y, yaw) in the map frame, returns None if the transform isn't available yet
    def lookup_pose_2d(self, frame_name):
        try:
            tf = self.tf_buffer.lookup_transform('map', frame_name, rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q.z, q.w)
        return (t.x, t.y, yaw)

    # Fetches all obstacle 2D positions currently visible in tf, skipping any not yet published
    def get_obstacle_positions(self):
        positions = []
        for frame in self.obstacle_frames:
            pose = self.lookup_pose_2d(frame)
            if pose is not None:
                positions.append((pose[0], pose[1]))
        return positions

    # Computes the waypoint path once, using the planner, from current AUV position to the mini-goal
    def compute_path(self, auv_pose):
        gate_pose = self.lookup_pose_2d('gate/base_link')
        if gate_pose is None:
            return None
        obstacles = self.get_obstacle_positions()
        gate_position = (gate_pose[0], gate_pose[1])
        gate_yaw = gate_pose[2]
        return self.planner.plan(auv_pose, gate_position, gate_yaw, obstacles)

    # Main control loop: plans once if needed, then drives toward the current waypoint with a P-controller
    def control_loop(self):
        auv_pose = self.lookup_pose_2d('auv/base_link')
        if auv_pose is None:
            return

        if self.path is None:
            self.path = self.compute_path(auv_pose)
            if self.path is None:
                self.get_logger().warn('No path found yet, waiting for tf data.')
                return
            self.current_waypoint_index = 0
            self.get_logger().info(f'Path computed with {len(self.path)} waypoints.')

        if self.current_waypoint_index >= len(self.path):
            self.cmd_pub.publish(Twist())  # path complete, hold zero velocity
            return

        target = self.path[self.current_waypoint_index]
        auv_x, auv_y, auv_yaw = auv_pose

        dx = target[0] - auv_x
        dy = target[1] - auv_y
        dist_to_target = math.hypot(dx, dy)

        if dist_to_target < self.waypoint_tolerance:
            self.current_waypoint_index += 1
            return

        self.send_velocity_command(auv_x, auv_y, auv_yaw, target[0], target[1])

    # Converts a map-frame position error into a body-frame P-controller velocity command, with a safety buffer
    def send_velocity_command(self, auv_x, auv_y, auv_yaw, target_x, target_y):
        dx = target_x - auv_x
        dy = target_y - auv_y

        target_yaw = math.atan2(dy, dx)
        yaw_error = math.atan2(math.sin(target_yaw - auv_yaw), math.cos(target_yaw - auv_yaw))

        # Rotate the map-frame error into the AUV's own forward/left frame
        local_x = dx * math.cos(auv_yaw) + dy * math.sin(auv_yaw)
        local_y = -dx * math.sin(auv_yaw) + dy * math.cos(auv_yaw)

        msg = Twist()
        # Safety buffer: clamp every axis so a large position error never produces a violent command
        msg.linear.x = max(min(local_x * self.position_gain, self.max_linear_speed), -self.max_linear_speed)
        msg.linear.y = max(min(local_y * self.position_gain, self.max_linear_speed), -self.max_linear_speed)
        msg.angular.z = max(min(yaw_error * self.yaw_gain, self.max_yaw_rate), -self.max_yaw_rate)
        self.cmd_pub.publish(msg)


# Creates the node and spins it (main navigation loop, meant to run after depth_controller finishes)
def main(args=None):
    rclpy.init(args=args)
    node = NavControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
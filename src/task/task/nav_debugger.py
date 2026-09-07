#!/usr/bin/env python3
"""
DEBUG VISUALIZER for AUV Navigation
Use this to debug whether path planning or navigation is causing crashes
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle, RegularPolygon, Arrow
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import time
from collections import deque

from .graph_planner import GraphPlanner, yaw_from_quaternion


class NavigationDebugger(Node):
    """ROS Node that visualizes the AUV's planned path and actual trajectory in real-time"""
    
    def __init__(self):
        super().__init__('nav_debugger')
        
        # Parameters
        self.declare_parameter('obstacle_frames', [
            'blue_flare/base_link', 'orange_flare/base_link',
            'red_flare/base_link', 'yellow_flare/base_link',
        ])
        self.declare_parameter('safety_radius', 0.75)
        self.declare_parameter('approach_distance', 2.0)
        
        self.obstacle_frames = self.get_parameter('obstacle_frames').value
        self.safety_radius = self.get_parameter('safety_radius').value
        self.approach_distance = self.get_parameter('approach_distance').value
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribe to velocity commands to see what's being sent
        self.cmd_sub = self.create_subscription(
            Twist, 
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            self.cmd_callback,
            10
        )
        
        # Store latest command
        self.latest_cmd = None
        
        # Store trajectory
        self.trajectory = deque(maxlen=1000)
        self.trajectory_timestamps = deque(maxlen=1000)
        
        # Planned path
        self.planned_path = None
        self.current_waypoint_index = 0
        
        # Planner
        self.planner = GraphPlanner(self.safety_radius, self.approach_distance)
        
        # Matplotlib setup
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 7))
        self.fig.suptitle('AUV Navigation Debugger - REAL-TIME', fontsize=14, fontweight='bold')
        
        # For real-time updates
        self.plot_update_timer = self.create_timer(0.5, self.update_plot)
        self.plot_lock = threading.Lock()
        
        self.get_logger().info('Navigation Debugger started! Visualizing in real-time...')
        
    def lookup_pose_2d(self, frame_name):
        """Look up 2D pose (x, y, yaw) of a frame"""
        try:
            tf = self.tf_buffer.lookup_transform('map', frame_name, rclpy.time.Time())
        except TransformException as e:
            # self.get_logger().debug(f'Transform lookup failed for {frame_name}: {e}')
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q.z, q.w)
        return (t.x, t.y, yaw)
    
    def get_auv_pose(self):
        """Get AUV pose"""
        return self.lookup_pose_2d('auv/base_link')
    
    def get_gate_pose(self):
        """Get gate pose"""
        return self.lookup_pose_2d('gate/base_link')
    
    def get_obstacle_positions(self):
        """Get all obstacle positions"""
        positions = []
        for frame in self.obstacle_frames:
            pose = self.lookup_pose_2d(frame)
            if pose is not None:
                positions.append((pose[0], pose[1]))
        return positions
    
    def cmd_callback(self, msg):
        """Store latest velocity command"""
        self.latest_cmd = msg
        # Also record trajectory
        auv_pose = self.get_auv_pose()
        if auv_pose:
            self.trajectory.append((auv_pose[0], auv_pose[1]))
            self.trajectory_timestamps.append(time.time())
    
    def compute_path(self):
        """Compute planned path from current AUV position to gate"""
        auv_pose = self.get_auv_pose()
        gate_pose = self.get_gate_pose()
        
        if auv_pose is None or gate_pose is None:
            return None
        
        obstacles = self.get_obstacle_positions()
        gate_position = (gate_pose[0], gate_pose[1])
        gate_yaw = gate_pose[2]
        
        # Plan path
        path = self.planner.plan(
            (auv_pose[0], auv_pose[1]),
            gate_position,
            gate_yaw,
            obstacles
        )
        
        if path:
            self.get_logger().info(f'Path computed with {len(path)} waypoints')
        else:
            self.get_logger().warn('No path found!')
            
        return path
    
    def update_plot(self):
        """Update the matplotlib plot with current state"""
        with self.plot_lock:
            self.ax1.clear()
            self.ax2.clear()
            
            # Get current state
            auv_pose = self.get_auv_pose()
            gate_pose = self.get_gate_pose()
            obstacles = self.get_obstacle_positions()
            
            # --- Plot 1: Top-down view with path and obstacles ---
            ax = self.ax1
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_title('Plan View (Top-Down)')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            
            # Plot obstacles and safety zones
            for i, obs in enumerate(obstacles):
                # Obstacle point
                ax.plot(obs[0], obs[1], 'rx', markersize=10, markeredgewidth=2)
                # Safety circle
                circle = Circle(obs, radius=self.safety_radius, 
                              facecolor='red', edgecolor='darkred', 
                              alpha=0.15, linewidth=1)
                ax.add_patch(circle)
                # Hexagon vertices
                vertices = self.planner.hexagon_vertices(obs, self.safety_radius)
                hex_points = np.array(vertices)
                ax.plot(hex_points[:, 0], hex_points[:, 1], 'r--', alpha=0.3)
                ax.plot(hex_points[:, 0], hex_points[:, 1], 'ko', markersize=3)
                
            # Plot gate
            if gate_pose:
                gx, gy, gyaw = gate_pose
                ax.plot(gx, gy, 'g*', markersize=20, label='Gate')
                # Gate direction
                arrow_len = 1.5
                ax.arrow(gx, gy, 
                        arrow_len * math.cos(gyaw), 
                        arrow_len * math.sin(gyaw),
                        head_width=0.3, head_length=0.3, 
                        fc='green', ec='green', alpha=0.7)
                # Mini-goal
                mini_goal = self.planner.compute_mini_goal((gx, gy), gyaw)
                ax.plot(mini_goal[0], mini_goal[1], 'g^', markersize=12, 
                       label='Mini-goal')
            
            # Plot AUV
            if auv_pose:
                ax.plot(auv_pose[0], auv_pose[1], 'bo', markersize=12, label='AUV')
                # AUV heading
                arrow_len = 0.8
                ax.arrow(auv_pose[0], auv_pose[1],
                        arrow_len * math.cos(auv_pose[2]),
                        arrow_len * math.sin(auv_pose[2]),
                        head_width=0.25, head_length=0.25,
                        fc='blue', ec='blue')
            
            # Compute and plot planned path
            if auv_pose and gate_pose:
                self.planned_path = self.compute_path()
            
            if self.planned_path:
                path_array = np.array(self.planned_path)
                ax.plot(path_array[:, 0], path_array[:, 1], 'b-', 
                       linewidth=2, label='Planned Path', alpha=0.7)
                ax.plot(path_array[:, 0], path_array[:, 1], 'b.', 
                       markersize=8, label='Waypoints')
                
                # Annotate waypoints
                for i, (wx, wy) in enumerate(self.planned_path):
                    if i == 0:
                        label = 'Start'
                    elif i == len(self.planned_path) - 1:
                        label = 'Goal'
                    else:
                        label = f'W{i}'
                    ax.annotate(label, (wx, wy), xytext=(5, 5),
                              textcoords='offset points', fontsize=8)
            
            # Plot actual trajectory
            if len(self.trajectory) > 1:
                traj_array = np.array(list(self.trajectory))
                ax.plot(traj_array[:, 0], traj_array[:, 1], 'c-', 
                       linewidth=1.5, alpha=0.5, label='Actual Trajectory')
                # Plot recent trajectory more prominently
                if len(traj_array) > 20:
                    ax.plot(traj_array[-20:, 0], traj_array[-20:, 1], 
                           'c-', linewidth=2, alpha=0.8)
            
            # Auto-set limits
            self.auto_limits(ax, auv_pose, gate_pose, obstacles)
            ax.legend(loc='upper right', fontsize=8)
            
            # --- Plot 2: Velocity commands ---
            ax2 = self.ax2
            ax2.grid(True, alpha=0.3)
            ax2.set_title('Velocity Commands (Last 10 sec)')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Velocity (m/s or rad/s)')
            
            if self.latest_cmd:
                # Show current command
                cmd = self.latest_cmd
                info_text = (
                    f'Current Command:\n'
                    f'vx = {cmd.linear.x:.2f} m/s\n'
                    f'vy = {cmd.linear.y:.2f} m/s\n'
                    f'wz = {cmd.angular.z:.2f} rad/s'
                )
                ax2.text(0.05, 0.95, info_text, transform=ax2.transAxes,
                        verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            
            # Show command history (would need to store history, simplified version)
            ax2.set_xlim(-1, 11)
            ax2.set_ylim(-1, 1)
            
            # Add status info
            status_text = (
                f'AUV Position: ({auv_pose[0]:.2f}, {auv_pose[1]:.2f})' if auv_pose else 'AUV: NOT FOUND\n'
                f'Gate Position: ({gate_pose[0]:.2f}, {gate_pose[1]:.2f})' if gate_pose else 'Gate: NOT FOUND'
            )
            ax2.text(0.05, 0.05, status_text, transform=ax2.transAxes,
                    verticalalignment='bottom',
                    bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
            
            plt.tight_layout()
            plt.draw()
            plt.pause(0.01)
    
    def auto_limits(self, ax, auv_pose, gate_pose, obstacles, padding=2.0):
        """Auto-set plot limits"""
        all_x = []
        all_y = []
        
        if auv_pose:
            all_x.append(auv_pose[0])
            all_y.append(auv_pose[1])
        if gate_pose:
            all_x.append(gate_pose[0])
            all_y.append(gate_pose[1])
        for obs in obstacles:
            all_x.append(obs[0])
            all_y.append(obs[1])
        
        if all_x and all_y:
            x_min = min(all_x) - padding
            x_max = max(all_x) + padding
            y_min = min(all_y) - padding
            y_max = max(all_y) + padding
            
            # Make sure we have reasonable limits
            if x_max - x_min < 5:
                center_x = (x_max + x_min) / 2
                x_min = center_x - 3
                x_max = center_x + 3
            if y_max - y_min < 5:
                center_y = (y_max + y_min) / 2
                y_min = center_y - 3
                y_max = center_y + 3
                
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)
    
    def show(self):
<<<<<<< HEAD
        """Save plots continuously"""
        import time
        while rclpy.ok():
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"/tmp/nav_debugger_{timestamp}.png"
            self.fig.savefig(filename, dpi=150, bbox_inches='tight')
            self.get_logger().info(f'Plot saved to {filename}')
            time.sleep(2.0)  # Save every 2 seconds
=======
        """Display the plot interactively"""
        plt.show()

>>>>>>> fc6372f (navigation+debugging suite)

def main(args=None):
    rclpy.init(args=args)
    debugger = NavigationDebugger()
    
    # Run ROS spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(debugger,))
    spin_thread.start()
    
    # Show the plot (blocking)
    debugger.show()
    
    # Cleanup
    debugger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
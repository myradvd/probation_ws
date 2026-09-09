#!/usr/bin/env python3

# PUBLISHER AND LISTENER NODE FOR NAVIGATION CONTROL

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import TransformException

from .graph_planner import yaw_from_quaternion, wrap_angle, compute_mini_goal


class GateNavigator(Node):

    def __init__(self):
        super().__init__('gate_navigator')

        # Publisher and TF listener setup
        self.cmd_pub=self.create_publisher(Twist,'/mavros/setpoint_velocity/cmd_vel_unstamped',10)
        
        self.tf_buffer=tf2_ros.Buffer()
        self.tf_listener=tf2_ros.TransformListener(self.tf_buffer,self)

        # Hardcoded configuration values
        self.approach_distance=2
        self.waypoint_tolerance=0.3
        self.yaw_tolerance=math.radians(5.0)
        self.target_depth=-1.5
        self.position_gain=0.5
        self.depth_gain=0.8
        self.yaw_gain=0.8
        self.max_linear_speed=0.5
        self.max_yaw_rate=0.4

        # State machine variables
        self.step='NAVIGATE'
        self.mini_goal=None

        # Timer running at 20 Hz (0.05 seconds)
        timer_period=0.05
        self.timer=self.create_timer(timer_period,self.control_loop)

    def lookup_pose_2d(self,frame_name):
        try:
            tf=self.tf_buffer.lookup_transform('map',frame_name,rclpy.time.Time())
        except TransformException:
            return None
        t=tf.transform.translation
        q=tf.transform.rotation
        yaw=yaw_from_quaternion(q.z,q.w)
        return (t.x,t.y,t.z,yaw)

    def control_loop(self):
        auv_pose=self.lookup_pose_2d('auv/base_link')
        if auv_pose is None:
            return

        if self.step=='NAVIGATE':
            self.run_navigate(auv_pose)
        elif self.step=='ALIGN':
            self.run_align(auv_pose)
        elif self.step=='DONE':
            self.publish_movement(0.0,0.0,0.0,0.0)

    def run_navigate(self,auv_pose):
        gate_pose=self.lookup_pose_2d('gate/base_link')
        if gate_pose is None:
            return

        if self.mini_goal is None:
            gate_position=(gate_pose[0],gate_pose[1])
            self.mini_goal=compute_mini_goal(gate_position,gate_pose[3],self.approach_distance)
            self.get_logger().info(f'Mini-goal set at {self.mini_goal}.')

        auv_x,auv_y,auv_z,auv_yaw=auv_pose
        dx=self.mini_goal[0]-auv_x
        dy=self.mini_goal[1]-auv_y
        dist=math.hypot(dx,dy)

        if dist<self.waypoint_tolerance:
            self.get_logger().info('Mini-goal reached, aligning to gate.')
            self.step='ALIGN'
            return

        target_yaw=math.atan2(dy,dx)
        yaw_error=wrap_angle(target_yaw-auv_yaw)

        forward_speed=min(dist*self.position_gain,self.max_linear_speed)
        vx=forward_speed*max(0.0,math.cos(yaw_error))
        vy=0.0 

        depth_error=self.target_depth-auv_z
        vz=max(min(depth_error*self.depth_gain,self.max_linear_speed),-self.max_linear_speed)
        wz=max(min(yaw_error*self.yaw_gain,self.max_yaw_rate),-self.max_yaw_rate)

        self.publish_movement(vx,vy,vz,wz)

    def run_align(self,auv_pose):
        gate_pose=self.lookup_pose_2d('gate/base_link')
        if gate_pose is None:
            return

        _,_,auv_z,auv_yaw=auv_pose
        gate_yaw=gate_pose[3]
        yaw_error=wrap_angle(gate_yaw-auv_yaw)

        depth_error=self.target_depth-auv_z
        vz=max(min(depth_error*self.depth_gain,self.max_linear_speed),-self.max_linear_speed)
        wz=max(min(yaw_error*self.yaw_gain,self.max_yaw_rate),-self.max_yaw_rate)

        self.publish_movement(0.0,0.0,vz,wz)

        if abs(yaw_error)<self.yaw_tolerance:
            self.get_logger().info('Aligned with gate.')
            self.publish_movement(0.0,0.0,0.0,0.0)
            self.step='DONE'

    def publish_movement(self,vx,vy,vz,wz):
        msg=Twist()
        msg.linear.x=vx
        msg.linear.y=vy
        msg.linear.z=vz
        msg.angular.x=0.0
        msg.angular.y=0.0
        msg.angular.z=wz
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node=GateNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':
    main()
#!/usr/bin/env python3 
 
# PUBLISHER AND LISTENER NODE FOR NAVIGATION CONTROL 
 
import math 
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
import tf2_ros 
from tf2_ros import TransformException 
 
from .graph_planner import yaw_from_quaternion, wrap_angle, compute_mini_goal 
 
from vision_msgs.msg import BoundingBoxArray 
 
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
        self.forward_speed=0.65 #STICK TO THIS 
        self.forward_time=6.5 #AFTER A LOT OF EXPERIMENTATION 
        self.forward_start_time=None 
        self.gate_lost=False 
 
        # State machine variables 
        self.step='NAVIGATE' 
        self.mini_goal=None 
        self.flip=False #for aligning and driving through the gate in the right direction when mini goal is not 
        #in direction with gate as per orientationof gate frame 
 
        self.gate_center_x=None 
        self.gate_lost_count=0 
 
        self.box_sub=self.create_subscription(BoundingBoxArray,'/main_camera/detection/bounding_boxes',self.box_callback,10) 
 
        # Timer running at 20 Hz (0.025 seconds) 
        timer_period=0.025 
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
 
    def box_callback(self,msg): #only for debug 
        self.gate_center_x=None 
 
        for box in msg.bounding_boxes: 
            if box.label_name=='gate': 
                self.gate_center_x=box.x+(box.w/2) 
                break 
 
 
    def control_loop(self): 
        auv_pose=self.lookup_pose_2d('auv/base_link') 
        if auv_pose is None: 
            return 
 
        if self.step=='NAVIGATE': 
            self.run_navigate(auv_pose) 
        elif self.step=='ALIGN': 
            self.run_align(auv_pose) 
        elif self.step=='DRIVE_THROUGH': 
            #self.get_logger().info('STATE: DRIVE_THROUGH ON') #debugging 
            self.run_drive_through() 
        elif self.step=='DONE': 
            self.publish_movement(0.0,0.0,0.0,0.0) 
 
    def run_navigate(self,auv_pose): 
        gate_pose=self.lookup_pose_2d('gate/base_link') 
        if gate_pose is None: 
            return 
 
        if self.mini_goal is None: 
     
            gate_position=(gate_pose[0],gate_pose[1]) 
            self.mini_goal,self.flip=compute_mini_goal(gate_position,gate_pose[3],self.approach_distance,(auv_pose[0],auv_pose[1]))  
            self.get_logger().info(f'Mini-goal set at {self.mini_goal}.') 
 
        auv_x,auv_y,auv_z,auv_yaw=auv_pose 
        dx=self.mini_goal[0]-auv_x 
        dy=self.mini_goal[1]-auv_y 
        dist=math.hypot(dx,dy) 
 
        if dist<self.waypoint_tolerance: 
            self.get_logger().info('Mini-goal reached, aligning to gate face.') 
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
        if self.flip: #180 flip cz of  mini goal on opp side of gate frame ori 
            gate_yaw=gate_yaw+math.pi #CHECKKK
        yaw_error=wrap_angle(gate_yaw-auv_yaw) 
 
        depth_error=self.target_depth-auv_z 
        vz=max(min(depth_error*self.depth_gain,self.max_linear_speed),-self.max_linear_speed) 
        wz=max(min(yaw_error*self.yaw_gain,self.max_yaw_rate),-self.max_yaw_rate) 
 
        self.publish_movement(0.0,0.0,vz,wz) 
 
        if abs(yaw_error)<self.yaw_tolerance: 
            self.get_logger().info('Aligned with gate.') 
            self.publish_movement(0.0,0.0,0.0,0.0) 
            self.step='DRIVE_THROUGH' 
 
    def run_drive_through(self): 
 
        # Gate is still visible 
        if self.gate_center_x is not None and not self.gate_lost: 
            if self.gate_center_x > 0.58: 
                # gate slightly to the right → move left gently 
                self.publish_movement(0.0, -0.05, 0.0, 0.0) 
 
            elif self.gate_center_x < 0.42: 
                # gate slightly to the left → move right gently 
                self.publish_movement(0.0, 0.05, 0.0, 0.0) 
 
            else: 
                # close enough to centre → move forward 
                self.publish_movement(0.60, 0.0, 0.0, 0.0) 
 
        # Gate has disappeared from camera 
        else: 
 
            if not self.gate_lost: 
                self.gate_lost=True 
                self.forward_start_time=self.get_clock().now() 
                self.get_logger().info( 
                    'GATE LOST - CONTINUING FORWARD 2.5m' 
                ) 
 
            elapsed=( 
                self.get_clock().now()-self.forward_start_time 
            ).nanoseconds/1e9 
 
            if elapsed < self.forward_time: 
                self.publish_movement(self.forward_speed,0.0,0.0,0.0) 
 
            else: 
                self.get_logger().info( 
                    'FORWARD DISTANCE COMPLETED - STOPPING' 
                ) 
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

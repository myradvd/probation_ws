#!/usr/bin/env python3
import math

#rotation about the z-axis to face a target yaw
def yaw_from_quaternion(qz: float, qw: float) -> float:
    return 2.0 * math.atan2(qz, qw)


#maps an angle difference into the rannge [-pi, pi]
def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# Computes a "mini-goal" point: a fixed distance in front of the gate, along its facing direction
def compute_mini_goal(gate_pos, gate_yaw, approach_distance):#all given in the /tf topic
    fx=math.cos(gate_yaw)
    fy=math.sin(gate_yaw)
    gx,gy=gate_pos[0], gate_pos[1]
    return (gx-approach_distance*fx, gy-approach_distance*fy)
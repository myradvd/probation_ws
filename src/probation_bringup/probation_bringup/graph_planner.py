    
#!/usr/bin/env python3 
import math 
 
#rotation about the z-axis to face a target yaw 
def yaw_from_quaternion(qz: float, qw: float) -> float: 
    return 2.0 * math.atan2(qz, qw) 
 
 
#maps an angle difference into the rannge [-pi, pi] 
def wrap_angle(angle): 
    return math.atan2(math.sin(angle), math.cos(angle)) 
 
 
# Computes a "mini-goal" point: a fixed distance in front of the gate, along its facing direction 
#bUt also chooses the mini-goal that is closest to the AUV's current position, so it doesntcrrash into the gate 
#doesnt solve issue of crashing into gate to get to target poitn if it starts too close,  
# now, instead, choosing mini goal on same SIDE. 
 
#EDGE CASE 1 THAT WILL NOT WORK: IF AUV verry close, like if it rotates, it hits.... 
#EDGE CASE 2: If AUV is parallel to gate and very close, it may crash into gate to get to the mini goal. 
def compute_mini_goal(gate_pos, gate_yaw, approach_distance, auv_pos): 
 
    fx=math.cos(gate_yaw) 
    fy=math.sin(gate_yaw) 
    gx,gy=gate_pos[0],gate_pos[1] 
    ax,ay=auv_pos[0],auv_pos[1] 
    #Vector from gate to AUV 
    dx=ax-gx 
    dy=ay-gy 
    # Determine which side of the gate the AUV is on 
    side=dx*fx+dy*fy 
    if side<0: 
        return (gx-approach_distance*fx,gy-approach_distance*fy),False 
    else: 
        return (gx+approach_distance*fx,gy+approach_distance*fy),True #need to flip the align to drive through in the right direction... 
import pytest
import numpy as np
import time
from termcolor import cprint

import pybullet_planning as pp
from pybullet_planning import INF
from pybullet_planning import connect, set_camera_pose, LockRenderer, wait_if_gui
from pybullet_planning import unit_pose, draw_pose
from pybullet_planning import compute_path_cost

import planner_2D_utils as mp_utils
from planner_2D_utils import create_aabb_box, get_aabb_center, draw_environment

def plan(self):
	path = pp.rrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, max_iterations=INF, max_time=max_time)
ik_result = p.calculateInverseKinematics(ur5,7,[0.2,0.3,0]) #calculate kinematics for the end effector (link 6) to go to specific coordinate returns a list of 6 values for each joint
print(ik_result)
ik_hand = [0,0,0,0,0,0]
for i in range(7): #go through all 6 joints and set the joints to that value
        p.setJointMotorControl2(bodyIndex=ur5,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=ik_result[i],
                                targetVelocity=0,
                                force=500,
                                positionGain=0.03,
                                velocityGain=1)

while True: # keep sim open
   p.stepSimulation()
   time.sleep(1./240.)
   
   

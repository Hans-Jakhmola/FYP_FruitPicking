import pybullet as p
import time
import math
import pybullet_data
from Environment import environment
import pybullet_planning as pp
from pybullet_planning import INF
from pybullet_planning import connect, set_camera_pose, LockRenderer, wait_if_gui
from pybullet_planning import unit_pose, draw_pose
from pybullet_planning import compute_path_cost
import pytest
import planner_2d_utils as mp_utils
from planner_2d_utils import create_aabb_box, get_aabb_center, draw_environment
import numpy as np
regions = {
        'env':   create_aabb_box(center=(.5, .5, base_z), extents=(1., 1., h)),
        'green': create_aabb_box(center=(.8, .8, base_z), extents=(.1, .1, h)),
    }

env = environment()
env.initialise(gui = True)
j = env.getjointconfiguration()
distance_fn = mp_utils.get_euclidean_distance_fn(weights=[1, 1])
goal = p.calculateInverseKinematics(env.ur5,10,[1,1,-1])
sample_fn, samples = mp_utils.get_box_sample_fn(regions['env'], obstacles=[]) # obstacles
extend_fn, roadmap = mp_utils.get_box_extend_fn(obstacles=obstacles)  # obstacles | []
collision_fn, cfree = mp_utils.get_box_collision_fn(obstacles)

path = pp.rrt(env.ur5_start_conf, goal, distance_fn, sample_fn, extend_fn, collision_fn, max_iterations=INF, max_time=2)
print(path)
print(j)
print('gonna close')
while True: # keep sim open
	#ik_result = p.calculateInverseKinematics(env.ur5,10,[1,1,-1]) #calculate kinematics for the end effector (link 6) to go to specific coordinate returns a list of 6 values for each joint
	#print(ik_result)
	#env.movetoconfig(ik_result)
	
	env.step()
   

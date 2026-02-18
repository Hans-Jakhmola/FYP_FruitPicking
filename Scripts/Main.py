import pybullet as p
import time
import math
import pybullet_data
from Environment import environment
import pybullet_planning as pp
env = environment()
env.initialise(gui = True)
j = env.getjointconfiguration()
path = pp.rrt(env.ur5_start_conf, goal, distance_fn, sample_fn, extend_fn, collision_fn, max_iterations=INF, max_time=max_time)
print(j)
print('gonna close')
while True: # keep sim open
	ik_result = p.calculateInverseKinematics(env.ur5,10,[1,1,-1]) #calculate kinematics for the end effector (link 6) to go to specific coordinate returns a list of 6 values for each joint
	#print(ik_result)
	env.movetoconfig(ik_result)
	
	env.step()
   

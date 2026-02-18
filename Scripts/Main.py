import pybullet as p
import time
import math
import pybullet_data
from Environment import environment

env = environment()
env.initialise(gui = True)

print('gonna close')
while True: # keep sim open
	ik_result = p.calculateInverseKinematics(env.ur5,10,[1,0.5,-0.5]) #calculate kinematics for the end effector (link 6) to go to specific coordinate returns a list of 6 values for each joint
	#print(ik_result)
	env.movetoconfig(ik_result)
	env.step()
   

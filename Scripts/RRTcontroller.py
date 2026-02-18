import pybullet as p
import time
import math
import pybullet_data
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion
import pybullet_planning as pp

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
   
   

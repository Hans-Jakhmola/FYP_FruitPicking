import pybullet as p
import time
import math
import pybullet_data
from pybullet_planning import BASE_LINK, RED, BLUE, GREEN
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui, apply_alpha
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import multiply, invert, get_distance
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion
from pybullet_planning import dump_world, set_pose
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box
from pybullet_planning import pairwise_collision, pairwise_collision_info, draw_collision_diagnosis, body_collision_info
physicsClient = p.connect(p.GUI)# p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #look in pybullet installation for models such as plane
p.setGravity(0,0,-9.8) #set gravity to earth gravity
planeId = p.loadURDF('plane.urdf') #load in a plane 

ur5 = p.loadURDF('Models/URDF/ur5.urdf',basePosition=[0, 0, 0],useFixedBase=True) #load ur5 robot at 0,0,0 with a base fixed to the plane
cube = p.loadURDF('cube.urdf',[1.5,0,0]) #load cube 
ik_joints = get_movable_joints(ur5) #get all moveable joints for the ur5
ik_joint_names = get_joint_names(ur5, ik_joints) #get the names of the joints
print('Joint {} \ncorresponds to:\n{}'.format(ik_joints, ik_joint_names)) #print joints
ur5_start_conf = [0,-1.65715,1.71108,-1.62348,0,0] #home configuration
set_joint_positions(ur5, ik_joints, ur5_start_conf) #set the joint to the configuration specified above, takes in 3 parameters (the robot,the joints,the configuration)
ik_result = p.calculateInverseKinematics(ur5,6,[1.5,0,0]) #calculate kinematics for the end effector (link 6) to go to specific coordinate returns a list of 6 values for each joint
print(ik_result) #print all joint values
wait_for_user()
for i in range(6): #go through all 6 joints and set the joints to that value
        p.setJointMotorControl2(bodyIndex=ur5,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=ik_result[i],
                                targetVelocity=0,
                                force=500,
                                positionGain=0.03,
                                velocityGain=1)

print('THIS IS WORKING RAAAAHHHH')

while True: # keep sim open
   p.stepSimulation()
   time.sleep(1./240.)
   cubePos, cubeOrn = p.getBasePositionAndOrientation(ur5)

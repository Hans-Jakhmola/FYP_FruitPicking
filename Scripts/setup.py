import pybullet as p
import time
import math
import pybullet_data
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion

physicsClient = p.connect(p.GUI)# p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #look in pybullet installation for models such as plane
p.setGravity(0,0,-9.8) #set gravity to earth gravity
planeId = p.loadURDF('plane.urdf') #load in a plane 

ur5 = p.loadURDF('Models/URDF/ur5.urdf',basePosition=[0, 0, 0],useFixedBase=True) #load ur5 robot at 0,0,0 with a base fixed to the plane

# Rotation
orientation = p.getQuaternionFromEuler([math.pi/2, 0, 0]) #rotate tree 90 degrees about the x axis so it is upright

treeshape = p.createVisualShape(shapeType=p.GEOM_MESH, fileName="Models/Tree/Tree.obj", meshScale=[0.13, 0.13, 0.13]) #create the visual component of the tree using obj file

treecollision = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="Models/Tree/Tree.obj", meshScale=[0.13, 0.13, 0.13]) #create collision for tree

tree = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=treecollision, baseVisualShapeIndex=treeshape, basePosition=[0.6, -0.1, 0], baseOrientation=orientation) #spawn in tree multibody using visual component, collision and base position. tree will be static so mass is 0

ur5_start_conf = [0,-1.65715,1.71108,-1.62348,0,0] #home configuration
ik_joints = get_movable_joints(ur5) #get all moveable joints for the ur5
set_joint_positions(ur5, ik_joints, ur5_start_conf) #set the joint to the configuration specified above, takes in 3 parameters (the robot,the joints,the configuration)

while True: # keep sim open
   p.stepSimulation()
   time.sleep(1./240.)
   

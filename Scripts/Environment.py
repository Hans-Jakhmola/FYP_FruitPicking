import pybullet as p
import time
import math
import pybullet_data
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion
import pybullet_planning as pp

class environment:
	def initialise(self,gui):
		if gui:
			p.connect(p.GUI)
		else:
            		p.connect(p.DIRECT)
		p.setAdditionalSearchPath(pybullet_data.getDataPath()) #look in pybullet installation for models such as plane
		p.setGravity(0,0,-9.8) #set gravity to earth gravity
		self.planeId = p.loadURDF('plane.urdf') #load in a plane 

		self.ur5 = p.loadURDF('Models/URDF/ur5_robotiq_85.urdf',basePosition=[0, 0, 0],useFixedBase=True) #load ur5 robot at 0,0,0 with a base fixed to the plane

		# Rotation
		self.treeorientation = p.getQuaternionFromEuler([math.pi/2, 0, 0]) #rotate tree 90 degrees about the x axis so it is upright

		self.treeshape = p.createVisualShape(shapeType=p.GEOM_MESH, fileName="Models/Tree/Tree.obj", meshScale=[0.13, 0.13, 0.13]) #create the visual component of the tree using obj file

		self.treecollision = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="Models/Tree/Tree.obj", meshScale=[0.13, 0.13, 0.13]) #create collision for tree

		self.tree = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=self.treecollision, baseVisualShapeIndex=self.treeshape, basePosition=[0.8, -0.1, 0], baseOrientation=self.treeorientation) #spawn in tree multibody using visual component, collision and base position. tree will be static so mass is 0

		self.ur5_start_conf = [0,-1.65715,1.71108,-1.62348,1,0,0,0,0,0,0,0] #home configuration first 6 indexes are joints on the robot rest are related to the robotiq-85 gripper
		self.ik_joints = get_movable_joints(self.ur5) #get all moveable joints for the ur5
		self.ik_joint_names = get_joint_names(self.ur5, self.ik_joints) #get the names of the joints
		print('Joint {} \ncorresponds to:\n{}'.format(self.ik_joints, self.ik_joint_names)) #print joints

		set_joint_positions(self.ur5,self.ik_joints, self.ur5_start_conf) #set the joint to the configuration specified above, takes in 3 parameters (the robot,the joints,the configuration)
		
	def movetoconfig(self,ik_result):
		for i in range(7): #go through all 6 joints and set the joints to that value
        			p.setJointMotorControl2(bodyIndex=self.ur5,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=ik_result[i],
                                targetVelocity=0,
                                force=500,
                                positionGain=0.03,
                                velocityGain=1)

	def step(self):
	        p.stepSimulation()
	        time.sleep(1./240.)
	        
	def close(self):
        	p.disconnect()

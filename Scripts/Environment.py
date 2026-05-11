import pybullet as p
import time
import math
import pybullet_data
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion
import pybullet_planning as pp

class environment: #environment class
	def initialise(self, gui): #initialise environment spawn in tree model robot model and fruit
		if gui:
			pp.connect(use_gui=gui)
		else:
			p.connect(p.DIRECT)
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		p.setGravity(0, 0, -9.8)
		self.planeId = p.loadURDF('plane.urdf')
		self.ur5 = p.loadURDF('Models/URDF/ur5_robotiq_85.urdf', basePosition=[0, 0, 0], useFixedBase=True)
		self.fruit = p.loadURDF("cube_small.urdf", [0.495, -0.054, 0.520])
		p.changeDynamics(self.fruit, -1, linearDamping=0.5, angularDamping=0.5, mass=0.05) #fruit dynamics
		p.changeVisualShape(self.fruit, -1, rgbaColor=[1, 0, 0, 1]) #make fruit red
		self.treeorientation = p.getQuaternionFromEuler([math.pi/2, 0, 0])
		self.treeshape = p.createVisualShape(shapeType=p.GEOM_MESH, fileName="Models/Tree/Tree.obj", meshScale=[0.13, 0.13, 0.13])
		self.treecollision = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="Models/Tree/Tree.obj", meshScale=[0.13, 0.13, 0.13])
		self.tree = p.createMultiBody(
			baseMass=0,
			baseCollisionShapeIndex=self.treecollision,
			baseVisualShapeIndex=self.treeshape,
			basePosition=[0.7, -0.1, 0],
			baseOrientation=self.treeorientation
		)
		self.ur5_start_conf = [0, -1.9, 1.71108, -1.62348, 1, 0, 0, 0, 0, 0, 0, 0]
		self.ik_joints = get_movable_joints(self.ur5)
		self.ik_joint_names = get_joint_names(self.ur5, self.ik_joints)
		print('Joint {} \ncorresponds to:\n{}'.format(self.ik_joints, self.ik_joint_names))
		set_joint_positions(self.ur5, self.ik_joints, self.ur5_start_conf)

	def step(self): #move simulation step
		p.stepSimulation()
		time.sleep(1. / 240.)


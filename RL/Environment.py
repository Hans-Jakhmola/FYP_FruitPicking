import pybullet as p
import time
import math
import random
import pybullet_data
from pybullet_planning import (
    get_num_joints, get_joint_names, get_movable_joints,
    set_joint_positions, joint_from_name, joints_from_names,
    get_sample_fn, plan_joint_motion,
)
import pybullet_planning as pp


class environment:
    def initialise(self, gui, include_tree=False, randomize_fruit=False, seed=None):
        if gui:
            pp.connect(use_gui=gui)
        else:
            p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        self.planeId = p.loadURDF('plane.urdf')
        self.ur5 = p.loadURDF(
            'Models/URDF/ur5_robotiq_85.urdf',
            basePosition=[0, 0, 0],
            useFixedBase=True,
        )
        rng = random.Random(seed)

        if randomize_fruit:
            # Spawn range passed in from the curriculum training script.
            # Defaults to Stage 1 jitter (±5cm) if not overridden.
            x_range = getattr(self, '_x_range', 0.05)
            y_range = getattr(self, '_y_range', 0.05)
            fx = 0.50 + rng.uniform(-x_range, x_range)
            fy = 0.00 + rng.uniform(-y_range, y_range)
        else:
            fx, fy = 0.50, 0.00

        fz = 0.025  # cube_small half-extent, rests on floor

        self.fruit = p.loadURDF("cube_small.urdf", [fx, fy, fz])
        p.changeDynamics(self.fruit, -1, linearDamping=0.5, angularDamping=0.5, mass=0.05)
        p.changeVisualShape(self.fruit, -1, rgbaColor=[1, 0, 0, 1])

        # Tree (disabled until tree environment stage)
        if include_tree:
            self.treeorientation = p.getQuaternionFromEuler([math.pi / 2, 0, 0])
            self.treeshape = p.createVisualShape(
                shapeType=p.GEOM_MESH,
                fileName="Models/Tree/Tree.obj",
                meshScale=[0.13, 0.13, 0.13],
            )
            self.treecollision = p.createCollisionShape(
                shapeType=p.GEOM_MESH,
                fileName="Models/Tree/Tree.obj",
                meshScale=[0.13, 0.13, 0.13],
            )
            self.tree = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=self.treecollision,
                baseVisualShapeIndex=self.treeshape,
                basePosition=[0.7, -0.1, 0],
                baseOrientation=self.treeorientation,
            )
        else:
            self.tree = None

        # Start config: downward-pointing gripper
        self.ur5_start_conf = [0, -1.57, 1.57, -1.57, -1.57, 0, 0, 0, 0, 0, 0, 0]
        self.ik_joints = get_movable_joints(self.ur5)
        self.ik_joint_names = get_joint_names(self.ur5, self.ik_joints)
        print('Joint {} \ncorresponds to:\n{}'.format(self.ik_joints, self.ik_joint_names))
        set_joint_positions(self.ur5, self.ik_joints, self.ur5_start_conf)

    def step(self):
        p.stepSimulation()
        time.sleep(1. / 240.)

    def close(self):
        pp.disconnect()

import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)# p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF('plane.urdf')

ur5 = p.loadURDF('Models/URDF/ur5.urdf',basePosition=[0, 0, 0],useFixedBase=True)

print('THIS IS WORKING RAAAAHHHH')

while True: # keep sim open
   p.stepSimulation()
   time.sleep(1./240.)
   cubePos, cubeOrn = p.getBasePositionAndOrientation(ur5)

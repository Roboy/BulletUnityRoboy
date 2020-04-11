#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet_data
import pybullet as p
import time

p.connect(p.SHARED_MEMORY_SERVER)
# p.connect(p.SHARED_MEMORY, p.SHARED_MEMORY_KEY)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.loadURDF("plane.urdf")
# p.loadURDF("r2d2.urdf")
# p.loadSDF("C:\\Users\\roboy\\Documents\\code\\BulletUnityRoboy\\Assets\\Urdf\\CheckersTable\\checkers.sdf")
# p.loadURDF("C:\\Users\\roboy\\Documents\\code\\BulletUnityRoboy\\Assets\\Urdf\\CheckersTable\\CheckersTable.urdf",0.0,1,0)
# p.loadURDF("C:\\Users\\roboy\\Documents\\code\\roboy3_models\\upper_body\\bullet.urdf", useFixedBase=1)
# p.setRealTimeSimulation(1)
# print("done loading")
while (1):
  #this is a no-op command, to allow GUI updates on Mac OSX (main thread)
  # p.setPhysicsEngineParameter()
  # p.syncBodyInfo()
  time.sleep(0.001)

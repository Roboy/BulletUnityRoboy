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
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.loadURDF("plane.urdf")
# p.loadURDF("C:\\Users\\alion\\Documents\\code\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\upper_body\\model.urdf", useFixedBase=1)
# p.setRealTimeSimulation(1)
# print("done loading")
while (1):
  #this is a no-op command, to allow GUI updates on Mac OSX (main thread)
  # p.setPhysicsEngineParameter()
  # p.syncBodyInfo()
  time.sleep(0.001)

import pybullet as p
import math
import time

p.connect(p.GUI)
ob = p.loadURDF("C:\\Users\\roboy\\Documents\\code\\roboy3_models\\upper_body\\bullet.urdf", useFixedBase=1)
p.setGravity(0,0,-10)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 0

useOrientation = 0

#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 1
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15
numJoints = p.getNumJoints(ob)
freeJoints = []

for i in range(numJoints):
    info = p.getJointInfo(ob,i)
    if info[2] == p.JOINT_REVOLUTE:
        freeJoints.append(i)
    if info[12] == b'hand_right':
        endEffectorId = i;
        print("EF id: " + str(i))
        break


def accurateCalculateInverseKinematics(ob, endEffectorId, targetPos, threshold, maxIter):
  closeEnough = False
  iter = 0
  dist2 = 1e30
  while (not closeEnough and iter < maxIter):
    jointPoses = p.calculateInverseKinematics(ob, endEffectorId, targetPos)
    #import pdb; pdb.set_trace()
    for i in range(len(freeJoints)):
      p.resetJointState(ob, freeJoints[i], jointPoses[i])
    ls = p.getLinkState(ob, endEffectorId)
    newPos = ls[4]
    diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
    dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
    closeEnough = (dist2 < threshold)
    iter = iter + 1
  #print ("Num iter: "+str(iter) + "threshold: "+str(dist2))
  return jointPoses

while True:
    if (useRealTimeSimulation):
        t = time.time()  #(dt, micro) = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f').split('.')
        #t = (dt.second/60.)*2.*math.pi
    else:
        t = t + 0.001

    if (useSimulation and useRealTimeSimulation == 0):
        p.stepSimulation()

    pos = [0.2 * math.cos(t)-0.4, -0.4, 0. + 0.2 * math.sin(t) + 0.7]
    threshold = 0.001
    maxIter = 100
    jointPoses = accurateCalculateInverseKinematics(ob, endEffectorId, pos,
                                                        threshold, maxIter)
    if (useSimulation):
      for i in range(len(freeJoints)):
        p.setJointMotorControl2(bodyIndex=ob,
                                jointIndex=freeJoints[i],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=0,
                                force=100,
                                positionGain=1,
                                velocityGain=0.1)
    ls = p.getLinkState(ob, endEffectorId)
    if (hasPrevPose):
        p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1

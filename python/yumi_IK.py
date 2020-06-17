import pybullet as p
import math
import time
import numpy as np
p.connect(p.SHARED_MEMORY, p.SHARED_MEMORY_KEY)
ob = 0 #p.loadURDF("C:/users/roboy/Documents/code/BulletUnityRoboy/Assets/Urdf/yumi_description/yumi-unity.urdf", useFixedBase=1, baseOrientation=(0,0,-0.7,0.7))
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
    print(i)
    info = p.getJointInfo(ob,i)
    if info[2] == p.JOINT_REVOLUTE or\
            info[2] == p.JOINT_PRISMATIC or \
            info[2] == p.JOINT_SPHERICAL:
        freeJoints.append(i)
    if info[12] == b'gripper_r_base':
        endEffectorRightId = i;
        print("EF id: " + str(i))
        #break
    if info[12] == b'gripper_l_base':
        endEffectorLeftId = i;
        print("EF id: " + str(i))
    if info[12] == b'yumi_body':
        baseLinkId = i


def getKinematicChain(ef):
    chain = []
    for i in range(baseLinkId, ef+1):
        chain.append(i)
    return chain

# import pdb; pdb.set_trace()
def accurateCalculateInverseKinematics(ob, endEffectorId, targetPos, threshold, maxIter):
# def accurateCalculateInverseKinematics(ob, endEffectorIds, targetPos, threshold, maxIter):
  closeEnough = False
  iter = 0
  dist2 = 1e30
  while (not closeEnough and iter < maxIter):
    # jointPoses = p.calculateInverseKinematics2(ob, endEffectorLinkIndices=endEffectorIds, targetPositions=targetPos)
    if endEffectorId == endEffectorLeftId:
        jointPoses = p.calculateInverseKinematics(ob, endEffectorId, targetPos, targetOrientation =(0,1,0,0))
    else:
        jointPoses = p.calculateInverseKinematics(ob, endEffectorId, targetPos,targetOrientation =(0,1,0,0))
    #import pdb; pdb.set_trace()
    for i in range(len(freeJoints)):
      p.resetJointState(ob, freeJoints[i], jointPoses[i])
    ls = p.getLinkState(ob, endEffectorRightId)
    newPos = ls[4]
    diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
    dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
    closeEnough = (dist2 < threshold)
    iter = iter + 1
  #print ("Num iter: "+str(iter) + "threshold: "+str(dist2))
  return jointPoses

base_pos = p.getBasePositionAndOrientation(ob)[0]
first = True
while True:

    if (useRealTimeSimulation):
        t = time.time()  #(dt, micro) = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f').split('.')
        #t = (dt.second/60.)*2.*math.pi
    else:
        t = t + 0.001

    if (useSimulation and useRealTimeSimulation == 0):
        p.stepSimulation()

    y = 16*np.sin(t)**3
    z = 13*np.cos(t)-5*np.cos(2*t)-2*np.cos(3*t)-np.cos(4*t)
    x = 0.05
    scale = 1/50.0
    # pos = posRight = [base_pos[0] - 0.5, base_pos[1] + abs(y)*scale , base_pos[2] + z*scale+0.45]
    # posLeft = [base_pos[0] - 0.5, base_pos[1] + -abs(y)*scale, base_pos[2] + z*scale+0.45]
    # posLeft = pos = [base_pos[0] + 0.2 * abs(math.cos(t)) + 0.4,  base_pos[1] + 0.1 * math.sin(t) - 0.3, base_pos[2]+0.4]
    pos = posLeft = [base_pos[0] + 0.13 * math.cos(t) + 0.51,  base_pos[1] + 0.13 * math.sin(t)-0.11, base_pos[2]+0.35]
    pos = posRight = [base_pos[0]+  0.21,  base_pos[1] -0.5, base_pos[2]+0.25]

    threshold = 0.001
    maxIter = 500

    if (first):
        # first = False
        jointPoses = accurateCalculateInverseKinematics(ob, endEffectorRightId, posRight,
                                                            threshold, maxIter)

        if (useSimulation):
          for i in range(len(freeJoints)):
            p.setJointMotorControl2(bodyIndex=ob,
                                    jointIndex=freeJoints[i],
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    targetVelocity=0,
                                    force=5,
                                    positionGain=2,
                                    velocityGain=0)

    jointPoses = accurateCalculateInverseKinematics(ob, endEffectorLeftId, posLeft,
                                                        threshold, maxIter)

    if (useSimulation):
      for i in range(len(freeJoints)):
        p.setJointMotorControl2(bodyIndex=ob,
                                jointIndex=freeJoints[i],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=0,
                                force=50,
                                positionGain=0.01,
                                velocityGain=0.01)
    ls = p.getLinkState(ob, endEffectorRightId)
    if (hasPrevPose):
        p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1
    # time.sleep(0.01)

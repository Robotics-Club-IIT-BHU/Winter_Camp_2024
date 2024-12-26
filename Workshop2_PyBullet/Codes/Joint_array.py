import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

# p.resetDebugVisualizerCamera(-10,-10,-110,[2,4,0])

planeId = p.loadURDF("samurai.urdf")
boxId = p.loadURDF("r2d2.urdf",[0,0,1],p.getQuaternionFromEuler([0,0,3.14]))
numJoints = p.getNumJoints(boxId)
print (numJoints)
for joint in range(numJoints):
    print(p.getJointInfo(boxId, joint))
p.setJointMotorControlArray(
    bodyIndex =boxId,
    jointIndices = [2,3,6,7],
    controlMode = p.VELOCITY_CONTROL,
    targetVelocities = [15,15,15,15]
)


for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
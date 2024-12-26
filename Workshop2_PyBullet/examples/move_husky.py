import time as t
import cv2 as cv
import numpy as np
import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)

CAR_LOCATION = [3,0,1.5]

BALL_LOCATION = [-3,0,1.5]

HUMANOID_LOCATION = [6,7,1.5]


p.loadURDF('urdf/arena/arena3/arena_v0.urdf')
husky = p.loadURDF('urdf/car/car.urdf', CAR_LOCATION, p.getQuaternionFromEuler([0, 0, 0]))
ball = p.loadURDF('urdf/ball/ball_red.urdf', BALL_LOCATION, p.getQuaternionFromEuler([0, 0, 0]))
humnaoid = p.loadURDF('urdf/humanoid/humanoid_red.urdf', HUMANOID_LOCATION,p.getQuaternionFromEuler([0, 0, 0]))


p.resetDebugVisualizerCamera(13,0,-110,[0,4,0])

def move(vels):
    vels = np.array(vels)
    [left_front, right_front, left_back, right_back] = vels.flatten()
    target_vels = [left_front, -right_front, left_back, -right_back]
    p.setJointMotorControlArray(
        bodyIndex=husky,
        jointIndices=[0, 1, 2, 3],
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=target_vels)
    

    
move(vels=[[2,2],
               [2,2]])
t.sleep(2)
move(vels=[[-2,2],
               [-2,2]])
t.sleep(2)
move(vels=[[-2,-2],
               [-2,-2]])
t.sleep(2)

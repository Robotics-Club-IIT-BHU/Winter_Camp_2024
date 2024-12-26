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

def open_grip(delay=1. / 240.):
    p.setJointMotorControl2(husky, 5, p.POSITION_CONTROL, targetPosition=np.pi / 2)
    p.setJointMotorControl2(husky, 6, p.POSITION_CONTROL, targetPosition=-np.pi / 2)
    t.sleep(delay)

def close_grip(delay=1. / 240.):
    p.setJointMotorControl2(husky, 5, p.POSITION_CONTROL, targetPosition=0)
    p.setJointMotorControl2(husky, 6, p.POSITION_CONTROL, targetPosition=0)
    t.sleep(delay)


def get_image(cam_height=0, dims=[512, 512]):
    orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(husky)[1])
    pos= p.getBasePositionAndOrientation(husky)[0]
    pos = np.add(pos, np.array([0, 0, cam_height]))

    camera_eye = [pos[0] + 0.4 * np.cos(orn[2]), pos[1] + 0.4 * np.sin(orn[2]), pos[2] + 1.15 * np.cos(orn[0])]
    target_pos = [pos[0] - 2 * np.cos(orn[2]), pos[1] - 2 * np.sin(orn[2]), pos[2] + 1.15 * np.cos(orn[0])]

    view_matrix = p.computeViewMatrix(camera_eye, target_pos, [0, 0, 1])
    proj_matrix = p.computeProjectionMatrixFOV(60, dims[0] / dims[1], 0.02, 50)
    images = p.getCameraImage(dims[0], dims[1], view_matrix, proj_matrix, shadow=True,
                                  renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgba_opengl = np.reshape(images[2], (dims[0], dims[1], 4))
    rgba_opengl = np.uint8(rgba_opengl)
    bgr = cv.cvtColor(rgba_opengl[:, :, 0:3], cv.COLOR_BGR2RGB)
    return bgr


def move(vels):
    vels = np.array(vels)
    [left_front, right_front, left_back, right_back] = vels.flatten()
    target_vels = [left_front, -right_front, left_back, -right_back]
    p.setJointMotorControlArray(
        bodyIndex=husky,
        jointIndices=[0, 1, 2, 3],
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=target_vels)

def get_orientation():
    orn_quat = p.getBasePositionAndOrientation(husky)[1]
    orn_euler = p.getEulerFromQuaternion(orn_quat)
    return orn_euler

def shoot(force=50):
    p.setJointMotorControl2(husky, 8, p.POSITION_CONTROL, targetPosition=-1.5, force=force)
    t.sleep(1. / 3.)
    p.setJointMotorControl2(husky, 8, p.POSITION_CONTROL, targetPosition=0, force=force)
    t.sleep(1. / 3.)






"""
Your Task is to grab the ball that is in front of the husky and then find the humanoid and shoot the ball. 
If you make the process of grabbing and shooting of ball autonomous then extra points will be awarded.

"""

#Hints:
# 1. Utilize the provided functions to complete the task (you may also create additional helper functions as needed).
# 2. Refer to the quickstart guide for getKeyboardEvents to manually control the Husky.
# 3. Use get_image() in OpenCV to detect ball and humanoid to automate the process.  





"""
Code here

"""





while True:
    img = get_image()
    cv.imshow("img", img)
    k = cv.waitKey(1)
    if k==ord('q'):
        break

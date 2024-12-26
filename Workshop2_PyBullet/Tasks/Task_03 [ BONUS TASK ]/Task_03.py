import time as t
import cv2 as cv
import numpy as np
import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)

CAR_LOCATION = [-25.5,0,1.5]


husky = p.loadURDF('urdf/car/car.urdf',CAR_LOCATION,p.getQuaternionFromEuler([0, 0, 1.57]))
Track = p.loadURDF('urdf/arena/urdf/Track.urdf', useFixedBase=1)
texture_id = p.loadTexture('urdf/arena/robo_lega/track.jpg')
p.changeVisualShape(Track, -1, textureUniqueId=texture_id)


p.resetDebugVisualizerCamera(13,0,-110,[0,4,0])


def get_image(cam_height=0, dims=[512, 512]):
        orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(husky)[1])
        pos = p.getBasePositionAndOrientation(husky)[0]
        pos = np.add(pos, np.array([0, 0, cam_height]))

        camera_eye = [pos[0] - 1 * np.cos(orn[2]), pos[1] - 1 * np.sin(orn[2]), pos[2] + 1.2 * np.cos(orn[0])]
        target_pos = [pos[0] - 1.5 * np.cos(orn[2]), pos[1] - 1.5 * np.sin(orn[2]), pos[2] + 0.4 * np.cos(orn[0])]
        proj_matrix = p.computeProjectionMatrixFOV(90, dims[0] / dims[1], 0.02, 50)

        view_matrix = p.computeViewMatrix(camera_eye, target_pos, [0, 0, 1])
        # proj_matrix = p.computeProjectionMatrixFOV(0, dims[0] / dims[1], 0.02, 50)
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


"""Problem Statement:

Your task is to make the Husky move inside the track and complete one lap of the track, without crossing lane boundaries.

The player must use precise movements to complete the task within a given time frame. The challenge lies in balancing the speed and 
accuracy of the movements by finding the optimal PID gains and processing the camera feed using OpenCV.

"""



# Hints:
# 1. Use get_image() in OpenCV to detect lane boundaries by converting the image to grayscale, applying edge detection (e.g., Canny), and identifying lane markers or contours.  
# 2. Calculate the Husky's deviation from the track center using the processed camera feed and adjust its trajectory accordingly.  
# 3. Create a PID function and implement it to minimize deviation errors, tuning the P, I, and D gains for smooth and accurate movement.  
# 4. Use move() function to adjust the velocity of husky. 
# 5. Track the Husky's position relative to the starting point or use track-specific markers to determine when the lap is completed.






"""
Code here

"""



while True:

    img = get_image()
    cv.imshow("img", img)
    k = cv.waitKey(1)
    if k==ord('q'):
        break

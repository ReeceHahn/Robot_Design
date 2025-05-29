import pybullet as p
import pybullet_data
import time
import numpy as np
import math

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally
    p.setGravity(0, 0, 0)

    planeId = p.loadURDF("plane.urdf")
    ObjId = p.loadURDF("Assignment_4_URDF.urdf")

    # represents roll, pitch and yaw in degrees
    target_angles = [90.0, 90.0, 0.0] 
    
    joint_angles = [math.radians(target_angles[2]), math.radians(target_angles[0]), math.radians(target_angles[1])]

    # uncomment if you want to set the rig to specific roll pitch and yaw values defined in target_angles
    #p.setJointMotorControl2(ObjId, 3, p.POSITION_CONTROL, joint_angles[1]) # roll
    #p.setJointMotorControl2(ObjId, 6, p.POSITION_CONTROL, joint_angles[2]) # pitch
    #p.setJointMotorControl2(ObjId, 1, p.POSITION_CONTROL, joint_angles[0]) # yaw

    while True:
        p.stepSimulation()

        imu_state = p.getLinkState(ObjId, 14)
        imu_position = imu_state[0]
        imu_rotation = np.array(p.getMatrixFromQuaternion(imu_state[1])).reshape(3, 3)

        print("IMU Position: {}\nIMU Rotation: \n{}\n".format(imu_position, np.round(imu_rotation, 3)))

        time.sleep(1./240.)
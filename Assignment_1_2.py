import pybullet as p
import pybullet_data
import time
import numpy as np
import math

# function to get 4x4 homogeneous transformation matrix from link position and rotation
def get_homogeneous_transform(position, rotation):
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = position
    return T

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally
    p.setGravity(0, 0, 0)

    planeId = p.loadURDF("plane.urdf")
    ObjId = p.loadURDF("Assignment_1_URDF.urdf")

    #---------------------------------------------------------------------------------------------------------
    # UNCOMMENT WHEN TESTING FORWARD KINEMATICS, COMMENT WHEN TESTING INVERSE KINEMATICS

    # represents alpha, beta, gamma (joint 1 angle, joint 2 angle, joint 3 angle) in degrees
    target_angles = [0.0, 45.0, 45.0] 

    # singularity whenever gamma = 0 because that means final arm link is fully stretched out
    singularity_angles = [274.0, 37.0, 0.0]

    joint_angles = [math.radians(target_angles[0]), math.radians(target_angles[1]), math.radians(target_angles[2])]

    # uncomment when want to test singularity configuration
    #joint_angles = [math.radians(singularity_angles[0]), math.radians(singularity_angles[1]), math.radians(singularity_angles[2])]

    p.setJointMotorControl2(ObjId, 0, p.POSITION_CONTROL, joint_angles[0])
    p.setJointMotorControl2(ObjId, 1, p.POSITION_CONTROL, joint_angles[1])
    p.setJointMotorControl2(ObjId, 2, p.POSITION_CONTROL, joint_angles[2])
    #---------------------------------------------------------------------------------------------------------

    #---------------------------------------------------------------------------------------------------------
    # UNCOMMENT WHEN TESTING INVERSE KINEMATICS, COMMENT WHEN TESTING FORWARD KINEMATICS
    #target_position1 = [0.5, 1, 1]

    #target_position2 = [0.54, 0.47, 0.13]

    #joint_positions = p.calculateInverseKinematics(ObjId, 3, target_position1)

    # uncomment when want to test joint_positions2
    #joint_positions = p.calculateInverseKinematics(ObjId, 3, target_position2)

    #print("alpha = {}, beta = {}, gamma = {}".format(math.degrees(joint_positions[0]), math.degrees(joint_positions[1]), math.degrees(joint_positions[2])))

    #p.setJointMotorControl2(ObjId, 0, p.POSITION_CONTROL, joint_positions[0])
    #p.setJointMotorControl2(ObjId, 1, p.POSITION_CONTROL, joint_positions[1])
    #p.setJointMotorControl2(ObjId, 2, p.POSITION_CONTROL, joint_positions[2])
    #---------------------------------------------------------------------------------------------------------

    while True:
        p.stepSimulation()

        link_1_state = p.getLinkState(ObjId, 0)
        link_2_state = p.getLinkState(ObjId, 1)
        link_3_state = p.getLinkState(ObjId, 2)
        end_effector_state = p.getLinkState(ObjId, 3)

        link_1_position = link_1_state[0]
        link_1_rotation = np.array(p.getMatrixFromQuaternion(link_1_state[1])).reshape(3, 3)
        link_1_transformation = get_homogeneous_transform(link_1_position, link_1_rotation)

        link_2_position = link_2_state[0]
        link_2_rotation = np.array(p.getMatrixFromQuaternion(link_2_state[1])).reshape(3, 3)
        link_2_transformation = get_homogeneous_transform(link_2_position, link_2_rotation)

        link_3_position = link_3_state[0] 
        link_3_rotation = np.array(p.getMatrixFromQuaternion(link_3_state[1])).reshape(3, 3)
        link_3_transformation = get_homogeneous_transform(link_3_position, link_3_rotation)

        end_effector_position = end_effector_state[0]
        end_effector_transformation = np.array(p.getMatrixFromQuaternion(end_effector_state[1])).reshape(3, 3)
        end_effector_transformation = get_homogeneous_transform(end_effector_position, end_effector_transformation)

        # change joint_angles to joint_positions when testing inverse kinematics, vice versa
        linear_jacobian, angular_jacobian = p.calculateJacobian(ObjId, 3, [0, 0, 0], joint_angles, [0, 0, 0], [0, 0, 0])
        jacobian = np.array(linear_jacobian)
        rank = np.linalg.matrix_rank(jacobian)

        print("Link 1 Homogeneous Transformation: \n{}\nLink 2 Homogeneous Transformation: \n{}\nLink 3 Homogeneous Transformation: \n{}\nEnd effector Homogeneous Transformation: \n{}\nLinear Jacobian: \n{}\nRank of Jacobian: {}\n".format(np.round(link_1_transformation, 3), np.round(link_2_transformation, 3), np.round(link_3_transformation, 3), np.round(end_effector_transformation, 3), np.round(jacobian, 3), rank))

        time.sleep(1./240.)
import pybullet as p
import time
import pybullet_data

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0, 0, 0)
    planeId = p.loadURDF("plane.urdf")
    ObjId = p.loadURDF("Assignment_1.urdf")

    # You can also specify the position and orientation by
    #------------------------------------------------------------------------
    # startPos = [0,0,1]
    # startOrientation = p.getQuaternionFromEuler([0,0,0])
    # boxId = p.loadURDF("example.urdf",startPos, startOrientation)
    #------------------------------------------------------------------------

    joint_angles = [2.3, 1.3, 2] # Represents alpha, beta, gamma (joint 1 angle, joint 2 angle, joint 3 angle)

    p.setJointMotorControl2(ObjId, 0, p.POSITION_CONTROL, joint_angles[0])
    p.setJointMotorControl2(ObjId, 1, p.POSITION_CONTROL, joint_angles[1])
    p.setJointMotorControl2(ObjId, 2, p.POSITION_CONTROL, joint_angles[2])

    link_1_state = p.getLinkState(ObjId, 0)
    link_2_state = p.getLinkState(ObjId, 1)
    link_3_state = p.getLinkState(ObjId, 2)

    while True:
        p.stepSimulation()

        end_effector_state = p.getLinkState(ObjId, 3)

        print("{}, {}".format(end_effector_state[0], end_effector_state[1]))

        time.sleep(1./240.)

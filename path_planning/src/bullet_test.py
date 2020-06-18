import pybullet as p
import time
import pybullet_data


if __name__ == '__main__':

    # p.connect(p.GUI)
    # p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    #
    # obUids = p.loadMJCF("mjcf/humanoid.xml")
    # humanoid = obUids[1]
    #
    # gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
    # jointIds = []
    # paramIds = []
    #
    # p.setPhysicsEngineParameter(numSolverIterations=10)
    # p.changeDynamics(humanoid, -1, linearDamping=0, angularDamping=0)
    #
    # for j in range(p.getNumJoints(humanoid)):
    #     p.changeDynamics(humanoid, j, linearDamping=0, angularDamping=0)
    #     info = p.getJointInfo(humanoid, j)
    #     #print(info)
    #     jointName = info[1]
    #     jointType = info[2]
    #     if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    #         jointIds.append(j)
    #         paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))
    #
    # # p.setRealTimeSimulation(1)
    # while (1):
    #     p.stepSimulation()
    #     p.setGravity(0, 0, p.readUserDebugParameter(gravId))
    #     for i in range(len(paramIds)):
    #         c = paramIds[i]
    #         targetPos = p.readUserDebugParameter(c)
    #         p.setJointMotorControl2(humanoid, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
    #     time.sleep(0.01)

    import pybullet as p
    p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -10)
    huskypos = [0, 0, 0.1]

    husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])
    numJoints = p.getNumJoints(husky)
    for joint in range(numJoints):
        print(p.getJointInfo(husky, joint))
    targetVel = 10  #rad/s
    maxForce = 100  #Newton

    for joint in range(2, 6):
        p.setJointMotorControl(husky, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
    for step in range(300):
        p.stepSimulation()
        time.sleep(0.01)

    targetVel = -10
    for joint in range(2, 6):
        p.setJointMotorControl(husky, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
    for step in range(400):
        p.stepSimulation()

    print p.getContactPoints(husky)

    p.disconnect()
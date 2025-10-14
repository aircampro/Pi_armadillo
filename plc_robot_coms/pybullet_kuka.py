# example using pybullet to simulate a kuka robot specified by the urdf model
#
import pybullet as p
import time
KUKA_IIWA_MODEL="kuka_iiwa/model.urdf"

p.connect(p.GUI)

p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True) 
kukaId = p.loadURDF(KUKA_IIWA_MODEL, [0, 0, 0], useFixedBase=True) 

gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
jointIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations=10)
p.changeDynamics(kukaId, -1, linearDamping=0, angularDamping=0)

for j in range(p.getNumJoints(kukaId)):
    p.changeDynamics(kukaId, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(kukaId, j)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        jointIds.append(j)
        paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))

p.setRealTimeSimulation(1)
while True:
    p.setGravity(0, 0, p.readUserDebugParameter(gravId))
    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)
        p.setJointMotorControl2(kukaId, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
    time.sleep(0.01)


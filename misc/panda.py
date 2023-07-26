import pybullet as p
import numpy as np
import pybullet_data
import os

p.connect(p.GUI)
p.resetDebugVisualizerCamera(cameraDistance=0.9, cameraYaw=180, cameraPitch=0, cameraTargetPosition=[0.,0.3,0.1])

p.resetSimulation()
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.setGravity(0,0,-10)
urdfRootPath=pybullet_data.getDataPath()
planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,-0.65])
rest_poses = [0, 0.32, 0, -2.41, 0, 1.16, 2.356, 0, 0]
panda1Uid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True, 
                                                                basePosition=[0,0,0], baseOrientation=[0, 0, 0.707, 0.707])
p.enableJointForceTorqueSensor(panda1Uid, 8, True)

for i in range(7):
    p.resetJointState(panda1Uid,i, rest_poses[i])

p.resetJointState(panda1Uid, 9, 0.08)
p.resetJointState(panda1Uid,10, 0.08)

tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0.2,-0.65])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

position = [0.,0.3,0.2]
orientation = [0.717,0.,0.,0.717]
fm_list = []
p.addUserDebugLine([0,0,0],[0.3,0,0],[1,0,0],parentObjectUniqueId=panda1Uid, parentLinkIndex=8)
p.addUserDebugLine([0,0,0],[0,0.3,0],[0,1,0],parentObjectUniqueId=panda1Uid, parentLinkIndex=8)
p.addUserDebugLine([0,0,0],[0,0,0.3],[0,0,1],parentObjectUniqueId=panda1Uid, parentLinkIndex=8)
for i in range(10000):
    jointPoses = p.calculateInverseKinematics(panda1Uid, 8, position, orientation)[0:7]
    p.setJointMotorControlArray(panda1Uid, list(range(7))+[9,10], p.POSITION_CONTROL, list(jointPoses)+2*[0])
    p.applyExternalForce(panda1Uid, -1, [0, 0, 100], [0, 0, 0], p.LINK_FRAME)
    #get measured force and torque
    fm = np.array(p.getJointState(panda1Uid, 8)[2])
    fm_list.append(fm)
    p.stepSimulation()

import matplotlib.pyplot as plt
list_size = len(fm_list)
fig = plt.figure(figsize=(20,7))
ax = fig.add_subplot(111)
plt.plot(range(list_size), np.array([elem[0] for elem in fm_list]), label='fm_fx')
plt.plot(range(list_size), np.array([elem[1] for elem in fm_list]), label='fm_fy')
plt.plot(range(list_size), np.array([elem[2] for elem in fm_list]), label='fm_fz')
plt.plot(range(list_size), np.array([elem[3] for elem in fm_list]), label='fm_mx')
plt.plot(range(list_size), np.array([elem[4] for elem in fm_list]), label='fm_my')
plt.plot(range(list_size), np.array([elem[5] for elem in fm_list]), label='fm_mz')
plt.title("measured force/torque")
plt.legend()
plt.show()

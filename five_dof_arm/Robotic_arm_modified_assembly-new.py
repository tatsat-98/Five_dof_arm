'''
import pybullet as p
import pybullet_data
import numpy as np
import time
import os

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load ground
p.loadURDF("plane.urdf",[0,0,0],[0,0,0,1])

# Set your URDF path
urdf_path = "D:/MASTERS/P.R.O.J.E.C.T.S/robotic_arm_modified_assembly/Assem7/urdf/Assem7.urdf"
robot = p.loadURDF(urdf_path, [0, 0, 0],[0,0,0,1], useFixedBase = True)
p.setJointMotorControl2(robot, 2, p.VELOCITY_CONTROL, force=-30)
p.setJointMotorControl2(robot, 3, p.VELOCITY_CONTROL, force=-30)
p.setJointMotorControl2(robot, 4, p.VELOCITY_CONTROL, force=0)

# Run the simulation loop
for i in range(1000):
    #p.setJointMotorControl2(robot, 0, p.POSITION_CONTROL, targetPosition=5*np.sin(i/50))
    #p.setJointMotorControl2(robot, 1, p.POSITsION_CONTROL, targetPosition=2*np.sin(i/50 + 0.5))
    #p.setJointMotorControl2(robot, 2, p.POSITION_CONTROL, targetPosition=100.0*np.sin(i/50))
    #p.setJointMotorControl2(robot, 3, p.POSITION_CONTROL, targetPosition=0*np.sin(i/50 + 0.5))
    p.stepSimulation()
    time.sleep(1./240.)


input("Simulation done. Press Enter to exit...")
p.disconnect()
'''
import pybullet as p
import pybullet_data
import time
import math
import os

# Set URDF file name
URDF_FILENAME = "D:/MASTERS/P.R.O.J.E.C.T.S/robotic_arm_modified_assembly/Assem7-new/urdf/Assem7-new.urdf"

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For plane.urdf, textures, etc.
p.setGravity(0, 0, -9.81)

# Load ground plane
p.loadURDF("plane.urdf")

# Load your 5-DOF URDF
start_pos = [0, 0, 0.0]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF(URDF_FILENAME, start_pos, start_orientation, useFixedBase=True)

# Print joint info
num_joints = p.getNumJoints(robot_id)
print(f"Robot has {num_joints} joints:")
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: Name = {joint_info[1].decode('utf-8')} | Type = {joint_info[2]}")

# Sinusoidal joint control loop
t = 0
while True:
    for j in range(num_joints):
        target = 0.5 * math.sin(t + j)
        p.setJointMotorControl2(robot_id,
                                jointIndex=j,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=target)
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
    t += 0.01

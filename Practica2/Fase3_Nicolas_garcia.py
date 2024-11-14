import pybullet as p
import pybullet_data
import argparse
import time
import math
import csv


# parser = argparse.ArgumentParser(description="URDF viewer example")
# parser.add_argument("--urdf", type=str, required=True, help="Ruta al archivo URDF.")
# args = parser.parse_args()
# urdf_path = args.urdf

APPROACH = 0
PICK = 1
ELEVATE = 2
ROTATE = 3
ROTATE2 = 4
PLACE = 5
DROP = 6
RETURN1 = 7
RETURN2 = 8
INIT = 9
FINISHED = 10
robot_urdf = "urdf/robot.urdf"
cube_urdf = "urdf/cube.urdf"


physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

startPos = [0,-4,1]
robot_startOrientation = p.getQuaternionFromEuler([0,0,math.pi/2])
robotId = p.loadURDF(robot_urdf, startPos, robot_startOrientation)

cube_startPos = [0,4,0.5]
cube_startOrientation = p.getQuaternionFromEuler([0,0,0])
cubeId = p.loadURDF(cube_urdf, cube_startPos, cube_startOrientation)


initial_time = time.time()
last_time = 0.0
measures = []
robotEndEffectorIndex=9

PICK_POS = [0,4,0.52]
INIT_POS = [0,4.1,1]
MOVE_2 = [0,3.5,3.2]
MOVE_3 = [0,4.5,2.8]
MOVE_R = [2.0,3.0,2.8]
MOVE_4 = [0,1.5,2.8]
MOVE_5 = [0,1.5,1.7]


wheels_joints = [19, 22, 25, 28] 
gripper_joints = [10, 11]
arm_joints= [1,2,4,6,7,8]

state = APPROACH
last_time = time.time()

# Marcas de timepo de cada estado
PICK_time = 0
ELEVATE_time = 0
ROTATE_time = 0
ROTATE2_time = 0
DROP_time = 0
DROP2_time = 0
RETURN1_time = 0
RETURN2_time = 0
INIT_time = 0
FINISHED_time = 0


# num_joints = p.getNumJoints(robotId)
# print("num_joints: ", num_joints)
# for i in range(num_joints):
#     info = p.getJointInfo(robotId, i)
#     print("%d: %s %d" % (info[0], info[1].decode("utf-8"), info[2]))

for i in arm_joints:
    p.enableJointForceTorqueSensor( robotId, i, enableSensor=True)
for i in gripper_joints:
    p.enableJointForceTorqueSensor( robotId, i, enableSensor=True)

def registerForce(time):
    
    total_force = 0
    for i in arm_joints:
        jointForces = p.getJointState(robotId, i)[2]
        jointForce = 0
        for i in range(3):
            jointForce += abs(jointForces[i])

        total_force += jointForce
    for i in gripper_joints:
        jointForces = p.getJointState(robotId, i)[2]
        jointForce = 0
        for i in range(3):
            jointForce += abs(jointForces[i])
            
        total_force += jointForce
    

    measures.append([time,10,total_force])

p.changeDynamics(cubeId, -1, lateralFriction=5)


while(1):
    
    if (time.time() - last_time > 0.01 and state != APPROACH):
        registerForce(time.time()-initial_time)
        last_time = time.time()

    if state == APPROACH:   
        
        p.setJointMotorControlArray(bodyIndex=robotId,  
                                    jointIndices=wheels_joints, 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities=[5,5,5,5], 
                                    forces=[100,100,100,100])
        
        if (p.getLinkState(robotId, 10)[0][1] >= 3.9):

            p.setJointMotorControlArray(bodyIndex=robotId,  
                                        jointIndices=wheels_joints,
                                        controlMode=p.VELOCITY_CONTROL, 
                                        targetVelocities=[0,0,0,0],
                                        forces=[100,100,100,100])
            initial_time = time.time()
            state = PICK
            PICK_time = time.time()- initial_time
            print("PICK init at " + str(PICK_time))
            
    elif state == PICK:
        jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex,
                                                   PICK_POS, 
                                                   targetOrientation=p.getQuaternionFromEuler([math.pi,0,math.pi/2]))
        
        positions = [jointPoses[0], jointPoses[1], jointPoses[2], jointPoses[3], jointPoses[4], jointPoses[5]]

        p.setJointMotorControlArray(bodyIndex=robotId,  jointIndices=arm_joints,
                                     controlMode=p.POSITION_CONTROL, 
                                     targetPositions=positions, 
                                     forces=[100,100,100,100,75,75],  
                                     positionGains=[0.02,0.02,0.02,0.02,0.02,0.02], 
                                     velocityGains=[1.3,1.3,1.3,1.3,1.3,1.3])
        
        if (PICK_POS[0] - 0.05 < p.getLinkState(robotId, robotEndEffectorIndex)[0][0] <= PICK_POS[0] + 0.05 and 
            PICK_POS[1] - 0.05 < p.getLinkState(robotId, robotEndEffectorIndex)[0][1] <= PICK_POS[1] + 0.05 and 
            PICK_POS[2] - 0.05 < p.getLinkState(robotId, robotEndEffectorIndex)[0][2] <= PICK_POS[2] + 0.05):
           
            p.setJointMotorControlArray(bodyIndex=robotId,  jointIndices=gripper_joints,
                                         controlMode=p.POSITION_CONTROL, 
                                         targetPositions=[0.055, 0.055], 
                                         forces=[100,100])
            
            if (p.getJointState(robotId, 10)[0] >= 0.045 and p.getJointState(robotId, 11)[0] >= 0.045):                 
                state = ELEVATE
                ELEVATE_time = time.time() -  initial_time
                print("ELEVATE init at " + str(ELEVATE_time) + "s")

    elif state == ELEVATE:
        
        jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, 
                                                  MOVE_3, 
                                                  targetOrientation=p.getQuaternionFromEuler([math.pi,0,math.pi/2]))
        
        positions = [jointPoses[0], jointPoses[1], jointPoses[2], jointPoses[3], jointPoses[4], jointPoses[5]]
        
        p.setJointMotorControlArray(bodyIndex=robotId,  jointIndices=arm_joints,
                                     controlMode=p.POSITION_CONTROL, 
                                     targetPositions=positions, 
                                     forces=[200,250,200,150,100,50], 
                                     positionGains=[0.03,0.03,0.03,0.03,0.03,0.03], 
                                     velocityGains=[1,1,1,1,1,1.7])
        
        if (MOVE_3[0] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][0] <= MOVE_3[0] + 0.1 and 
            MOVE_3[1] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][1] <= MOVE_3[1] + 0.1 and 
            MOVE_3[2] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][2] <= MOVE_3[2] + 0.1):

            state = ROTATE
            ROTATE_time = time.time() - initial_time 
            print("ROTATE init at " + str(ROTATE_time) + "s") 
                 
            
    elif state == ROTATE:
        
        jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, 
                                                  MOVE_R, 
                                                  targetOrientation=p.getQuaternionFromEuler([math.pi,0,0]))
        
        positions = [jointPoses[0], jointPoses[1], jointPoses[2], jointPoses[3], jointPoses[4], jointPoses[5]]

        p.setJointMotorControlArray(bodyIndex=robotId,  jointIndices=arm_joints, 
                                    controlMode=p.POSITION_CONTROL, 
                                    targetPositions=positions, 
                                    forces=[150,300,200,150,50,50], 
                                    positionGains=[0.04,0.04,0.04,0.04,0.04,0.04], 
                                    velocityGains=[1.5,1.5,1.5,1.5,1.5,1.5])
        
        if (MOVE_R[0] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][0] <= MOVE_R[0] + 0.1 and 
            MOVE_R[1] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][1] <= MOVE_R[1] + 0.1 and
            MOVE_R[2] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][2] <= MOVE_R[2] + 0.1):
                state = ROTATE2
                ROTATE2_time = time.time() - initial_time
                print("ROTATE2 init at " + str(ROTATE2_time) + "s")
                

    elif state == ROTATE2:
        
        jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, 
                                                  MOVE_4, 
                                                  targetOrientation=p.getQuaternionFromEuler([math.pi,0,-math.pi/2]))
        
        positions = [jointPoses[0], jointPoses[1], jointPoses[2], jointPoses[3], jointPoses[4], jointPoses[5]]

        p.setJointMotorControlArray(bodyIndex=robotId,  jointIndices=arm_joints, 
                                    controlMode=p.POSITION_CONTROL, 
                                    targetPositions=positions, 
                                    forces=[200,300,200,150,100,50], 
                                    positionGains=[0.03,0.03,0.03,0.03,0.03,0.03], 
                                    velocityGains=[1.5,1.5,1.5,1.5,1.5,1.5])
        
        if (MOVE_4[0] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][0] <= MOVE_4[0] + 0.1 and
            MOVE_4[1] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][1] <= MOVE_4[1] + 0.1 and 
            MOVE_4[2] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][2] <= MOVE_4[2] + 0.1):
                state = PLACE
                DROP2_time = time.time() - initial_time
                print("PLACE init at " + str(DROP2_time) + "s")
                

    elif state == PLACE:
        
        jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex,
                                                  MOVE_5, 
                                                  targetOrientation=p.getQuaternionFromEuler([math.pi,0,-math.pi/2]))
        
        positions = [jointPoses[0], jointPoses[1], jointPoses[2], jointPoses[3], jointPoses[4], jointPoses[5]]
        p.setJointMotorControlArray(bodyIndex=robotId,  jointIndices=arm_joints, 
                                    controlMode=p.POSITION_CONTROL, 
                                    targetPositions=positions, 
                                    forces=[200,300,200,75,75,50], 
                                    positionGains=[0.03,0.03,0.03,0.03,0.03,0.03], 
                                    velocityGains=[1.7,1.7,1.7,1.7,1.7,1.7])
        
        if (MOVE_5[0] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][0] <= MOVE_5[0] + 0.1 and 
            MOVE_5[1] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][1] <= MOVE_5[1] + 0.1 and 
            MOVE_5[2] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][2] <= MOVE_5[2] + 0.1):
                state = DROP
                DROP_time = time.time() - initial_time
                print("DROP init at " + str(DROP_time) + "s")
                

    elif state == DROP:
        p.setJointMotorControlArray(bodyIndex=robotId,  jointIndices=gripper_joints, controlMode=p.POSITION_CONTROL, targetPositions=[0.0,0.0], forces=[50,50])

        if (p.getJointState(robotId, 10)[0] <= 0.005 and p.getJointState(robotId, 11)[0] <= 0.005):
            state = RETURN1
            RETURN2_time = time.time() - initial_time
            print("RETURN1 init at " + str(RETURN2_time) + "s")
            

    elif state == RETURN1:
        
        jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, 
                                                  MOVE_4, 
                                                  targetOrientation=p.getQuaternionFromEuler([math.pi,0,-math.pi/2]))
        
        positions = [jointPoses[0], jointPoses[1], jointPoses[2], jointPoses[3], jointPoses[4], jointPoses[5]]

        p.setJointMotorControlArray(bodyIndex=robotId,  jointIndices=arm_joints, 
                                    controlMode=p.POSITION_CONTROL, 
                                    targetPositions=positions, 
                                    forces=[100,300,200,150,100,50],
                                    positionGains=[0.03,0.03,0.03,0.03,0.03,0.03],
                                    velocityGains=[1.3,1.3,1.3,1.3,1.3,1.3])
        
        if (MOVE_4[0] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][0] <= MOVE_4[0] + 0.1 and
            MOVE_4[1] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][1] <= MOVE_4[1] + 0.1 and
            MOVE_4[2] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][2] <= MOVE_4[2] + 0.1):
                state = RETURN2
                RETURN1_time = time.time() - initial_time
                print("RETURN2 init at " + str(RETURN1_time) + "s")
                

    elif state == RETURN2:
        
        jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, 
                                                    MOVE_R, 
                                                    targetOrientation=p.getQuaternionFromEuler([math.pi,0,0]))
        
        positions = [jointPoses[0], jointPoses[1], jointPoses[2], jointPoses[3], jointPoses[4], jointPoses[5]]

        p.setJointMotorControlArray(bodyIndex=robotId,  jointIndices=arm_joints,
                                    controlMode=p.POSITION_CONTROL, 
                                    targetPositions=positions, 
                                    forces=[100,300,200,75,75,50],
                                    positionGains=[0.03,0.03,0.03,0.03,0.03,0.03],
                                    velocityGains=[1,1,1,1,1,1])
        
        if (MOVE_R[0] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][0] <= MOVE_R[0] + 0.1 and
            MOVE_R[1] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][1] <= MOVE_R[1] + 0.1 and 
            MOVE_R[2] - 0.1 < p.getLinkState(robotId, robotEndEffectorIndex)[0][2] <= MOVE_R[2] + 0.1):
                state = INIT
                INIT_time = time.time() - initial_time
                print("INIT init at " + str(INIT_time) + "s")

    elif state == INIT:
        
        p.setJointMotorControlArray(bodyIndex=robotId,  jointIndices=arm_joints,
                                    controlMode=p.POSITION_CONTROL, 
                                    targetPositions=[0,0,0,0,0,0], 
                                    forces=[200,200,200,75,75,75],
                                    positionGains=[0.03,0.03,0.03,0.03,0.03,0.03],
                                    velocityGains=[1.3,1.3,1.3,1.3,1.3,1.3])
        
        if all((-0.1 < p.getJointState(robotId, i)[0] < 0.1) for i in arm_joints):
            state = FINISHED
            FINISHED_time = time.time() - initial_time
            print("FINISHED init at " + str(FINISHED_time) + "s")
            

    elif state == FINISHED:
        G_total = 0

        for i in measures:
            G_total += i[2]
            
        with open('Fase3_nicolas_garcia2.csv', 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(measures)
        
        exit(0)
    p.stepSimulation()
    time.sleep(1./200.)
    
    




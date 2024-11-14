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
ramp_urdf = "ramp.urdf"
robot_urdf = "husky/husky_mod.urdf"
#robot_urdf = "husky/husky.urdf"
barrier_urdf = "barrier.urdf"
mat_urdf = "mat.urdf"


physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

startPos = [0,0,0.1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
rampId = p.loadURDF(ramp_urdf,startPos, startOrientation)

robot_startOrientation = p.getQuaternionFromEuler([0,0,math.pi/2])

robotId = p.loadURDF(robot_urdf,startPos, robot_startOrientation)

barrier_startPos = [-1.5, 17, 0.5]
barrier_startOrientation = p.getQuaternionFromEuler([0,0,0])
barrierId = p.loadURDF("barrier.urdf", barrier_startPos, startOrientation)
p.changeDynamics(barrierId, 0, localInertiaDiagonal=[3.751666667,3.751666667,3.751666667])

mat_startPos = [0, 20, 0.5]
mat_startOrientation = p.getQuaternionFromEuler([0,0,0])
matId = p.loadURDF("mat.urdf", mat_startPos, startOrientation)



init_time = time.time()
p.setRealTimeSimulation(1)
last_distance = 0.0
measures = []
vel = 11.4
force = 30
try:
    
    while True:

        # #set the joint friction
        if (p.getBasePositionAndOrientation(robotId)[0][1] - last_distance > 0.01):
            act_time = time.time() - init_time
            last_distance = p.getBasePositionAndOrientation(robotId)[0][1]
            act_vel = p.getBaseVelocity(robotId)
            vel = 11.4
            force = 30

            measures.append((act_time, last_distance,act_vel[0][1],vel,vel,vel,vel,force,force,force,force))
            #print("Time: ", act_time, "Distance: ", last_distance, "Velocity: ", act_vel[0][1])


            

        p.setJointMotorControlArray(robotId, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=[vel,vel,vel,vel], forces=[force,force,force,force])
       
        if (p.getBasePositionAndOrientation(robotId)[0][1]>20.5):
             #print(measures)
             print("Robot reached the mat")
             while(p.getBaseVelocity(robotId)[0][1]>0.01):
                p.setJointMotorControlArray(robotId, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=[0,0,0,0], forces=[100,100,100,100])

             with open('Fase3_3.csv', 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(measures)   
             time.sleep(1)
             p.disconnect()
             exit(0)
            
except KeyboardInterrupt:
      pass
	
p.disconnect() 

import pybullet as p
import pybullet_data
import argparse
import time
import math
import csv



ramp_urdf = "ramp.urdf"
robot_urdf = "husky/husky_mod.urdf"
# robot_urdf = "husky/husky.urdf"
barrier_urdf = "barrier.urdf"
mat_urdf = "mat.urdf"


physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

startPos = [0,0,0]
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
vels = []
forces = []
prev_error = 0
prev_time = 0
error_sum = 0
past_state = 0
PLANE = 0
RAMP_UP = 1
RAMP_DOWN = 2

try:
    
    while True:
        angle_y_rad = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robotId)[1])[1]
        angle_y = math.degrees(angle_y_rad)
        vel_y = p.getBaseVelocity(robotId)[0][1]
        error = 2-vel_y
        act_time = time.time() - init_time
        error_sum += error


        Kp = 50
        Kd = 25
        ki = 0.0001

       
        vel = Kp*error + Kd*(error-prev_error)/(act_time-prev_time) + ki*error_sum
        prev_error = error
        prev_time = act_time
        
        if (vel > 11.5):
             vel = 11.2

        if (vel < -12):
            vel = -12
        
        
        if (angle_y < -10):
            if (vel_y < 1.8):
                t_vel_ad = 17
                t_vel_tr = 15
                force_ad = 80
                force_tr = 70
               
            elif (vel_y < 1.5):
                t_vel_ad = 40
                t_vel_tr = 35
                force_ad = 135
                force_tr = 135

            elif (vel_y < 1):
                t_vel_ad = 40
                t_vel_tr = 40
                force_ad = 140
                force_tr = 140
            elif (vel_y < 0.5):
                t_vel_ad = 40
                t_vel_tr = 40
                force_ad = 150
                force_tr = 150
            else:
                t_vel_ad = 40
                t_vel_tr = 13
                force_ad = 150
                force_tr = 110
        elif (5 < angle_y < 15):
            if (vel_y <= 2.1):
                t_vel_ad = 9
                t_vel_tr = 9
                force_ad = 30
                force_tr = 30
            elif (vel_y > 2.1):
                t_vel_ad = 8
                t_vel_tr = 8
                force_ad = 30
                force_tr = 30
            
        elif(15 < angle_y < 25):
            t_vel_ad = 11
            t_vel_tr = 11
            force_ad = 30
            force_tr = 30

        elif (angle_y > 25):
            t_vel_ad = -11
            t_vel_tr = -12
            force_ad = 30
            force_tr = 30
        elif (angle_y > 28):
            if (vel_y <= 2.1):
                t_vel_ad = 10
                t_vel_tr = 10
                force_ad = 35
                force_tr = 35
            elif (vel_y > 2.1):
                t_vel_ad = -14
                t_vel_tr = -12
                force_ad = 30
                force_tr = 30
        elif (-0.3 <= angle_y <= 0.3):
            if (1.6< vel_y < 1.90):
                
                t_vel_ad = vel
                t_vel_tr = vel
                force_ad = 30
                force_tr = 30
            else:
                t_vel_ad = 11.2
                t_vel_tr = 11.2
                force_ad = 30
                force_tr = 30
       
        p.setJointMotorControlArray(robotId, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=[t_vel_ad,t_vel_ad,t_vel_tr,t_vel_tr], forces=[force_ad,force_ad,force_tr,force_tr])   
        
        if (p.getBasePositionAndOrientation(robotId)[0][1] - last_distance > 0.01):
            act_time = time.time() - init_time
            last_distance = p.getBasePositionAndOrientation(robotId)[0][1]
            vel = p.getBaseVelocity(robotId)
            measures.append((act_time, last_distance,vel[0][1],t_vel_ad,t_vel_ad,t_vel_tr,t_vel_tr,force_ad,force_ad,force_tr,force_tr))
            #print("Time: ", act_time, "Distance: ", last_distance, "Velocity: ", vel[0][1])


        
       
        if (p.getBasePositionAndOrientation(robotId)[0][1]>20.5):
             #print(measures)
             print("Robot reached the mat")
             while(p.getBaseVelocity(robotId)[0][1]>0.01):
                p.setJointMotorControlArray(robotId, [2, 3, 4, 5], 
                            p.VELOCITY_CONTROL, targetVelocities=[0,0,0,0], forces=[100,100,100,100])

             with open('measures4.csv', 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(measures)   
             time.sleep(1)
             p.disconnect()
             exit(0)
            
except KeyboardInterrupt:
      pass
	
p.disconnect() 

# -*- coding: utf-8 -*-
"""
Created on Tue Jan 06 22:00:39 2015

@author: Nikolai K.
"""
#Import Libraries:
import vrep                  #V-rep library
import sys
import time                #used to keep track of time
import numpy as np         #array library
import math
import matplotlib as mpl   #used for image plotting

#Pre-Allocation

PI=math.pi  #pi=3.14..., constant

vrep.simxFinish(-1) # just in case, close all opened connections

clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

# params: auxillaryPackets from simxReadVisionSensor function
# returns: steer amount
def auxPacketsToSteer(auxPackets):
    blobCount = auxPackets[1][0]
    print("blob count", blobCount)
    # If we have more than 0 blobs then use that data
    # If more than 2 blobs detected, throw out reading because it probably contains noise
    if blobCount > 0 and blobCount < 3:
        xtarget = auxPackets[1][5]
        ytarget = auxPackets[1][6]
        print("x: ", xtarget, ", y: ", ytarget)
        return 1*(0.5-xtarget)
        

if clientID != -1:  #check if client connection successful
    print('Connected to remote API server')

else:
    print('Connection not successful')
    sys.exit('Could not connect')


#retrieve motor  handles
errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

# Get visionSensorHandle
res1, visionSensorHandle = vrep.simxGetObjectHandle(clientID, 'cam1', vrep.simx_opmode_oneshot_wait)

# Setup and 
result, detectionState, auxPackets = vrep.simxReadVisionSensor(clientID, visionSensorHandle, vrep.simx_opmode_streaming)

sensor_h = [] #empty list for handles
sensor_val = np.array([]) #empty array for sensor measurements

#orientation of all the sensors: 
sensor_loc = np.array([-PI/2, -50/180.0*PI, -30/180.0*PI, -10/180.0*PI, 10/180.0*PI, 30/180.0*PI, 50/180.0*PI, PI/2 ,PI/2 ,130/180.0*PI ,150/180.0*PI ,170/180.0*PI ,-170/180.0*PI ,-150/180.0*PI ,-130/180.0*PI ,-PI/2]) 

#for loop to retrieve sensor arrays and initiate sensors
for x in range(1, 16 + 1):
        errorCode, sensor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(x), vrep.simx_opmode_oneshot_wait)
        sensor_h.append(sensor_handle) #keep list of handles        
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensor_handle, vrep.simx_opmode_streaming)                
        sensor_val = np.append(sensor_val, np.linalg.norm(detectedPoint)) #get list of values
        
t = time.time()

while (time.time() - t) < 600:
    #Loop Execution
    sensor_val = np.array([])    
    for x in range(1, 16 + 1):
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensor_h[x - 1],vrep.simx_opmode_buffer)                
        sensor_val = np.append(sensor_val,np.linalg.norm(detectedPoint)) #get list of values

    result, detectionState, auxPackets = vrep.simxReadVisionSensor(clientID, visionSensorHandle, vrep.simx_opmode_buffer)

    if auxPackets:
        steerValue = auxPacketsToSteer(auxPackets)
        print("STEER VALUE = ", steerValue)

    #controller specific
    sensor_sq = sensor_val[0:8] * sensor_val[0:8] #square the values of front-facing sensors 1-8
        
    max_ind = np.where(sensor_sq == np.max(sensor_sq))
    max_ind = max_ind[0][0]

    # print("max_ind = ", max_ind)
    # print("SENSOR VALUES = ", sensor_val)
    # print("sensor_sq[max_ind] = ", sensor_sq[max_ind])
    if sensor_sq[max_ind] > 0.8:
        steer = -1 / sensor_loc[max_ind]
        forwardVelocity = 0 # set forward velocity to 0 so it rotates in place
    else: # set steer 
        steer = 0
        forwardVelocity = 1 
        
    print("SENSOR_LOC = ", sensor_loc[max_ind])
    print("OBJ AVOID STEER = ", steer)
    steeringGain = 0.5	#steering gain
    leftMotorVelocity = forwardVelocity + steeringGain * steer
    rightMotorVelocity = forwardVelocity - steeringGain * steer
    print("V_l =", leftMotorVelocity)
    print("V_r =", rightMotorVelocity)

    errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, leftMotorVelocity, vrep.simx_opmode_streaming)
    errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, rightMotorVelocity, vrep.simx_opmode_streaming) 

    # pixelRed = image[b * (Y)]
    print("------------------------------------------------------------------")
    time.sleep(0.2) #loop executes once every 0.2 seconds (= 5 Hz)

#Post Allocation
errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)


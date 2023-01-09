
# Import libraries
from controller import Robot, Motor, DistanceSensor, Lidar
import csv
#execfile('functionsLibrary.py')

# Literals
TIME_STEP = 16
MAX_SPEED = 6.5
BASE_SPEED = 3
SAFETY_DISTANCE = 0.40


# Robot instance
robot = Robot()

# Get components of robot
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
lidar = robot.getDevice("LDS-01")
lidarMainMotor = robot.getDevice("LDS-01_main_motor")
lidarSecondaryMotor = robot.getDevice("LDS-01_secondary_motor")

#PID values
lastError = 0
cumError = 0
lLastError = 0
lastCorrection = 0


# MOVEMENT FUNCTIONS

#Functions
def moveForward():
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(BASE_SPEED)
    rightMotor.setVelocity(BASE_SPEED)
    print("Moving forward")
    return(BASE_SPEED, BASE_SPEED)

def turnLeft():    
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(BASE_SPEED/9)
    rightMotor.setVelocity(BASE_SPEED)
    print("Turning left")
    return(BASE_SPEED/9, BASE_SPEED)
    
def turnRight():    
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(BASE_SPEED)
    rightMotor.setVelocity(BASE_SPEED/9)
    print("Turning right")
    return(BASE_SPEED, BASE_SPEED/9)
    
def rotateLeft():
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(-BASE_SPEED)
    rightMotor.setVelocity(BASE_SPEED)
    print("Rotating left")
    return(-BASE_SPEED, BASE_SPEED)
    
def rotateRight():
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(BASE_SPEED)
    rightMotor.setVelocity(-BASE_SPEED)
    print("Rotating right")
    return(BASE_SPEED, -BASE_SPEED)
    
    
def moveBackward():
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(-BASE_SPEED)
    rightMotor.setVelocity(-BASE_SPEED)
    print("Moving backward")
    return(-BASE_SPEED, -BASE_SPEED)

def stop():
    leftMotor.setPosition(0) # Set position es absoluto o relativo?
    rightMotor.setPosition(0)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    print("Stopping")
    return (0, 0)
    
def moveSpeeds(leftSpeed, rightSpeed):
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    print("Moving speed")
    return(leftSpeed, rightSpeed)

def constrain(val, min_val, max_val):
    if val < min_val: return min_val
    if val > max_val: return max_val
    return val



# ML PART ********************************************************************
# Dependencies import
print("Import libaries...")
import math
import pandas as pd
import numpy as np
import sklearn as sk


# Read CSV
print("Reading CSV...")
from numpy import genfromtxt
data = np.genfromtxt('data.csv', delimiter=',')

# NANS and INFS  filtering
print("Filtering data...")

nCols = data.shape[1]
nRows = data.shape[0]

counterNan = 0
counterInf = 0

for j in range(nCols):
  for i in range(nRows):

    if math.isnan(data[i, j]):
      data[i, j] = 0
      counterNan += 1

    if math.isinf(data[i, j]):
      data[i, j] = 0
      counterInf += 1

print(f"Filtrados {counterNan} Nans")
print(f"Filtrados {counterInf} Infs")

# Inputs and targets split
X = data[:, : 3]
y = data[:, 3 :]

# Split train, validation, test

X_train = X[: 12000, :]
X_val = X[12000:17000, :]
X_test = X[17000 : , :]

y_train = y[: 12000, :]
y_val = y[12000:17000, :]
y_test = y[17000 :, :]


# MODEL IMPLEMENTATION
N_INPUTS = 3
N_TARGETS = 2

# Train
print("Training model...")
from sklearn.tree import DecisionTreeRegressor
from sklearn.neural_network import MLPRegressor

#model = DecisionTreeRegressor()
model = MLPRegressor()

model.fit(X_train, y_train)

print("Model trained successfully!")
print("* Program start *")
# END ML PART ************************************************************




def leftWallFollower(rangeVec, LAT_MARGIN=0.20,FRONT_MARGIN=0.25):
    
    HYSTERESIS = 0.005
    FRONT_ANGLE = 20
    LEFT_ANGLE = 40
    
    # Minimize measurements from Lidar for values
    frontRange = min(rangeVec[int(180-FRONT_ANGLE/2):int(180+FRONT_ANGLE/2)])
    cornerRange = min(rangeVec[int(135-LEFT_ANGLE/2):int(135+LEFT_ANGLE/2)])
    leftRange = min(rangeVec[int(90-LEFT_ANGLE/2):int(90+LEFT_ANGLE/2)])
    
    # Ruled-based behaviour
    if frontRange < FRONT_MARGIN:
        act = rotateRight()
    else:
        # In case of inf value...
        frontRange = constrain(frontRange, -100, 100)
        leftRange = constrain(leftRange, -100, 100)
        cornerRange = constrain(cornerRange, -100, 100)
        
        predictions = model.predict([[frontRange, leftRange, cornerRange]])

        leftSp = predictions[0][0]
        rightSp = predictions[0][1]
        
        # Limitate actuation in available motor range
        if (leftSp > MAX_SPEED): leftSp = MAX_SPEED
        if (leftSp < -MAX_SPEED): leftSp = - MAX_SPEED
        if (rightSp > MAX_SPEED): rightSp = MAX_SPEED
        if (rightSp < -MAX_SPEED): rightSp = -MAX_SPEED   
        
        act = moveSpeeds(leftSp, rightSp)
    return act
    
def printInfo():
    #print(lidarRangeImage)
    print("Min distance: ", end="")
    print(min(lidarRangeImage))
    print("Min distance index: ", end="")
    print(lidarRangeImage.index(min(lidarRangeImage)))
    print("Forward distance: ", end="")
    print(lidarRangeImage[179])
 
 
# ++++++>>>>>>  PROGRAM  <<<<<<++++++

# LIDAR 
# Move lidar motors (just visualization)
lidarMainMotor.setPosition(float('inf'))
lidarSecondaryMotor.setPosition(float('inf'))
lidarMainMotor.setVelocity(40.0)
lidarSecondaryMotor.setVelocity(60.0)

#Enable lidar
lidar.enable(TIME_STEP)
lidar.enablePointCloud() 

#Variables
lidarWidth = lidar.getHorizontalResolution()
lidarMaxRange = lidar.getMaxRange()
  
# LOOP
loopCounter = 0
timeCounter = 0
while robot.step(TIME_STEP) != -1:
    timeCounter += TIME_STEP
    
    lidarRangeImage = lidar.getRangeImage()
    setLidarRangeImage = set(lidarRangeImage) # Take elements that are not repeated
    if len(setLidarRangeImage) > 1:
        printInfo()
        action = leftWallFollower(lidarRangeImage)
    
    
    #♠ Save values to CSV
    frontMeasure = lidarRangeImage[165]
    leftMeasure = lidarRangeImage[90]
    cornerMeasure = lidarRangeImage[135]
        
  
    # Update loop counter
    print("Loop count: ", loopCounter)
    loopCounter = loopCounter + 1
   
  

# Enter here exit cleanup code.
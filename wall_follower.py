# Import libraries
from controller import Robot, Motor, DistanceSensor, Lidar
import csv
#from functionsLibrary import *

# Literals
TIME_STEP = 16
MAX_SPEED = 10
BASE_SPEED = 5
SAFETY_DISTANCE = 0.40

# Robot instance
robot = Robot()

# Get components of robot
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
lidar = robot.getDevice("LDS-01")
lidarMainMotor = robot.getDevice("LDS-01_main_motor")
lidarSecondaryMotor = robot.getDevice("LDS-01_secondary_motor")

# * PROGRAM FUNCTIONS *
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
     
def leftWallFollow(rangeVec):
    
    # Detection of walls
    front_wall = rangeVec[180] < 0.30
    left_wall = rangeVec[90] < 0.30
    left_corner = rangeVec[135] < 0.30
    
    print("Front wall: ", front_wall)
    print("Left corner: ", left_corner)
    print("Left wall: ", left_wall)
    
    # Reactive action
    if front_wall:
        action = rotateRight()
        
    else:
       if left_wall:
           action = moveForward()
           
       else:
           action = turnLeft()
           
       if left_corner:
           action = rotateRight()
           

    return(action)
    
    
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
loopCounter = 0

# CSV opening in write mode
with open('data.csv', 'w', newline='') as csvfile:
    # Crea un objeto csv.writer
    csv_writer = csv.writer(csvfile)

# * PROGRAM LOOP *
    while robot.step(TIME_STEP) != -1:
        
        lidarRangeImage = lidar.getRangeImage()
        setLidarRangeImage = set(lidarRangeImage) # Take elements that are not repeated
        
        if len(setLidarRangeImage) > 1: # Could avoid lidar non-detection fail
            action = leftWallFollow(lidarRangeImage)

        #♠ Save values to CSV
        frontMeasure = lidarRangeImage[165]
        leftMeasure = lidarRangeImage[90]
        cornerMeasure = lidarRangeImage[135]
        
        # Write CSV
        csv_writer.writerow([frontMeasure, leftMeasure, cornerMeasure, action[0], action[1]])
        
        # Update loop counter
        print("Loop count: ", loopCounter)
        loopCounter = loopCounter + 1
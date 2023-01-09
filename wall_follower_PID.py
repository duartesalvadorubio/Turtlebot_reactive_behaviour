# Import libraries
from controller import Robot, Motor, DistanceSensor, Lidar
import csv
#execfile('functionsLibrary.py')

# Literals
TIME_STEP = 16
MAX_SPEED = 6.5
BASE_SPEED = 3

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
    
def PIDactuation(LAT_MARGIN, leftRange):
    global lastError, cumError, lLastError, lastCorrection
    ANTI_WINDUP = 20
    
    # PID Gains
    # Standar PID best results: Kp = 30, Kd = 12, Ki = 0
    # Tustin PID best results: Kp 50, Kd = 25, Ki = 0
    Kp = 30
    Kd = 12
    Ki = 0
    
    # Obtain error and error marks
    error = leftRange - LAT_MARGIN
    rateError= error - lastError
    cumError = error + cumError
    
    # Obtain PID actuation
    # Main PID
    correction = Kp * error + Kd * rateError + Ki * cumError
    
    # Tustin discrete PID
    # Obtain integrative part and apply ANTI-WINDUP method
    integralValue = Ki * error
    integralValue = constrain(integralValue, -ANTI_WINDUP, ANTI_WINDUP)
    
    #correction = lastCorrection + Kp * (error - lastError) + Kd * (error + lLastError - 2*lastError)
    
    # Compute speed for each wheel based on actuation
    leftSpeed = BASE_SPEED - correction
    rightSpeed = BASE_SPEED + correction
    
    # Limitate actuation in available motor range
    if (leftSpeed > MAX_SPEED): leftSpeed = MAX_SPEED
    if (leftSpeed < -MAX_SPEED): leftSpeed = - MAX_SPEED
    if (rightSpeed > MAX_SPEED): rightSpeed = MAX_SPEED
    if (rightSpeed < -MAX_SPEED): rightSpeed = -MAX_SPEED
    
    # Save last error for next iteration
    lastError = error
    # Only for Tustin discretized PID
    lLastError = lastError 
    lastCorrection = correction
    
    return(leftSpeed, rightSpeed)
        
        

def leftWallFollower(rangeVec, LAT_MARGIN=0.20,FRONT_MARGIN=0.25):
    
    #HYSTERESIS = 0.005
    FRONT_ANGLE = 20
    LEFT_ANGLE = 40
    
    # Minimize measurements from Lidar for values
    frontRange = min(rangeVec[int(180-FRONT_ANGLE/2):int(180+FRONT_ANGLE/2)])
    leftRange = min(rangeVec[int(135-LEFT_ANGLE/2):int(135+LEFT_ANGLE/2)])
    
    # Ruled-based behaviour
    if frontRange < FRONT_MARGIN:
        act = rotateRight()
    else:
        actuation = PIDactuation(LAT_MARGIN, leftRange)        
        act = moveSpeeds(actuation[0], actuation[1])
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


# CSV opening in write mode
with open('data.csv', 'w', newline='') as csvfile:
    # Crea un objeto csv.writer
    csv_writer = csv.writer(csvfile)
    
    # LOOP
    loopCounter = 0
    timeCounter = 0
    while robot.step(TIME_STEP) != -1:
        timeCounter += TIME_STEP
        
        lidarRangeImage = lidar.getRangeImage()
        setLidarRangeImage = set(lidarRangeImage) # Take elements that are not repeated
        if len(setLidarRangeImage) > 1:
            #printInfo()
            action = leftWallFollower(lidarRangeImage)
        else:
            frontMeasure = 0 
            leftMeasure = 0 
            cornerMeasure = 0
            action = [0, 0]
        
        #♠ Save values to CSV
        frontMeasure = lidarRangeImage[180]
        leftMeasure = lidarRangeImage[90]
        cornerMeasure = lidarRangeImage[135]
            
        # Write CSV
        csv_writer.writerow([frontMeasure, leftMeasure, cornerMeasure, action[0], action[1]])
        
        # Update loop counter
        print("Loop count: ", loopCounter)
        loopCounter = loopCounter + 1
       
  

# Enter here exit cleanup code.
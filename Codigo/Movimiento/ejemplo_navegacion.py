from controller import Robot, Motor, DistanceSensor, Camera, Emitter, GPS
import struct
import numpy as np
import cv2 as cv

timeStep = 32            # Set the time step for the simulation
max_velocity = 6.28      # Set a maximum velocity time constant

robot = Robot()

# Create an object to control the left wheel
wheel_left = robot.getDevice("wheel1 motor")
# Create an object to control the right wheel
wheel_right = robot.getDevice("wheel2 motor")

#[left wheel speed, right wheel speed]
speeds = [max_velocity, max_velocity]

#Create objects for all robot sensors
leftDist = robot.getDevice("leftDist")      # Get robot's left distance sensor
leftDist.enable(timeStep)     # Enable left distance sensor

frontDist = robot.getDevice("frontDist")
frontDist.enable(timeStep)

rightDist = robot.getDevice("rightDist")
rightDist.enable(timeStep)

cam = robot.getDevice("camera")
cam.enable(timeStep)

colorSensor = robot.getDevice("color")
colorSensor.enable(timeStep)

emitter = robot.getDevice("emitter")    # Emitter doesn't need enable

gps = robot.getDevice("gps")
gps.enable(timeStep)

wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))


def turn_right():
    #set left wheel speed
    speeds[0] = 0.6 * max_velocity
    #set right wheel speed
    speeds[1] = -0.2 * max_velocity


def turn_left():
    #set left wheel speed
    speeds[0] = -0.2 * max_velocity
    #set right wheel speed
    speeds[1] = 0.6 * max_velocity


def spin():
    #set left wheel speed
    speeds[0] = 0.6 * max_velocity
    #set right wheel speed
    speeds[1] = -0.6 * max_velocity


def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        # If time elapsed (converted into ms) is greater than value passed in
        if (robot.getTime() - initTime) * 1000.0 > ms:
            break


def getColor():
    img = colorSensor.getImage()    # Grab color sensor camera's image view
    # Return grayness of the only pixel (0-255)
    return colorSensor.imageGetGray(img, colorSensor.getWidth(), 0, 0)


def checkVic(img):
    # Convert img to RGBA format (for OpenCV)
    img = np.frombuffer(img, np.uint8).reshape(
        (cam.getHeight(), cam.getWidth(), 4))
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)  # Grayscale image
    # Inverse threshold image (0-80 -> white; 80-255 -> black)
    img, thresh = cv.threshold(img, 80, 255, cv.THRESH_BINARY_INV)
    # Find all shapes within thresholded image
    contours, hierarchy = cv.findContours(
        thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        x, y, w, h = cv.boundingRect(cnt)   # Find width and height of contour
        contArea = cv.contourArea(cnt)   # Area covered by the shape
        ratio = w / h    # Calculate width to height ratio of contour
        # if the contour area and width to height ratio are within certain ranges
        if contArea > 300 and contArea < 1000 and ratio > 0.65 and ratio < 0.95:
            return True
    return False


def report(victimType):
    # Struct package to be sent to supervisor to report victim/hazard
    # First four bytes store robot's x coordinate
    # Second four bytes store robot's z coordinate
    # Last byte stores type of victim
    #     Victims: H, S, U, T
    #     Hazards: F, P, C, O
    wheel_left.setVelocity(0)   # Stop for 1 second
    wheel_right.setVelocity(0)
    delay(1300)
    # Convert victimType to character for struct.pack
    victimType = bytes(victimType, "utf-8")
    posX = int(gps.getValues()[0] * 100)    # Convert from cm to m
    posZ = int(gps.getValues()[2] * 100)
    message = struct.pack("i i c", posX, posZ, victimType)
    emitter.send(message)
    robot.step(timeStep)


while robot.step(timeStep) != -1:
    speeds[0] = max_velocity
    speeds[1] = max_velocity

    # Check left and right sensor to avoid walls
    # for sensor on the left, either
    if leftDist.getValue() < 0.05:
        turn_right()      # We see a wall on the left, so turn right away from the wall

    if rightDist.getValue() < 0.05:		# for sensor on the right too
        turn_left()

    # for front sensor
    if frontDist.getValue() < 0.05:
        spin()

    # if on black, turn away
    if getColor() < 80:
        spin()
        wheel_left.setVelocity(speeds[0])
        wheel_right.setVelocity(speeds[1])
        delay(600)

    # if sees victim, report it
    if checkVic(cam.getImage()):
        report('T')  # Cannot determine type of victim, so always try 'T' for now

    # Send the speed values we have choosen to the robot
    wheel_left.setVelocity(speeds[0])
    wheel_right.setVelocity(speeds[1])

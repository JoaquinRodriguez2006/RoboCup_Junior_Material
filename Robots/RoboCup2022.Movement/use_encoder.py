from controller import Robot
from controller import Motor
from controller import PositionSensor

robot = Robot()  # Create robot object
timeStep = 32   # timeStep = number of milliseconds between world updates
max_velocity = 6.28

wheel_left = robot.getDevice("wheel1 motor")    # Motor initialization
wheel_right = robot.getDevice("wheel2 motor")
wheel_left.setPosition(float('inf'))
wheel_right.setPosition(float('inf'))

leftEncoder = wheel_left.getPositionSensor()    # Encoder initialization
rightEncoder = wheel_right.getPositionSensor()
leftEncoder.enable(timeStep)
rightEncoder.enable(timeStep)

wheel_left.setVelocity(max_velocity)
wheel_right.setVelocity(max_velocity)

while robot.step(timeStep) != -1:
    print("Left motor has spun " +
          str(leftEncoder.getValue()) + " radians")  # Step 3
    print("Right motor has spun " +
          str(rightEncoder.getValue()) + " radians")
    if leftEncoder.getValue() > 25:   # If left motor has spun more than 20 radians
        break

wheel_left.setVelocity(0.0)
wheel_right.setVelocity(0.0)

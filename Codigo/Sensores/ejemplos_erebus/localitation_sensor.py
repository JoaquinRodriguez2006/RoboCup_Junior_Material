from controller import Robot, GPS
import math

robot = Robot()

gps = robot.getDevice("gps")

timestep = int(robot.getBasicTimeStep())

gps.enable(timestep)

robot.step(timestep)  # Must step once to update sensor reading
startX = gps.getValues()[0]  # Store initial robot position
startY = gps.getValues()[1]
startZ = gps.getValues()[2]

while robot.step(timestep) != -1:

    x = gps.getValues()[0]
    y = gps.getValues()[1]
    z = gps.getValues()[2]

    # Use the distance formula to calculate distance to starting position
    dist = math.sqrt((x-startX)**2 + (z-startZ)**2)

    print(f"Distance in x axis: {round(dist * 100)}" )

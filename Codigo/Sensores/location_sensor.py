from controller import Robot, GPS # Step 1: Import GPS

robot = Robot() 

gps = robot.getDevice("gps") # Step 2: Retrieve the sensor, named "gps", from the robot. Note that the sensor name may differ between robots

timestep = int(robot.getBasicTimeStep())

gps.enable(timestep) # Step 3: Enable the sensor, using the timestep as the update rate

while robot.step(timestep) != -1:

    x = gps.getValues()[0] # Step 4: Use the getValues() function to get the sensor readings
    y = gps.getValues()[1] # Note that the gps returns a list of 3 values for x, y, z, position
    z = gps.getValues()[2]
    
    print("x: " + str(round(x * 100)) +  "\t" + " z: " + str(round(z * 100)))
    print()

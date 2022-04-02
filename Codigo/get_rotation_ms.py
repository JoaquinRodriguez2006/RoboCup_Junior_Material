from controller import Robot, GPS
from controller import Motor
from controller import PositionSensor
from controller import Supervisor
import sys

timeStep = 32
supervisor = Supervisor()

orientation = supervisor.getSelf().getOrientation()
print(orientation)

"""
robot_node = supervisor.getFromDef("MY_ROBOT")
if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
trans_orientation = robot_node.getOrientation("translation")

while supervisor.step(timeStep) != -1:
    # this is done repeatedly
    values = trans_orientation.getSFRotation()
    print("MY_ROBOT's Orientation is: %g %g %g" % (values[0], values[1], values[2]))
"""
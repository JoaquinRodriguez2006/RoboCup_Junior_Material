from controller import Robot, GPS
from controller import Motor
from controller import PositionSensor
from controller import Supervisor
import sys

timeStep = 32
supervisor = Supervisor()

robot_node = supervisor.getFromDef("MY_ROBOT")
if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("translation")

while supervisor.step(timeStep) != -1:
    # this is done repeatedly
    values = trans_field.getSFVec3f()
    print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))
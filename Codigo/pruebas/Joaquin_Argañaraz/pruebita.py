from controller import Robot
from controller import Motor
from controller import PositionSensor
from controller import Robot, DistanceSensor, GPS, Camera, Receiver, Emitter
import cv2
import numpy as np
import math
import time

robot = Robot()
timeStep = 32
tile_size = 0.12
speed = 6.28
media_baldoza = 0.06
estado = 1
start = 0
r = 0
g = 0
b = 0
Dist = 0
# start = robot.getTime()

# Camera initialization
camera = robot.getDevice("camera1")
camera.enable(timeStep)

# camera 3 = Izquierda
# camera 2 = Derecha
# camera 1 = Medio


while robot.step(timeStep) != -1:
    camera.getImage()
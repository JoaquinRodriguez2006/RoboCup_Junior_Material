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
global r
global g
global b
global Dist
r = 0
g = 0
b = 0
Dist = 0
# start = robot.getTime()

# Camera initialization
camera = robot.getDevice("camera3")
camera.enable(timeStep)

# Colour sensor initialization
colour_sensor = robot.getDevice("colour_sensor")
colour_sensor.enable(timeStep)

# Distance sensor initialization
distancia_sensor1 = robot.getDevice("distance sensor1")
distancia_sensor1.enable(timeStep)
distancia_sensorDer = robot.getDevice("distance sensor2")
distancia_sensorDer.enable(timeStep)
distancia_sensorIzq = robot.getDevice("distance sensor3")
distancia_sensorIzq.enable(timeStep)


while robot.step(timeStep) != -1:
    print(distancia_sensor1.getValue())
    print(distancia_sensorDer.getValue())
    print(distancia_sensorIzq.getValue())
    print(".")
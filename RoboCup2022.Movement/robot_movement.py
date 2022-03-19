from tracemalloc import start
from controller import Robot
from controller import Motor
from controller import PositionSensor
from controller import Robot, DistanceSensor, GPS, Camera, Receiver, Emitter
import math
import time

robot = Robot() 
timeStep = 32
tile_size = 0.12
speed = 6.28

start = robot.getTime()

# Distance sensor initialization
distancia_sensor1 = robot.getDevice("distance sensor1")
distancia_sensor1.enable(timeStep)

# Motor initialization
ruedaIzquierda = robot.getDevice("wheel1 motor")
ruedaDerecha = robot.getDevice("wheel2 motor")
ruedaIzquierda.setPosition(float('inf'))
ruedaDerecha.setPosition(float('inf'))

# Functions
def avanzar(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(vel)

def girar(vel):
    ruedaIzquierda.setVelocity(-vel)
    ruedaDerecha.setVelocity(vel)

def girar_izq(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(-vel)

while robot.step(timeStep) != -1:
    avanzar(6.28)
    for i in range(2):
        if robot.getTime() >= start + 4.8:
            girar(6.28)
            if robot.getTime() >= start + 5.15:
                avanzar(6.28)
                if robot.getTime() >= start + 9.91:
                    girar(6.28)
                    if robot.getTime() >= start + 10.26:
                        avanzar(6.28)
                        if robot.getTime() >= start + 15.2:
                            girar(6.28)
                            if robot.getTime() >= start + 16:
                                avanzar(6.28)
                                if robot.getTime() >= start + 16.3:
                                    avanzar(0)

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
media_baldoza = 0.06
estado = 1
start = 0
# start = robot.getTime()

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

def girito(vel):
    ruedaIzquierda.setVelocity(0.7)
    ruedaDerecha.setVelocity(vel)

def girar_izq(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(-vel)

while robot.step(timeStep) != -1:
    if estado == 0:
        avanzar(6.28)
        if distancia_sensor1.getValue() <= 0.15:
            start = robot.getTime()
            estado = 1
    
    # Giro 180 grados
    if estado == 1:
        girito(6.28)
        if robot.getTime() >= start + 0.79:
            avanzar(0)
            estado = 0
    
    
    
    
    
    # print(robot.getTime())
    # if estado == 0:
    #     avanzar(6.28)
    #     if distancia_sensor1.getValue() <= media_baldoza:
    #         avanzar(0)
    #         start = robot.getTime()
    #         print(robot.getTime())
    #         estado = 1

    # if estado == 1:
    #     girar(5.96)
    #     if robot.getTime() >= start + 0.36:
    #         avanzar(0)
    #         estado = 0



    # if estado == 1:
    #     start = robot.getTime()
    #     estado = 0
    # if estado == 0:
    #     girar_izq(6.28)
    #     if robot.getTime() >= start + 0.352:
    #         ruedaIzquierda.setVelocity(0)
    #         ruedaDerecha.setVelocity(0)
    #         print("no")
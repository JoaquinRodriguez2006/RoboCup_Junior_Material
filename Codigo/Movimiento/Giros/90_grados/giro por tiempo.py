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
    
    # Robot avanza  hasta que encuentra un obstaculo
    if estado == 0:
        avanzar(6.28)
        if distancia_sensor1.getValue() <= media_baldoza:
            avanzar(0)
            start = robot.getTime()
            estado = 1
    # Robot gira 90 grados
    if estado == 1:
        girar(5.96)
        if robot.getTime() >= start + 0.36:
            avanzar(0)
            estado = 0

from controller import Robot
from controller import Motor
from controller import PositionSensor
from controller import Robot, DistanceSensor, GPS, Camera, Receiver, Emitter
from controller import PositionSensor
import math
import time

robot = Robot() 
timeStep = 32
tile_size = 0.12
speed = 6.28
media_baldoza = 0.06
estado = 0
start = 0

# Motor initialization
ruedaIzquierda = robot.getDevice("wheel1 motor")
ruedaDerecha = robot.getDevice("wheel2 motor")
ruedaIzquierda.setPosition(float('inf'))
ruedaDerecha.setPosition(float('inf'))

rIzq_encoder = ruedaIzquierda.getPositionSensor()
rDer_encoder = ruedaDerecha.getPositionSensor()

rIzq_encoder.enable(timeStep)
rDer_encoder.enable(timeStep)

def avanzar_baldoza():
    estado = 0
    start = 0
    if estado == 0:
        start = rDer_encoder.getValue()
        estado = 1
        
    if estado == 1:
        ruedaIzquierda.setVelocity(6.28)
        ruedaDerecha.setVelocity(6.28)
        if rDer_encoder.getValue() >= start + 5.9:
            ruedaIzquierda.setVelocity(0)
            ruedaDerecha.setVelocity(0)

def avanzar_interseccion():
    estado = 0
    start = 0
    if estado == 0:
        start = rDer_encoder.getValue()
        estado = 1
        
    if estado == 1:
        ruedaIzquierda.setVelocity(6.28)
        ruedaDerecha.setVelocity(6.28)
        if rDer_encoder.getValue() >= start + 2.9:
            ruedaIzquierda.setVelocity(0)
            ruedaDerecha.setVelocity(0)


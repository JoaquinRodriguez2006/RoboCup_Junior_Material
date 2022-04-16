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


def avanzar_baldosa(t_inicial,status=1):
    global estado
    global start
    avanzar(6.28)
    if rDer_encoder.getValue() >= t_inicial + 5.9:
            estado = status

def avanzar_media_baldosa(t_inicial,status=1):
    global estado
    global start
    avanzar(6.28)
    if rDer_encoder.getValue() >= t_inicial + 2.9:
            estado = status

while robot.step(timeStep) != -1:
    if estado == 0:
        start = rDer_encoder.getValue()
        estado = 1
        
    if estado == 1:
        avanzar_media_baldosa(start)
        avanzar_media_baldosa(start+3)
        avanzar_media_baldosa(start+6)
        avanzar_media_baldosa(start+9)
        avanzar_media_baldosa(start+12)
        avanzar_media_baldosa(start+15,"parar")

    if estado == "parar":
        avanzar(0)
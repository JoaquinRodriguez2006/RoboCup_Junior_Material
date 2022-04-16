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


gyro = robot.getDevice("gyro")
gyro.enable(timeStep)


def rotar(angulo):
    global angulo_actual
    global tiempo_anterior
    #  iniciar_rotacion
    girar(0.5)
    # Mientras no llego al angulo solicitado sigo girando
    if (abs(angulo - angulo_actual) > 1):
        tiempo_actual = robot.getTime()
        # print("Inicio rotacion angulo", angulo, "Angulo actual:",angulo_actual)
        tiempo_transcurrido = tiempo_actual - tiempo_anterior  # tiempo que paso en cada timestep
        # rad/seg * mseg * 1000
        radsIntimestep = abs(gyro.getValues()[1]) * tiempo_transcurrido
        degsIntimestep = radsIntimestep * 180 / math.pi
        print("rads: " + str(radsIntimestep) + " | degs: " + str(degsIntimestep))
        angulo_actual += degsIntimestep
        # Si se pasa de 360 grados se ajusta la rotacion empezando desde 0 grados
        angulo_actual = angulo_actual % 360
        # Si es mas bajo que 0 grados, le resta ese valor a 360
        if angulo_actual < 0:
            angulo_actual += 360
        tiempo_anterior = tiempo_actual
        print("Angulo actual:", angulo_actual)
        return False
    print("Rotacion finalizada.")
    angulo_actual = 0
    return True

estado = 0
angulo_actual = 0
tiempo_anterior = robot.getTime()
while robot.step(timeStep) != -1:
    
    if rotar(45) and estado == 0:
        print("Estoy girando")
        start = rDer_encoder.getValue()
        estado = 1
        # avanzar(0)

    if estado == 1:
        avanzar(6.28)
        if rDer_encoder.getValue() >= start + 4.1:
            avanzar(0)
            break
avanzar(0)

      

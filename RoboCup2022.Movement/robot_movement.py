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

# Gyroscope
gyro = robot.getDevice("gyro")
gyro.enable(timeStep)

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
        radsIntimestep = abs(gyro.getValues()[1]) * tiempo_transcurrido   # rad/seg * mseg * 1000
        degsIntimestep = radsIntimestep * 180 / math.pi
        # print("rads: " + str(radsIntimestep) + " | degs: " + str(degsIntimestep))
        angulo_actual += degsIntimestep
        # Si se pasa de 360 grados se ajusta la rotacion empezando desde 0 grados
        angulo_actual = angulo_actual % 360
        # Si es mas bajo que 0 grados, le resta ese valor a 360
        if angulo_actual < 0:
            angulo_actual += 360
        tiempo_anterior = tiempo_actual
        return False
    print("Rotacion finalizada.")
    angulo_actual = 0
    return True

while robot.step(timeStep) != -1:
    avanzar(6.28)
    for i in range(2):
        if robot.getTime() >= start + 4.8:
            girar(6.28)
            if robot.getTime() >= start + 5.15:
                avanzar(6.28)
                if robot.getTime() >= start + 9.91:
                    girar(6.28)
                    if robot.getTime() >= start + 10.265:
                        avanzar(6.28)
                        if robot.getTime() >= start + 15.2:
                            girar(6.28)
                            if robot.getTime() >= start + 16:
                                avanzar(6.28)
                                if robot.getTime() >= start + 16.3:
                                    avanzar(0)

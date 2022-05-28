from controller import Robot
from controller import Motor
from controller import PositionSensor
from controller import Robot, DistanceSensor, GPS, Camera, Receiver, Emitter
import cv2
import numpy as np
import math
import time

# MISCELANOUS THINGS
robot = Robot()
timeStep = 32
tile_size = 0.12
speed = 6.28
media_baldoza = 0.06
start = 0
global r
global g
global b
r = 0
g = 0
b = 0

# INITIALIZATIONS
# Camera initialization
camera = robot.getDevice("camera3")
camera.enable(timeStep)

# Colour sensor initialization
colour_sensor = robot.getDevice("colour_sensor")
colour_sensor.enable(timeStep)

# Distance sensor initialization
distancia_sensor1 = robot.getDevice("distance sensor1") # FRENTE
distancia_sensor1.enable(timeStep)
distancia_sensor2 = robot.getDevice("distance sensor2") # DERECHA
distancia_sensor2.enable(timeStep)
distancia_sensor3 = robot.getDevice("distance sensor3") # IZQUIERDA
distancia_sensor3.enable(timeStep)

# Motor initialization
ruedaIzquierda = robot.getDevice("wheel1 motor")
ruedaDerecha = robot.getDevice("wheel2 motor")
ruedaIzquierda.setPosition(float('inf'))
ruedaDerecha.setPosition(float('inf'))

# Encoder initialization
rIzq_encoder = ruedaIzquierda.getPositionSensor()
rDer_encoder = ruedaDerecha.getPositionSensor()
rIzq_encoder.enable(timeStep)
rDer_encoder.enable(timeStep)

# Gps initialization
gps = robot.getDevice("gps")
gps.enable(timeStep)
robot.step(timeStep) # Actualizo los valores de los sensores
startX = gps.getValues()[0] # Cargo La posicion inicial
startY = gps.getValues()[1]
startZ = gps.getValues()[2]
x = 0
y = 0
z = 0

# FUNCTIONS
def leer_sensores():
    global r
    global g
    global b
    # Color sensor
    color = colour_sensor.getImage()
    r = colour_sensor.imageGetRed(color, 1, 0, 0)
    g = colour_sensor.imageGetGreen(color, 1, 0, 0)
    b = colour_sensor.imageGetBlue(color, 1, 0, 0)
    # Distance Sensor
    distance = distancia_sensor1.getValue()

def avanzar(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(vel)
    
def retroceder(vel):
    ruedaIzquierda.setVelocity(-vel)
    ruedaDerecha.setVelocity(-vel)

def girar_der(vel):
    ruedaIzquierda.setVelocity(-vel)
    ruedaDerecha.setVelocity(vel)

def girar_izq(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(-vel)

gyro = robot.getDevice("gyro")
gyro.enable(timeStep)


def rotar(angulo):
    global angulo_actual
    global tiempo_anterior
    #  iniciar_rotacion
    if angulo > 0:
        girar_der(0.5)
    else:
        girar_izq(0.5)
    # Mientras no llego al angulo solicitado sigo girando
    if (abs(abs(angulo) - angulo_actual) > 1):
        tiempo_actual = robot.getTime()
        # print("Inicio rotacion angulo", angulo, "Angulo actual:",angulo_actual)
        tiempo_transcurrido = tiempo_actual - \
            tiempo_anterior  # tiempo que paso en cada timestep
        # rad/seg * mseg * 1000
        radsIntimestep = abs(gyro.getValues()[1]) * tiempo_transcurrido
        degsIntimestep = radsIntimestep * 180 / math.pi
        # print("rads: " + str(radsIntimestep) +
        #     " | degs: " + str(degsIntimestep))
        angulo_actual += degsIntimestep
        # Si se pasa de 360 grados se ajusta la rotacion empezando desde 0 grados
        angulo_actual = angulo_actual % 360
        # Si es mas bajo que 0 grados, le resta ese valor a 360
        if angulo_actual < 0:
            angulo_actual += 360
        tiempo_anterior = tiempo_actual
        # print("Angulo actual:", angulo_actual)
        return False
    print("Rotacion finalizada.")
    angulo_actual = 0
    return True

def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        print("delay")
        if (robot.getTime() - initTime) * 1000.0 > ms: # If time elapsed (converted into ms) is greater than value passed in
            avanzar(0)
            break

def rotar_enclavado(angulo):
    while robot.step(timeStep) != -1:
        leer_sensores()
        # print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
        if rotar(angulo) == True: # If time elapsed (converted into ms) is greater than value passed in
            avanzar(0)
            if distancia_sensor1.getValue() < 0.06:
                rotar_enclavado(180)
            break
      
def avance(tipo_avance):
    start = rDer_encoder.getValue()
    velocidad = 0
    avance = 0
    if tipo_avance == "medio":
        velocidad = 6.28
        avance = 2.9
    elif tipo_avance == "largo":
        avance = 5.9
        velocidad = 5.96
    elif tipo_avance == "esquina":
        avance = 4.1
        velocidad = 6.28
    while robot.step(timeStep) != -1:
        avanzar(velocidad)
        leer_sensores()
        # print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
        if rDer_encoder.getValue() >= start + avance:
            avanzar(0)
            break  

def retroceso(tipo_retroceso):
    start = rDer_encoder.getValue()
    velocidad = 0
    retroceso = 0
    if tipo_retroceso == "medio":
        velocidad = 6.28
        retroceso = 2.9
    elif tipo_retroceso == "largo":
        retroceso = 5.9
        velocidad = 5.96
    elif tipo_retroceso == "esquina":
        retroceso = 4.1
        velocidad = 6.28
    elif tipo_retroceso == "poquito":
        retroceso = 1.9
        velocidad = 6.28
    while robot.step(timeStep) != -1:
        retroceder(velocidad)
        leer_sensores()
        # print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
        if start - retroceso >= rDer_encoder.getValue():
            avanzar(0)
            break

def localizacion():
    while robot.step(timeStep) != -1:
        x = gps.getValues()[0]
        y = gps.getValues()[1]
        z = gps.getValues()[2]
        distancia = math.sqrt((x-startX)**2 + (z-startZ)**2)
        print(f"Distance in Axis {round(distancia * 100)}")
        # print("X =" + x  + "||  Y =" + y + "||  Z =" + z)
        solución_LaberintoPruebas1()

def solución_LaberintoPruebas1():
    print(distancia_sensor1.getValue())
    if (distancia_sensor1.getValue() < 0.06) and (distancia_sensor2.getValue() < 0.06):
        print("Izquierda liberada")
        rotar_enclavado(-90)
    if (distancia_sensor1.getValue() < 0.06) and (distancia_sensor3.getValue() < 0.06):
        print("Derecha Liberada")
        rotar_enclavado(90)
    if (distancia_sensor1.getValue() < 0.06) and (distancia_sensor2.getValue() < 0.09) and (distancia_sensor3.getValue() < 0.9):
        print("CALLEJÓN SIN SALIDA")
        rotar_enclavado(180)
    else:
        avanzar(4)

# MAIN CODE
angulo_actual = 0
tiempo_anterior = robot.getTime()

while robot.step(timeStep) != -1:
    localizacion()
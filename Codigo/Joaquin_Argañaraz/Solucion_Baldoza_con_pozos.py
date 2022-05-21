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
r = 0
g = 0
b = 0

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

def leer_sensores():
    global r
    global g
    global b
    # Color sensor
    color = colour_sensor.getImage()
    r = colour_sensor.imageGetRed(color, 1, 0, 0)
    g = colour_sensor.imageGetGreen(color, 1, 0, 0)
    b = colour_sensor.imageGetBlue(color, 1, 0, 0)
    # azul: r=65 g=65 b=252
    # rojo: r=252 g=65 b=65
    # print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
"""
    # Camara
    image = camera.getImage()
    imagen = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    frame = cv2.cvtColor(imagen, cv2.COLOR_BGRA2BGR)
    
    cv2.imshow("frame", frame)
    
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Grayscale
    cv2.imshow("grayScale", frame)
    
    cv2.threshold(frame, 80, 255, cv2.THRESH_BINARY) # Threshold
    cv2.imshow("thresh", frame)
    
    cv2.waitKey(1)

    # Sensor de Distancia
    print("Distancia: " + str(distancia_sensor1.getValue()))
    """


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
            break
      
def avance(tipo_avance):
    start = rDer_encoder.getValue()
    velocidad = 0
    avance = 0
    if tipo_avance == "medio" and tipo_pizza() != "swamp":
        velocidad = 3
        avance = 2.9
    if tipo_avance == "medio" and tipo_pizza() == "swamp":
        velocidad = 2
        avance = 2.9
    elif tipo_avance == "largo" and tipo_pizza() != "swamp":
        avance = 5.9
        velocidad = 5.96
    elif tipo_avance == "esquina" and tipo_pizza() != "swamp":
        avance = 4.1
        velocidad = 6.28
    while robot.step(timeStep) != -1:
        avanzar(velocidad)
        leer_sensores()
        # print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
        if tipo_pizza() == "hole":
            retroceso("poquito")
            rotar_enclavado(90)
            break
        if rDer_encoder.getValue() >= start + avance:
            avanzar(0)
            break  

def retroceso(tipo_retroceso):
    start = rDer_encoder.getValue()
    velocidad = 0
    retroceso = 0
    if tipo_retroceso == "medio" and tipo_pizza() == "swamp":
        velocidad = 2
        retroceso = 3.6
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

def tipo_pizza():
    
    # print("valores(1): r:" + str(r) + " , g:" + str(g) + " , b:" + str(b))
    if 255 >= r >= 248 and 69 >= b >= 61 and 69 >= g >= 61:
        print("(Red)pasaje zona 3 a 1")
    elif 151 >= r >= 143 and 232 >= b >= 224 and 69 >= g >= 61:
        print("(Vaiolet)pasaje zona 2 a 3")
    elif 69 >= r >= 61 and 255 >= b >= 248 and 69 >= g >= 61:
        print("(Blue)pasaje zona 1 a 2")
    elif 216 >= r >= 208 and 108 >= b >= 100 and 183 >= g >= 174:
        print("Entered swamp")
        return "swamp"
    elif 255 >= r >= 245 and 255 >= b >= 245 and 255 >= g >= 245:
        print("Found Checkpoint")
    elif 235 >= r >= 229 and 235 >= b >= 229 and 235 >= g >= 229:
        print("Azulejo normal")
    elif 46 >= r >= 38 and 46 >= b >= 38 and 46 >= g >= 38:
        print("un agujero negro we")
        return "hole"
        # retroceso("medio")
        # rotar_enclavado(180)
    else:
        return "prueba"




angulo_actual = 0
tiempo_anterior = robot.getTime()
contador = 0
avanzar(0)
while robot.step(timeStep) != -1:
    tipo_pizza()
    avance("medio")
    print("valores(1): r:" + str(r) + " , g:" + str(g) + " , b:" + str(b))

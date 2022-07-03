from pkg_resources import to_filename
from controller import Robot
from controller import Motor
from controller import PositionSensor
from controller import Robot, DistanceSensor, GPS, Camera, Receiver, Emitter
import math
from time import sleep
import cv2
import numpy as np

robot = Robot()
timeStep = 1
tile_size = 0.12
speed = 6.28
media_baldoza = 0.06
estado = 1
start = 0

camera = robot.getDevice("camera1")
camera2 = robot.getDevice("camera2")
camera3 = robot.getDevice("camera3")
camera.enable(timeStep)
camera2.enable(timeStep)
camera3.enable(timeStep)
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
         #print("rads: " + str(radsIntimestep) +
         #   " | degs: " + str(degsIntimestep))
        angulo_actual += degsIntimestep
        # Si se pasa de 360 grados se ajusta la rotacion empezando desde 0 grados
        angulo_actual = angulo_actual % 360
        # Si es mas bajo que 0 grados, le resta ese valor a 360
        if angulo_actual < 0:
            angulo_actual += 360
        tiempo_anterior = tiempo_actual
        #print("Angulo actual:", angulo_actual)
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
        if rotar(angulo) == True: # If time elapsed (converted into ms) is greater than value passed in
            avanzar(0)
            captar()
            break
      
def avance(tipo_avance):
    start = rDer_encoder.getValue()
    velocidad = 0
    avance = 0
    if tipo_avance == "medio":
        avance = 5.9
        velocidad = 5.96
        captar()
    elif tipo_avance == "S_largo":
        avance = 30
        velocidad = 5.96
    elif tipo_avance == "esquina":
        avance = 19
        velocidad = 8
    while robot.step(timeStep) != -1:
        avanzar(velocidad)
        if rDer_encoder.getValue() >= start + avance:
            avanzar(0)
            break  

def retroceso(tipo_retroceso):
    start = rDer_encoder.getValue()
    velocidad = 0
    retroceso = 0
    if tipo_retroceso == "medio":
        retroceso = 5.9
        velocidad = 5.96
        captar()
    elif tipo_retroceso == "esquina":
        retroceso = 19
        velocidad = 6.28
    while robot.step(timeStep) != -1:
        retroceder(velocidad)
        if start - retroceso >= rDer_encoder.getValue():
            avanzar(0)
            break      
        
angulo_actual = 0
tiempo_anterior = robot.getTime()
contador = 0
estado = 0
def captar():
    contador = 0
    while robot.step(timeStep) != -1:
      avanzar(0)
      img = camera.getImage()
      img2 = camera2.getImage()
      img3 = camera3.getImage()
      img = np.array(np.frombuffer(img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
      img2 = np.array(np.frombuffer(img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
      img3 = np.array(np.frombuffer(img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
      cv2.imwrite(f"C:/Users/iita01/Desktop/RoboCup_Junior_Material/Codigo/pruebas/Jose_Gonzalez/imagenes_webots/world3_movimiento/imagen_webot_{contador}.png", img)
      cv2.imwrite(f"C:/Users/iita01/Desktop/RoboCup_Junior_Material/Codigo/pruebas/Jose_Gonzalez/imagenes_webots/world3_movimiento/imagen_webot_{contador}.png", img2)
      cv2.imwrite(f"C:/Users/iita01/Desktop/RoboCup_Junior_Material/Codigo/pruebas/Jose_Gonzalez/imagenes_webots/world3_movimiento/imagen_webot_{contador}.png", img3)
      print("Tomo la imagen")
      delay(1000)
      cv2.imshow("Image", img)
      cv2.imshow("Image", img2)
      cv2.imshow("Image", img3)
      cv2.waitKey(1)
      contador += 1
      break


while robot.step(timeStep) != -1:
    rotar_enclavado(180)
    avance("medio")
    rotar_enclavado(180)
    avance("medio")
    avance("medio")
    rotar_enclavado(90)
    avance("medio")
    rotar_enclavado(90)
    avance("medio")
    rotar_enclavado(-90)
    avance("medio")
    rotar_enclavado(90)
    avance("medio")
    rotar_enclavado()
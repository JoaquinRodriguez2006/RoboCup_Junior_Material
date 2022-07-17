from time import sleep
from controller import Robot
import cv2
import numpy as np

robot = Robot()
timeStep = 32
camera_derecha = robot.getDevice("camera2")
camera_derecha.enable(timeStep)
camera_centro = robot.getDevice("camera1")
camera_centro.enable(timeStep)
camera_izquierda = robot.getDevice("camera3")
camera_izquierda.enable(timeStep)
contador = 0
ruedaIzquierda = robot.getDevice("wheel1 motor")
ruedaDerecha = robot.getDevice("wheel2 motor")
ruedaIzquierda.setPosition(float('inf'))
ruedaDerecha.setPosition(float('inf'))

def girar_izq(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(-vel)

def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        print("delay")
        if (robot.getTime() - initTime) * 1000.0 > ms: # If time elapsed (converted into ms) is greater than value passed in
            avanzar(0)
            break

def avanzar(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(vel)

def girito():
    inicio = robot.getTime()
    while robot.step(timeStep) != -1:
        girar_izq(-5.96)
        if robot.getTime() >= inicio + 0.39:
            avanzar(0)
            break

estado = 1
time_inicio = robot.getTime()
while robot.step(timeStep) != -1:
    if estado == 1:
        girito()
        estado = 0
    avanzar(4)
    img_izq = camera_izquierda.getImage()
    img_izq = np.array(np.frombuffer(img_izq, np.uint8).reshape((camera_izquierda.getHeight(), camera_izquierda.getWidth(), 4)))
    cv2.imwrite(f"C:/Users/JOAKOL/Desktop/robotica/Imagenes/camara_izquierda/imagen_izquierda_webot_{contador}.png", img_izq)
    print("Tomo la imagen")
    cv2.imshow("Image", img_izq)

    img_cen = camera_centro.getImage()
    img_cen = np.array(np.frombuffer(img_cen, np.uint8).reshape((camera_centro.getHeight(), camera_centro.getWidth(), 4)))
    cv2.imwrite(f"C:/Users/JOAKOL/Desktop/robotica/Imagenes/camara_centro/imagen_centro_webot_{contador}.png", img_cen)
    print("Tomo la imagen")
    cv2.imshow("Image2", img_cen)

    img_der = camera_derecha.getImage()
    img_der = np.array(np.frombuffer(img_der, np.uint8).reshape((camera_derecha.getHeight(), camera_derecha.getWidth(), 4)))
    cv2.imwrite(f"C:/Users/JOAKOL/Desktop/robotica/Imagenes/camara_derecha/imagen_derecha_webot_{contador}.png", img_der)
    print("Tomo la imagen")
    cv2.imshow("Image3", img_der)

    cv2.waitKey(1)
    contador += 1
    if(robot.getTime() - time_inicio > 10):
        print("Fin de Capturas")
        break
from time import sleep
from controller import Robot
import cv2
import numpy as np

robot = Robot()
timeStep = 32
camera = robot.getDevice("camera1")
camera.enable(timeStep)
contador = 0
ruedaIzquierda = robot.getDevice("wheel1 motor")
ruedaDerecha = robot.getDevice("wheel2 motor")
ruedaIzquierda.setPosition(float('inf'))
ruedaDerecha.setPosition(float('inf'))

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

while robot.step(timeStep) != -1:
    avanzar(0)
    img = camera.getImage()
    img = np.array(np.frombuffer(img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
    cv2.imwrite(f"C:/Users/iita01/Desktop/RoboCup_Junior_Material/Codigo/pruebas/Jose_Gonzalez/imagenes_world3/imagen_webot_{contador}.png", img)
    print("Tomo la imagen")
    delay(3000)
    cv2.imshow("Image", img)
    cv2.waitKey(1)
    contador += 1
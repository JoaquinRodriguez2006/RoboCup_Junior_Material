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
        # If time elapsed (converted into ms) is greater than value passed in
        if (robot.getTime() - initTime) * 1000.0 > ms:
            avanzar(0)
            break


def avanzar(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(vel)


contador = 0
time_inicio = robot.getTime()
while robot.step(timeStep) != -1:
    if contador == 0:
        video = cv2.VideoWriter(
        'video.avi', -1, 1, (64, 40))
        contador = 1
    avanzar(0)
    img = camera.getImage()
    img = np.array(np.frombuffer(img, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)))
    print("Tomo la imagen y la subo al video")
    video.write(img)
    if(robot.getTime() - time_inicio > 10):
        video.release()
        break
    
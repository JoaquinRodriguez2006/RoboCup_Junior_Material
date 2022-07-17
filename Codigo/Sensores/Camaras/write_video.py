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
video = cv2.VideoWriter(
    'db2.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 31, (64, 40))
while robot.step(timeStep) != -1:
    avanzar(0)
    img = camera.getImage()
    img = np.array(np.frombuffer(img, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)))
    cv2.imwrite("C:/Users/enzzo/OneDrive/Documentos/Github/Sabado-IITA-robotica_2022/RoboCup_Junior_Material/Codigo/Sensores/Camaras/imagenes_camara_izquierda/imagen_webot_0.png", img)
    print("Tomo la imagen y la subo al video")
    path = "C:/Users/enzzo/OneDrive/Documentos/Github/Sabado-IITA-robotica_2022/RoboCup_Junior_Material/Codigo/Sensores/Camaras/imagenes_camara_izquierda/imagen_webot_0.png"
    img = cv2.imread(path)
    video.write(img)
    if(robot.getTime() - time_inicio > 10):
        video.release()
        print("Fin de capturas")
        break
    
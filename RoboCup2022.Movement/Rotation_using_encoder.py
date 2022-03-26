from controller import Robot, GPS
from controller import Motor
from controller import PositionSensor
import time

robot = Robot() # Create robot object
timeStep = 32   # timeStep = numero de milisegundos entre actualizaciones mundiales (del mundo)
noventaGrados = 2.3

# Wheels initialization
ruedaIzquierda = robot.getDevice("wheel1 motor")    # Motor initialization
ruedaDerecha = robot.getDevice("wheel2 motor")
ruedaIzquierda.setPosition(float('inf'))
ruedaDerecha.setPosition(float('inf'))

# Encoder initialization
encoderIzquierdo = ruedaIzquierda.getPositionSensor()    # Encoder initialization
encoderDerecho = ruedaDerecha.getPositionSensor()
encoderIzquierdo.enable(timeStep)
encoderDerecho.enable(timeStep)


#Create your functions here
def avanzar(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(vel)

def girar(vel):
    ruedaIzquierda.setVelocity(-vel)
    ruedaDerecha.setVelocity(vel)

while robot.step(timeStep) != -1:
    girar(5.96)
    ruedaDerecha.setPosition(float(noventaGrados))
    print("Diferencia del encoder:", encoderDerecho.getValue() - noventaGrados  )
    if(abs(encoderDerecho.getValue() - noventaGrados) < 0.01):
        print("Detenerse")
        avanzar(0)
        break


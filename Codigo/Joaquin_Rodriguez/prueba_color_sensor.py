from controller import Robot
from controller import Motor
from controller import Robot, GPS
import time
import math

robot = Robot() # Create robot object
timeStep = 32   # timeStep = numero de milisegundos entre actualizaciones mundiales (del mundo)
tile_size = 0.12 # Tamaño de casilla
angulo_actual = 0
tiempo_anterior = 0
media_baldoza = 0.06
speed = 6.28
global start
global finalLetter

# Distance sensor initialization
distancia_sensor1 = robot.getDevice("distance sensor1")
distancia_sensor1.enable(timeStep)
distancia_sensorDer = robot.getDevice("distance sensor2")
distancia_sensorDer.enable(timeStep)
distancia_sensorIzq = robot.getDevice("distance sensor2")
distancia_sensorIzq.enable(timeStep)
maxima_distancia = 0.4
valor_sensor_distancia_Izq = distancia_sensorIzq.getValue() * 1000

# Motor initialization
ruedaIzquierda = robot.getDevice("wheel1 motor")
ruedaDerecha = robot.getDevice("wheel2 motor")
ruedaIzquierda.setPosition(float('inf'))
ruedaDerecha.setPosition(float('inf'))

# Gyroscope initialization
gyro = robot.getDevice("gyro")
gyro.enable(timeStep)

#Gps initialization
gps = robot.getDevice("gps")
gps.enable(timeStep)
robot.step(timeStep) # Actualizo los valores de los sensores
startX = gps.getValues()[0] # Cargo La posicion inicial
startY = gps.getValues()[2]
offset_xy = [0, 2]
x = 0
y = 0
global lista_victim
lista_victim = []

# Color sensor initialization
colorSensor = robot.getDevice("colour_sensor")
colorSensor.enable(timeStep)

# Util Functions
def avanzar(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(vel)

def girar(vel):
    ruedaIzquierda.setVelocity(vel//3)
    ruedaDerecha.setVelocity(vel)

def rotar(angulo):
    global angulo_actual
    global tiempo_anterior
    #  iniciar_rotacion
    girar(0.8)
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

def type_floor():
    image = colorSensor.getImage()
    r = colorSensor.imageGetRed(image, 1, 0, 0)
    # print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
    if r == 212 :
        return 'arena'
    if r >= 242:
        return 'checkpoint'
    if r == 233 :
        return 'common'
    if r <= 110:
        return 'pozo'

# Main Code
start = robot.getTime()
while robot.step(timeStep) != -1:
# TYPES OF FLOOR DETERMINE EACH STATE
    image = colorSensor.getImage()
    r = colorSensor.imageGetRed(image, 1, 0, 0)
    floor = type_floor()
    if floor == 'common':
        print("Comun")
        avanzar(3)
    
    if floor == 'arena':
        print("Arena")
        avanzar(2)
        rotar(90)
    
    if floor == 'checkpoint':
        print("Checkpoint")
        avanzar(2)
        tiempo_anterior = robot.getTime()
        if robot.getTime() >= start + 14:
            if rotar(90):
                break
        
    if floor == 'pozo':
        print('Pozo')
        avanzar(0)


"""if r >= 212 :
        return 'arena'
    if r >= 242:
        return 'checkpoint'
    if r < 240 and r >= 233 :
        return 'common'"""
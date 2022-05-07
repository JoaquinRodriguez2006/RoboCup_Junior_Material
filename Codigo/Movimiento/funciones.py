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
media_baldoza = 0.06
estado = 1
start = 0
# start = robot.getTime()

# Distance sensor initialization
distancia_sensor1 = robot.getDevice("distance sensor1")
distancia_sensor1.enable(timeStep)

# Step 2: Retrieve the sensor, named "gps", from the robot. Note that the sensor name may differ between robots
gps = robot.getDevice("gps")
gps.enable(timeStep)

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
        print("rads: " + str(radsIntimestep) +
            " | degs: " + str(degsIntimestep))
        angulo_actual += degsIntimestep
        # Si se pasa de 360 grados se ajusta la rotacion empezando desde 0 grados
        angulo_actual = angulo_actual % 360
        # Si es mas bajo que 0 grados, le resta ese valor a 360
        if angulo_actual < 0:
            angulo_actual += 360
        tiempo_anterior = tiempo_actual
        print("Angulo actual:", angulo_actual)
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
            break
      
def avance(tipo_avance):
    start = rDer_encoder.getValue()
    velocidad = 0
    avance = 0
    if tipo_avance == "recto":
        velocidad = 6.28
        avance = 2.9
    elif tipo_avance == "esquina":
        avance = 4.1
        velocidad = 6.28
    while robot.step(timeStep) != -1:
        # Sensor de Distancia
        print("El valor del sensor distancia es: " + str(distancia_sensor1.getValue()))
        #Sensor de GPS
        # Step 4: Use the getValues() function to get the sensor readings
        # Note that the gps returns a list of 3 values for x, y, z, position
        x = gps.getValues()[0]
        y = gps.getValues()[1]
        z = gps.getValues()[2]
        print("x: " + str(round(x * 100)) + "\t" + " z: " + str(round(z * 100)))
        print()
        #Encoders
        encoder_izq = rIzq_encoder.getValue()
        encoder_der =  rDer_encoder.getValue()
        print(f"Valor del encoder izquierdo: {encoder_izq}")
        print(f"Valor del encoder derecho: {encoder_der}")
        
        avanzar(velocidad)
        if rDer_encoder.getValue() >= start + avance:
            avanzar(0)
            break  

def retroceso(tipo_retroceso):
    start = rDer_encoder.getValue()
    velocidad = 0
    retroceso = 0
    if tipo_retroceso == "recto":
        velocidad = 6.28
        retroceso = 2.9
    elif tipo_retroceso == "esquina":
        retroceso = 4.1
        velocidad = 6.28
    while robot.step(timeStep) != -1:
        
        # Lectura de Sensores
        #Encoders
        encoder_izq = rIzq_encoder.getValue()
        encoder_der = rDer_encoder.getValue()
        print(f"Valor del encoder izquierdo: {encoder_izq}")
        print(f"Valor del encoder derecho: {encoder_der}")
        retroceder(velocidad)
        if start - retroceso >= rDer_encoder.getValue():
            avanzar(0)
            break      
        
angulo_actual = 0
tiempo_anterior = robot.getTime()
contador = 0
estado = 0


while robot.step(timeStep) != -1:
    retroceso("recto")
    retroceso("recto")
    rotar_enclavado(90)
        

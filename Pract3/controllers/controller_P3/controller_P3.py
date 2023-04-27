# -*- coding: utf-8 -*-
""" 
Webots Khepera IV controller in Python for obstacle avoidance.
"""

from controller import Robot, Camera, DistanceSensor, Motor
import numpy as np
import random
import math

# Máxima velocidad de las ruedas soportada por el robot (khepera4).
MAX_SPEED = 47.6
# Velocidad por defecto para este comportamiento.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 32
# Odometría
RADIO_RUEDA = 0.021
ESPACIO_ENTRE_RUEDAS = 0.10819
RADIO_ENTRE_RUEDAS = ESPACIO_ENTRE_RUEDAS/2

MOV_RECTO = 0.25/RADIO_RUEDA
ANGULO = (45 * (math.pi/180))
GIRO = ANGULO*RADIO_ENTRE_RUEDAS/RADIO_RUEDA
ESTADOS = 6
ACCIONES = 3
MAP_SIZE = ESTADOS,ACCIONES
matriz = np.zeros((MAP_SIZE))
giroL = False
giroR = False
recto = False


ultrasonic_sensors_names = [
    "left ultrasonic sensor",
    "front left ultrasonic sensor",
    "front ultrasonic sensor",
    "front right ultrasonic sensor",
    "right ultrasonic sensor"
]

infrared_sensors_names = [
    # turret sensors
    "rear left infrared sensor", 
    "left infrared sensor", 
    "front left infrared sensor", 
    "front infrared sensor",
    "front right infrared sensor", 
    "right infrared sensor", 
    "rear right infrared sensor", 
    "rear infrared sensor",
    # ground sensors
    "ground left infrared sensor", 
    "ground front left infrared sensor", 
    "ground front right infrared sensor",
    "ground right infrared sensor"
]

# store the last time a message was displayed
last_display_second = 0


def init_devices(timeStep):
    """
    Obtener y configurar los dispositivos necesarios.

    timeStep: tiempo (en milisegundos) de actualización por defecto para los sensores/actuadores
      (cada dispositivo puede tener un valor diferente).
    """

    # Get pointer to the robot.
    robot = Robot()

    # Si queremos obtener el timestep de la simulación.
    # simTimeStep = int(robot.getBasicTimeStep())

    # Obtener dispositivos correspondientes a los motores de las ruedas.
    leftWheel = robot.getDevice("left wheel motor")
    rightWheel = robot.getDevice("right wheel motor")

    # Configuración inicial para utilizar movimiento por posición (necesario para odometría).
    # En movimiento por velocidad, establecer posición a infinito (wheel.setPosition(float('inf'))).
    leftWheel.setPosition(0)
    rightWheel.setPosition(0)
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

    # Obtener el dispositivo de la cámara
    camera = robot.getDevice("camera")
    # Activar el dispositivo de la cámara (el tiempo de actualización de los frames
    # de la cámara no debería ser muy alto debido al alto volumen de datos que implica).
    camera.enable(timeStep * 10)

    # Obtener y activar los sensores de posición de las ruedas (encoders).
    posL = robot.getDevice("left wheel sensor")
    posR = robot.getDevice("right wheel sensor")
    posL.enable(timeStep)
    posR.enable(timeStep)
        # get the LED actuators
    leds = [
        robot.getDevice("front left led"),
        robot.getDevice("front right led"),
        robot.getDevice("rear led")
    ]
    
        # enable the ultrasonic sensors
    ultrasonic_sensors = []
    for name in ultrasonic_sensors_names:
        sensor = robot.getDevice(name)
        sensor.enable(TIME_STEP)
        ultrasonic_sensors.append(sensor)
    
    # enable the infrared sensors
    infrared_sensors = []
    for name in infrared_sensors_names:
        sensor = robot.getDevice(name)
        sensor.enable(TIME_STEP)
        infrared_sensors.append(sensor)

    return robot, leftWheel, rightWheel, ultrasonic_sensors, infrared_sensors, posL, posR, camera, leds

def girarL(leftWheel, rightWheel, posL, posR,robot):
   encoderL = posL.getValue() - GIRO
   encoderR = posR.getValue() + GIRO
   leftWheel.setPosition(posL.getValue() - GIRO)
   rightWheel.setPosition(posR.getValue() + GIRO)
   leftWheel.setVelocity(CRUISE_SPEED)
   rightWheel.setVelocity(CRUISE_SPEED)
   while(posR.getValue() < encoderR - 0.02):
      robot.step(TIME_STEP)
   return encoderL, encoderR
          
def girarR(leftWheel, rightWheel, posL, posR,robot):
    encoderL = posL.getValue() + GIRO
    encoderR = posR.getValue() - GIRO
    leftWheel.setPosition(posL.getValue() + GIRO)
    rightWheel.setPosition(posR.getValue() - GIRO)
    leftWheel.setVelocity(CRUISE_SPEED)
    rightWheel.setVelocity(CRUISE_SPEED)
    while(posL.getValue() < encoderL - 0.01):
        robot.step(TIME_STEP)
    return encoderL, encoderR

def irRecto(leftWheel, rightWheel, posL, posR,robot):
       leftWheel.setVelocity(CRUISE_SPEED)
       rightWheel.setVelocity(CRUISE_SPEED)
       leftWheel.setPosition(posL.getValue() + MOV_RECTO)
       rightWheel.setPosition(posR.getValue() + MOV_RECTO)    
       encoderL = posL.getValue() + MOV_RECTO
       encoderR = posR.getValue() + MOV_RECTO
       return encoderL, encoderR

def getEstado():
    global giroL, giroR, recto
    if(DistanceSensor.getValue(infrared_sensors[2]) > 250):
        encoderL, encoderR = girarL(leftWheel, rightWheel, posL, posR,robot)
    elif(DistanceSensor.getValue(infrared_sensors[4]) > 180):
        encoderL, encoderR = girarL(leftWheel, rightWheel, posL, posR,robot)
    elif(DistanceSensor.getValue(infrared_sensors[9]) > 750 and DistanceSensor.getValue(infrared_sensors[11]) < 500):
        # sale de la linea por la izq
        encoderL, encoderR = girarL(leftWheel, rightWheel, posL, posR,robot)
        print("izq")
        giroL = True
    elif(DistanceSensor.getValue(infrared_sensors[10]) > 750 and DistanceSensor.getValue(infrared_sensors[8]) < 500):
       # sale de la linea por la dcha
       encoderL, encoderR = girarR(leftWheel, rightWheel, posL, posR,robot)
       print("dcha")
       giroR = True
    else:
        recto = True
        encoderL, encoderR = irRecto(leftWheel, rightWheel, posL, posR,robot)
    return giroL, giroR, recto

def getAccion(count,):
    global giroL, giroR, recto
    giroL, giroR, recto = getEstado()
    probabilidad = 1 - count/500
        if(random.random() <= probabilidad):
            index = random.randint(0,2)

robot, leftWheel, rightWheel, ultrasonic_sensors, infrared_sensors, posL, posR, camera,leds = init_devices(TIME_STEP)
timestep = int(robot.getBasicTimeStep())
robot.step(TIME_STEP)
# main loop
while robot.step(TIME_STEP) != -1:
    # display some sensor data every second
    # and change randomly the led colors
    display_second = int(robot.getTime())
    if display_second != last_display_second:
        for led in leds:
            led.set(random.randint(0, 0xFFFFFF))
        last_display_second = display_second
    giroL, giroR, recto = getEstado()
robot.cleanup()

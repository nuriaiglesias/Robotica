# -*- coding: utf-8 -*-
""" 
Webots Khepera IV controller in Python for obstacle avoidance.
"""

from controller import Robot, Camera, DistanceSensor, Motor
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

robot = Robot()

time_step = int(robot.getBasicTimeStep())

# enable the camera
camera = robot.getDevice("camera")
camera.enable(time_step)

# enable the ultrasonic sensors
ultrasonic_sensors = []
for name in ultrasonic_sensors_names:
    sensor = robot.getDevice(name)
    sensor.enable(time_step)
    ultrasonic_sensors.append(sensor)

# enable the infrared sensors
infrared_sensors = []
for name in infrared_sensors_names:
    sensor = robot.getDevice(name)
    sensor.enable(time_step)
    infrared_sensors.append(sensor)

# get the LED actuators
leds = [
    robot.getDevice("front left led"),
    robot.getDevice("front right led"),
    robot.getDevice("rear led")
]

# get the motors and set target position to infinity (speed control)
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# store the last time a message was displayed
last_display_second = 0


def girarL(left_motor, right_motor, speed_offset, speed_delta):
    left_motor.setVelocity(speed_offset + speed_delta)
    right_motor.setVelocity(speed_offset - speed_delta)
          
def girarR(left_motor, right_motor, speed_offset, speed_delta):
    left_motor.setVelocity(speed_offset + speed_delta)
    right_motor.setVelocity(speed_offset - speed_delta)

def irRecto(left_motor, right_motor):
       left_motor.setVelocity(CRUISE_SPEED)
       right_motor.setVelocity(CRUISE_SPEED)

# main loop
while robot.step(time_step) != -1:
    # display some sensor data every second
    # and change randomly the led colors
    display_second = int(robot.getTime())
    if display_second != last_display_second:
        for led in leds:
            led.set(random.randint(0, 0xFFFFFF))
        last_display_second = display_second
    speed_offset = 0.2 * (MAX_SPEED - 0.03 * infrared_sensors[3].getValue())
    speed_delta = 0.03 * infrared_sensors[2].getValue() - 0.03 * infrared_sensors[4].getValue()
    if(DistanceSensor.getValue(infrared_sensors[9]) > 750 and DistanceSensor.getValue(infrared_sensors[11]) < 500):
        girarL(left_motor,right_motor, speed_offset, speed_delta)
        print("izq")
    elif(DistanceSensor.getValue(infrared_sensors[10]) > 750 and DistanceSensor.getValue(infrared_sensors[8]) < 500):
       girarR(left_motor,right_motor, speed_offset, speed_delta)
       print("dcha")
    else:
        print("recto")
        irRecto(left_motor, right_motor)

robot.cleanup()

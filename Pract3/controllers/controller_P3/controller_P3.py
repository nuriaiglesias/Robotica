import random
from enum import Enum
import numpy as np

from controller import Robot, Motor, DistanceSensor

TIME_STEP = 32
MAX_SPEED = 10

last_display_second = 0
learning_rate = 0.5
mat_q = np.zeros((3,3))
visitas = np.zeros((3,3))
sensors_hist = []

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
    
def init_devices():

    robot = Robot()
    
    f_camera = robot.getDevice("camera")
    f_camera.enable(TIME_STEP)
    
    u_sensors = []
    for sensor in ultrasonic_sensors_names:
        u_sens = robot.getDevice(sensor)
        u_sens.enable(TIME_STEP)
        u_sensors.append(u_sens)
    
    infrared_sensors = []
    for sensor in infrared_sensors_names:
        ir_sens = robot.getDevice(sensor)
        ir_sens.enable(TIME_STEP)
        infrared_sensors.append(ir_sens)
    
    leds = [robot.getDevice("front left led"), robot.getDevice("front right led"), robot.getDevice("rear led")]
    
    leftWheel = robot.getDevice("left wheel motor")
    rightWheel = robot.getDevice("right wheel motor")
    leftWheel.getPositionSensor().enable(TIME_STEP)
    rightWheel.getPositionSensor().enable(TIME_STEP)
    leftWheel.setPosition(float('inf'))
    rightWheel.setPosition(float('inf'))
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    
    return leds,infrared_sensors,u_sensors,robot,leftWheel,rightWheel



leds,infrared_sensors,u_sensors,robot,leftWheel,rightWheel = init_devices()
leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(MAX_SPEED)
    

estado_actual = 2
accion_actual = 0
    
def check_sensors():
    return [infrared_sensors[8].getValue(), infrared_sensors[9].getValue(),
            infrared_sensors[10].getValue(), infrared_sensors[11].getValue()]
    
def getEstado(sensor_values):
    if sensor_values[1] > 750 and sensor_values[3] < 500:
        return 0
    elif sensor_values[2] > 750 and sensor_values[0] < 500:
        return 1
    return 2
    
def check_refuerzo(new_sensor_values, prev_sensor_values):
    if all(value < 500 for value in prev_sensor_values) and all(value < 500 for value in new_sensor_values):
        return 1
    elif all(value < 500 for value in prev_sensor_values) and not all(value < 500 for value in new_sensor_values):
        return -1
    elif all(value > 750 for value in prev_sensor_values) and all(value > 750 for value in new_sensor_values):
        return -1
    elif all(value > 750 for value in prev_sensor_values) and not all(value > 750 for value in new_sensor_values):
        return 1
    elif sum(i > 750 for i in prev_sensor_values) < sum(i > 750 for i in new_sensor_values):
        return -1
    else:
        return 1

def actualizar_refuerzo(refuerzo, action, prev_estado, nuevo_estado, learning_rate):
    visitas[prev_estado][action] += 1
    learning_rate = 1 / (1 + visitas[prev_estado][action])
    mat_q[prev_estado][action] = (1-learning_rate) * mat_q[prev_estado][action] + learning_rate * (refuerzo + 0.5 * np.argmax(mat_q[nuevo_estado]))
    return learning_rate


    
def pick_action(estado_actual):
    return np.argmax(mat_q[estado_actual])
    
def go_straight():
    accion_actual = 0
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
    
def turn_left():
    accion_actual = 1
    leftWheel.setVelocity(-MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
    
def turn_right():
    accion_actual = 2
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(-MAX_SPEED)

def perform_action(action):
    if action == 0:
        turn_right()
    elif action == 1:
        turn_left()
    elif action == 2:
        go_straight()
    

while robot.step(TIME_STEP) != -1:
    display_second = robot.getTime()
    if display_second != last_display_second:
        last_display_second = display_second
        
        if (infrared_sensors[2].getValue() > 300 or infrared_sensors[3].getValue() > 300 or infrared_sensors[4].getValue() > 300):
            speed_offset = 0.2 * (MAX_SPEED - 0.03 * infrared_sensors[3].getValue());
            speed_delta = 0.03 * infrared_sensors[2].getValue() - 0.03 * infrared_sensors[4].getValue()
            leftWheel.setVelocity(speed_offset + speed_delta)
            rightWheel.setVelocity(speed_offset - speed_delta)
        else:
            sensor_values = check_sensors()
            sensors_hist.append(sensor_values)
            estado_actual = getEstado(sensor_values)
            
            action = pick_action(estado_actual)
            print(mat_q)
            perform_action(action)
        
            sensors_hist.append(check_sensors())
            new_sensor_values = sensors_hist[len(sensors_hist)-3]
            nuevo_estado = getEstado(new_sensor_values)
            refuerzo = check_refuerzo(sensor_values, new_sensor_values)
            learning_rate = actualizar_refuerzo(refuerzo, action, estado_actual, nuevo_estado, learning_rate)
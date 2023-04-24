from controller import Robot, Motor, DistanceSensor  # Módulo de Webots para el control el robot.
from controller import Camera  # Módulo de Webots para el control de la cámara.
# Importar las librerías necesarias

from queue import PriorityQueue
import time  # Si queremos utilizar time.sleep().
import numpy as np  # Si queremos utilizar numpy para procesar la imagen.
import cv2  # Si queremos utilizar OpenCV para procesar la imagen.
import math
import heapq


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
ANGULO = (90 * (math.pi/180))
GIRO = ANGULO*RADIO_ENTRE_RUEDAS/RADIO_RUEDA


STARTX = 15
STARTY = 15
START = STARTX,STARTY
MAP_SIZE = 30,30
MAX_VALUE = 99

MAPA_VACIO = 0
MAPA_OCUPADO = 1
mapa = np.zeros((MAP_SIZE), dtype=np.int8)
distMap = np.zeros((MAP_SIZE))

for i in range(mapa.shape[0]):
        for j in range(mapa.shape[1]):
            distMap[i][j] = (abs(i-STARTX)+abs(j-STARTY))
position = START
startingPos = 0,0
orientacion = 0
giroL = False
mapaGuardado = False
ruta = ""
posInicial = False
meta = False
# Nombres de los sensores de distancia basados en infrarrojo.
INFRARED_SENSORS_NAMES = [
    "rear left infrared sensor",
    "left infrared sensor",
    "front left infrared sensor",
    "front infrared sensor",
    "front right infrared sensor",
    "right infrared sensor",
    "rear right infrared sensor",
    "rear infrared sensor",
]

def enable_distance_sensors(robot, timeStep, sensorNames):
    """
    Obtener y activar los sensores de distancia.

    Return: lista con los sensores de distancia activados, en el mismo orden
    establecido en la lista de  nombres (sensorNames).
    """
    sensorList = []

    for name in sensorNames:
        sensorList.append(robot.getDevice(name))

    for sensor in sensorList:
        sensor.enable(timeStep)

    return sensorList


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

    # Obtener una lista con los sensores infrarrojos ya activados
    irSensorList = enable_distance_sensors(robot, timeStep, INFRARED_SENSORS_NAMES)

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


    return robot, leftWheel, rightWheel, irSensorList, posL, posR, camera


def process_image_rgb(camera,imgdata):

    W = camera.getWidth()
    H = camera.getHeight()
    image = bytes(np.frombuffer(imgdata, np.uint8).reshape((H, W, 4)))

    # Si es suficiente, podríamos procesar solo una parte de la imagen para optimizar .
    for y in range(1*H//3,2*H//3):
        for x in range(2*W//5,3*W//5):
            green = Camera.imageGetGreen(image, W, x, y)
            red = Camera.imageGetRed(image, W, x, y)
            blue = Camera.imageGetBlue(image, W, x, y)
            if ((red > 240) and (green > 240) and (blue < 50)):
                return True
    return False



def girarL(leftWheel, rightWheel, posL, posR,robot):
   encoderL = posL.getValue() - GIRO
   encoderR = posR.getValue() + GIRO
   leftWheel.setPosition(posL.getValue() - GIRO)
   rightWheel.setPosition(posR.getValue() + GIRO)
   leftWheel.setVelocity(CRUISE_SPEED)
   rightWheel.setVelocity(CRUISE_SPEED)
   giroL = True
   while(posR.getValue() < encoderR - 0.02):
      robot.step(TIME_STEP)
   time.sleep(0.5)
   return encoderL, encoderR, giroL
          
def girarR(leftWheel, rightWheel, posL, posR,robot):
    encoderL = posL.getValue() + GIRO
    encoderR = posR.getValue() - GIRO
    leftWheel.setPosition(posL.getValue() + GIRO)
    rightWheel.setPosition(posR.getValue() - GIRO)
    leftWheel.setVelocity(CRUISE_SPEED)
    rightWheel.setVelocity(CRUISE_SPEED)
    giroL = False
    while(posL.getValue() < encoderL - 0.01):
        robot.step(TIME_STEP)
    time.sleep(0.5)
    return encoderL, encoderR, giroL

def irRecto(leftWheel, rightWheel, posL, posR,robot):
       leftWheel.setVelocity(CRUISE_SPEED)
       rightWheel.setVelocity(CRUISE_SPEED)
       leftWheel.setPosition(posL.getValue() + MOV_RECTO)
       rightWheel.setPosition(posR.getValue() + MOV_RECTO)    
       encoderL = posL.getValue() + MOV_RECTO
       encoderR = posR.getValue() + MOV_RECTO
       giroL = False
       while(posR.getValue() < encoderR - 0.01 or posL.getValue() < encoderL - 0.01):
          robot.step(TIME_STEP)
       time.sleep(0.5)
       return encoderL, encoderR, giroL
       
def obtenerPos(position, orientacion):
    x, y = position
    posiciones = [(x+1, y), (x, y+1), (x-1, y), (x, y-1)]
    return (posiciones[int(orientacion)], posiciones[(int(orientacion)+2)%4], posiciones[(int(orientacion)+1)%4], posiciones[(int(orientacion)-1)%4])
        
def crearMapa(delante, izq, dcha, irSensorList):
    posiciones = [izq, delante, dcha]
    direcciones = [1, 3, 5]
    for pos, dir in zip(posiciones, direcciones):
        x, y = pos
        if DistanceSensor.getValue(irSensorList[dir]) > 180:
            mapa[x][y] = MAPA_OCUPADO
        else:
            mapa[x][y] = MAPA_VACIO
    return mapa

def guardarmapa():
    global ruta
    ruta = r"" #write your path to store the map
    np.savetxt(ruta, mapa, fmt="%d")  

def seguirParedes(leftWheel,rightWheel,posL,posR,irSensorList,robot,imgdata,camera):
     global giroL,position,orientacion,mapaGuardado,posInicial
     if(mapaGuardado == False):
         delante,atras,izq,dcha = obtenerPos(position,orientacion)
         
         mapa = crearMapa(delante,izq,dcha,irSensorList)
        
         if(not giroL and DistanceSensor.getValue(irSensorList[1]) < 180):
            
             encoderL, encoderR, giroL = girarL(leftWheel, rightWheel, posL, posR,robot)
             orientacion = (orientacion + 1) % 4
         elif(giroL or DistanceSensor.getValue(irSensorList[3]) < 180):
            
             position = delante
             posInicial = True
             encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR,robot)
         elif(DistanceSensor.getValue(irSensorList[3]) >= 180):
             
             encoderL, encoderR, giroL = girarR(leftWheel, rightWheel, posL, posR,robot)
             orientacion = (orientacion - 1 + 4) % 4
         if(posInicial): 
             if(position == (STARTX,STARTY)):
                 print(mapa)
                 mapaGuardado = True
                 guardarmapa()



def A_star(mapa, start, goal):
   
    def heuristic(a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    
    lista = [(0, start)]
    lista2 = set()
    coste = {start: 0}
    visitado = {}

  
    while lista:
        
        actual = min(lista)[1]
        lista = [node for node in lista if node[1] != actual]


       
        if actual == goal:
            break

    
        lista2.add(actual)

       
        for adj in [(actual[0] + 1, actual[1]), (actual[0] - 1, actual[1]), (actual[0], actual[1] + 1), (actual[0], actual[1] - 1)]:
          
            if adj[0] < 0 or adj[0] >= len(mapa) or adj[1] < 0 or adj[1] >= len(mapa[0]) or mapa[adj[0]][adj[1]] == 1:
                continue

        
            nuevo_coste = coste[actual] + 1

            
            if adj in lista2 and nuevo_coste >= coste[adj]:
                continue

         
            visitado[adj] = actual
            coste[adj] = nuevo_coste
            priority = nuevo_coste + heuristic(goal, adj)
            if adj not in [node[1] for node in lista]:
                lista.append((priority, adj))

 
    if goal not in visitado:
        return None


    ruta = [goal]
    actual = goal
    while actual != start:
        actual = visitado[actual]
        ruta.append(actual)
    ruta.reverse()

  
    return ruta


    
def volverinicial(leftWheel, rightWheel, posL, posR, irSensorList, robot, imgdata, camera):
    global giroL, position, orientacion, mapaGuardado, ruta, meta
    if mapaGuardado and not meta: 
        
        delante,atras,izq,dcha = obtenerPos(position,orientacion)
      
        if(not giroL and DistanceSensor.getValue(irSensorList[1]) < 170):
             
             encoderL, encoderR, giroL = girarL(leftWheel, rightWheel, posL, posR,robot)
             orientacion = (orientacion + 1) % 4
        elif(giroL or DistanceSensor.getValue(irSensorList[3]) < 170):
             
             position = delante
             encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR,robot)
        elif(DistanceSensor.getValue(irSensorList[3]) >= 170):
           
             encoderL, encoderR, giroL = girarR(leftWheel, rightWheel, posL, posR,robot)
             orientacion = (orientacion - 1 + 4) % 4
        if(process_image_rgb(camera,imgdata)):
        
            ruta = A_star(mapa,position, (STARTX, STARTY))
            if ruta is not None:
                print(ruta)
                for nodo in ruta[::1]:
                    x, y = position
                    
                   
                    delante,atras,izq,dcha = obtenerPos(position,orientacion)  
                    if(orientacion == 0):
                       
                        if x < nodo[0]:
                           
                            encoderL, encoderR, giroL = girarR(leftWheel, rightWheel, posL, posR, robot)
                            orientacion = (orientacion - 1 + 4) % 4
                            position = (x+1, y)
                        elif x > nodo[0]:
                            
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            position = (x-1, y)
                        elif y < nodo[1]:
                           
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            position = (x, y+1)
                        elif y > nodo[1]:
                          
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            position = (x, y-1)
                    elif(orientacion == 1):
                 
                        if x < nodo[0]:
                           
                            encoderL, encoderR, giroL = girarR(leftWheel, rightWheel, posL, posR, robot)
                            orientacion = (orientacion - 1 + 4) % 4
                            position = (x+1, y)
                        elif x > nodo[0]: 
                            
                            encoderL, encoderR, giroL = girarL(leftWheel, rightWheel, posL, posR, robot)
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            orientacion = (orientacion + 1) % 4
                            position = (x-1, y)
                        elif y < nodo[1]: 
                          
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            position = (x, y+1)
                        elif y > nodo[1]:
                            
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            position = (x, y-1)
                    elif(orientacion == 2):
                      
                        if x < nodo[0]:
                           
                          
                            encoderL, encoderR, giroL = girarR(leftWheel, rightWheel, posL, posR, robot)
                            orientacion = (orientacion - 1 + 4) % 4
                            position = (x+1, y)
                        elif x > nodo[0]: 
                            
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            position = (x-1, y)
                        elif y < nodo[1]: 
                           
                            encoderL, encoderR, giroL = girarR(leftWheel, rightWheel, posL, posR, robot)
                            orientacion = (orientacion - 1 + 4) % 4
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            position = (x, y+1)
                        elif y > nodo[1]:
                            
                            encoderL, encoderR, giroL = girarL(leftWheel, rightWheel, posL, posR, robot)
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            orientacion = (orientacion + 1) % 4
                            position = (x, y-1)
                    elif(orientacion == 3):
                        
                        if x < nodo[0]:
                            
                            encoderL, encoderR, giroL = girarL(leftWheel, rightWheel, posL, posR, robot)
                            orientacion = (orientacion - 1 + 4) % 4
                            position = (x+1, y)
                        elif x > nodo[0]: 
                            
                            encoderL, encoderR, giroL = girarR(leftWheel, rightWheel, posL, posR, robot)
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            orientacion = (orientacion - 1 + 4) % 4
                            position = (x-1, y)
                        elif y < nodo[1]:
                         
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            position = (x, y+1)
                        elif y > nodo[1]: 
                            
                            
                            encoderL, encoderR, giroL = irRecto(leftWheel, rightWheel, posL, posR, robot)
                            position = (x, y-1)
            if position == (STARTX,STARTY):
               meta = True
              
            
           
               


def main():
    
    robot, leftWheel, rightWheel, irSensorList, posL, posR, camera = init_devices(TIME_STEP)

    
   
    robot.step(TIME_STEP)
    while robot.step(TIME_STEP) != -1:

        imgdata = camera.getImage()
        seguirParedes(leftWheel,rightWheel,posL,posR,irSensorList,robot,imgdata,camera)
        volverinicial(leftWheel,rightWheel,posL,posR,irSensorList,robot,imgdata,camera)


if __name__ == "__main__":
    main()

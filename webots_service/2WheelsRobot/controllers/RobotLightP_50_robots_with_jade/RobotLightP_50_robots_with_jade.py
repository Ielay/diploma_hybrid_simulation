"""RobotLightP controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import Emitter
from controller import Receiver
from controller import DistanceSensor
from controller import LightSensor
from controller import Compass
from random import randint
import math
import struct
import os

# Посылаем в jade:
# 1. bearing
# 2. q
# 3. light
# 4. id/robot_number
# 5. d
# 6. coordinates
def build_msg_for_jade(bearing, q, light, robot_number, d, coordinates):
    converted_light = ','.join([str(l) for l in light])
    converted_coordinates = ','.join([str(c) for c in coordinates])

    msg_template = '''bearing={0}\nq={1}\nlight={2}\nid={3}\nd={4}\ncoordinates={5}'''

    return msg_template.format(bearing, q, converted_light, robot_number, d, converted_coordinates)

#Расчитываем азимут
def calc_azimuth(com):
    north = com.getValues()
    rad = math.atan2(north[0], north[2])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360 
    # cos_com = north[0]
    # sin_com = north[2]
    #print(cos_com, sin_com)

    return bearing

#Расчет азимута движения на источник по четырем сенсорам света.
#Выбираем датчик с максимальным уровнем излучения. Сравниваем с соседними.
#Выбираем второй по мощности излучения датчик.
#Расчитываем доворо по часовой стролке до источника излучения.
#Расчитываем желаемый азимут.
def calc_dbearing(bearing, light):
    dbearing = bearing
    if light_max != 0:
        a = 0
        b = 0
        if light_max == light[0]:
            a = light[0] - light[3] 
            b = light[0] - light[1]
            if a < b and light[3] != 0: 
                dbearing = bearing - 45 + (light[0]*90)/(light[0]+light[3])
            elif b < a and light[1] != 0:
                dbearing = bearing + 45 + (light[1]*90)/(light[0]+light[1])
            else: 
                dbearing = bearing + 45
        elif light_max == light[1]:
            a = light[1] - light[0]
            b = light[1] - light[2]
            if a <= b and light[0] != 0:
                dbearing = bearing + 45 + (light[1]*90)/(light[0]+light[1])
            elif b < a and light[2] != 0:
                dbearing = bearing + 135 + (light[2]*90)/(light[2]+light[1])
            else: 
                dbearing = bearing + 135
        elif light_max == light[2]:
            a = light[2] - light[1] 
            b = light[2] - light[3] 
            if a <= b and light[1] != 0: 
                dbearing = bearing + 135 + (light[2]*90)/(light[2]+light[1])
            elif b < a and light[3] != 0:
                dbearing = bearing + 225 + (light[3]*90)/(light[2]+light[3])
            else: 
                dbearing = bearing + 225
        elif light_max == light[3]:
            a = light[3] - light[2] 
            b = light[3] - light[0]
            if a <= b and light[2] != 0: 
                dbearing = bearing + 225 + (light[3]*90)/(light[2]+light[3])
            elif b < a and light[0] != 0:
                dbearing = bearing + 315 + (light[0]*90)/(light[0]+light[3])
            else: 
                dbearing = bearing + 315
        if dbearing > 360:
          dbearing = dbearing - 360

    return dbearing

def calc_q(light, a_q, d):
    if light[0]+light[3] == 0:
        return 0
    else:
        return (1-a_q) * (1 - abs((light[0]-light[3]) / (light[0]+light[3]))) + a_q * (d/1000)

def change_wheels_velocity(wheels, leftSpeed, rightSpeed):
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)

#Передаем сообщение соседям
def send_msg(emmiter, bearing, q):
    message = struct.pack("dd",bearing,q)
    emmiter.send(message)

#Получаем сообщение от соседа
def receive_msg(receiver):
    message = receiver.getData()
    return struct.unpack("dd",message)


out_file_prefix = os.path.join('C:' + os.sep, 'diploma', 'webots_messaging', 'robots_with_jade', 'out_')
in_file_prefix = os.path.join('C:' + os.sep, 'diploma', 'webots_messaging', 'robots_with_jade', 'in_')

#Returns robot number in the hive (range: from 0 to +INF)
def extract_robot_number(robot_name) -> int:
    idx1 = robot_name.find('(')
    if idx1 == -1:
        return 0
    idx2 = robot_name.find(')')
    
    return int(robot_name[idx1 + 1 : idx2])

# Write data to local fs file
def write_data(robot_number, data):
    with open(out_file_prefix + str(robot_number), 'w+') as out_file:
        out_file.write(data)

# Read data from local fs file
def read_data(robot_number):
    data = None
    
    in_file_path = in_file_prefix + str(robot_number)
    with open(in_file_path, 'r') as in_file:
        line = in_file.readline()
        print('line = {0}'.format(line))
        if not line:
            data = None
        else:
            data = line

    if not data:
        return None
    else:
        return float(data)

# create the Robot instance.
robot = Robot()
robot_name = robot.getName()
robot_number = extract_robot_number(robot_name)
robot_own_channel = 1 + robot_number

print('Robot {0} configuration started'.format(robot_name))

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

# init empty files with in data (where new bearing should be located)
with open(in_file_prefix + str(robot_number), 'w+') as in_file:
    in_file.write('')

# initialize motors
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

#Вводим количество членов группы не включая самого робота (т.е. n-1)
# TODO v_p: not needed for this implementation
robots_number = 50 - 1 

# initialize distance sensor
ds = robot.getDevice('ds')
ds.enable(TIME_STEP)

# initialize motors
ls = []
lsNames = ['ls1', 'ls2', 'ls3', 'ls4']
for i in range(4):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(TIME_STEP)

# initialize compass   
com = robot.getDevice('com')
com.enable(TIME_STEP)

# init gps device
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

print('Robot {0} configuring is over'.format(robot_name))

#Начальное значение на двигатели
leftSpeed = 0
rightSpeed = 0 

# Переменные для задания обхода препятствия
avoidObstacleCounter = 0
p = 0 # choosing between left and right direction (0 - left, 1 - right)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    print('Robot {0} turn'.format(robot_name))

    #Расчитываем азимут
    bearing = calc_azimuth(com)

    #Ищем максимум из датчиков
    light = []
    for i in range(4):
        light.append(ls[i].getValue())
        # print('Robot {0} light {1}: {2}'.format(robot_name, i, light[i]))

    light_max = max(light)
    print('Max light: {0}'.format(light_max))
    
    #Вводим уверенность в курсе q по датчикам света 
    # датчику направления. a_q - коэфициент
    # d - показаия датчика дистанции.
    a_q = 0.5 #beta в дипломе?
    d = ds.getValue()
    q = calc_q(light, a_q, d)

    coords = gps.getValues()

    msg_for_jade = build_msg_for_jade(bearing, q, light, robot_number, d, coords)

    # send robot metrics to JADE
    write_data(robot_number, msg_for_jade)
    
    # get previously calculated dbearingG from JADE
    dbearingG = read_data(robot_number)
    print('dbearingG = {0}'.format(dbearingG))
    if dbearingG == None:
        # we can't change the velocity and direction if we don't have calculated dbearingG
        continue

    #Задаем движение
    if bearing == dbearingG and light[0]+light[1]+light[2]+light[3] > 0:
        leftSpeed = 3.14 
        rightSpeed = 3.14
    elif dbearingG > bearing and dbearingG < bearing + 180:
        leftSpeed = 3.14 
        rightSpeed = 2
    elif dbearingG > bearing and dbearingG > bearing + 180: 
        leftSpeed = 2
        rightSpeed = 3.14 
    elif bearing > dbearingG and bearing < dbearingG + 180:
        leftSpeed = 2
        rightSpeed = 3.14 
    elif bearing > dbearingG and bearing > dbearingG + 180:
        leftSpeed = 3.14 
        rightSpeed = 2
    else: 
        leftSpeed = 0
        rightSpeed = 0
    
    #Обход препятствий
    if d <= 400 and avoidObstacleCounter == 0:
        avoidObstacleCounter = 1
        if light[0] == light[1] == light[2] == light[3]:
            p = randint(0,1)
        elif light_max == light[0] or light_max == light[1]: 
            p = 0 #право
        elif light_max == light[2] or light_max == light[3]:
            p = 1 #влево
    
    if avoidObstacleCounter != 0:
        if d > 400:
            avoidObstacleCounter = 0
        else:
            avoidObstacleCounter -= 1
            if p == 1:
                leftSpeed = -2
                rightSpeed = 2  
            elif p == 0:
                leftSpeed = 2
                rightSpeed = -2
    
    #Отправляем значение на моторы
    change_wheels_velocity(wheels, leftSpeed, rightSpeed)
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.

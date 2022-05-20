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
from random import choice
import math
import struct


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
#Расчитываем поворот по часовой стролке до источника излучения.
#Расчитываем желаемый азимут.
def calc_light_azimuth(cur_bearing, light, light_max):
    light_bearing = cur_bearing
    # light_max = 0
    # light_min = 0
    if light_max != 0:
        a = 0
        b = 0
        if light_max == light[0]:
            a = light[0] - light[3] 
            b = light[0] - light[1]
            if a < b and light[3] != 0: 
                light_bearing = cur_bearing - 45 + (light[0]*90)/(light[0]+light[3])
                #light_max = light[3]+light[0]
                #light_min = light[2]+light[1]
            elif b < a and light[1] != 0:
                light_bearing = cur_bearing + 45 + (light[1]*90)/(light[0]+light[1])
                #light_max = light[0]+light[1]
                #light_min = light[3]+light[2]
            else: 
                light_bearing = cur_bearing + 45 
                #light_max = light[0]+light[1]
                #light_min = light[3]+light[2]
        elif light_max == light[1]:
            a = light[1] - light[0]
            b = light[1] - light[2]
            if a <= b and light[0] != 0:
                light_bearing = cur_bearing + 45 + (light[1]*90)/(light[0]+light[1])
                #light_max = light[1]+light[0]
                #light_min = light[3]+light[2]
            elif b < a and light[2] != 0:
                light_bearing = cur_bearing + 135 + (light[2]*90)/(light[2]+light[1])
                #light_max = light[1]+light[2]
                #light_min = light[3]+light[0]
            else:
                light_bearing = cur_bearing + 135
                #light_max = light[1]+light[2]
                #light_min = light[3]+light[0]
        elif light_max == light[2]:
            a = light[2] - light[1] 
            b = light[2] - light[3] 
            if a <= b and light[1] != 0: 
                light_bearing = cur_bearing + 135 + (light[2]*90)/(light[2]+light[1])
                #light_mqax = light[1]+light[2]
                #light_min = light[3]+light[0]
            elif b < a and light[3] != 0:
                light_bearing = cur_bearing + 225 + (light[3]*90)/(light[2]+light[3])
                #light_max = light[3]+light[2]
                #light_min = light[1]+light[0]
            else: 
                light_bearing = cur_bearing + 225
                #light_max = light[3]+light[2]
                #light_min = light[1]+light[0]
        elif light_max == light[3]:
            a = light[3] - light[2]
            b = light[3] - light[0]
            if a <= b and light[2] != 0:
                light_bearing = cur_bearing + 225 + (light[3]*90)/(light[2]+light[3])
                #light_max = light[3]+light[2]
                #light_min = light[1]+light[0]
            elif b < a and light[0] != 0:
                light_bearing = cur_bearing + 315 + (light[0]*90)/(light[0]+light[3])
                #light_max = light[3]+light[0]
                #light_min = light[1]+light[2]
            else: 
                light_bearing = cur_bearing + 315  
                #light_max = light[3]+light[0]
                #light_min = light[1]+light[2]                              
        if light_bearing > 360:
          light_bearing = light_bearing - 360

    return light_bearing

#Передаем сообщение соседям
def send_msg(emmiter, bearing, q, robot_number):
    message = struct.pack("ddi",bearing,q,robot_number)
    emmiter.send(message)

#Получаем сообщение от соседа
def receive_msg(receiver):
    message = receiver.getData()
    return struct.unpack("ddi",message)

#Returns robot number in the hive (range: from 0 to +INF)
def extract_robot_number(robot_name) -> int:
    idx1 = robot_name.find('(')
    if idx1 == -1:
        return 0
    idx2 = robot_name.find(')')
    
    return int(robot_name[idx1 + 1 : idx2])

# create the Robot instance.
robot = Robot()
robot_name = robot.getName()
robot_number = extract_robot_number(robot_name)
robot_own_channel = 1 + robot_number

# improt and configure logging
import logging

root_logger = logging.getLogger()
root_logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('C:\diploma\webots_logs\{0}.log'.format(robot_number), 'w+', 'utf-8')
handler.setFormatter(logging.Formatter('%(asctime)s | %(message)s'))
root_logger.addHandler(handler)

print('Robot {0} configuration started'.format(robot_name))

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

# initialize motors
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# initialize emmiters
emm = robot.getDevice('emitter')

# Вводим количество членов группы не включая самого робота (т.е. n-1)
robots_number = 25 

#initialize receivers
receiver = robot.getDevice('receiver')
receiver.enable(TIME_STEP)

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

print('Robot {0} configuring is over'.format(robot_name))

#Начальное значение на двигатели
leftSpeed = 0
rightSpeed = 0

# min/max allowable distance to the obstacle
# (not sure if it is 4 or 400 meters)
d_min = 400
d_max = 1000

# Переменные для задания обхода препятствия
avoidObstacleCounter = 0
j = 0
p = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    logging.debug('-------')
    print('------\nRobot {0} turn'.format(robot_number))

    #Расчитываем текущий азимут (bearing t)
    cur_bearing = calc_azimuth(com)
    logging.debug('cur bearing = %s', int(cur_bearing))

    #Ищем максимум из датчиков
    light = []
    for i in range(4):
        light.append(ls[i].getValue())

    light_max = max(light)
    # print('Max light: {0}'.format(light_max))
    
    #Calc light bearing (direction to the max light)
    light_bearing = calc_light_azimuth(cur_bearing, light, light_max)
    logging.debug('light bearing = %s', int(light_bearing))
    
    #Вводим уверенность в курсе q^i_t по датчикам света и датчику направления.
    # betha - коэфициент (0 - сенсор дистанции не учитывается, 1 - сенсоры света не учитываются)
    # d - показания датчика дистанции.
    q = 0
    betha = 0.5
    d = ds.getValue()
    logging.debug('d = %d', d)
    if light[0] + light[3] == 0:
        # q = 0
        q = betha * (min(d, d_max)/d_max)
    else:
        q = (1 - betha) * (1 - abs((light[0] - light[3]) / (light[0] + light[3]))) + betha * (min(d, d_max)/d_max)
    
    #Передаем сообщение соседям
    send_msg(emm, cur_bearing, q, robot_number)
    
    #Принимаем сообщения от соседей
    neighbours_number = 0
    # bearing_n: [0] - bearing, [1] - q
    bearing_n = [0] * robots_number
    for i in range(robots_number):
        bearing_n[i] = [0] * 2
        bearing_n[i][1] = -1
    
    while receiver.getQueueLength() > 0:
        dataList = receive_msg(receiver)

        neighbour_number = dataList[2]

        bearing_n[neighbour_number][0] = dataList[0]
        bearing_n[neighbour_number][1] = dataList[1]
        logging.debug('Bearing [{1}]: {0}'.format(bearing_n[neighbour_number][0], neighbour_number))

        receiver.nextPacket()

        neighbours_number += 1
    logging.debug('neighbours_number = %s', neighbours_number)

    #Расчитываем sigma_t
    # 0.8 is the optimal as follows from the Veronika's diploma
    alpha = 0.8
    deltaq = 0
    for i in range(robots_number):
        if bearing_n[i][1] != -1:
            deltaq += bearing_n[i][1] - q
    # print('deltaq = {0}'.format(deltaq))

    # calc sigma_t
    # sigma_t = 0
    # if neighbours_number == 0:
    #     sigma_t = (alpha*deltaq)
    # else:
    #     sigma_t = (alpha*deltaq)/neighbours_number
    sigma_t = 0
    if neighbours_number == 0:
        sigma_t = 0
    else:
        sigma_t = (alpha * deltaq) / neighbours_number
    
    #Расчитываем gamma_t 
    # if q == 0:
    #     q = 0.01
    # gamma_t = 1/(q+sigma_t)
    gamma_t = 0
    if (q + sigma_t) == 0:
        gamma_t = 0
    else:
        gamma_t = 1 / (q + sigma_t)
    
    def calc_new_bearing(light_bearing, neighbours_number, alpha, sigma_t, gamma_t, bearing_n):
        if neighbours_number == 0:
            return (light_bearing * (1 - (sigma_t * gamma_t)))
        else:
            delta_rq = 0
            for i in range(robots_number):
                if bearing_n[i][1] != -1:
                    delta_rq += (bearing_n[i][0] * bearing_n[i][1]) - (cur_bearing * q)
            
            return (light_bearing * (1 - (sigma_t * gamma_t))) + (((alpha * gamma_t) / neighbours_number) * delta_rq)

    # Calc new bearing (t+1)
    rotation_step = 10
    new_bearing = light_bearing
    if d >= d_max:
        avoidObstacleCounter = 0
        # just calculate new bearing using the formula
        new_bearing = calc_new_bearing(light_bearing, neighbours_number, alpha=alpha, sigma_t=sigma_t, gamma_t=gamma_t, bearing_n=bearing_n)
    elif d_min < d and d < d_max:
        avoidObstacleCounter = 0
        # make a rotation
        light_bearing_corrected = light_bearing
        if light[0] == 0 and light[3] == 0:
            if light[1] != 0 or light[2] != 0:
                if light[1] > light[2]:
                    light_bearing_corrected += rotation_step
                else:
                    light_bearing_corrected -= rotation_step
        else:
            if light[0] > light[3]:
                light_bearing_corrected += rotation_step
            else:
                light_bearing_corrected -= rotation_step
        
        new_bearing = calc_new_bearing(light_bearing_corrected, neighbours_number, alpha=alpha, sigma_t=sigma_t, gamma_t=gamma_t, bearing_n=bearing_n)
    elif d <= d_min:
        # random rotation
        if avoidObstacleCounter == 0:
            # choose new rotation side
            avoidObstacleCounter = 1
            
            p = choice([1, -1])
            # new_bearing += p * rotation_step
        # else:
            # use already chosen rotation side
            # new_bearing += p * rotation_step
    
    if new_bearing > 360:
        new_bearing = new_bearing - 360
    
    logging.debug('new bearing = %s', int(new_bearing))

    # previously:
    # slow = 2
    # fast = 3.14
    slow_velocity = math.pi / 2
    fast_velocity = math.pi
    #Задаем движение
    if d <= d_min:
        if p == -1:
            leftSpeed = 0
            rightSpeed = fast_velocity
        elif p == 1:
            leftSpeed = fast_velocity
            rightSpeed = 0
    else:
        if int(cur_bearing) == int(new_bearing) and light[0]+light[1]+light[2]+light[3] > 0:
            # keep forward
            logging.debug('keep forward')
            leftSpeed = fast_velocity
            rightSpeed = fast_velocity
        elif new_bearing > cur_bearing and new_bearing <= cur_bearing + 180:
            # turn right
            logging.debug('turn right')
            leftSpeed = fast_velocity
            rightSpeed = slow_velocity
        elif new_bearing > cur_bearing and new_bearing >= cur_bearing + 180: 
            # turn left
            logging.debug('turn left')
            leftSpeed = slow_velocity
            rightSpeed = fast_velocity
        elif cur_bearing > new_bearing and cur_bearing <= new_bearing + 180:
            # turn left
            logging.debug('turn left')
            leftSpeed = slow_velocity
            rightSpeed = fast_velocity
        elif cur_bearing > new_bearing and cur_bearing >= new_bearing + 180:
            # turn right
            logging.debug('turn right')
            leftSpeed = fast_velocity
            rightSpeed = slow_velocity
        else:
            # stay still
            logging.debug('stay still')
            leftSpeed = 0
            rightSpeed = 0
    
    # #Обход препятствий
    # #print(d)
    # if d <= d_min and avoidObstacleCounter == 0:
    #     avoidObstacleCounter = 1
    #     if light[0] == light[1] == light[2] == light[3]:
    #         p = randint(0,1)
    #     elif light_max == light[0] or light_max == light[1]: 
    #         p = 0 #право
    #     elif light_max == light[2] or light_max == light[3]:
    #         p = 1 #влево
    
    # if avoidObstacleCounter != 0:
    #     if d > d_min:
    #         avoidObstacleCounter = 0
    #     else:
    #         avoidObstacleCounter -= 1
    #         if p == 1:
    #             leftSpeed = -2
    #             rightSpeed = 2  
    #         elif p == 0:
    #             leftSpeed = 2
    #             rightSpeed = -2
    
    #Отправляем значение на моторы
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.

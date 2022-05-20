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

#Передаем сообщение соседям
def send_msg(emmiter, bearing, q):
    message = struct.pack("dd",bearing,q)
    emmiter.send(message)

#Получаем сообщение от соседа
def receive_msg(receiver):
    message = receiver.getData()
    return struct.unpack("dd",message)


# create the Robot instance.
robot = Robot()
robot_name = robot.getName()

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
emm = robot.getDevice('trans')

#Вводим количество членов группы не включая самого робота (т.е. n-1)
robots_number = 24

#initialize receiver 
rec = []  
recNames = ['rec1', 'rec2', 'rec3', 'rec4', 'rec5', 'rec6', 'rec7', 'rec8', 'rec9', 'rec10',
            'rec11', 'rec12', 'rec13', 'rec14', 'rec15', 'rec16', 'rec17', 'rec18', 'rec19', 'rec20',
            'rec21', 'rec22', 'rec23', 'rec24']
for i in range(robots_number):
    rec.append(robot.getDevice(recNames[i]))
    rec[i].enable(TIME_STEP)

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

#Начальное значение на двигатели
leftSpeed = 0
rightSpeed = 0 

# Переменные для задания обхода препятствия
avoidObstacleCounter = 0
j = 0
p = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    #Расчитываем азимут
    bearing = calc_azimuth(com)

    #Ищем максимум из датчиков
    light = []
    for i in range(4):
        light.append(ls[i].getValue())
        # print('Robot {0} light {1}: {2}'.format(robot_name, i, light[i]))

    light_max = max(light)
    print('Robot {0} max light: {1}'.format(robot_name, light_max))
    
    #Расчет азимута движения на источник по четырем сенсорам света.
    #Выбираем датчик с максимальным уровнем излучения. Сравниваем с соседними.
    #Выбираем второй по мощности излучения датчик.
    #Расчитываем доворо по часовой стролке до источника излучения.
    #Расчитываем желаемый азимут.
    dbearing = bearing
    # light_max = 0
    # light_min = 0
    if light_max != 0:
        a = 0
        b = 0
        if light_max == light[0]:
            a = light[0] - light[3] 
            b = light[0] - light[1]
            if a < b and light[3] != 0: 
                dbearing = bearing - 45 + (light[0]*90)/(light[0]+light[3])
                #light_max = light[3]+light[0]
                #light_min = light[2]+light[1]
            elif b < a and light[1] != 0:
                dbearing = bearing + 45 + (light[1]*90)/(light[0]+light[1])
                #light_max = light[0]+light[1]
                #light_min = light[3]+light[2]
            else: 
                dbearing = bearing + 45 
                #light_max = light[0]+light[1]
                #light_min = light[3]+light[2]
        elif light_max == light[1]:
            a = light[1] - light[0]
            b = light[1] - light[2]
            if a <= b and light[0] != 0:
                dbearing = bearing + 45 + (light[1]*90)/(light[0]+light[1])
                #light_max = light[1]+light[0]
                #light_min = light[3]+light[2]
            elif b < a and light[2] != 0:
                dbearing = bearing + 135 + (light[2]*90)/(light[2]+light[1])
                #light_max = light[1]+light[2]
                #light_min = light[3]+light[0]
            else: 
                dbearing = bearing + 135
                #light_max = light[1]+light[2]
                #light_min = light[3]+light[0]
        elif light_max == light[2]:
            a = light[2] - light[1] 
            b = light[2] - light[3] 
            if a <= b and light[1] != 0: 
                dbearing = bearing + 135 + (light[2]*90)/(light[2]+light[1])
                #light_max = light[1]+light[2]
                #light_min = light[3]+light[0]
            elif b < a and light[3] != 0:
                dbearing = bearing + 225 + (light[3]*90)/(light[2]+light[3])
                #light_max = light[3]+light[2]
                #light_min = light[1]+light[0]
            else: 
                dbearing = bearing + 225
                #light_max = light[3]+light[2]
                #light_min = light[1]+light[0]
        elif light_max == light[3]:
            a = light[3] - light[2] 
            b = light[3] - light[0]
            if a <= b and light[2] != 0: 
                dbearing = bearing + 225 + (light[3]*90)/(light[2]+light[3])
                #light_max = light[3]+light[2]
                #light_min = light[1]+light[0]
            elif b < a and light[0] != 0:
                dbearing = bearing + 315 + (light[0]*90)/(light[0]+light[3])
                #light_max = light[3]+light[0]
                #light_min = light[1]+light[2]
            else: 
                dbearing = bearing + 315  
                #light_max = light[3]+light[0]
                #light_min = light[1]+light[2]                              
        if dbearing > 360:
          dbearing = dbearing - 360
   
    # print("Hello")
    # print (robot_name)
    # print("bearing =", bearing)
    # print("dbearing =", dbearing)
    
    #Вводим уверенность в курсем q по датчикам света 
    # датчику направления. a_q - коэфициент
    # d - показаия датчика дистанции.
    q = 0
    a_q = 0.5 #beta в дипломе?
    d = ds.getValue()
    if light[0]+light[3] == 0:
        q = 0
    else:
        q = (1-a_q)*(1 - abs((light[0]-light[3])/(light[0]+light[3]))) + a_q*(d/1000)    
    
    #Передаем сообщение соседям
    send_msg(emm, bearing, q)
    
    #Принимаем сообщение
    k_t = robots_number
    bearing_n = [0] * robots_number
    for i in range(robots_number):
        bearing_n [i] = [0] * 2 
        #print (bearingn [i][1])
    for i in range(robots_number):
        if rec[i].getQueueLength() > 0:
            dataList = receive_msg(rec[i])

            bearing_n [i][0] = dataList[0]
            bearing_n [i][1] = dataList[1]
            print('Bearing: {0}; {1}'.format(bearing_n[i][0], bearing_n[i][1]))

            rec[i].nextPacket()
        else:
            k_t -= 1
            bearing_n [i][1] = -1
    print('k_t = {0}'.format(k_t))

    #Расчитываем sigma
    # alpha = 1.5
    alpha = 0.8
    deltaq = 0
    for i in range (robots_number):
        if bearing_n [i][1] != -1:
            deltaq += bearing_n[i][1] - q
    if k_t == 0:
        k_t = 1
   
    print('deltaq = {0}'.format(deltaq))
    sigma_t = (alpha*deltaq)/k_t
    
    #Расчитываем гамма^i_t 
    if q == 0:
        q = 0.01
    gamma_t = 1/(q+sigma_t)
    
    """
    #Считаем среднюю уверенность соседей q_sr
    q_sum = 0
    k_i = k
    q_sr = q
    for i in range (k):
        if bearingn [i][1] != 0:
            q_sum += bearingn [i][1]
        else:
            k_i -= 1
    if k_i > 0:
        q_sr = q_sum/k_i
    else:
        q_sr = 1
    
    
    #Считаем средневзвешенный курс соседей dbearingn_sr_w
    
    dbearingn_sr_w = 0
    cos_sum_sr_w = 0
    sin_sum_sr_w = 0
    sin_sum_w = 0
    cos_sum_w = 0
    k_i_w = 0
    for i in range (k):
        if q_sum != 0:
            q_i_sr_w = (bearingn[i][1])/q_sum
        else:
            q_i_sr_w = (bearingn[i][1]) 
        if bearingn [i][1] > 0:
            cos_sum_w += math.cos(math.radians(bearingn[i][0]))*q_i_sr_w
            sin_sum_w += math.sin(math.radians(bearingn[i][0]))*q_i_sr_w
            k_i_w += 1 
    if k_i_w != 0:
        cos_sum_sr_w = cos_sum_w*q_sum/k_i_w
        sin_sum_sr_w = sin_sum_w*q_sum/k_i_w
    """
    # Расчитываем сумму разностей
    cos_delta_sum = 0
    sin_delta_sum = 0
    cos_bearing = math.cos(math.radians(bearing))
    sin_bearing = math.sin(math.radians(bearing)) 
    cos_dbearing = math.cos(math.radians(dbearing))
    sin_dbearing = math.sin(math.radians(dbearing))

    for i in range (robots_number):
        if bearing_n [i][1] != -1:
            cos_bearingn = math.cos(math.radians(bearing_n [i][0]))
            sin_bearingn = math.sin(math.radians(bearing_n [i][0])) 
            cos_delta_sum += cos_bearingn*bearing_n [i][1] - cos_bearing*q
            sin_delta_sum += sin_bearingn*bearing_n [i][1] - sin_bearing*q
    #Расчитываем курс в группе dbearingG исходя из данных группы
    #alpha - коэфициент, p - уверенность к курсу при пересчете от группы
    dbearingG = 0
    if k_t == 0:
        dbearingG = dbearing
    else:     
        cos_db_G = cos_dbearing*(1-(sigma_t*gamma_t))+(alpha*gamma_t*cos_delta_sum)/k_t  
        sin_db_G = sin_dbearing*(1-(sigma_t*gamma_t))+(alpha*gamma_t*sin_delta_sum)/k_t
        #cos_db_G = ((1-alpha)*cos_bearing*q + alpha*cos_sum_sr_w)/((1-alpha)*q + alpha*q_sr)
        #sin_db_G = ((1-alpha)*sin_bearing*q + alpha*sin_sum_sr_w)/((1-alpha)*q + alpha*q_sr)
        
        if cos_db_G < -1:
            cos_db_G = -1
        if cos_db_G > 1:
            cos_db_G = 1
        if cos_db_G > 0 and sin_db_G > 0:
            dbearingG = math.degrees(math.acos(cos_db_G))
        elif cos_db_G > 0 and sin_db_G < 0:
            dbearingG = 360 - math.degrees(math.acos(cos_db_G))
        elif cos_db_G < 0 and sin_db_G > 0:
            dbearingG = 180 - math.degrees(math.acos(cos_db_G))   
        elif cos_db_G < 0 and sin_db_G < 0:
            dbearingG = 180 + math.degrees(math.acos(cos_db_G))
    
    # print("dbearingG=", dbearingG)
        
    if d < 1000 and d > 400:
        if light[0] == light[1] == light[2] == light[3]:
            j = randint(1,2)
        elif light_max == light[0] or light_max == light[1]: 
            j = 1 #право
        elif light_max == light[2] or light_max == light[3]:
            j = 2 #влево 
       
        if j == 2:
            dbearing = dbearing-10
        elif j == 1:
            dbearing = dbearing+10
    #print (j)   
    if dbearingG > 360:
        dbearingG = dbearingG-360
          
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
    #print(d)
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

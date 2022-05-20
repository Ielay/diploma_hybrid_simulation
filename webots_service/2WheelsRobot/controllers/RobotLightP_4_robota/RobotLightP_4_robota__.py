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

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
ROBOT_TIME_STEP = int(robot.getBasicTimeStep())
RECEIVER_TIME_STEP = ROBOT_TIME_STEP
DISTANCE_SENSOR_TIME_STEP = ROBOT_TIME_STEP
MOTOR_TIME_STEP = ROBOT_TIME_STEP
COMPASS_TIME_STEP = ROBOT_TIME_STEP

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
recNames = ['rec1', 'rec2', 'rec3', 'rec4', 'rec5', 'rec6', 'rec7', 'rec8', 'rec9', 'rec10', 'rec11', 'rec12', 'rec13', 'rec14', 'rec15', 'rec16', 'rec17', 'rec18', 'rec19', 'rec20', 'rec21', 'rec22', 'rec23', 'rec24']
for i in range(robots_number):
    rec.append(robot.getDevice(recNames[i]))
    rec[i].enable(RECEIVER_TIME_STEP)

# initialize distance sensor   
ds = robot.getDevice('ds')
ds.enable(DISTANCE_SENSOR_TIME_STEP)

# initialize motors
ls = []
lsNames = ['ls1', 'ls2', 'ls3', 'ls4']
for i in range(4):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(MOTOR_TIME_STEP)

# initialize compass
com = robot.getDevice('com')
com.enable(COMPASS_TIME_STEP)

#Начальное значение на двигатели
leftSpeed = 0
rightSpeed = 0 

# Переменные для задания обхода препятствия
avoidObstacleCounter = 0
counter = 0
j = 0
p = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(ROBOT_TIME_STEP) != -1:
    print ('Robot = {0}'.format(robot.getName()))

    #Ищем максимум из датчиков
    light = []
    for i in range(4):
        light.append(ls[i].getValue())
        #print (light[i])
    # max_light = light[0]
    # for i in range(4):
    #     if light[i] > max_light:
    #         max_light = light[i]
    max_light = max(light)
    print('Max light = {0}'.format(max_light))


    #Расчет азимута движения на источник по четырем сенсорам света.
    #Выбираем датчик с максимальным уровнем излучения. Сравниваем с соседними.
    #Выбираем второй по мощности излучения датчик.
    #Расчитываем доворо по часовой стролке до источника излучения.
    #Расчитываем желаемый азимут.
    error = 0
    light_max = 0
    light_min = 0
    if max_light != 0:
        a = 0
        b = 0               
        if max_light == light[0]:
            a = light[0] - light[3] 
            b = light[0] - light[1]
            if a < b and light[3] != 0: 
                error = (light[0]*90)/(light[0]+light[3])-45
              
            elif b < a and light[1] != 0:
                error = 45 + (light[1]*90)/(light[0]+light[1])
               
            else: 
                # TODO v_p: это ошибка, и тут должно быть 'error' ?
                #  нет определения переменной dbearing в этом скоупе
                dbearing = 45 
                
        elif max_light == light[1]:
            a = light[1] - light[0]
            b = light[1] - light[2]
            if a <= b and light[0] != 0:
                error = 45 + (light[1]*90)/(light[0]+light[1])
                
            elif b < a and light[2] != 0:
                error = 135 + (light[2]*90)/(light[2]+light[1])
                
            else: 
                error = 135
              
        elif max_light == light[2]:
            a = light[2] - light[1] 
            b = light[2] - light[3] 
            if a <= b and light[1] != 0: 
                error = 135 + (light[2]*90)/(light[2]+light[1])
                
            elif b < a and light[3] != 0:
                error = 225 + (light[3]*90)/(light[2]+light[3])
                
            else: 
                error = 225
                
        elif max_light == light[3]:
            a = light[3] - light[2] 
            b = light[3] - light[0]
            if a <= b and light[2] != 0: 
                error = 225 + (light[3]*90)/(light[2]+light[3])
                
            elif b < a and light[0] != 0:
                error = 315 + (light[0]*90)/(light[0]+light[3])
                
            else: 
                error = 315                          
   
    print('error = {0}'.format(error))
    # print (robot.getName())
    # print("bearing =", bearing)
    # print("dbearing =", dbearing)
    
    #Вводим уверенность в курсе q по датчикам света и датчику направления
    # a_q - коэфициент
    # d - показания датчика дистанции
    q = 0
    a_q = 0.5
    d = ds.getValue()
    if light[0]+light[3] == 0:
        q = 0
    else:
        q = (1-a_q)*(1 - abs((light[0]-light[3])/(light[0]+light[3]))) + a_q*(d/1000)    
    
    #Передаем сообщение соседям (азимут + уверенность в курсе)
    message = struct.pack("dd",error,q)
    emm.send(message)
    
    #Принимаем сообщение
    k_t = robots_number
    bearingn = [0] * robots_number
    for i in range(robots_number):
        bearingn [i] = [0] * 2 
        #print (bearingn [i][1])
    for i in range(robots_number):
        if rec[i].getQueueLength() > 0:
            message = rec[i].getData()
            dataList = struct.unpack("dd",message)
            bearingn [i][0] = dataList[0]
            bearingn [i][1] = dataList[1]
            rec[i].nextPacket()
            # print("Hello")
            print(bearingn[i][0], bearingn[i][1])
        else:
            k_t -= 1    
            bearingn [i][1] = -1
    print ("k_t = ", k_t) 
    #Расчитываем sigma
    alpha = 0.8
    deltaq = 0
    sigma_t = 0
    for i in range (robots_number):
        if bearingn [i][1] != -1:
            deltaq += bearingn[i][1] - q
    print ("deltaq= ", deltaq)
    if k_t == 0:
        sigma_t = 0 
    else:    
        sigma_t = (alpha*deltaq)/k_t
    
    #Расчитываем гамма^i_t 
    if q == 0:
        q=0.01
    gamma_t = 1/(q+sigma_t)
    
    # Расчитываем сумму разностей
    cos_delta_sum = 0
    sin_delta_sum = 0
    cos_error = math.cos(math.radians(error))
    sin_error = math.sin(math.radians(error)) 
   
    for i in range (robots_number):
        if bearingn [i][1] != -1:
            cos_bearingn = math.cos(math.radians(bearingn [i][0]))
            sin_bearingn = math.sin(math.radians(bearingn [i][0])) 
            cos_delta_sum += cos_bearingn*bearingn [i][1] - cos_error*q
            sin_delta_sum += sin_bearingn*bearingn [i][1] - sin_error*q

    #Расчитываем курс в группе dbearingG исходя из данных группы
    #alpha - коэфициент, p - уверенность к курсу при пересчете от группы
    dbearingG = 0
    if k_t == 0:
        dbearingG = error
    else:     
        cos_dbearingG = cos_error*(1-(sigma_t*gamma_t))+(alpha*gamma_t*cos_delta_sum)/k_t  
        sin_dbearingG = sin_error*(1-(sigma_t*gamma_t))+(alpha*gamma_t*sin_delta_sum)/k_t
        
        print("cos", cos_dbearingG)
            
        if cos_dbearingG < -1:
            cos_dbearingG = -1
        if cos_dbearingG > 1:
            cos_dbearingG = 1
        if cos_dbearingG > 0 and sin_dbearingG > 0:
            dbearingG = math.degrees(math.acos(cos_dbearingG))
        elif cos_dbearingG > 0 and sin_dbearingG < 0:
            dbearingG = 360 - math.degrees(math.acos(cos_dbearingG))
        elif cos_dbearingG < 0 and sin_dbearingG > 0:
            dbearingG = 180 - math.degrees(math.acos(cos_dbearingG))   
        elif cos_dbearingG < 0 and sin_dbearingG < 0:
            dbearingG = 180 + math.degrees(math.acos(cos_dbearingG))
            
    print("errorG=", dbearingG)
          
    #Задаем движение
    if dbearingG <= 5 or dbearingG >= 355: 
        leftSpeed = 3.14 
        rightSpeed = 3.14
    elif dbearingG <= 175:
        leftSpeed = 3.14
        rightSpeed = 3.14*(1-dbearingG/180) 
    elif dbearingG >= 185: 
        leftSpeed = 3.14*((dbearingG/180)-1)
        rightSpeed = 3.14 
    else:
        counter = 50
        
    #Обход препятствий
    print("d",d)
    if d <= 600 and avoidObstacleCounter == 0:
        avoidObstacleCounter = 1
        if light[0] == light[1] == light[2] == light[3]:
            p = randint(0,1)
        elif max_light == light[0] or max_light == light[1]: 
            p = 0 #право
        elif max_light == light[2] or max_light == light[3]:
            p = 1 #влево       
   
    
    if avoidObstacleCounter != 0:
        if d > 600:
           avoidObstacleCounter = 0
        else:
           avoidObstacleCounter -= 1
                
           if p == 1:
               leftSpeed = -2
               rightSpeed = 2  
           elif p == 0:
               leftSpeed = 2
               rightSpeed = -2
    
    if counter != 0:
         leftSpeed = -2
         rightSpeed = 2
         counter -= 1 
         
    print ("LeftSpeed", leftSpeed)
    print ("RightSpeed", rightSpeed) 
                 
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

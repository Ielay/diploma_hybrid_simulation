"""RobotLightP controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from random import randint
import math
import struct
import os
import json

# ROS
import rospy
from std_msgs.msg import String

class RosMessaging:
    def __init__(self, robot_number):
        self.robot_id = robot_number
        self.azimuth = None

        rospy.init_node('ros_robot_{}'.format(self.robot_id), anonymous=True)

        self.publisher = rospy.Publisher('webots_to_jade/{}/robot_info'.format(self.robot_id), 
                                        String, queue_size=10)
        self.subscriber = rospy.Subscriber('jade_to_webots/{}/new_azimuth'.format(self.robot_id), 
                                        String, self.sub_callback)
    
    
    def publish(self, robot_number, bearing, q, light, d, neighbours):
        converted_light = ','.join([str(l) for l in light])
        converted_neighbours = ','.join([str(int(c)) for c in neighbours])

        robot_data = {
            'id': robot_number,
            'azimuth': bearing,
            'q': q,
            'light': converted_light,
            'neighbours': converted_neighbours,
            'd': d
        }

        msg = json.dumps(robot_data)
        self.publisher.publish(msg)
    
    def sub_callback(self, data):
        print('Received data {} for robot {}!'.format(data, self.robot_id))

        self.azimuth = float(data.data)
    
    def get_azimuth(self):
        return self.azimuth


class Calculations:
    @staticmethod
    def calc_azimuth(com):
        '''
        Raschityvaem azimut
        '''
        north = com.getValues()
        rad = math.atan2(north[0], north[2])
        bearing = (rad - 1.5708) / math.pi * 180.0
        if bearing < 0.0:
            bearing = bearing + 360
        # cos_com = north[0]
        # sin_com = north[2]
        # print(cos_com, sin_com)

        return bearing

    @staticmethod
    def calc_q(light, a_q, d):
        '''
        Calculating confidence
        '''
        if light[0]+light[3] == 0:
            return 0
        else:
            return (1-a_q) * (1 - abs((light[0]-light[3]) / (light[0]+light[3]))) + a_q * (d/1000)


class BotInternal:
    @staticmethod
    def send_msg(emmiter, robot_number):
        '''
        Peredaem soobshhenie sosedjam
        '''
        message = struct.pack("d", robot_number)
        emmiter.send(message)

    @staticmethod
    def receive_msg(receiver):
        '''
        Poluchaem soobshhenie ot soseda
        '''
        message = receiver.getData()
        return struct.unpack("d", message)
    
    @staticmethod
    def change_wheels_velocity(wheels, leftSpeed, rightSpeed):
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        wheels[2].setVelocity(leftSpeed)
        wheels[3].setVelocity(rightSpeed)
    
    @staticmethod
    def extract_robot_number(robot_name):
        idx1 = robot_name.find('(')
        if idx1 == -1:
            return 0
        idx2 = robot_name.find(')')

        return int(robot_name[idx1 + 1: idx2])

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    robot_name = robot.getName()
    robot_number = BotInternal.extract_robot_number(robot_name)
    ros_messager = RosMessaging(robot_number)

    robot_own_channel = 1 + robot_number

    print('Robot {0} configuration started'.format(robot_number))

    # get the time step of the current world.
    TIME_STEP = int(robot.getBasicTimeStep())
    print('Time step: {0} ms.'.format(TIME_STEP))

    # initialize motors
    wheels = []
    wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

    for i in range(4):
        wheels.append(robot.getDevice(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(0.0)


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

    # initialize emmiter
    emitter = robot.getDevice('emitter')

    # init receiver
    receiver = robot.getDevice('receiver')
    receiver.enable(TIME_STEP)

    # print('Robot {0} configuring is over'.format(robot_name))

    # Nachal'noe znachenie na dvigateli
    leftSpeed = 0
    rightSpeed = 0

    # Peremennye dlja zadanija obhoda prepjatstvija
    avoidObstacleCounter = 0
    p = 0  # choosing between left and right direction (0 - left, 1 - right)
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(TIME_STEP) != -1:

        print('----\nRobot {0} turn'.format(robot_number))

        # Raschityvaem azimut
        bearing = Calculations.calc_azimuth(com)

        # Ishhem maksimum iz datchikov
        light = []
        for i in range(4):
            light.append(ls[i].getValue())
            # print('Robot {0} light {1}: {2}'.format(robot_name, i, light[i]))

        light_max = max(light)

        print('Max light: {0}'.format(light_max))

        # Vvodim uverennost' v kurse q po datchikam sveta
        # datchiku napravlenija. a_q - kojeficient
        # d - pokazaija datchika distancii.
        a_q = 0.5  # beta v diplome?
        d = ds.getValue()
        q = Calculations.calc_q(light, a_q, d)

        # Peredaem soobshhenie sosedjam
        BotInternal.send_msg(emitter, robot_number)

        # Prinimaem soobshhenie
        neighbours = []
        while (receiver.getQueueLength() > 0):
            dataList = BotInternal.receive_msg(receiver)

            neighbours.append(dataList[0])

            receiver.nextPacket()

        # send robot metrics to JADE through ROS

        ros_messager.publish(robot_number, bearing, q, light, d, neighbours)

        # get previously calculated dbearingG from JADE through ROS

        dbearingG = ros_messager.get_azimuth()

        print('dbearingG = {0}'.format(dbearingG))

        if dbearingG == None:
            print('Stay still as no data received')
            # we can't change the velocity and direction if we don't have calculated dbearingG
            continue

        # Zadaem dvizhenie
        if bearing == dbearingG and light[0]+light[1]+light[2]+light[3] > 0:
            print('Keep forward')
            leftSpeed = 3.14
            rightSpeed = 3.14
        elif dbearingG > bearing and dbearingG < bearing + 180:
            print('Turn right')
            leftSpeed = 3.14
            rightSpeed = 2
        elif dbearingG > bearing and dbearingG > bearing + 180:
            print('Turn left')
            leftSpeed = 2
            rightSpeed = 3.14
        elif bearing > dbearingG and bearing < dbearingG + 180:
            print('Turn left')
            leftSpeed = 2
            rightSpeed = 3.14
        elif bearing > dbearingG and bearing > dbearingG + 180:
            print('Turn right')
            leftSpeed = 3.14
            rightSpeed = 2
        else:
            print('Stay still')
            leftSpeed = 0
            rightSpeed = 0

        # Obhod prepjatstvij
        if d <= 400 and avoidObstacleCounter == 0:
            avoidObstacleCounter = 1
            if light[0] == light[1] == light[2] == light[3]:
                p = randint(0, 1)
            elif light_max == light[0] or light_max == light[1]:
                p = 0  # pravo
            elif light_max == light[2] or light_max == light[3]:
                p = 1  # vlevo

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


        print('speeds:', leftSpeed, rightSpeed)
        # Otpravljaem znachenie na motory
        BotInternal.change_wheels_velocity(wheels, leftSpeed, rightSpeed)


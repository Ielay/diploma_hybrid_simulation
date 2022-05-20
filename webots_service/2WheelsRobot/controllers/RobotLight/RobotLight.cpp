#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cstring>



#define TIME_STEP 64
using namespace webots;
double leftSpeed;
double rightSpeed;

// Функии поворотов и движения
// Поворот вправо
void right()
  {
    leftSpeed = 3.14;
    rightSpeed = 2;
  }

// Поворот влево
void left()
  {
    leftSpeed = 2;
    rightSpeed = 3.14;
  }   

//Движение вперед
void forward()
  {
    leftSpeed = 3.14;
    rightSpeed = 3.14;
  } 

//Остановка
void stop()
  {
    leftSpeed = 0;
    rightSpeed = 0;
  }
  
//Вращение вокруг оси влево
void laround()
  {
    leftSpeed = -2;
    rightSpeed = 2;
  }

//Вращение вокруг оси вправо
void raround()
  {
    leftSpeed = 2;
    rightSpeed = -2;
  }
// Инициализация робота
int main(int argc, char **argv) {
  Robot *robot = new Robot();

  // Инициализация датчика дистанции
  DistanceSensor *ds = robot->getDistanceSensor("ds");
  ds->enable(TIME_STEP);
   
  // Инициализация компаса  
  Compass *com = robot->getCompass("com");
  com->enable(TIME_STEP);  
    
  // Инициализация четырех сенсоров света  
  LightSensor *ls[4];
  char lsNames[4][10] = {"ls1", "ls2", "ls3", "ls4"};
  for (int i = 0; i < 4; i++) {  
    ls[i] = robot->getLightSensor(lsNames[i]);
    ls[i]->enable(TIME_STEP); 
  }
  
  // Инициализация четырех моторов  
  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  //int robot_type;
  //if (robot -> getName() == "RL")
   // {
    //Инициализация передающих модулей связи
    //robot_type = 1;
    Emitter* trans = robot->getEmitter("trans1");
   // } 
  //if (robot -> getName() == "RL1")
   // {
    // Инициализация принимающих модулей связи
   // robot_type = 2;
    //Receiver *rec = robot->getReceiver("rec1");
    //rec->enable(TIME_STEP);
   // }
  //Задаем начальные значения для счетчиков
  int avoidObstacleCounter = 0;
  int j = 0;
  
  // Основной цикл
  while (robot->step(TIME_STEP) != -1) 
  {      
    //Расчитываем азимут.
    const double *north = com->getValues();
    double rad = atan2(north[0], north[2]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0) 
      {
          bearing = bearing + 360.0;
      } 
    // Ищем максимум из датчиков
    double light[4];
    for (int i = 0; i < 4; i++)
      {
          light[i] = ls[i]->getValue();
      }
    double max = light[0];
    for (int i = 1; i < 4; i++)
      {
         if (light[i] > max)
           {
             max = light[i];
           }
      }
      
      //std::cout << light[3] << "|"  << light[0] << std::endl;
      //std::cout << light[2] << "|"  << light[1] << std::endl;
// Расчет желаемого азимута движения на источник по четырем сенсорам света.
// Выбираем датчик с максимальным уровнем излучения. Сравниваем с соседними.
// Выбираем второй по мощности излучения датчик.
// Расчитываем доворо по часовой стролке до источника излучения.
// Расчитываем желаемый азимут.     
    double dbearing = bearing;
    if (max != 0)
      {
        double a = 0;
        double b = 0;                
        if (max == light[0])
          {
            a = light[0] - light[3]; 
            b = light[0] - light[1];
            if (a < b && light[3] != 0) 
              {
                dbearing = bearing - 45 + (light[0]*90)/(light[0]+light[3]);
              }
           else if (b < a && light[1] != 0)
              {
                dbearing = bearing + 45 + (light[1]*90)/(light[0]+light[1]);
              }
           else 
               {
                 dbearing = bearing + 45;
               }  
          } 
        else if (max == light[1])
          {
            a = light[1] - light[0]; 
            b = light[1] - light[2]; 
            if (a <= b && light[0] != 0) 
              {
                dbearing = bearing + 45 + (light[1]*90)/(light[0]+light[1]);
              }
            if (b < a && light[2] != 0)
              {
                dbearing = bearing + 135 + (light[2]*90)/(light[2]+light[1]);
              }
            else 
              {
                dbearing = bearing + 135;
              } 
          }
        else if (max == light[2])
          {
            a = light[2] - light[1]; 
            b = light[2] - light[3]; 
            if (a <= b && light[1] != 0) 
              {
                dbearing = bearing + 135 + (light[2]*90)/(light[2]+light[1]);
              }
            if (b < a && light[3] != 0)
              {
                dbearing = bearing + 225 + (light[3]*90)/(light[2]+light[3]);
              }
            else 
              {
                dbearing = bearing + 225;
              }   
          }
        else if (max == light[3])
          {
            a = light[3] - light[2]; 
            b = light[3] - light[0];
            if (a <= b && light[2] != 0) 
              {
                dbearing = bearing + 225 + (light[3]*90)/(light[2]+light[3]);
              }
            if (b < a && light[0] != 0)
              {
                dbearing = bearing + 315 + (light[0]*90)/(light[0]+light[3]);
              }
            else 
              {
                dbearing = bearing + 315;
              }   
          }                                     
        if (dbearing > 360)
          {
          dbearing = dbearing - 360;  
          }
      }
    
      //const double* message = &dbearing;
      double message[] = {dbearing};
      trans->send(message, 10);
      //const void* buffer = rec->getData();  
      //rec->nextPacket();
     std::cout << "emm" << message << "bearing" << dbearing << std::endl;
    // Main-loop
    if (bearing == dbearing && light[0]+light[1]+light[2]+light[3] > 0)
      {
        forward();
      }
    else if (dbearing > bearing && dbearing < bearing + 180)
      {
        right();
      }
    else if (dbearing > bearing && dbearing > bearing + 180) 
      {
        left();
      }
    else if (bearing > dbearing && bearing < dbearing + 180) 
      {
        left();
      }
    else if (bearing > dbearing && bearing > dbearing + 180) 
      {
        right();
      }
    else 
      {
        stop();
      }  
 // Обход препятствий
    double d = ds->getValue();
    if (d >= 1000 && avoidObstacleCounter == 0)
      {
        j = 0;
      }
    else if (d < 1000 && j == 0)
      { 
        j = rand ();
        avoidObstacleCounter = 70;
      }
    else if (d <= 400 && j != 0) 
      {
        if (j < 16383) 
          {
            raround();
          } 
        else 
          {
            laround();
          }
      }
    else if (avoidObstacleCounter != 0)  
      {
        avoidObstacleCounter --;
        if (j > 16383) 
          {
            left();
          } 
        else 
          {
            right();
          }
      }
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
    wheels[2]->setVelocity(leftSpeed);
    wheels[3]->setVelocity(rightSpeed);
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}
#include <ros.h>
#include <std_msgs/String.h>

#include <string>

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h> 

#ifndef STASSID
#define STASSID "hon"
#define STAPSK  "111123457"
#endif

// --- Настройки сети для роботов ---
const char* ssid = STASSID;             // SSID сети, к которой осуществляется подключение
const char* host="esp8266-webupdate";   // hosted host 
const char* password_firmware = STAPSK; // пароль сети, к которой осуществляется подключение
const char* appassword = "0000000001";  // пароль сети робота в режиме AP

const std::string robot_id = "2";
const int robot_id_int = 2;
IPAddress rosserial_host(192,168,12,1);
const uint16_t rosserial_port = 11412;

int number_robot = 502; //why...

// --- Web server for update purposes ---
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

const int ESP_BUILTIN_LED = 2; //Порт светодиода для моргания

// --- Константы алгоритма ---
const int n_neighbors = 10; //Количество соседей, из которых выбираем
const int n_neighbors_vibor = 5; //Количество соседей, которые выбираем случайно из n_neighbors
const float alpha = 0.3; //Коэффициент расчета курса группы

// --- Переменные алгоритма ---
int n = 50; //Счетчик для запроса сервера при обновлении
int new_azimuth = -1; // '-1' is initial value which could never be resulted during chosen LVP algorithm

// --- subscriber handler ---
void sub_callback(const std_msgs::String& azimuth_msg){
  new_azimuth = std::stoi(azimuth_msg.data);
}

// --- ROS-related vars ---
ros::NodeHandle nh;
std::string robot_topic = "/robot/" + robot_id + "/info";
std::string agent_topic = "/agent/" + robot_id + "/result";
// Make a chatter publisher
std_msgs::String str_msg; // object that is used for transferring any msg to ros
ros::Publisher chatter(robot_topic.c_str(), &str_msg); //chatter expects the trasferring msg to be in 'str_msg'; topic name is 'chatter'
ros::Subscriber<std_msgs::String> sub(agent_topic.c_str(), sub_callback);

void connectRos() {
  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(rosserial_host, rosserial_port);
  //nh.getHardware()->setBaud(9600)
  nh.initNode();

  nh.advertise(chatter);
  nh.subscribe(sub);
}

void setup() {
  Serial.begin(9600);
  //WiFi.mode(WIFI_AP_STA); // Ставим режим модуля в AP+STA
  //WiFi.softAP(apssid); // Start AP mode
  //WiFi.softAPConfig (local_ip, gateway, subnet); // Настройка адресации

  Serial.print("Connecting WiFi");
  WiFi.begin(ssid, password_firmware);
  delay(5000);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.print(".");
    WiFi.begin(ssid, password_firmware);

    //blinking with LED while waiting for WiFi connection
    digitalWrite(ESP_BUILTIN_LED, LOW);
    delay(500);
    digitalWrite(ESP_BUILTIN_LED, HIGH);
    delay(500);
  }

  Serial.println("Successful");

  //configure update service
  MDNS.begin(host); 

  httpUpdater.setup(&httpServer);
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
  pinMode(ESP_BUILTIN_LED, OUTPUT);

  //configure connection to ROS
  connectRos();
}

int processUpdateServer(int n) {
    //processing updater calls to update firmware
    httpServer.handleClient();
    MDNS.update();

    return n;
}

void transferDataToNeighbours(int prefix_code) {
    // Записываем данные для передачи соседям в имя сети
    //int apsid = prefix_code*10000 + (dbearing/100)*1000 + ((dbearing%100)/10)*100 + (dbearing%10)*10 + q; 
    int apsid = prefix_code*1000 + robot_id_int;
    String apsd = String(apsid); 
    const char* apssid = apsd.c_str();

    //Создаем сеть с новым названием
    WiFi.softAP(apssid, appassword); // Start AP mode
    //delay(1000);
}

std::string commaSeparatedStr(int neighbours_ids[n_neighbors], int neighbours_found) {
    std::string msg = "";
    for (int i = 0; i < neighbours_found; ++i) {
        msg += std::to_string(neighbours_ids[i]);
        if (i != neighbours_found - 1) {
            msg += ",";
        }
    }

    return msg;
}

void transferDataToROS(int dbearing, int q, int neighbours_ids[n_neighbors], int neighbours_found) {
  // change data of str_msg
  std::string msg_to_send = "i=" + robot_id +
                            ";b=" + std::to_string(dbearing) + 
                            ";q=" + std::to_string(q) +
                            ";n=" + commaSeparatedStr(neighbours_ids, neighbours_found);
  str_msg.data = msg_to_send.c_str();
  // publish str_msg
  chatter.publish( &str_msg );

  nh.spinOnce();
  delay(2000);
}

void loop() {
  if (n > 0) { //Если счетчик есть, то процессим запросы на обновление от клиента
    n = processUpdateServer(n);
  }
    
  //Считываем данные из буфера обмена с arduino (Serial)
  int dbearing = number_robot;
  int q = 1;
  int dbearingG; // результат пересчета курса от группы

  if (Serial.available() != 0){
    int i_main = 0;
    byte ardrec[64];
    while (Serial.available() > 0){
      ardrec[i_main] = Serial.read();
      i_main++;
    }
    //Получаем переменные для обработки и передачи
    dbearing = ardrec[i_main-4]*100 + ardrec[i_main - 3]*10 + ardrec[i_main-2];
    q = ardrec[i_main-1]; 

    transferDataToNeighbours(111); // 111 в начале названия - идентификатор необходимой сети.
  } else {
    transferDataToNeighbours(332); // 333 в начале названия - ошибка в сообщениях.
  }

  if (dbearing > 360) {
    dbearing -= 360;
  }
  
  //Сканируем сети
  int networksFound = WiFi.scanNetworks();

  //Смотрим список сетей и заполняем массив с данными о направлении, уверенности и уровнем сигнала
  int neighbours_ids[n_neighbors]; //найденные соседи робота
  int neighbours_found = 0; //количество найденных соседей
  int rssi; // мощность сигнала
  int ssid_name; //название сети с данными
  //Если сети есть, то начинаем считывать данные
  if (networksFound > 0){
    for (int net_idx = 0; net_idx < networksFound; net_idx++){
      rssi = WiFi.RSSI(net_idx);
      ssid_name = WiFi.SSID(net_idx).toInt();
      
      if (ssid_name/1000 == 111) { // check if it's some robot AP
        neighbours_ids[neighbours_found] = ssid_name%1000;
        neighbours_found++;
      }

      if (neighbours_found == n_neighbors){
        break;
      }
    }
  }

  transferDataToROS(dbearing, q, neighbours_ids, neighbours_found);

  if (new_azimuth != -1) {
    dbearingG = new_azimuth;
  } else {
    dbearingG = dbearing;
  }

  if (dbearingG > 360) {
    dbearingG -= 360;
  }

  //Преобразуем данные для передачи в arduino 
  byte esp_write[3];
  esp_write[0] = dbearingG/100;
  esp_write[1] = (dbearingG%100)/10;
  esp_write[2] = dbearingG%10;
  Serial.write(esp_write, 3);
}

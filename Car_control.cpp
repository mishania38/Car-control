
/*
Данная программа служит для реализации функции дистанционного запуска двигателя на микроконтроллере
Atmel AVR Mega 2560 в связке с GSM модемом SIM800
Обмен данными происходит через интернет по протоколу Mqtt
*/

#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <OneWire.h>


TinyGsm      modem(Serial2);         // Создаем объект TinyGsm(использующий gsm модем, на шине UART, пин 16,17)
TinyGsmClient client(modem);         
PubSubClient   mqtt(client);
OneWire         ds18b20(47);         // Создаем объект OneWire для шины I2C, с помощью которого будет осуществляться работа с датчиком температуры

/*------------------------------Порты вывода управляющего сигнала------------------*/
#define IGN_PIN         32           // пин зажигание
#define STARTER_PIN     33           // пин стартер
#define ACC_PIN         34           // пин ACC на замке зажигания / обходчик иммобилайзера
#define CAR_OP_PIN      35           // сигнал на отпирание
#define CAR_CL_PIN      36           // сигнал на запирание
#define HEAT_ENG_PIN    38           // сигнал на включение подогрева антифриза

/*------------------------------Порты ввода аналогового сигнала----------------------*/
#define TACH_PIN       A0            // Сигнал с тахометра
#define BAT_V          A2            // Напряжение аккумулятора

/*------------------------------Переменные хранения статуса-----------------------------*/
bool alarmOn = false;                      // Статус охраны
bool engineHeated = false;                 // Статус подогрева двигателя
bool engineOn = false;                     // Статус включения автозапуска
bool hasEngineStarted = false;             // Статус заведенного двигателя

                
/*------------------------------Переменные с таймерами----------------------------------*/
unsigned long whenEngineStarted = 0;       // Время, когда был запущен двигатель
unsigned long EngineWorkPeriod = 900000;   // Интервал, на который двигатель запускается
unsigned long lastMqttUpdate = millis();   // Время последнего обновления соединения
unsigned long lastTimerUpdate = 0;         
unsigned int _startTimer = 0;
unsigned int starterPeriod = 1200;         // Интервал включения стартера

/*------------------------------Переменные для авторизации на MQTT сервере--------------*/
const char* broker = "mqtt.by";          // Адрес сервера MQTT брокера
const char mqtt_user[] = "*******";      // Имя пользователя
const char mqtt_pass[] = "******";       // Пароль сервера MQTT брокера
const char mqtt_cid[] = "***";           // Уникальное имя устройства в сети MQTT
unsigned int PORT = 1883;                // Порт MQTT брокера
const char apn[]  = "internet.mts.by";   // Точка доступа в мобильный интернет
const char user[] = "";           
const char pass[] = "";

/*-----------------------------Топики-------------------------------------------------*/
const char startenginecom[] = "/user/38mishania/fiat/startenginecom";
const char startengine[] = "/user/38mishania/fiat/startengine";
const char alarmoncom[] = "/user/38mishania/fiat/alarmoncom";
const char alarmon[] = "/user/38mishania/fiat/alarmon";
const char heatenginecom[] = "/user/38mishania/fiat/heatenginecom";
const char heatengine[] = "/user/38mishania/fiat/heatengine";
const char refreshcom[] = "/user/38mishania/fiat/refreshcom";
const char batteryvolt[] = "/user/38mishania/fiat/batteryvolt";
const char startperiodcom[] = "/user/38mishania/fiat/startperiodcom";
const char startperiod[] = "/user/38mishania/fiat/startperiod";
const char starterperiodcom[] = "/user/38mishania/fiat/starterperiodcom";
const char starterperiod[] = "/user/38mishania/fiat/starterperiod";
const char cartemp[] = "/user/38mishania/fiat/cartemp";
const char totalerrorcount[] = "/user/38mishania/fiat/totalerrorcount";
const char rpminfo[] = "/user/38mishania/fiat/rpminfo";
const char rpmcom[] = "/user/38mishania/fiat/rpmcom";
const char startTimer[] = "/user/38mishania/fiat/starttimer";

String batteryVoltage = "";                 // Переменная для хранения напряжения аккумулятора
String tempCar = "-0";

int RPM = 300;                              // Пороговое значение заведенного двигателя (в попугаях, 0-1023)
int countNetError = 0;                      // Количество неудачных попыток коннекта (после 3-х рестарт модема и обнуление)
int totalcountNetError = 0;                 // Количество рестартов модема



bool mqttConnect()                          // Авторизация на MQTT сервере и подписка на топики
{
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  bool status = mqtt.connect(mqtt_cid, mqtt_user, mqtt_pass);

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" OK");
  checkStatus();
  mqtt.subscribe(startenginecom);
  mqtt.subscribe(alarmoncom);
  mqtt.subscribe(heatenginecom);
  mqtt.subscribe(refreshcom);
  mqtt.subscribe(startperiodcom);
  mqtt.subscribe(starterperiodcom);
  return mqtt.connected();
}

void checkStatus()                               // Сбор данных о состоянии автомобиля и отправка данных на сервер
{
  
  String _modemRestart = String(totalcountNetError);
  float _starterperiod = starterPeriod;
  
  
  batteryVoltage = "";                            // Замеряем напряжение бортсети
  float _volt = analogRead(BAT_V);
  _volt = _volt / 42.94;                          // переводим попугаи в вольты
  batteryVoltage += _volt;
  SerialMon.print("Voltage");
  SerialMon.println(batteryVoltage);
    
  byte data[2];                                                  // Переменная для хранения двух байт значения температуры
  ds18b20.reset(); ds18b20.write(0xCC); ds18b20.write(0x44);     // Делаем сброс всех предыдущих команд и параметров. Даем датчику DS18b20 команду измерить температуру.
  delay( 1000 );                                                 // Ждем пока датчик проведет измерения            
  ds18b20.reset(); ds18b20.write(0xCC); ds18b20.write(0xBE);
  data[0] = ds18b20.read(); data[1] = ds18b20.read();            // Получаем и считываем ответ. Читаем младший байт значения температуры. А потом старший
  float temperature =  ((data[1] << 8) | data[0]) * 0.0625;      // Формируем итоговое значение: - сперва "склеиваем" значение, - затем умножаем его на коэффициент

  tempCar = "";
  tempCar += temperature;       // Возвращаем полученное значение температуры
  SerialMon.print("Temp");
  SerialMon.println(tempCar);
  delay(100);

  mqtt.publish(startengine, (analogRead(TACH_PIN) >= RPM) ? "1" : "0");
  mqtt.publish(alarmon, alarmOn ? "1" : "0");
  mqtt.publish(batteryvolt, batteryVoltage.c_str());
  mqtt.publish(heatengine, engineHeated ? "1" : "0");
  mqtt.publish(cartemp, tempCar.c_str());
  mqtt.publish(totalerrorcount, (String(totalcountNetError)).c_str());

  mqtt.publish(startperiod, (String(EngineWorkPeriod / 60000)).c_str());
  mqtt.publish(starterperiod, (String(_starterperiod / 1000)).c_str());
  mqtt.publish(rpminfo, (String(analogRead(TACH_PIN))).c_str());
  mqtt.publish(startTimer, (String(_startTimer)).c_str());
  mqtt.publish(startTimer, (String(_startTimer)).c_str());
}

void modemInit()                                 // Инициализация модема при старте
{
  SerialMon.println("Initializing modem...");
  modem.restart();                               //Перезагрузка модема
  totalcountNetError++;

  String modemInfo = modem.getModemInfo();       //Получение данных о модеме
  SerialMon.print("Modem: ");
  SerialMon.println(modemInfo);

  SerialMon.print("Waiting for network...");     //Подключение к сети оператора
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    while (true);
  }
  SerialMon.println(" OK");

  SerialMon.print("Connecting to ");             //Подключение к интернету
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println(" fail");
    while (true);
  }
  SerialMon.println(" OK");
  
  
  mqtt.setServer(broker, PORT);                  //Настройки брокера MQTT
  mqtt.setCallback(mqttCallback);

}

void mqttCallback(char* topic, byte* payload, unsigned int len)
{
  String _topic = String(topic);
  String _val = "";
  
  for (unsigned int i = 0; i != len; i++){
    _val += char(*(payload + i));
  }
  
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();
  SerialMon.println(_topic + _val);


  if (_topic.indexOf("startenginecom") > -1) {
    if (_val.indexOf("1") > -1) {startEngine(true, 5000);}
    if (_val.indexOf("0") > -1) {stopEngine();}
  }

  else if (_topic.indexOf("alarmoncom") > -1) {
    if (_val.indexOf("1") > -1) {carClose();}
    if (_val.indexOf("0") > -1) {carOpen();}
  }

  else if (_topic.indexOf("heatenginecom") > -1) {
    if (_val.indexOf("1") > -1) {engineHeat(true);}
    if (_val.indexOf("0") > -1) {engineHeat(false);}
  }
  
  else if (_topic.indexOf("refreshcom") > - 1) {
  }

  else if (_topic.indexOf("startperiodcom") > - 1) {
    EngineWorkPeriod = _val.toInt();
    EngineWorkPeriod *= 60000;
    SerialMon.println(_val);
  }

  else if (_topic.indexOf("starterperiodcom") > - 1) {
    float value;
    value = _val.toFloat();
    value *= 1000;
    starterPeriod = value;
    SerialMon.println(starterPeriod);
  }
  
  checkStatus();
}

void allPinOff ()
{
  int i;
  for (i = 32; i <= 38; i++) {
    digitalWrite(i, LOW);
  }
  for (i = 39; i <= 45; i++) {
    digitalWrite(i, HIGH);
  }

}
/*
void startEngine(bool onTimer)        // Запускаем двигатель
{
  if (analogRead(TACH_PIN) <= RPM) {                  //Проверяем заведен ли двигатель
    int i;
    int timeIgn = 3000;
    int timeSt = starterPeriod;
    
    for (i = 1; i <= 3; i++) {                        //3 попытки запуска

      digitalWrite(IGN_PIN, HIGH);                    //Включаем зажигание,
      digitalWrite(ACC_PIN, HIGH);                    //Включаем аксесуары
      
      delay( timeIgn );                               //Ждем прогрев свечей накала 3/5/7 секунд
      
      digitalWrite(STARTER_PIN, HIGH);                //Включаем стартер, на установленное время
      delay( timeSt );                                //Время прокрута стартером
      digitalWrite(STARTER_PIN, LOW);                 //Выключаем стартер
      
      delay( 500 );
      
      if (analogRead(TACH_PIN) >= RPM) {              //Проверяем по тахометру, завелся ли двигатель
        if (onTimer && alarmOn) {                     //Если стоял флаг таймера и авто закрыт,
          whenEngineStarted = millis();               //то сбрасываем счетчик
          lastTimerUpdate = millis();
          engineOn = true;                            //и ставим флаг активного таймера на автозапуск
        }
        else {digitalWrite(BTN_LED_PIN, HIGH);}       
      break;
      }
      
      else {                                          //Если же двигатель не завелся, то:
        timeIgn += 2000;                              //Увеличиваем время накала свечей
        timeSt += 500;                                //Увеличиваем время вращения стартером
        digitalWrite(IGN_PIN, LOW);
        digitalWrite(ACC_PIN, LOW);
        delay(500);
      }
    }
  }
}
*/
void startEngine(bool onTimer, int maxInterval)        // Запускаем двигатель
{
  if (analogRead(TACH_PIN) <= RPM) {                  //Проверяем заведен ли двигатель
    int i;
    int timeIgn = 3000;
    SerialMon.println("Запуск двигателя!");
    //int timeSt = starterPeriod;
    
    digitalWrite(IGN_PIN, LOW);
    digitalWrite(ACC_PIN, LOW);

    for (i = 1; i <= 3; i++) {
      SerialMon.print("Попытка №");
      SerialMon.println(i);
      int t = 0;
      
      digitalWrite(IGN_PIN, HIGH);                    //Включаем зажигание,
      digitalWrite(ACC_PIN, HIGH);                    //Включаем аксесуары
      
      delay( timeIgn );                               //Ждем прогрев свечей 3/5/7 секунд
      
      digitalWrite(STARTER_PIN, HIGH);                //Включаем стартер, на установленное время
      SerialMon.println("Включение стартера");
      
      while((analogRead(TACH_PIN) <= RPM) && (t <= maxInterval))
      {
        delay(50);
        t += 50;
        SerialMon.println(t);
      }
      
      digitalWrite(STARTER_PIN, LOW);                 //Выключаем стартер
      SerialMon.println("Выключение стартера");
      delay( 1000 );
      
      if (analogRead(TACH_PIN) >= RPM) {              //Проверяем по тахометру, завелся ли двигатель
        if (onTimer && alarmOn) {                     //Если стоял флаг таймера и авто под охраной,
          whenEngineStarted = millis();               //то сбрасываем счетчик
          lastTimerUpdate = millis();
          engineOn = true;                            //и ставим флаг активного таймера на автозапуск
        }       
        break;
      }
      
      else {                                          //Если же двигатель не завелся, то:
        timeIgn += 2000;                              //Увеличиваем время накала свечей
        digitalWrite(IGN_PIN, LOW);
        digitalWrite(ACC_PIN, LOW);
        delay(1000);
        SerialMon.println("Двигатель не запущен!");
      }
    }
  }
}

void stopEngine()                                // Глушим двигатель
{
  digitalWrite(IGN_PIN, LOW);                    //Выключаем зажигание
  digitalWrite(ACC_PIN, LOW);                    //Выключаем ACC
  engineHeat(false);                             //Выключаем подогрев двигателя
  whenEngineStarted = 0;                         //Сбрасываем таймер
  engineOn = false;                              //Сбрасываем флаг дистанционного запуска
}

void carOpen()                                   // Открываем автомобиль
{
  if (alarmOn) {
    if (engineOn) {                              // Если до открытия автомобиля, двигатель был запущен на прогрев,
      engineOn = false;                          // то оставляем его работающим. Деактивировав таймер.
    }
    digitalWrite(CAR_OP_PIN, HIGH);             
    delay( 500 );
    digitalWrite(CAR_OP_PIN, LOW);
    alarmOn = false;

  }
}

void carClose()                                  // Закрываем автомобиль
{
  if (alarmOn == false) {
    if (analogRead(TACH_PIN) >= RPM) {           // Если двигатель был запущен то включаем таймер. 
      engineOn = true;                           // По истечении которого глушим двигатель
    }
    digitalWrite(CAR_CL_PIN, HIGH);
    delay( 500 );
    digitalWrite(CAR_CL_PIN, LOW);
    alarmOn = true;
    SerialMon.println("Car Close");
  }
}

void engineHeat(bool on)                         // Включаем/выключаем подогрев двигателя
{
  if ((analogRead(TACH_PIN) >= RPM) && on) {
    digitalWrite(HEAT_ENG_PIN, HIGH);
    engineHeated = true;
  }
  else {
    digitalWrite(HEAT_ENG_PIN, LOW);
    engineHeated = false;
  }
}

void MqttThread()
{
    if (engineOn) {
      if (whenEngineStarted + EngineWorkPeriod < millis()) {
        whenEngineStarted = 0;
        stopEngine();
        _startTimer = 0;
      }
      if (lastTimerUpdate + 30000 < millis()) {
        _startTimer = (whenEngineStarted + EngineWorkPeriod - millis())/60000;
        mqtt.publish(startTimer, (String(_startTimer)).c_str());
        lastTimerUpdate = millis();
      }
    }
    
    if (countNetError > 2) {
      SerialMon.println("Count errors: " + String(countNetError));
      countNetError = 0;
      modemInit();
    }

    if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    countNetError++;
    unsigned long t = millis();
    if (t - lastMqttUpdate > 10000L) {
      SerialMon.println("check connect");
      lastMqttUpdate = t;
      if (mqttConnect()) {
        countNetError = 0;
        lastMqttUpdate = 0;
      }
    }
    delay(100);
    return;
    }
    mqtt.loop();
}


void setup() {
  pinMode(TACH_PIN,      INPUT);
  pinMode(BAT_V,         INPUT);
  analogReference(DEFAULT);         //Устанавливаем диапазон напряжений на АЦП. 0-5v
  
  
  pinMode(IGN_PIN,      OUTPUT);
  pinMode(STARTER_PIN,  OUTPUT);
  pinMode(ACC_PIN,      OUTPUT);
  pinMode(CAR_OP_PIN,   OUTPUT);
  pinMode(CAR_CL_PIN,   OUTPUT);
  pinMode(HEAT_ENG_PIN,   OUTPUT);

  SerialMon.begin(115200);
  Serial2.begin(115200);

  allPinOff();
  modemInit();
}

void loop() 
{

  MqttThread();

}

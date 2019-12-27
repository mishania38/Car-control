

/*
Serial2 - физический последовательный порт, на котором висит GSM модем. Пин 16 и 17.
begin("speed") Инициализация порта, задаем скорость
end()          Закрытие порта
available()    Функция получает количество байт доступных для чтения. Это те байты которые уже записаны в буфер. Буфер может хранить до 64 байт.
read()         Cчитывает очередной доступный байт из буфера последовательного соединения.
print()        Передает данные через порт как ASCII текст.
println()      Передает данные как ASCII текст с следующим за ним символом переноса строки (ASCII символ 13 или '\r') и символом новой строки (ASCII 10 или '\n').
write()        Функция передает данные как бинарный код. Данные послаются как один или серия байтов.
peek()         Возвращает следующий доступный байт (символ) из буфера входящего последовательно соединения, не удаляя его из этого буфера.


*/
#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <OneWire.h>



/*------------------------------Создаем объекты библиотечных классов-------------*/
TinyGsm      modem(Serial2);         // Создаем объект класса TinyGsm(использующий gsm модем, на шине UART, пин 16,17)
TinyGsmClient client(modem);
PubSubClient   mqtt(client);
OneWire         ds18b20(47);         // Создаем объект класса OneWire для шины I2C, с помощью которого будет осуществляться работа с датчиком температуры

/*------------------------------Объявляем задачи---------------------------------*/


/*------------------------------Порты вывода ULN2003-32-38-----------------------*/
#define IGN_PIN         32           // пин зажигание
#define STARTER_PIN     33           // пин стартер
#define ACC_PIN         34           // пин ACC
#define CAR_OP_PIN      35           // сигнал на отпирание
#define CAR_CL_PIN      36           // сигнал на запирание
#define BTN_LED_PIN     37           // Сигнал на подсветку кнопки Старт-Стоп
#define HEAT_ENG_PIN    38           // сигнал на обогрев двигателя

/*------------------------------Порты вывода BD140-39-45-------------------------*/

#define HEAT_GL_PIN     40           // сигнал на обогрев зеркал
#define TRUNK_OP_PIN    41           // Выход на открытие багажника
#define HORN_PIN        42           // Выход на сирену
#define BLINK_PIN       43           // Выход на аварийку


/*------------------------------Порты ввода аналогового сигнала------------------*/
#define TACH_PIN       A0            // Сигнал с тахометра
#define START_BTN      A1            // Сигнал с кнопки Старт/Стоп
#define BAT_V          A2            // Напряжение аккумулятора
#define DOOR_PIN       A3            // Сигнал с концевиков дверей
#define HOOD_PIN       A4            // Сигнал с концевиков капота и багажника
#define BRAKE_PIN      A5            // Сигнал с концевика педали тормоза
#define HANDBRAKE_PIN  A6            // Сигнал с концевика ручника
#define IGN_IN_PIN     A9            // Сигнал включенного зажигания

/*------------------------------Переменные хранения статуса-----------------------------*/
bool alarmOn = false;                      // Статус охраны
bool engineHeated = false;                 // Статус подогрева двигателя
bool glassHeated = false;                  // Статус подогрева стекол
bool engineOn = false;                     // Статус автозапуска
bool silentMode = false;                   // Режим без звука

bool hasEngineStarted = false;             // Статус заведенного двигателя
bool hasStartButtonClicked = false;        // Статус нажатой кнопки старт-стоп
bool hasBrakeClicked = false;              // Статус активированного ручника
bool hasIgnitionOn = false;                // Статус включенного зажигания
bool hasDoorOpen = false;                  // Статус концевиков дверей
bool hasHoodOpen = false;                  // Статус концевиков капота и багажника

byte blinkBtnstate = 0;                    

/*------------------------------Переменные с таймерами----------------------------------*/
unsigned long whenEngineStarted = 0;       // Время, когда был запущен двигатель
unsigned long EngineWorkPeriod = 900000;   // Интервал, на который двигатель запускается
unsigned long lastMqttUpdate = millis();   // Время последнего обновления
unsigned long lastBtnBlink = millis();
//unsigned int updateMqttPeriod   = 10000;   // Период переподключения к серверу
unsigned int starterPeriod = 1200;         // Интервал включения стартера
unsigned int blinkBtnPeriod = 1000;        // Интервал мигания подсветки кнопки Старт-стоп

/*------------------------------Переменные для авторизации на MQTT сервере--------------*/
const char* broker = "m24.cloudmqtt.com";   // Адрес сервера MQTT брокера
const char mqtt_user[10] = "nsbmespx";      // Имя пользователя
const char mqtt_pass[15] = "1HTighZdc_RU";  // Пароль сервера MQTT брокера
const char mqtt_cid[15] = "fiat";           // Уникальное имя устройства в сети MQTT
unsigned int PORT = 19793;                  // Порт MQTT брокера НЕ SSL !
const char apn[]  = "internet.life.com.by"; // Точка доступа в мобильный интернет
const char user[] = "";
const char pass[] = "";

String adminPhone = "+375297907775";        // Номер телефона администратора
String batteryVoltage = "";                 // Переменная для хранения напряжения аккумулятора
String tempCar = "-0";

int RPM = 300;                     // Пороговое значение заведенного двигателя (в попугаях)
int countNetError = 0;                      // Количество неудачных попыток коннекта (после 3-х рестарт модема и обнуление)
int totalcountNetError = 0;                 // Количество рестартов модема

bool mqttConnect();
void checkStatus();
void modemInit();
void mqttCallback(char* topic, byte* payload, unsigned int len);
void allPinOff ();
void startEngine(bool onTimer, int count);
void stopEngine();
void carOpen();
void carClose();
void engineHeat(bool on);
void glassHeat(bool on);
void DetectionThread();
void StartStopThread();
void MqttThread();

bool mqttConnect()                               // Авторизация на MQTT сервере и подписка на топики
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
  mqtt.subscribe("startenginecom");
  mqtt.subscribe("alarmoncom");
  mqtt.subscribe("heatenginecom");
  mqtt.subscribe("refreshcom");
  mqtt.subscribe("startperiod");
  return mqtt.connected();
  
}

void checkStatus()                               // Сбор данных о состоянии автомобиля и отправка данных на сервер
{
  DetectionThread();
  String state = "";
  String _modemRestart = String(totalcountNetError);

    batteryVoltage = "";                            // Замеряем напряжение бортсети
    float _volt = analogRead(BAT_V);
    _volt = _volt / 42.94;                          // переводим попугаи в вольты
    batteryVoltage += _volt;
    SerialMon.print("Voltage");
    SerialMon.println(batteryVoltage);

    byte data[2];             // Переменная для хранения двух байт значения температуры
    ds18b20.reset(); ds18b20.write(0xCC); ds18b20.write(0x44);     // Делаем сброс всех предыдущих команд и параметров. Даем датчику DS18b20 команду измерить температуру.
    delay( 1000 );                       // Ждем пока датчик проведет измерения            
    ds18b20.reset(); ds18b20.write(0xCC); ds18b20.write(0xBE);
    data[0] = ds18b20.read(); data[1] = ds18b20.read();            // Получаем и считываем ответ. Читаем младший байт значения температуры. А потом старший
    float temperature =  ((data[1] << 8) | data[0]) * 0.0625;      // Формируем итоговое значение: - сперва "склеиваем" значение, - затем умножаем его на коэффициент

    tempCar = "";
    tempCar += temperature;       // Возвращаем полученное значение температуры
    SerialMon.print("Temp");
    SerialMon.println(tempCar);
    delay(100);

  state += ("Start:" + String(analogRead(START_BTN)) + " ,");
  state += ("Break:" + String(analogRead(BRAKE_PIN)) + " ,");
  state += ("DOOR:" + String(analogRead(DOOR_PIN)) + " ,");
  state += ("error:" + String(countNetError) + ".");
  
  mqtt.publish("startengine", hasEngineStarted ? "1" : "0");
  mqtt.publish("alarmon", alarmOn ? "1" : "0");
  mqtt.publish("batteryvolt", batteryVoltage.c_str());
  mqtt.publish("heatengine", engineHeated ? "1" : "0");
  mqtt.publish("status", state.c_str());
  mqtt.publish("cartemp", tempCar.c_str());
  mqtt.publish("totalerrorcount", _modemRestart.c_str());
  mqtt.publish("dooropen", hasDoorOpen ? "1" : "0");
  mqtt.publish("hoodopen", hasHoodOpen ? "1" : "0");
  mqtt.publish("brake", hasBrakeClicked ? "1" : "0");
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
  
  

  if (_topic.indexOf("startenginecom") > -1) {
    if (_val.indexOf("1") > -1) {startEngine(true, 1);}
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

  else if (_topic.indexOf("startperiod") > - 1) {
    EngineWorkPeriod = _val.toInt();
    EngineWorkPeriod *= 60000;
    SerialMon.println("Start period is " + String(EngineWorkPeriod));
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

void startEngine(bool onTimer, int count)        // Запускаем двигатель
{
  if (analogRead(TACH_PIN) <= RPM) {     //Проверяем заведен ли двигатель
    int i;
    int timeIgn = 3000;
    int timeSt = starterPeriod;
    
    digitalWrite(IGN_PIN, LOW);
    digitalWrite(ACC_PIN, LOW);
    
    for (i = count; i <= 3 && (analogRead(TACH_PIN) <= RPM); i++) {

      digitalWrite(IGN_PIN, HIGH);                    //Включаем зажигание,
      digitalWrite(ACC_PIN, HIGH);                    //Включаем аксесуары
      
      delay( timeIgn );     //Ждем прогрев свечей 3/5/7 секунд
      
      digitalWrite(STARTER_PIN, HIGH);                //Включаем стартер, на установленное время
      delay( timeSt );      //Время прокрута стартером
      digitalWrite(STARTER_PIN, LOW);                 //Выключаем стартер
      

      delay( 1000 );
      
      if (analogRead(TACH_PIN) >= RPM) {              //Проверяем по тахометру, завелся ли двигатель
        hasEngineStarted = true;                      //Ставим флаг об успешном запуске
        if (onTimer && alarmOn) {                     //Если стоял флаг таймера и авто под охраной,
          whenEngineStarted = millis();               //то сбрасываем счетчик
          engineOn = true;                            //и ставим флаг активного таймера на автозапуск
        }
        else {digitalWrite(BTN_LED_PIN, HIGH);}       
      }
      
      else {                                          //Если же двигатель не завелся, то:
        timeIgn += 2000;                              //Увеличиваем время накала свечей
        timeSt += 500;                                //Увеличиваем время вращения стартером
        digitalWrite(IGN_PIN, LOW);
        digitalWrite(ACC_PIN, LOW);
        delay(1000);
      }
    }
  }
  DetectionThread();
}

void stopEngine()                                // Глушим двигатель
{
  digitalWrite(IGN_PIN, LOW);                    //Выключаем зажигание
  digitalWrite(ACC_PIN, LOW);                    //Выключаем ACC
  digitalWrite(BTN_LED_PIN, LOW);                //Выключаем подсветку кнопки старт-стоп
  engineHeat(false);                             //Выключаем подогрев двигателя
  glassHeat(false);                              //Выключаем подогрев зеркал
  whenEngineStarted = 0;                         //Сбрасываем таймер
  engineOn = false;                              //Сбрасываем флаг дистанционного запуска
}

void carOpen()                                   // Открываем автомобиль
{
  if (alarmOn) {
    if (engineOn) {                              // Если до открытия автомобиля, двигатель был запущен на прогрев,
      engineOn = false;                          // то оставляем его работающим. Деактивировав таймер.
      digitalWrite(BTN_LED_PIN, HIGH);           // Оставляя постоянно гореть подсветку на кнопке
    }
    digitalWrite(CAR_OP_PIN, HIGH);
    digitalWrite(BLINK_PIN, LOW);
    
    delay( 500 );
    digitalWrite(CAR_OP_PIN, LOW);
    alarmOn = false;
    delay( 2500 );
    digitalWrite(BLINK_PIN, HIGH);

  }
}

void carClose()                                  // Закрываем автомобиль
{
  if (alarmOn == false) {
    if (analogRead(TACH_PIN) >= RPM) {           // Если двигатель был запущен
      stopEngine();
    }
    digitalWrite(CAR_CL_PIN, HIGH);
    delay( 500 );
    digitalWrite(CAR_CL_PIN, LOW);
    digitalWrite(BTN_LED_PIN, LOW);
    alarmOn = true;
    SerialMon.println("Car Close");
  }
}

void engineHeat(bool on)                         // Включаем/выключаем подогрев двигателя
{
  if (hasEngineStarted && on) {
    digitalWrite(HEAT_ENG_PIN, HIGH);
    engineHeated = true;
  }
  else {
    digitalWrite(HEAT_ENG_PIN, LOW);
    engineHeated = false;
  }
}

void glassHeat(bool on)                          // Включаем/выключаем подогрев зеркал
{
  if (hasEngineStarted && on) {
    digitalWrite(HEAT_GL_PIN, LOW);
    glassHeated = true;
  }
  else {
    digitalWrite(HEAT_GL_PIN, HIGH);
    glassHeated = false;
  }
}

void DetectionThread()         // Задача-поток считывания показаний параметров автомобиля
{
      hasEngineStarted = (analogRead(TACH_PIN) >= RPM) ? true : false;
      hasStartButtonClicked = (analogRead(START_BTN) >= RPM) ? true : false;
      hasBrakeClicked = (analogRead(BRAKE_PIN) <= 100) ? true : false;
      hasIgnitionOn = (analogRead(IGN_IN_PIN) >= RPM) ? true : false;
      hasDoorOpen = (analogRead(DOOR_PIN) <= 100) ? true : false;
      hasHoodOpen = (analogRead(HOOD_PIN) <= 100) ? true : false;
}

void StartStopThread()         // Задача-поток отслеживания нажатий кнопки старт-стоп, таймер прогрева двигателя,
{                                                // мигание подсветкой кнопки старт-стоп.

    if (engineOn) {
      if (whenEngineStarted + EngineWorkPeriod < millis()) {
        whenEngineStarted = 0;
        stopEngine();
        checkStatus(); 
      }
    }
  
    if (engineOn || (!alarmOn && !hasEngineStarted)){           // Если двигатель не запущен или в режиме прогрева,
      if (lastBtnBlink + blinkBtnPeriod < millis()) {           // то мигаем подсветкой кнопки Старт-стоп,
        lastBtnBlink = millis();                                // в ожидании нажатия на кнопку запуска
        digitalWrite(BTN_LED_PIN, blinkBtnstate);
        blinkBtnstate = !blinkBtnstate;
      }
    }
  
    if (engineOn && !hasEngineStarted){                         // Если во время прогрева двигатель заглох,
      stopEngine();                                             // то выключаем зажигание
    }
  
    if (!alarmOn) {                                             // Если авто снят с охраны
      if (!engineOn && !hasEngineStarted){                      // И двигатель не запущен
        if (hasStartButtonClicked){                             // Проверяем нажата ли кнопка старт-стоп
          digitalWrite(BTN_LED_PIN, HIGH);                      
          startEngine(false, 3);                                // Только тогда заводим двигатель, отключив таймер
        }
      }
      else if (hasEngineStarted && hasStartButtonClicked) {     // Если двигатель был запущен и нажали кнопку старт-стоп
        stopEngine();                                           // то глушим двигатель
      }
    }
}

void MqttThread()
{
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


void setup() 
{
  pinMode(TACH_PIN,      INPUT);
  pinMode(START_BTN,     INPUT);
  pinMode(BAT_V,         INPUT);
  pinMode(DOOR_PIN,      INPUT);
  pinMode(HOOD_PIN,      INPUT);
  pinMode(BRAKE_PIN,     INPUT);
  pinMode(HANDBRAKE_PIN, INPUT);
  analogReference(DEFAULT);         //Устанавливаем диапазон напряжений на АЦП. 0-5v
  
  
  pinMode(IGN_PIN,      OUTPUT);
  pinMode(STARTER_PIN,  OUTPUT);
  pinMode(ACC_PIN,      OUTPUT);
  pinMode(CAR_OP_PIN,   OUTPUT);
  pinMode(CAR_CL_PIN,   OUTPUT);

  pinMode(HEAT_ENG_PIN, OUTPUT);
  pinMode(HEAT_GL_PIN,  OUTPUT);
  pinMode(TRUNK_OP_PIN, OUTPUT);
  pinMode(HORN_PIN,     OUTPUT);
  pinMode(BLINK_PIN,    OUTPUT);
  pinMode(BTN_LED_PIN,  OUTPUT);

  SerialMon.begin(9600);
  Serial2.begin(57600);

  allPinOff();
  modemInit();
}

void loop() 
{
  StartStopThread();
  //DetectionThread();
  MqttThread();

}
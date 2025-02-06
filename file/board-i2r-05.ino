// Boards Manager 버젼은 esp32 by Espressif System 3.1.1 으로 개발했어요
// 버젼을 3.1.1으로 하세요
#include <WiFi.h>
#include <PubSubClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include <FS.h>
#include <Wire.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include "esp_system.h"  // 메모리 체
#include <Adafruit_NeoPixel.h>
#include "DHT.h"  //DHT sensor library by Adafruit
#include <map>
#include <vector>

#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>

// 전역 변수: 센서 종류
const String sensorTypes[] = {"temp", "humi", "light", "analog"};
const int numTypes = sizeof(sensorTypes) / sizeof(sensorTypes[0]); // 배열 크기 계산
// Output pin numbers
const int outputPins[2] = {38, 47};
// Input pin numbers
const int inputPins[2] = {8, 9};
const int numberOfPins = sizeof(outputPins) / sizeof(outputPins[0]);
#define TRIGGER_PIN 41 // Factory Reset trigger pin 
#define ledPin 48      // RGB LED가 연결된 핀 번호
#define NUM_PIXELS 1    // 제어할 LED 개수
//const int ledPin = 48;  // RGB Led
#define DHTPIN 10       // DHT 센서가 연결된 핀 번호
#define DHTTYPE DHT11   // 사용하는 센서의 유형 (DHT11)
#define LIGHT_SENSOR_PIN 2  // Light 센서가 연결된 핀
#define ANALOG_PIN 1  // 아날로그 입력 핀 번호

DHT dht(DHTPIN, DHTTYPE); // DHT 센서 객체 생성
Adafruit_NeoPixel strip(NUM_PIXELS, ledPin, NEO_GRB + NEO_KHZ800);
std::vector<String> measurementTypes;  // measurementTypes 벡터 선언

// Define the Data structure
struct PinStateChange {
  //bool portState; // 상태 변경 여부 (true: ON, false: OFF)
  String mac;
  int port;
  bool value;
  unsigned long timestamp; // 상태 변경 시간
};

struct SensorChange {
  float value;
  String mac;
  int port;
  bool state;
  unsigned long timestamp; // 상태 변경 시간
};

struct Device {
public:
  int type = 5;
  int shield = 1;
  String mac=""; // Bluetooth mac address 를 기기 인식 id로 사용한다.
  unsigned long lastTime = 0;  // 마지막으로 코드가 실행된 시간을 기록할 변수
  const long interval = 5000;  // 실행 간격을 밀리초 단위로 설정 (5초)
  int out[numberOfPins];
  int in[numberOfPins];
  String sendData="",sendDataPre=""; // 보드의 입력,출려,전압 데이터를 json 형태로 저장
  void checkFactoryDefault();
  void loop();
  void sendStatusCheckChange(bool dataChange); // 현재 상태를 전송하는 함수
  void sendOut(int port, bool portState); // 핀 상태를 MQTT로 전송하는 함수
  std::vector<PinStateChange> pinStateChanges[numberOfPins+2][2]; // 포트당 2개의 상태 변경 내역 저장 (0=false,1=true)
  // temp:numberOfPins+2  humi:numberOfPins+2
  void loadPinStatesFromSPIFFS();
  void digitalWriteUpdateData(int pin, bool value);
  // RGB 색상 설정 함수
  void setColor(uint8_t red, uint8_t green, uint8_t blue) {
    strip.setPixelColor(0, strip.Color(red, green, blue)); // 첫 번째 LED의 색상 설정
    strip.show();                                         // 설정한 색상을 출력
  };
} dev;

struct Ble {
public:
  char *service_uuid="4fafc201-1fb5-459e-8fcc-c5c9c331914b";
  char *characteristic_uuid="beb5483e-36e1-4688-b7f5-ea07361b26a8";
  bool boot=false;
  bool isConnected=false;
  void setup();
  void readBleMacAddress();
  void writeToBle(int order);
} ble;

// Global variable to store program start time
unsigned long programStartTime;

WiFiClient espClient;
PubSubClient client(espClient);

struct WifiMqtt {
public:
  bool selectMqtt=false;
  bool isConnected=false;
  bool isConnectedMqtt=false;
  String ssid="";
  String password="";
  String email="";
  String mqttBroker = ""; // 브로커 
  char outTopic[50]="i2r/"; 
  char inTopic[50]="i2r/";  
  unsigned long statusSendCounter = 0; // MQTT 연결 시 상태 전송 횟수를 기록할 변수
  unsigned long startupTime; // 프로그램 시작 시간
  const unsigned long ignoreDuration = 5000; // 무시할 시간 (밀리초 단위), 예: 5000ms = 5초

  void loop();
  void connectToWiFi();
  void publishMqtt();
  void reconnectMQTT();
  void readWifiMacAddress();
};

struct Sensor {
public:
  std::map<String, float> sensorData;
  std::map<String, float> sensorLastData;
  std::map<String, float> calSensorData;
  std::map<String, std::vector<SensorChange>> SensorChanges;
  std::vector<String> measurementTypes;  // 벡터 선언
 
  unsigned long lastTimeSensor = 0;
  const long intervalSensor = 5000;

  void sendOutSensorChange(String type, bool rise);
  void saveToSPIFFS();
  void loadFromSPIFFS();
  void printSensorData();

  // 생성자에서 전역 변수 sensorTypes를 measurementTypes에 복사
  Sensor() {
    measurementTypes.assign(sensorTypes, sensorTypes + numTypes);  // 값 할당
    for(int i = 0; i < numTypes; i++) {
      SensorChanges[sensorTypes[i]].push_back({-10000, "00:00:00:00:00:00", -1, false, millis()});
      SensorChanges[sensorTypes[i]].push_back({10000, "00:00:00:00:00:00", -1, false, millis()});
    }
    this->loadFromSPIFFS();
  }

  void setup() {
    // 온습도 센서를 사용할 때만 초기화
    dht.begin();
    //this->printSensorData();
  }

  void loop() {
    unsigned long currentTime = millis();
    if (currentTime - this->lastTimeSensor >= this->intervalSensor) {
      this->lastTimeSensor = currentTime;
      this->measure();
      dev.sendStatusCheckChange(true);
      for (const auto& type : measurementTypes) {
        SensorChange changeLow = SensorChanges[type][0]; // 참조로 가져오기
        SensorChange changeHigh = SensorChanges[type][1]; // 참조로 가져오기
        if (sensorData[type] >= changeHigh.value && sensorLastData[type] < changeHigh.value) {
          this->sendOutSensorChange(type, true);
        }
        if (sensorData[type] <= changeLow.value && sensorLastData[type] > changeLow.value) {
          this->sendOutSensorChange(type, false);
        }
      }
    }
  }

  void measure() {
    for (const auto& type : measurementTypes) {
      sensorLastData[type] = sensorData[type];
      if(type == "temp") {
        //sensorData[type] = dht.readTemperature();
        sensorData[type] = round(dht.readTemperature() * 10) / 10.0;
      }
      else if(type == "humi")
        sensorData[type] = int(dht.readHumidity());
      else if(type == "light")
        sensorData[type] = int(analogRead(LIGHT_SENSOR_PIN));
      else if(type == "analog")
        sensorData[type] = int(analogRead(ANALOG_PIN));

      sensorData[type] = sensorData[type] + calSensorData[type];
    }
  }
} sensor; // 기본 측정값 설정

struct Config {
public:
  bool initializeSPIFFS();
  void loadConfigFromSPIFFS();
  void saveConfigToSPIFFS();
} config;

struct Tool {
public:
  // 업데이트 상태를 저장할 경로
  const char* updateStateFile = "/updateState.txt";
  const char* firmwareFileNameFile = "/firmwareFileName.txt";  // 파일 이름 저장 경로
  String firmwareFileName = "";  // 다운로드할 파일 이름 저장

  void download_program(String fileName);
  void blinkLed(int iteration);
} tool;

// Create an instance of the Data structure
WifiMqtt wifi,wifiSave;

unsigned int counter = 0;
BLECharacteristic *pCharacteristic;

void parseJSONPayload(byte* payload, unsigned int length);
void setup();

//=========================================================
// NTP 서버 설정
const long utcOffsetInSeconds = 3600 * 9;  // 한국 표준시(KST) UTC+9
const unsigned long ntpUpdateInterval = 3600000;  // 1시간(3600000ms)마다 NTP 서버 업데이트

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

// 핀 설정
const int controlPins[] = {26, 27, 32, 33};
bool pinStates[] = {false, false, false, false};  // 각 핀의 현재 상태 저장
bool previousPinStates[] = {false, false, false, false};  // 각 핀의 이전 상태 저장

// TimeSlot 클래스 정의
class TimeSlot {
public:
  int startHour;
  int startMinute;
  int endHour;
  int endMinute;
  String repeatMode;  // "daily"="d" 또는 "weekly"="w"
  int dayOfWeek;  // 요일 (0 = 일요일, 1 = 월요일, ..., 6 = 토요일)
  //time schedule 위한 프로그램
  unsigned long lastMsgTime = 0; // 마지막 메시지 전송 시간
  unsigned long lastIntTime = 0; // 마지막 메시지 전송 시간
  int slotIndexToSend = -1; // 전송할 타임슬롯 인덱스
  int currentPinIndex = 0; // 전송할 핀 인덱스
  time_t lastNtpTime;
  unsigned long lastMillis;
  unsigned long lastNtpUpdateMillis;

    void setup();
    void loop();
    TimeSlot(int sh, int sm, int eh, int em, String rm, int dow = -1)
      : startHour(sh), startMinute(sm), endHour(eh), endMinute(em), repeatMode(rm), dayOfWeek(dow) {}

    TimeSlot() // 기본 생성자 추가
      : startHour(0), startMinute(0), endHour(0), endMinute(0), repeatMode("d"), dayOfWeek(-1) {}

    bool isActive(struct tm * timeinfo) {
        int currentHour = timeinfo->tm_hour;
        int currentMinute = timeinfo->tm_min;
        int currentDayOfWeek = timeinfo->tm_wday;

        //Serial.printf("Checking if active: Current time %02d:%02d, Current day %d\n", currentHour, currentMinute, currentDayOfWeek);
        //Serial.println(repeatMode);
        if (repeatMode == "d") {
            return isTimeInRange(currentHour, currentMinute);
        } else if (repeatMode == "w") {
            if (currentDayOfWeek == dayOfWeek) {
                return isTimeInRange(currentHour, currentMinute);
            }
        }
        return false;
    }

private:
    bool isTimeInRange(int currentHour, int currentMinute) {
        int startTotalMinutes = startHour * 60 + startMinute;
        int endTotalMinutes = endHour * 60 + endMinute;
        int currentTotalMinutes = currentHour * 60 + currentMinute;

        //Serial.printf("Checking time range: %02d:%02d - %02d:%02d, Current time: %02d:%02d\n", startHour, startMinute, endHour, endMinute, currentHour, currentMinute);

        if (startTotalMinutes < endTotalMinutes) {
            return currentTotalMinutes >= startTotalMinutes && currentTotalMinutes < endTotalMinutes;
        } else {
            return currentTotalMinutes >= startTotalMinutes || currentTotalMinutes < endTotalMinutes;
        }
    }
} timeManager;  // 변수 이름을 time에서 timeManager로 변경


// 각 핀에 대한 동적 시간대 관리
std::vector<TimeSlot> timeSlots[4];

// 함수 선언
void addTimeSlot(int pinIndex, int startHour, int startMinute, int endHour, int endMinute, String repeatMode, int dayOfWeek = -1);
void removeTimeSlot(int pinIndex, int slotIndex);
void removeAllTimeSlots(int pinIndex); // 새로운 함수 선언
void loadTimeSlotsFromSPIFFS(int pinIndex);
void saveTimeSlotsToSPIFFS(int pinIndex);
String getTimeSlotJson(int pinIndex, int slotIndex);
void startSendingTimeSlots(int pinIndex);
void printCurrentTime();
void printSchedules();
void sendNextTimeSlot();
void TimeSlot::setup() {
  timeClient.begin();
  timeClient.update();
  this->lastNtpTime = timeClient.getEpochTime();
  this->lastMillis = millis();
  this->lastNtpUpdateMillis = millis();

  // 현재 시간 출력
  printCurrentTime();

  // SPIFFS에서 시간 슬롯 로드
  for (int i = 0; i < 4; i++) {
    loadTimeSlotsFromSPIFFS(i);
  }

  // 현재 스케줄 출력
  //printSchedules();
}
void TimeSlot::loop() {
  unsigned long currentMillis = millis();
  // 타임슬롯 전송 로직
  if (this->slotIndexToSend >= 0) {
    if (currentMillis - this->lastMsgTime >= 1000) { // 1초 간격으로 전송
      sendNextTimeSlot();
      this->lastMsgTime = currentMillis;
    }
  }

  if (currentMillis - this->lastIntTime < 1000) { // 1초 loop 실행
    return;
  }
  this->lastIntTime = currentMillis;

  if (WiFi.status() == WL_CONNECTED) {
    wifi.isConnected = true;
    if (millis() - this->lastNtpUpdateMillis > ntpUpdateInterval) {
      timeClient.update();
      this->lastNtpTime = timeClient.getEpochTime();
      this->lastMillis = millis();
      this->lastNtpUpdateMillis = millis();
      Serial.println("NTP time updated");
    }
  } else {
    wifi.isConnected = false;
  }

  time_t currentTime = this->lastNtpTime + ((millis() - this->lastMillis) / 1000);
  struct tm* timeinfo = localtime(&currentTime);

  //Serial.printf("Current loop time: %02d:%02d:%02d\n", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

  for (int i = 0; i < 4; i++) {
    bool pinOn = false;
    for (TimeSlot slot : timeSlots[i]) {
      if (slot.isActive(timeinfo)) {
        pinOn = true;
        break;
      }
    }

    pinStates[i] = pinOn;

    if (pinStates[i] != previousPinStates[i]) {
      //digitalWrite(controlPins[i], pinOn ? HIGH : LOW);
      dev.digitalWriteUpdateData(i, pinOn);
      Serial.printf("%02d:%02d:%02d - Pin %d is %s\n", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, controlPins[i], pinOn ? "ON" : "OFF");
      previousPinStates[i] = pinStates[i];
    }
  }
}
//=========================================================

void setup() {
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  // Set each output pin as an output
  for (int i = 0; i <numberOfPins; i++) {
    pinMode(outputPins[i], OUTPUT);
  }
  // Set each input pin as an input
  for (int i = 0; i < numberOfPins; i++) {
    pinMode(inputPins[i], INPUT);
  }

  dev.checkFactoryDefault();
  config.loadConfigFromSPIFFS();
  if (wifi.ssid.isEmpty()) {
    Serial.println("Bluetooth 셋업");
    ble.setup();
    // BLE이 제대로 초기화될 수 있도록 약간의 시간을 기다립니다.
    delay(1000);
    Serial.println("BLE ready!");
  }
  else {
    sensor.setup();
    // Wi-Fi 연결 설정
    wifi.connectToWiFi();
    // MQTT 설정
    client.setServer(wifi.mqttBroker.c_str(), 1883);
    client.setCallback(callback);
    // 프로그램 시작 시간 기록
    wifi.startupTime = millis();
    programStartTime = millis(); // Record program start time
    timeManager.setup();
  }
  // Load pin states from SPIFFS
  dev.loadPinStatesFromSPIFFS();
  
  // rgb led
  strip.begin();         // LED 제어를 위한 초기화
  strip.show();          // LED를 초기화 상태로 설정
  dev.setColor(255, 0, 0);   // 빨강색

  //light
  pinMode(LIGHT_SENSOR_PIN, INPUT);  // Light 센서 핀을 입력으로 설정
  analogReadResolution(12);       // 12비트 ADC 해상도 설정 (0~4095)
  
  // setup이 끝나는 시점에서 메모리 사용량 출력
  Serial.print("Free heap memory after setup: ");
  Serial.println(esp_get_free_heap_size());
}

/* 블루투스 함수 ===============================================*/
// 받은 order의 리턴정보
void Ble::writeToBle(int order) {
  // Create a JSON object
  DynamicJsonDocument responseDoc(1024);
  // Fill the JSON object based on the order
  if (order == 1) {
    responseDoc["order"] = order;
    responseDoc["ssid"] = wifi.ssid;
    responseDoc["password"] = wifi.password;
    responseDoc["email"] = wifi.email;
  }

  // Serialize JSON object to string
  String responseString;
  serializeJson(responseDoc, responseString);

  if (order == 0) {
    responseString = "프로그램 다운로드";
  } else if (order == 2) {
    responseString = dev.sendData;
  } else if (order == 101) {
    responseString = "와이파이 정보가 잘못되었습니다.";
  } else if (order == 102) {
    responseString = "와이파이 정보가 저장되었습니다.";
  }

  // String 타입으로 변경 후 전송
  if (pCharacteristic) {
    pCharacteristic->setValue(responseString); // String 타입으로 전달
    pCharacteristic->notify(); // If notification enabled
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Ble Device connected");
      ble.isConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
      wifi.isConnected = false; // Set the isConnected flag to false on disconnection
      Serial.println("Device disconnected");
      ble.isConnected = false;
      BLEDevice::startAdvertising();  // Start advertising again after disconnect
    }
};

// 전송된 문자를 받는다.
class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue().c_str(); // std::string 대신 String 사용
      if (value.length() > 0) {
        Serial.println("BLE 받음:");
        for (int i = 0; i < value.length(); i++) {
          Serial.print(value[i]);
        }
        Serial.println();

        // `std::string` 대신 `String`을 사용
        parseJSONPayload((byte*)value.c_str(), value.length());
      }
    }
};

void Ble::setup() {
  ble.boot = true;
  String namePlc = "i2r-" + String(dev.type) + "-IoT PLC";
  BLEDevice::init(namePlc.c_str());
  BLEServer *pServer = BLEDevice::createServer();

  // Set server callbacks
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(ble.service_uuid);
  pCharacteristic = pService->createCharacteristic(
                                         ble.characteristic_uuid,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setValue(""); // 초기 값 설정
  pCharacteristic->setValue(String(200, ' ')); // 최대 길이를 200으로 설정 (String 타입으로 변경)

  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(ble.service_uuid);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE service started");
  // 이제 BLE MAC 주소를 읽어 봅니다.
  ble.readBleMacAddress();
}


void Ble::readBleMacAddress() {
  // BLE 디바이스에서 MAC 주소를 가져옵니다.
  BLEAddress bleAddress = BLEDevice::getAddress();
  // MAC 주소를 String 타입으로 변환합니다.
  String mac = bleAddress.toString().c_str();
  // MAC 주소를 모두 대문자로 변환합니다.
  mac.toUpperCase();
  // 시리얼 모니터에 BLE MAC 주소를 출력합니다.
  Serial.print("BLE MAC Address: ");
  Serial.println(mac);
}
/* 블루투스 함수 ===============================================*/

/* 와이파이 MQTT 함수 ===============================================*/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("WiFi 받음: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // 프로그램 시작 후 일정 시간 동안 메시지 무시
  unsigned long currentMillis = millis();
  if (currentMillis - wifi.startupTime < wifi.ignoreDuration) {
    Serial.println("프로그램 시작 후 초기 메시지 무시 중...");
    return;
  }

  // JSON 파싱
  parseJSONPayload(payload, length);
}

void WifiMqtt::publishMqtt()
{ 
  Serial.println("publish: "+dev.sendData);
  client.publish(wifi.outTopic, dev.sendData.c_str());
}

void WifiMqtt::connectToWiFi() {
  if (wifi.ssid == NULL) {
    Serial.println("SSID is NULL or empty, returning...");
    return; // SSID가 null이거나 빈 문자열이면 함수를 빠져나갑니다.
  }
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(wifi.ssid, wifi.password);

  int wCount = 0;
  wifi.isConnected = true;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    dev.checkFactoryDefault();
    wCount++;
    if(wCount > 10) {
      wifi.isConnected = false;
      break; // while 루프를 벗어납니다.
    }
  }

  this->readWifiMacAddress();
  
  if(this->isConnected == true) {
    Serial.println("\nConnected to Wi-Fi");

    // 이메일 기반으로 MQTT 토픽 이름 설정
    String outTopicBase = "i2r/" + this->email + "/out";
    String inTopicBase = "i2r/" + this->email + "/in";
    strncpy(this->outTopic, outTopicBase.c_str(), sizeof(this->outTopic) - 1);
    this->outTopic[sizeof(this->outTopic) - 1] = '\0'; // 널 종료 보장
    strncpy(this->inTopic, inTopicBase.c_str(), sizeof(this->inTopic) - 1);
    this->inTopic[sizeof(this->inTopic) - 1] = '\0'; // 널 종료 보장

  } else {
    Serial.println("\nWi-Fi를 찾을 수 없습니다.");
  }
  
}

void WifiMqtt::loop() {
  if(this->isConnected == false)
    return;
  if (!client.connected()) {
    this->reconnectMQTT();
  }
  client.loop();

  // LED 점멸 로직 mqtt가 연결되면 2초간격으로 점멸한다.
  if (isConnectedMqtt) {
    dev.setColor(0, 255, 0);   // 녹색
    if (statusSendCounter < 3) {
      dev.sendStatusCheckChange(false);
      statusSendCounter++;
      delay(1000); // 잠시 대기 후 전송 (필요 시 조정)
    }
  } else {
    dev.setColor(255, 0, 0);   // 빨강색
  }
}

void WifiMqtt::reconnectMQTT() {
  if(wifi.isConnected == false)
    return;
  while (!client.connected()) {
    dev.checkFactoryDefault();
    Serial.println("Connecting to MQTT...");
    if (client.connect(dev.mac.c_str())) {
      Serial.println("Connected to MQTT");
      client.subscribe(wifi.inTopic); // MQTT 토픽 구독
      wifi.isConnectedMqtt=true;
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      wifi.isConnectedMqtt=false;
      delay(5000);
    }
  }
}

void WifiMqtt::readWifiMacAddress() {
  // Wi-Fi 디바이스에서 MAC 주소를 가져옵니다.
  String macAddress = WiFi.macAddress();
  // MAC 주소를 String 타입으로 변환합니다.
  dev.mac = macAddress;
  // MAC 주소를 모두 대문자로 변환합니다.
  dev.mac.toUpperCase();
  // 시리얼 모니터에 Wi-Fi MAC 주소를 출력합니다.
  Serial.print("Wi-Fi MAC Address: ");
  Serial.println(dev.mac);
}
/* 와이파이 MQTT 함수 ===============================*/
/* Time Schedule =====================================================*/
void sendNextTimeSlot() {
  if (timeManager.slotIndexToSend < timeSlots[timeManager.currentPinIndex].size()) {
    dev.sendData=getTimeSlotJson(timeManager.currentPinIndex, timeManager.slotIndexToSend);
    wifi.publishMqtt();
    timeManager.slotIndexToSend++;
  } else {
    timeManager.slotIndexToSend = -1; // 전송 완료
    Serial.println("All time slots sent for pin " + String(timeManager.currentPinIndex));
  }
}

void printSchedules() {
  Serial.println("Current Schedules:");
  for (int i = 0; i < 4; i++) {
    Serial.printf("Pin %d:\n", controlPins[i]);
    for (const TimeSlot& slot : timeSlots[i]) {
      if (slot.repeatMode == "weekly") {
        Serial.printf("  %02d:%02d - %02d:%02d on day %d (%s)\n",
          slot.startHour, slot.startMinute, slot.endHour, slot.endMinute, slot.dayOfWeek, slot.repeatMode.c_str());
      }
      else {
        Serial.printf("  %02d:%02d - %02d:%02d (%s)\n",
          slot.startHour, slot.startMinute, slot.endHour, slot.endMinute, slot.repeatMode.c_str());
      }
    }
  }
}

void printCurrentTime() {
  timeClient.update();
  time_t currentTime = timeClient.getEpochTime();
  struct tm* timeinfo = localtime(&currentTime);

  Serial.printf("Current time: %02d:%02d:%02d\n", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
}

void addTimeSlot(int pinIndex, int startHour, int startMinute, int endHour, int endMinute, String repeatMode, int dayOfWeek) {
  if (pinIndex >= 0 && pinIndex < 4) {
    timeSlots[pinIndex].push_back(TimeSlot(startHour, startMinute, endHour, endMinute, repeatMode, dayOfWeek));
    Serial.println("Time slot added.");
  } else {
    Serial.println("Invalid index.");
  }
}

void removeTimeSlot(int pinIndex, int slotIndex) {
  if (pinIndex >= 0 && pinIndex < 4 && slotIndex >= 0 && slotIndex < timeSlots[pinIndex].size()) {
    timeSlots[pinIndex].erase(timeSlots[pinIndex].begin() + slotIndex);
    Serial.println("Time slot removed.");
  } else {
    Serial.println("Invalid index or slot index.");
  }
}

void removeAllTimeSlots(int pinIndex) {
  if (pinIndex >= 0 && pinIndex < 4) {
    timeSlots[pinIndex].clear();
    Serial.println("All time slots removed.");
  } else {
    Serial.println("Invalid index.");
  }
}

void saveTimeSlotsToSPIFFS(int pinIndex) {
  String fileName = "/timeslots_" + String(pinIndex) + ".json";
  File file = SPIFFS.open(fileName, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  DynamicJsonDocument doc(512);  // Adjust size according to expected payload
  JsonArray pinArray = doc.to<JsonArray>();
  for (const TimeSlot& slot : timeSlots[pinIndex]) {
    JsonObject slotObj = pinArray.createNestedObject();
    slotObj["sH"] = slot.startHour;
    slotObj["sM"] = slot.startMinute;
    slotObj["eH"] = slot.endHour;
    slotObj["eM"] = slot.endMinute;
    slotObj["rM"] = slot.repeatMode;
    slotObj["dW"] = slot.dayOfWeek;
  }

  if (serializeJson(doc, file) == 0) {
    Serial.println("Failed to write to file");
  }
  file.close();
}

void loadTimeSlotsFromSPIFFS(int pinIndex) {
  String fileName = "/timeslots_" + String(pinIndex) + ".json";
  File file = SPIFFS.open(fileName, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  size_t size = file.size();
  if (size > 512) {  // Adjust size according to expected payload
    Serial.println("File size is too large");
    file.close();
    return;
  }

  std::unique_ptr<char[]> buf(new char[size]);
  file.readBytes(buf.get(), size);

  DynamicJsonDocument doc(512);  // Adjust size according to expected payload
  DeserializationError error = deserializeJson(doc, buf.get());

  if (error) {
    //Serial.println("Failed to parse time-slot file");
    file.close();
    return;
  }

  JsonArray pinArray = doc.as<JsonArray>();
  for (JsonObject slotObj : pinArray) {
    int startHour = slotObj["sH"];
    int startMinute = slotObj["sM"];
    int endHour = slotObj["eH"];
    int endMinute = slotObj["eM"];
    String repeatMode = slotObj["rM"].as<String>();
    int dayOfWeek = slotObj["dW"];

    timeSlots[pinIndex].push_back(TimeSlot(startHour, startMinute, endHour, endMinute, repeatMode, dayOfWeek));
  }

  file.close();
}

String getTimeSlotJson(int pinIndex, int slotIndex) {
  DynamicJsonDocument doc(256);
  JsonObject slotObj = doc.to<JsonObject>();
  const TimeSlot& slot = timeSlots[pinIndex][slotIndex];
  slotObj["order"] = 4;
  slotObj["mac"] = dev.mac;
  slotObj["pI"] = pinIndex;
  slotObj["index"] = slotIndex;
  slotObj["sH"] = slot.startHour;
  slotObj["sM"] = slot.startMinute;
  slotObj["eH"] = slot.endHour;
  slotObj["eM"] = slot.endMinute;
  slotObj["rM"] = slot.repeatMode;
  slotObj["dW"] = slot.dayOfWeek;
  // 전체 타임슬롯 중 현재 타임슬롯의 인덱스 + 1 / 전체 타임슬롯 수
  slotObj["pN"] = String(slotIndex + 1) + "/" + String(timeSlots[pinIndex].size());

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}


void startSendingTimeSlots(int pinIndex) {
  timeManager.slotIndexToSend = 0;
  timeManager.currentPinIndex = pinIndex;
}
/* Time Schedule =====================================================*/

/* Tools ===========================================================*/
void Sensor::sendOutSensorChange(String type, bool rise) {
  SensorChange change;
  if (rise)
    change = SensorChanges[type][1];  // 상승 조건일 때
  else
    change = SensorChanges[type][0];  // 하강 조건일 때
   // 참조로 가져오기
  if (change.mac == dev.mac) {
    // MAC 주소가 현재 장치의 MAC 주소와 동일하면 직접 포트로 출력
    dev.digitalWriteUpdateData(change.port, change.state);
    if(rise)
      Serial.println("Temperature up: Direct output to port " + String(change.port) + " with value " + String(change.value));
    else
      Serial.println("Temperature down: Direct output to port " + String(change.port) + " with value " + String(change.value));
  } else {
    // MAC 주소가 다르면 MQTT 메시지 전송
    DynamicJsonDocument doc(256);
    doc["order"] = 2;
    doc["mac"] = change.mac;
    doc["no"] = change.port;
    doc["value"] = change.state;
    String output;
    serializeJson(doc, output);
    client.publish(wifi.inTopic, output.c_str());
    if(rise)
      Serial.println("Temperature up: " + output);  
    else
      Serial.println("Temperature down: " + output);
  }
}


// 🔵 SPIFFS에 데이터 저장 함수
void Sensor::saveToSPIFFS() {
  StaticJsonDocument<2048> jsonDoc; // JSON 데이터 저장을 위한 문서

  // sensorData 저장
  JsonObject jsonSensorData = jsonDoc.createNestedObject("sensorData");
  for (const auto& entry : sensorData) {
      jsonSensorData[entry.first] = entry.second;
  }

  // sensorLastData 저장
  JsonObject jsonSensorLastData = jsonDoc.createNestedObject("sensorLastData");
  for (const auto& entry : sensorLastData) {
      jsonSensorLastData[entry.first] = entry.second;
  }

  // calSensorData 저장
  JsonObject jsonCalSensorData = jsonDoc.createNestedObject("calSensorData");
  for (const auto& entry : calSensorData) {
      jsonCalSensorData[entry.first] = entry.second;
  }

  // SensorChanges 저장
  JsonObject jsonSensorChanges = jsonDoc.createNestedObject("SensorChanges");
  for (const auto& entry : SensorChanges) {
      JsonArray changesArray = jsonSensorChanges.createNestedArray(entry.first);
      for (const auto& change : entry.second) {
          JsonObject changeObj = changesArray.createNestedObject();
          changeObj["value"] = change.value;
          changeObj["mac"] = change.mac;
          changeObj["port"] = change.port;
          changeObj["state"] = change.state;
          changeObj["timestamp"] = change.timestamp;
      }
  }

  // JSON 문자열로 변환
  String jsonString;
  serializeJson(jsonDoc, jsonString);

  // SPIFFS에 저장
  File file = SPIFFS.open("/sensor_config.json", "w");
  if (!file) {
      Serial.println("Failed to open file for writing.");
      return;
  }
  file.print(jsonString);
  file.close();

  Serial.println("Sensor data saved to SPIFFS:");
  Serial.println(jsonString);
}

// 🔵 SPIFFS에서 데이터 로드 함수
void Sensor::loadFromSPIFFS() {
  File file = SPIFFS.open("/sensor_config.json", "r");
  if (!file) {
      Serial.println("Failed to open file for reading.");
      return;
  }

  // 파일 내용을 JSON 문서로 변환
  StaticJsonDocument<2048> jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, file);
  file.close();

  if (error) {
      Serial.println("Failed to parse JSON.");
      return;
  }

  // sensorData 복원
  JsonObject jsonSensorData = jsonDoc["sensorData"];
  for (JsonPair kv : jsonSensorData) {
      sensorData[kv.key().c_str()] = kv.value().as<float>();
  }

  // sensorLastData 복원
  JsonObject jsonSensorLastData = jsonDoc["sensorLastData"];
  for (JsonPair kv : jsonSensorLastData) {
      sensorLastData[kv.key().c_str()] = kv.value().as<float>();
  }

  // calSensorData 복원
  JsonObject jsonCalSensorData = jsonDoc["calSensorData"];
  for (JsonPair kv : jsonCalSensorData) {
      calSensorData[kv.key().c_str()] = kv.value().as<float>();
  }

  // SensorChanges 복원
  JsonObject jsonSensorChanges = jsonDoc["SensorChanges"];
  for (JsonPair kv : jsonSensorChanges) {
      String key = kv.key().c_str();
      JsonArray changesArray = kv.value().as<JsonArray>();
      std::vector<SensorChange> changes;

      for (JsonObject changeObj : changesArray) {
          SensorChange change;
          change.value = changeObj["value"];
          change.mac = changeObj["mac"].as<String>();
          change.port = changeObj["port"];
          change.state = changeObj["state"];
          change.timestamp = changeObj["timestamp"];
          changes.push_back(change);
      }
      SensorChanges[key] = changes;
  }

  Serial.println("Sensor data loaded from SPIFFS.");
}

void Sensor::printSensorData() {
  Serial.println("\n===== Sensor Data =====");
  
  // sensorData 출력
  Serial.println("Sensor Data:");
  for (const auto& entry : sensorData) {
      Serial.println("  " + entry.first + ": " + String(entry.second));
  }

  // sensorLastData 출력
  Serial.println("\nLast Sensor Data:");
  for (const auto& entry : sensorLastData) {
      Serial.println("  " + entry.first + ": " + String(entry.second));
  }

  // calSensorData 출력
  Serial.println("\nCalibrated Sensor Data:");
  for (const auto& entry : calSensorData) {
      Serial.println("  " + entry.first + ": " + String(entry.second));
  }

  // SensorChanges 출력
  Serial.println("\n===== Sensor Changes =====");
  for (const auto& entry : SensorChanges) {
      Serial.println("Sensor: " + entry.first);
      for (size_t i = 0; i < entry.second.size(); i++) {
          const auto& change = entry.second[i];
          Serial.println("  [" + String(i) + "] Value: " + String(change.value) +
                          ", MAC: " + change.mac +
                          ", Port: " + String(change.port) +
                          ", State: " + String(change.state) +
                          ", Timestamp: " + String(change.timestamp));
      }
  }
  Serial.println("=========================\n");
}

void Device::loop() {
  unsigned long currentTime = millis();  // 현재 시간을 가져옵니다

  if (currentTime - this->lastTime >= this->interval) {
    this->lastTime = currentTime;
    this->sendStatusCheckChange(true); // 입력 핀 상태 체크 및 변화 감지
  }
  sensor.loop();
}

void Device::digitalWriteUpdateData(int pin, bool value) {
  // dev.out 업데이트
  dev.out[pin] = value;
  // 출력 포트로 값 설정
  digitalWrite(outputPins[pin], value ? HIGH : LOW);
  this->sendStatusCheckChange(false);
}

void Device::loadPinStatesFromSPIFFS() {
  for (int port = 0; port < numberOfPins+2; ++port) {
    String fileName = "/pinState_" + String(port) + ".json";
    if (!SPIFFS.exists(fileName)) {
      // pinStateChanges[port][0], pinStateChanges[port][1] 모든 데이터를 null 또는 -1로 초기화
      dev.pinStateChanges[port][0].clear();
      dev.pinStateChanges[port][1].clear();
      
      PinStateChange defaultChange;
      defaultChange.mac = "";
      defaultChange.port = -1;
      defaultChange.value = false;
      defaultChange.timestamp = 0;

      dev.pinStateChanges[port][0].push_back(defaultChange);
      dev.pinStateChanges[port][1].push_back(defaultChange);
      //file 만들고 저장해줘
      //Serial.println("Initialized pinStateChanges for port " + String(port));
    } else {
      // 파일이 존재하면 내용을 읽어서 로드
      File file = SPIFFS.open(fileName, FILE_READ);
      if (!file) {
        Serial.println("Failed to open file for reading");
        continue;
      }

      size_t size = file.size();
      if (size > 1024) {  // Adjust size according to expected payload
        Serial.println("File size is too large");
        file.close();
        continue;
      }

      std::unique_ptr<char[]> buf(new char[size]);
      file.readBytes(buf.get(), size);
      Serial.println(buf.get());

      DynamicJsonDocument doc(2048);  // Adjust size according to expected payload
      DeserializationError error = deserializeJson(doc, buf.get());

      if (error) {
        Serial.println("Failed to parse pin-state file");
        file.close();
        continue;
      }

      JsonArray changesArray = doc["changes"];
      dev.pinStateChanges[port][0].clear();
      dev.pinStateChanges[port][1].clear();
      
      int index=0;
      for (JsonObject changeObj : changesArray) {
        PinStateChange change;
        change.mac = changeObj["mac"].as<String>();
        change.port = changeObj["port"].as<int>();
        change.value = changeObj["value"].as<bool>();
        change.timestamp = changeObj["timestamp"];
        dev.pinStateChanges[port][index++].push_back(change);
      }

      file.close();
    }
  }

  // PinStateChange 프린트
  /*
  Serial.println("---------------");
  for (int port = 0; port < numberOfPins; ++port) {
    Serial.printf("Port %d - false state:\n", port);
    for (const auto& change : this->pinStateChanges[port][0]) {
      Serial.printf("Loaded PinStateChange: MAC=%s, port=%d, value=%s, timestamp=%lu\n", 
                    change.mac.c_str(), change.port, change.value ? "true" : "false", change.timestamp);
    }

    Serial.printf("Port %d - true state:\n", port);
    for (const auto& change : this->pinStateChanges[port][1]) {
      Serial.printf("Loaded PinStateChange: MAC=%s, port=%d, value=%s, timestamp=%lu\n", 
                    change.mac.c_str(), change.port, change.value ? "true" : "false", change.timestamp);
    }
  }
  Serial.println("---------------");
  */
}
// 핀 상태를 다이렉트 또는 MQTT로 전송하는 함수 정의
void Device::sendOut(int port, bool portState) {
  PinStateChange change = dev.pinStateChanges[port][int(portState)][0];  // pinStateChanges[port][1]에서 첫 번째 항목 가져오기
  if (change.mac == dev.mac) {
    // MAC 주소가 현재 장치의 MAC 주소와 동일하면 직접 포트로 출력
    this->digitalWriteUpdateData(change.port,change.value);
    Serial.println("Direct output to port " + String(change.port) + " with value " + String(change.value));
  } else {
    // MAC 주소가 다르면 MQTT 메시지 전송
    DynamicJsonDocument doc(256);
    doc["order"] = 2;
    doc["mac"] = change.mac;
    doc["no"] = change.port;
    doc["value"] = change.value;
    String output;
    serializeJson(doc, output);
    client.publish(wifi.inTopic, output.c_str());
    Serial.println("WiFi 보냄: " + output);
  }
}

//dataChange=true이전값과 비교하여 값이 변했으면 데이터 보낸다.
//dataChange=false 무조건 데이터 보낸다.
void Device::sendStatusCheckChange(bool dataChange) {
  DynamicJsonDocument responseDoc(1024);
  responseDoc["type"] = dev.type;
  responseDoc["email"] = wifi.email;
  responseDoc["shield"] = dev.shield;
  responseDoc["mac"] = dev.mac;

  for (const auto& type : sensor.measurementTypes) {
    responseDoc[type] = sensor.sensorData[type];
  }

  JsonArray inArray = responseDoc.createNestedArray("in");
  for (int i = 0; i < numberOfPins; i++) {
    inArray.add(dev.in[i]); // mqtt보내기위한 문장 작성
    //in 포트 입력 변화시 여기 설정된 출력값 실행
    int invertedState = digitalRead(inputPins[i]);
    // 읽은 값 반전: 1 -> 0, 0 -> 1
    int currentState = !invertedState;
    if (dev.in[i] != currentState) {
      dev.in[i] = currentState;
      sendOut(i, currentState); // 상태 변화를 MQTT로 전송
    }
  }

  JsonArray outArray = responseDoc.createNestedArray("out");
  for (int i = 0; i < numberOfPins; i++) {
    outArray.add(dev.out[i]);
  }
  dev.sendData="";
  serializeJson(responseDoc, dev.sendData);

  if(dataChange == false && wifi.isConnectedMqtt == true) {
    wifi.publishMqtt();
  }

  if( !dev.sendData.equals(dev.sendDataPre)) {
    dev.sendDataPre = dev.sendData;
    if(wifi.isConnectedMqtt == true) {
      //Serial.println(dev.sendData);
      wifi.publishMqtt();
    }
  }
}
// Config 파일을 SPIFFS에서 읽어오는 함수
void Config::loadConfigFromSPIFFS() {
  Serial.println("파일 읽기");

  if (!config.initializeSPIFFS()) {
    Serial.println("Failed to initialize SPIFFS.");
    return;
  }

  if (!SPIFFS.exists("/config.txt")) {
    Serial.println("Config file does not exist.");
    return;
  }

  File configFile = SPIFFS.open("/config.txt", FILE_READ);
  if (!configFile) {
    Serial.println("Failed to open config file");
    return;
  }

  size_t size = configFile.size();
  std::unique_ptr<char[]> buf(new char[size]);
  configFile.readBytes(buf.get(), size);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, buf.get());
  
  if (error) {
    Serial.println("Failed to parse config file");
    return;
  }

  wifi.ssid = doc["ssid"] | "";
  wifi.password = doc["password"] | "";
  wifi.email = doc["email"] | "";
  wifi.mqttBroker = doc["mqttBroker"] | "";

  Serial.print("wifi.ssid: "); Serial.println(wifi.ssid);
  Serial.print("wifi.password: "); Serial.println(wifi.password);
  Serial.print("wifi.email: "); Serial.println(wifi.email);
  Serial.print("wifi.mqttBroker: "); Serial.println(wifi.mqttBroker);
  configFile.close();
}

void Config::saveConfigToSPIFFS() {
  Serial.println("config.txt 저장");

  if (!config.initializeSPIFFS()) {
    Serial.println("SPIFFS 초기화 실패.");
    return;
  }

  // SPIFFS 초기화를 시도합니다.
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS failed to initialize. Formatting...");
    // 초기화 실패 시 포맷을 시도합니다.
    if (!SPIFFS.format()) {
      Serial.println("SPIFFS format failed.");
      return;
    }
    // 포맷 후에 다시 초기화를 시도합니다.
    if (!SPIFFS.begin()) {
      Serial.println("SPIFFS failed to initialize after format.");
      return;
    }
  }

  File configFile = SPIFFS.open("/config.txt", FILE_WRITE);
  
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return;
  }

  DynamicJsonDocument doc(1024);

  // 데이터를 구조체에서 가져온다고 가정합니다.
  doc["ssid"] = wifiSave.ssid;
  doc["password"] = wifiSave.password;
  doc["email"] = wifiSave.email;
  doc["mqttBroker"] = wifiSave.mqttBroker;

  Serial.print("wifi.ssid: "); Serial.println(wifiSave.ssid);
  Serial.print("wifi.password: "); Serial.println(wifiSave.password);
  Serial.print("wifi.email: "); Serial.println(wifiSave.email);
  Serial.print("wifi.mqttBroker: "); Serial.println(wifiSave.mqttBroker);

  if (serializeJson(doc, configFile) == 0) {
    Serial.println("Failed to write to file");
    configFile.close();
    return;
  }

  configFile.close();
  // 파일이 제대로 닫혔는지 확인합니다.
  if (configFile) {
    Serial.println("파일이 여전히 열려있습니다.");
  } else {
    Serial.println("파일이 성공적으로 닫혔습니다.");
  }
  Serial.println("파일 저장 끝");

  // 파일이 제대로 저장되었는지 확인합니다.
  if (SPIFFS.exists("/config.txt")) {
    Serial.println("Config file saved successfully.");
    // 저장이 확인된 후 재부팅을 진행합니다.
    Serial.println("Rebooting...");
    delay(1000); // 재부팅 전에 짧은 지연을 줍니다.
    ESP.restart();
  } else {
    Serial.println("Config file was not saved properly.");
  }
  
  // ESP32 재부팅
  delay(1000);
  ESP.restart();
}

// SPIFFS를 초기화하고 필요한 경우 포맷하는 함수를 정의합니다.
bool Config::initializeSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS 초기화 실패!");
    if (!SPIFFS.format()) {
      Serial.println("SPIFFS 포맷 실패!");
      return false;
    }
    if (!SPIFFS.begin()) {
      Serial.println("포맷 후 SPIFFS 초기화 실패!");
      return false;
    }
  }
  return true;
}

// checkFactoryDefault에서 다운로드 중단을 처리
void Device::checkFactoryDefault() {
  if (digitalRead(TRIGGER_PIN) == LOW) {
    //digitalWrite(ledPin, LOW);
    dev.setColor(255, 0, 0);   // 빨강색
    Serial.println("Please wait over 3 min");
    SPIFFS.format();
    delay(1000);
    ESP.restart();
    delay(1000);
  }
}


// httpsupdate()
// 다운로드 함수 
void Tool::download_program(String fileName) {
  // 다운로드 시작 전에 LED 깜박이기
  this->blinkLed(10);
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClientSecure clientSecure;
    clientSecure.setInsecure();  // 인증서 검증 무시

    // Add optional callback notifiers
    httpUpdate.onStart([]() {
      Serial.println("Update Started");
    });
    httpUpdate.onEnd([]() {
      Serial.println("Update Finished");
      tool.blinkLed(30);
    });
    httpUpdate.onProgress([](int cur, int total) {
      Serial.printf("Progress: %d%%\n", (cur * 100) / total);
    });
    httpUpdate.onError([](int error) {
      Serial.printf("Update Error: %d\n", error);
    });

    httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    String url = "https://github.com/kdi6033/download/raw/main/" + fileName;
    Serial.println("Downloading from: " + url);
    
    // 서버에서 HTTP 응답 코드 확인 추가
    t_httpUpdate_return ret = httpUpdate.update(clientSecure, url);
    Serial.printf("HTTP Code: %d\n", clientSecure.connected() ? clientSecure.available() : -1);
  
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
  }
}


void Tool::blinkLed(int iteration) {
  for (int i = 0; i < iteration; i++) {
    ///digitalWrite(ledPin, HIGH); // LED 켜기
    dev.setColor(255, 255, 255);   // 빨강색
    delay(100);                 // 0.1초 대기
    dev.setColor(0, 0, 0);   // 빨강색  // LED 끄기
    delay(100);                 // 0.1초 대기
  }
}

/* Tools ===========================================================*/

void parseJSONPayload(byte* payload, unsigned int length) {
  char payloadStr[length + 1];
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';  // Null-terminate the string
  //Serial.println(payloadStr);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payloadStr);

  if (error) {
    Serial.println("JSON 파싱 실패!");
    return;
  }

  int order = doc["order"] | -1;
  if (order == 1) { //order=1 에는 mac 데이터 없음
    const char *ssid = doc["ssid"] | "";
    const char *password = doc["password"] | "";
    const char *email = doc["email"] | "";
    const char *mqttBroker = doc["mqttBroker"] | "";

    wifiSave.ssid = ssid;
    wifiSave.password = password;
    wifiSave.email = email;
    wifiSave.mqttBroker = mqttBroker;

    Serial.print("wifi.ssid: "); Serial.println(wifiSave.ssid);
    Serial.print("wifi.password: "); Serial.println(wifiSave.password);
    Serial.print("wifi.email: "); Serial.println(wifiSave.email);
     Serial.print("wifi.mqttBroker: "); Serial.println(wifiSave.mqttBroker);
    config.saveConfigToSPIFFS();
  }

  // 수신된 메시지에서 mac 주소를 읽어옵니다.
  String receivedMac = doc["mac"] | "";
  // 이 기기의 MAC 주소와 비교합니다.
  if (receivedMac != dev.mac) {
    Serial.println("Received MAC address does not match device MAC address. Ignoring message.");
    return;
  }

  if (order == 0) {
    //펌웨어 다운로드
    const char *fileName = doc["fileName"] | "";
    tool.download_program(fileName);
  }
  else if (order == 2) {
    //mqtt 로 전송된 출력을 실행한다.
    // JSON 메시지에서 "no"와 "value" 값을 읽어옵니다.
    int no = doc["no"] | -1;  // 유효하지 않은 인덱스로 초기화
    bool value = doc["value"] | false;
    dev.digitalWriteUpdateData(no, value);
  }
  else if (order==3) {
    //폰에서 화면을 초기화하기 위한 메세지 요청
    dev.sendStatusCheckChange(false);
  }
  else if (order == 4) {
    String oper = doc["oper"] | "";
    int pinIndex = doc["pI"] | -1;
    if (oper == "insert") {
      int startHour = doc["sH"] | 0;
      int startMinute = doc["sM"] | 0;
      int endHour = doc["eH"] | 0;
      int endMinute = doc["eM"] | 0;
      String repeatMode = doc["rM"] | "d";
      int dayOfWeek = doc["dW"] | -1;

      addTimeSlot(pinIndex, startHour, startMinute, endHour, endMinute, repeatMode, dayOfWeek);
      saveTimeSlotsToSPIFFS(pinIndex); // 타임슬롯 변경 시 SPIFFS에 저장
      Serial.println("Time slot added and saved to SPIFFS");

      // 해당 핀 인덱스의 스케줄 리스트 전송
      startSendingTimeSlots(pinIndex);
    }
    else if (oper == "delete") {
      int slotIndex = doc["slotIndex"] | -1;

      if (slotIndex >= 0 && slotIndex < timeSlots[pinIndex].size()) {
        removeTimeSlot(pinIndex, slotIndex);
        saveTimeSlotsToSPIFFS(pinIndex); // 타임슬롯 변경 시 SPIFFS에 저장
        Serial.println("Time slot removed and saved to SPIFFS");
        startSendingTimeSlots(pinIndex);
      }
      else {
        Serial.println("Invalid slot index.");
      }
    }
    else if (oper == "deleteAll") {
      removeAllTimeSlots(pinIndex);
      saveTimeSlotsToSPIFFS(pinIndex); // 모든 타임슬롯 삭제 후 SPIFFS에 저장
      Serial.println("All time slots removed for pin " + String(pinIndex));
    }
    else if (oper == "list") {
      Serial.println("Time slots list request received for pin " + String(pinIndex));
      startSendingTimeSlots(pinIndex);
    }
  }
  
  else if (order == 5) {
    String oper = doc["oper"] | "";
    if (oper == "save") {
      int portNo = doc["portNo"] | -1;
      JsonArray portStates = doc["portState"].as<JsonArray>();

      if (portNo >= 0 && portNo < (numberOfPins+2)) {
        dev.pinStateChanges[portNo][0].clear();  // 기존 데이터를 초기화
        dev.pinStateChanges[portNo][1].clear();  // 기존 데이터를 초기화

        int index = 0; // 배열 인덱스 변수

        for (JsonObject portState : portStates) {
          String mac = portState["mac"] | "";
          int port = portState["port"] | -1;
          bool value = portState["value"] | false;

          Serial.println("mac:" + mac + "  port: " + String(port) + "  value: " + String(value));
          // PinStateChange 객체 생성 및 데이터 설정
          PinStateChange change;
          change.mac = mac;
          change.port = port;
          change.value = value;
          change.timestamp = millis();

          // 인덱스에 따라 pinStateChanges에 저장
          dev.pinStateChanges[portNo][index].push_back(change);

          index++;
        }

        // 데이터 SPIFFS에 저장
        DynamicJsonDocument saveDoc(2048);
        JsonArray changesArray = saveDoc.createNestedArray("changes");

        for (const auto& ch : dev.pinStateChanges[portNo][0]) {
          JsonObject obj = changesArray.createNestedObject();
          obj["mac"] = ch.mac;
          obj["port"] = ch.port;
          obj["value"] = ch.value;
          obj["timestamp"] = ch.timestamp;
        }

        for (const auto& ch : dev.pinStateChanges[portNo][1]) {
          JsonObject obj = changesArray.createNestedObject();
          obj["mac"] = ch.mac;
          obj["port"] = ch.port;
          obj["value"] = ch.value;
          obj["timestamp"] = ch.timestamp;
        }

        String saveData;
        serializeJson(saveDoc, saveData);
        String fileName = "/pinState_" + String(portNo) + ".json";
        Serial.println("fileName : " + fileName);
        File file = SPIFFS.open(fileName, FILE_WRITE);
        if (!file) {
          Serial.println("Failed to open file for writing");
          return;
        }
        file.print(saveData);
        file.close();
        //Serial.println("Data saved to SPIFFS: " + saveData);
      }
    }
    else if (oper == "list") {
      int port = doc["portNo"] | -1;
      if (port >= 0 && port < numberOfPins) {
        DynamicJsonDocument responseDoc(2048);
        responseDoc["order"] = 6;
        responseDoc["mac"] = dev.mac;
        responseDoc["portNo"] = port;
        JsonArray portStates = responseDoc.createNestedArray("portState");

        for (const auto& change : dev.pinStateChanges[port][0]) {
          JsonObject obj = portStates.createNestedObject();
          obj["mac"] = change.mac;
          obj["port"] = change.port;
          obj["value"] = change.value;
        }
        
        for (const auto& change : dev.pinStateChanges[port][1]) {
          JsonObject obj = portStates.createNestedObject();
          obj["mac"] = change.mac;
          obj["port"] = change.port;
          obj["value"] = change.value;
        }

        String output;
        serializeJson(responseDoc, output);
        client.publish(wifi.outTopic, output.c_str());
        Serial.println("MQTT message sent: " + output);
      } else {
        Serial.println("Invalid port number: " + String(port));
      }
    }
    else if (oper == "delete") {
      int port = doc["portNo"] | -1;
      if (port >= 0 && port < numberOfPins) {
        dev.pinStateChanges[port][0].clear();
        dev.pinStateChanges[port][1].clear();
        
        PinStateChange defaultChange;
        defaultChange.mac = "";
        defaultChange.port = -1;
        defaultChange.value = false;
        defaultChange.timestamp = 0;

        dev.pinStateChanges[port][0].push_back(defaultChange);
        dev.pinStateChanges[port][1].push_back(defaultChange);

        Serial.println("Deleted pinStateChanges for port " + String(port));

        // 데이터 SPIFFS에 저장
        DynamicJsonDocument saveDoc(2048);
        JsonArray changesArray = saveDoc.createNestedArray("changes");

        for (const auto& ch : dev.pinStateChanges[port][0]) {
          JsonObject obj = changesArray.createNestedObject();
          obj["mac"] = ch.mac;
          obj["port"] = ch.port;
          obj["value"] = ch.value;
          obj["timestamp"] = ch.timestamp;
        }

        for (const auto& ch : dev.pinStateChanges[port][1]) {
          JsonObject obj = changesArray.createNestedObject();
          obj["mac"] = ch.mac;
          obj["port"] = ch.port;
          obj["value"] = ch.value;
          obj["timestamp"] = ch.timestamp;
        }

        String saveData;
        serializeJson(saveDoc, saveData);
        String fileName = "/pinState_" + String(port) + ".json";
        Serial.println("fileName : " + fileName);
        File file = SPIFFS.open(fileName, FILE_WRITE);
        if (!file) {
          Serial.println("Failed to open file for writing");
          return;
        }
        file.print(saveData);
        file.close();
        Serial.println("Data saved to SPIFFS: " + saveData);
      } else {
        Serial.println("Invalid port number: " + String(port));
      }
    }
    
  }
  else if (order == 8) {
    String type = doc["type"] | "";
    String oper = doc["oper"] | "";
    if (oper == "save") {
      SensorChange& changeLow = sensor.SensorChanges[type][0]; // 참조로 가져오기
      SensorChange& changeHigh = sensor.SensorChanges[type][1]; // 참조로 가져오기
      JsonArray portStates = doc["portState"].as<JsonArray>();
      if (portStates.size() >= 2) {
        JsonObject lowData = portStates[0];
        JsonObject highData = portStates[1];

        // Low 값 저장
        changeLow.value = doc["valueLow"].as<float>();
        changeLow.mac = lowData["mac"].as<String>();
        changeLow.port = lowData["port"].as<int>();
        changeLow.state = lowData["value"].as<bool>();

        // High 값 저장
        changeHigh.value = doc["valueHigh"].as<float>();
        changeHigh.mac = highData["mac"].as<String>();
        changeHigh.port = highData["port"].as<int>();
        changeHigh.state = highData["value"].as<bool>();
      }
      sensor.saveToSPIFFS();
      //sensor.printSensorData();
      Serial.println("Sensor data saved to SPIFFS.");
    }
    else if (oper == "list") {
      SensorChange changeLow = sensor.SensorChanges[type][0]; // 참조로 가져오기
      SensorChange changeHigh = sensor.SensorChanges[type][1]; // 참조로 가져오기
      String mac = doc["mac"] | "";
      // 응답 JSON 문서 생성
      DynamicJsonDocument responseDoc(1024);
      responseDoc["order"] = 8;
      responseDoc["type"] = type;
      responseDoc["mac"] = mac;
      responseDoc["calCali"] = sensor.sensorData[type];
      responseDoc["valueHigh"] = changeHigh.value;
      responseDoc["valueLow"] = changeLow.value;

      JsonArray responsePortStates = responseDoc.createNestedArray("portState");

      // SensorChanges에서 mac, port, state 값을 가져와 추가
      for (int i = 0; i < 2; i++) {
          JsonObject obj = responsePortStates.createNestedObject();
          SensorChange change = sensor.SensorChanges[type][i]; // i가 0이면 Low, 1이면 High
          obj["mac"] = change.mac;
          obj["port"] = change.port;
          obj["value"] = change.state;
      }

      // JSON 직렬화 및 MQTT 전송
      String responseData;
      serializeJson(responseDoc, responseData);
      
      //Serial.println(responseData);
      client.publish(wifi.outTopic, responseData.c_str());
    }
    else if (oper == "delete") {
      sensor.SensorChanges[type].clear();
      sensor.SensorChanges[type].push_back({-10000, "00:00:00:00:00:00", -1, false, millis()});
      sensor.SensorChanges[type].push_back({10000, "00:00:00:00:00:00", -1, false, millis()});
      //sensor.printSensorData();
      sensor.saveToSPIFFS();
    }
    else if (oper == "cali") {
      float valueCali = doc["valueCali"].as<float>();
      String mac = doc["mac"] | "";
      sensor.calSensorData[type] += (valueCali - sensor.sensorData[type]);
      //sensor.printSensorData();
      sensor.saveToSPIFFS();
    }
  }
}

void loop() {
  if(!ble.boot) {
    dev.loop();
    wifi.loop();
    timeManager.loop();
  }
  dev.checkFactoryDefault();
}

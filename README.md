
# 개요
GitHub의 i2r-05 레포지터리를 기반으로 학생들에게 IoT 및 임베디드 시스템을 교육합니다. 여기서는 i2r-05 보드를 활용하여 기본적인 하드웨어 이해부터 실습을 진행하고, On-Device AI(온디바이스 AI) 구현합니다. 그리고 스마트팜을 하시는 분들을 위해서 각종 센서와 액튜에이터를 자동으로 연결하여 스마트폰과 PC에서 모니터링/제어 할 수 있게 합니다.

# 교육 목표
- i2r-05 보드를 활용하여 하드웨어 및 펌웨어 개발의 기초를 익힘
- Wi-Fi 및 MQTT 기반의 IoT 통신을 실습하여 네트워크 프로그래밍 이해
- 머신러닝 및 On-Device AI 개념을 학습하고 실제 보드에서 실행 가능하도록 구현
- ESP32-S3의 강력한 연산 성능을 활용하여 실시간 데이터 분석 및 처리

# 교육 과정 구성
## (1) 기초 과정: i2r-05 보드 이해 및 환경 설정
- i2r-05 보드 개요 및 사양 분석
- 개발 환경 구축(Arduino IDE, ESP-IDF, PlatformIO)
- 기본적인 GPIO 제어 및 센서 연결 실습
- 시리얼 통신 및 Wi-Fi 설정
## (2) 중급 과정: IoT 및 네트워크 프로그래밍
- MQTT 프로토콜을 활용한 데이터 송수신
- 클라우드 서버(AWS, Firebase, Node.js 서버)와의 연동
- MongoDB와 연계하여 데이터 저장 및 분석
## (3) 고급 과정: On-Device AI 구현
- ESP32-S3의 AI 가속 기능(TensorFlow Lite Micro, Edge Impulse) 소개
- 간단한 머신러닝 모델(이미지 인식, 음성 인식) 학습 및 배포
- 실시간 센서 데이터를 활용한 AI 모델 적용
- 최적화 및 배터리 절약을 위한 저전력 설계

# 기대 효과
- 학생들이 IoT 및 AI 시스템의 기본 개념을 이해하고 직접 구현할 수 있도록 지원
- AI가 탑재된 임베디드 시스템을 직접 설계하고 응용할 수 있는 능력 배양
- 실제 산업 현장에서 활용 가능한 IoT 및 AI 기술 습득

ESP32 S3 16M Flash, AI IoT 개발용 보드
![i2r-05](https://github.com/user-attachments/assets/4ba415d9-08e6-4b2c-9396-1d3f35ffdfe2)
![i2r-05 회로도](https://github.com/user-attachments/assets/ed673ea3-f054-4370-ae92-979a29e72c4d)

# ESP32-S3-WROOM-1-N16R8 설명
ESP32-S3-WROOM-1-N16R8은 Espressif Systems에서 개발한 고성능 Wi-Fi 및 Bluetooth LE 모듈입니다. 이 모듈은 ESP32-S3 SoC를 기반으로 하며, **16MB의 플래시 메모리(N16)와 8MB의 PSRAM(R8)**을 탑재하고 있습니다.

### 1. 주요 특징
#### (1) CPU 및 성능
- 듀얼코어 Xtensa® LX7 32비트 프로세서
- 최대 240MHz 클럭 속도
- AI 및 DSP 기능 향상 (벡터 연산 가속 지원)
#### (2) 메모리
- 16MB 플래시 메모리 (N16)
- 8MB PSRAM (R8) → AI, 이미지 처리, 데이터 버퍼링에 유용
#### (3)무선 기능
- Wi-Fi 802.11 b/g/n (2.4GHz)
- Bluetooth 5.0 LE (Low Energy)
- WPA3 보안 지원
#### (4) USB 기능
- Full-Speed USB OTG (USB 1.1)
- USB CDC/JTAG 디버깅 지원
- USB-Serial 통신 가능 (UART 필요 없음)
#### (5) GPIO 및 확장성
- 최대 45개의 GPIO 핀 제공
- SPI, I2C, UART, PWM, ADC, DAC 지원
- JTAG 디버깅 지원
#### (6) 저전력 설계
- Deep Sleep 모드에서 μA 단위의 전력 소모
- ULP (Ultra-Low Power) 코프로세서 내장
#### (7) AI 및 머신러닝
- Vector Extension (AI 가속기) 포함
- TensorFlow Lite, ESP-DSP 최적화 지원

### 2. ESP32-S3-WROOM-1-N16R8의 장점
- 대용량 PSRAM (8MB) 탑재 : 이미지 처리, AI 모델, 오디오 스트리밍 등에 적합
- Wi-Fi + Bluetooth 동시 지원 : BLE를 활용한 IoT 기기 및 스마트 디바이스 개발 가능
- USB OTG 지원 : 직접 USB 장치와 통신 가능 (키보드, 마우스, 스토리지 연결)
- 보안 기능 강화 : Secure Boot, Flash Encryption, AES-XTS 암호화 지원
- ESP-IDF 및 Arduino 지원 : ESP-IDF, Arduino, MicroPython 등 다양한 개발 환경 제공

### 3. ESP32-S3-WROOM-1-N16R8의 활용 사례
- AI 및 머신러닝
- 온디바이스 AI (On-Device AI)
- 얼굴 인식, 음성 인식, AIoT (AI + IoT)
- IoT 및 스마트 디바이스
- Wi-Fi 기반 스마트 홈 시스템
- BLE 센서 네트워크
- 임베디드 시스템 개발
- USB 기반 주변기기 개발
- 드론, 로봇 제어 시스템
- 실시간 데이터 처리
- MQTT 및 WebSocket을 활용한 실시간 데이터 전송
- 센서 데이터를 클라우드로 전송하여 모니터링

### 4. ESP32-S3-WROOM-1-N16R8 개발 환경
- ESP-IDF (Espressif 공식 SDK)
- Arduino IDE
- Visual Code
- PlatformIO
- MicroPython
- FreeRTOS 기반 멀티태스킹 지원

arduino IDE를 사용하여 컴파일 하려면 tool 에서 보드설정은 아래와 같습니다.
<img width="300" src="https://github.com/user-attachments/assets/786d4d17-e62d-4c16-bae7-ef9bf459cffe" />    


# 1. ESP32 블루투스(BLE) 통신 개요 및 실습
ESP32는 Bluetooth Classic과 Bluetooth Low Energy(BLE)를 모두 지원하는 강력한 무선 모듈입니다.
BLE는 저전력으로 데이터를 전송할 수 있어, IoT(사물인터넷) 기기에서 널리 사용됩니다.
블루투스는 스마트폰의 블루투스와 연결하여 와이파이 정보를 보드로 전송하여 통신 접속을 하게 합니다. 

# 1. i2r-05 Shield V1 보드
<img width="482" alt="i2r Shield V1" src="https://github.com/user-attachments/assets/fcc54b19-4a71-4e17-8461-90e9ae864907" />    

 Arduino 기반의 다목적 학습 보드로, 추가적인 납땜이나 복잡한 배선 없이 다양한 실험을 수행할 수 있습니다. 보드에 내장된 다양한 모듈을 활용하여, 코드를 다운로드하는 것만으로 실험을 완료할 수 있도록 설계되었습니다.
 
| i2r-05 | name | 설명 |
|--------|-------|-------|
| 47 | LED1 D13 파랑 | LED 출력 |
| 38 | LED2 D12 빨강 | LED 출력 |
| 8 | SW1 D2 | 입력 버튼  |
| 9 | SW2 D3 | 입력 버튼  |
| 1 | Rotation A0 | 가변 저항 ADC |
| 11 | Buzzer D5 | 부저 알람  |
| 2 | Light A1 | 조도 센서 |
| 10 | DHT11 D4 | 	온도 측정  |
| 4  | LM35 A2 | 	온도 측정  |
| 15 | RGB LED red | 빨간색 Led 입력 핀 |
| 21 | RGB LED green | 녹색 Led 입력 핀 |
| 16 | RGB LED blue | 파란색 Led 입력 핀 |
| 12 | IR Receiver D6 | 	적외선 리모컨 제어 |


## 1.1 RGB Led 제어 (48번핀)
48번 RGB Led가 연결되어 있어서 색상을 제어하는 프로그램 입니다.    
<img src="https://github.com/user-attachments/assets/8d3ef6cc-9df4-47de-a5eb-6bd3402c9eb4" alt="RGB Led" width="100">  48번 RGB Led 제어    
```
esp32 s3 를 사용하고 48번핀에 RGB Led  가 연결되 있습니다. 이 한선으로 칼라를 제어 합니다. 빨강 파랑 녹색 불들어오게 프로그램 해줘
```
아두이노 프로그램
```
#include <Adafruit_NeoPixel.h>

#define LED_PIN 48      // RGB LED가 연결된 핀 번호
#define NUM_PIXELS 1    // 제어할 LED 개수

Adafruit_NeoPixel strip(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();         // LED 제어를 위한 초기화
  strip.show();          // LED를 초기화 상태로 설정
}

void loop() {
  setColor(255, 0, 0);   // 빨강색
  delay(1000);           // 1초 대기
  setColor(0, 255, 0);   // 녹색
  delay(1000);           // 1초 대기
  setColor(0, 0, 255);   // 파랑색
  delay(1000);           // 1초 대기
}

// RGB 색상 설정 함수
void setColor(uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(0, strip.Color(red, green, blue)); // 첫 번째 LED의 색상 설정
  strip.show();                                         // 설정한 색상을 출력
}
```
## 1.2 Led 제어 (47, 38 번핀)
ESP32-S3 보드의 47번 핀과 38번 핀에 연결된 LED를 1초 간격으로 교차로 점등하는 Arduino 프로그램입니다.    

<img src="https://github.com/user-attachments/assets/8d3ef6cc-9df4-47de-a5eb-6bd3402c9eb4" alt="Led Control" width="100">  47, 38번 Led 제어    
```
esp32 s3 보드 47, 38번 핀에 led가 연결되어 있습니다. 1초 간격으로 교차로 점등하게 프록램 해줘요
```
아두이노 프로그램
```
#define LED1_PIN 47 // 첫 번째 LED 핀 번호
#define LED2_PIN 38 // 두 번째 LED 핀 번호

void setup() {
  pinMode(LED1_PIN, OUTPUT); // LED1 핀을 출력 모드로 설정
  pinMode(LED2_PIN, OUTPUT); // LED2 핀을 출력 모드로 설정

  digitalWrite(LED1_PIN, LOW); // LED1 초기 상태 꺼짐
  digitalWrite(LED2_PIN, LOW); // LED2 초기 상태 꺼짐
}

void loop() {
  digitalWrite(LED1_PIN, HIGH); // LED1 켜기
  digitalWrite(LED2_PIN, LOW);  // LED2 끄기
  delay(1000);                 // 1초 대기

  digitalWrite(LED1_PIN, LOW); // LED1 끄기
  digitalWrite(LED2_PIN, HIGH); // LED2 켜기
  delay(1000);                 // 1초 대기
}
```

## 1.3 스위치 입력 (8, 9 번핀)
ESP32-S3 보드의 8번 핀과 9번 핀에 연결된 스위치를 감지하고, 스위치를 누를 때마다 시리얼 포트로 해당 상태를 출력하는 Arduino 프로그램입니다.    

<img src="https://github.com/user-attachments/assets/8d3ef6cc-9df4-47de-a5eb-6bd3402c9eb4" alt="Switch Input" width="100">  8,9번 스위치 입력    
```
esp32 s3 보드 8,9 번핀에 스위치가 연결되어 있습니다. 스위치를 누르면 시리얼포트로 이를 알려줘
```
아두이노 프로그램
```
#define SWITCH1_PIN 8  // 첫 번째 스위치 핀 번호
#define SWITCH2_PIN 9  // 두 번째 스위치 핀 번호

void setup() {
  Serial.begin(115200);              // 시리얼 통신 초기화
  pinMode(SWITCH1_PIN, INPUT_PULLUP); // 스위치1 핀을 입력 모드로 설정 및 내부 풀업 저항 활성화
  pinMode(SWITCH2_PIN, INPUT_PULLUP); // 스위치2 핀을 입력 모드로 설정 및 내부 풀업 저항 활성화
}

void loop() {
  if (digitalRead(SWITCH1_PIN) == LOW) { // 스위치1이 눌렸을 때 (풀업 상태에서 LOW)
    Serial.println("Switch 1 Pressed");
    delay(200); // debounce 방지를 위한 딜레이
  }
  if (digitalRead(SWITCH2_PIN) == LOW) { // 스위치2가 눌렸을 때 (풀업 상태에서 LOW)
    Serial.println("Switch 2 Pressed");
    delay(200); // debounce 방지를 위한 딜레이
  }
}
```

## 1.4 아나로그 입력 (1 번핀)
ESP32-S3 보드의 1번 핀에 아날로그 입력이 연결하여 해당 값을 읽어서 시리얼 포트로 출력하는 Arduino 프로그램입니다.
<img src="https://github.com/user-attachments/assets/8d3ef6cc-9df4-47de-a5eb-6bd3402c9eb4" alt="Analog Input" width="100">  1번 아나로그 입력    
```
esp32 s3 보드 1 번핀에 아나로그 입력이 연결되어 있습니다. 입력 값을 시리얼포트로 알려줘
```
- ANALOG_PIN은 1번 핀으로 설정하여 ADC1 채널을 사용합니다.
- analogReadResolution(12)를 사용하여 아날로그 입력 값을 **12비트 해상도(0 ~ 4095)**로 읽습니다. 기본값은 12비트입니다.
- analogRead(ANALOG_PIN) 함수로 아날로그 값을 읽습니다. 읽은 값은 0에서 4095 범위의 정수로 반환됩니다.
- 
아두이노 프로그램
```
#define ANALOG_PIN 1  // 아날로그 입력 핀 번호

void setup() {
  Serial.begin(115200);       // 시리얼 통신 초기화
  analogReadResolution(12);   // 12비트 해상도 설정 (0 ~ 4095)
}

void loop() {
  int analogValue = analogRead(ANALOG_PIN);  // 아날로그 값 읽기
  Serial.print("Analog Value: ");
  Serial.println(analogValue);              // 값을 시리얼로 출력
  delay(500);                               // 500ms 대기
}
```

## 1.5 Buzzer 출력 (11 번핀)

ESP32-S3 보드의 11번 핀에 연결된 부저를 제어하기 위해 Arduino 코드를 작성했습니다. 부저는 PWM 신호를 통해 제어되며, 이를 통해 다양한 음을 생성할 수 있습니다.    

<img src="https://github.com/user-attachments/assets/8d3ef6cc-9df4-47de-a5eb-6bd3402c9eb4" alt="Buzzer 출력" width="100">  11번 Buzzer 출력    
```
esp32 s3 보드 11 번핀에 Buzzer가 연결되어 있습니다. 출력이 나오게 프로그램해줘
```
아두이노 프로그램
```
#define ANALOG_PIN 1  // 아날로그 입력 핀 번호

void setup() {
  Serial.begin(115200);       // 시리얼 통신 초기화
  analogReadResolution(12);   // 12비트 해상도 설정 (0 ~ 4095)
}

void loop() {
  int analogValue = analogRead(ANALOG_PIN);  // 아날로그 값 읽기
  Serial.print("Analog Value: ");
  Serial.println(analogValue);              // 값을 시리얼로 출력
  delay(500);                               // 500ms 대기
}
```
<img src="https://github.com/user-attachments/assets/8d3ef6cc-9df4-47de-a5eb-6bd3402c9eb4" alt="음악연주" width="100">  음악연주   
```
부져로 음악연주하는 프로그램 만들어줘
```
ESP32-S3의 부저를 사용해 간단한 음악을 연주하는 프로그램을 작성할 수 있습니다. 아래는 아두이노의 tone() 함수를 사용하여 **"Twinkle Twinkle Little Star"**를 연주하는 예제입니다. 
```
#define BUZZER_PIN 11  // 부저가 연결된 핀 번호

// 음계의 주파수 정의 (도, 레, 미, 파, 솔, 라, 시)
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523

// 멜로디 음계 배열
int melody[] = {
  NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
  NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4
};

// 각 음계의 지속 시간 (4분의 1, 8분의 1 등)
int noteDurations[] = {
  4, 4, 4, 4, 4, 4, 2,
  4, 4, 4, 4, 4, 4, 2
};

void setup() {
  pinMode(BUZZER_PIN, OUTPUT); // 부저 핀을 출력 모드로 설정
}

void loop() {
  // 멜로디 재생
  for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
    int noteDuration = 1000 / noteDurations[i];  // 음 길이 계산
    tone(BUZZER_PIN, melody[i], noteDuration);   // 음 출력
    delay(noteDuration * 1.3);                   // 음 사이 간격 (지속 시간의 30% 추가)
    noTone(BUZZER_PIN);                          // 음 끄기
  }

  delay(2000); // 전체 멜로디 끝난 후 2초 대기
}
```

## 1.6 Light 센서 (2 번핀)

ESP32-S3 보드의 11번 핀에 연결된 부저를 제어하기 위해 Arduino 코드를 작성했습니다. 부저는 PWM 신호를 통해 제어되며, 이를 통해 다양한 음을 생성할 수 있습니다.    

<img src="https://github.com/user-attachments/assets/8d3ef6cc-9df4-47de-a5eb-6bd3402c9eb4" alt="Light 센서" width="100">  Light 센서
```
sp32 s3 보드 2 번핀에 Light 센서가 연결되어 있습니다. 조도를 출력하는 프로그램해줘
```
- LIGHT_SENSOR_PIN 설정: 센서가 연결된 핀 번호를 #define으로 정의합니다.
- analogReadResolution(12): ESP32-S3의 ADC는 최대 12비트 해상도를 지원합니다. 이를 설정해 ADC 값이 0~4095 범위로 출력되도록 합니다.
- ADC 값을 전압으로 변환: 조도 센서에서 출력된 값은 ADC를 통해 읽은 뒤, 전압 값으로 변환됩니다.
계산 공식: 전압 = (ADC 값 / 최대 ADC 값) * 기준 전압
- Serial 출력: 읽은 ADC 값과 전압 값을 시리얼 모니터에 출력합니다
  
아두이노 프로그램
```
#define LIGHT_SENSOR_PIN 2  // Light 센서가 연결된 핀

void setup() {
  Serial.begin(115200);           // 시리얼 통신 초기화
  pinMode(LIGHT_SENSOR_PIN, INPUT);  // Light 센서 핀을 입력으로 설정
  analogReadResolution(12);       // 12비트 ADC 해상도 설정 (0~4095)
}

void loop() {
  int lightValue = analogRead(LIGHT_SENSOR_PIN);  // Light 센서 값 읽기
  float voltage = (lightValue / 4095.0) * 3.3;    // ADC 값을 전압으로 변환 (ESP32-S3의 기본 전압 기준은 3.3V)

  Serial.print("Light Sensor Value: ");
  Serial.print(lightValue);                       // ADC 값 출력
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);                       // 전압 값 출력
  Serial.println(" V");

  delay(500);  // 500ms 대기
}
```

## 1.7 DHT11 온습도 센서 (10 번핀)

이 프로그램은 DHT 라이브러리를 사용하여 온도와 습도를 읽고 시리얼 모니터에 출력합니다.   

<img src="https://github.com/user-attachments/assets/8d3ef6cc-9df4-47de-a5eb-6bd3402c9eb4" alt="DHT11 온습도 센서" width="100">  DHT11 온습도 센서
```
sp32 s3 보드 10 번핀에  DHT11 온습도 센서가 연결되어 있습니다. 온습도를 출력하는 프로그램해
```
- 라이브러리 설치 : DHT sensor library by Adafruit 
- #define DHTTYPE DHT11: DHT 센서의 타입을 지정합니다.
- 데이터 읽기:
readHumidity(): 습도를 읽습니다.
readTemperature(): 온도를 읽습니다
- 출력 형식: 온도는 °C, 습도는 % 단위로 출력됩니다.
- 지연 시간: 센서의 데이터 갱신 주기(최대 2초)에 맞춰 delay(2000)을 사용하여 2초마다 값을 읽습니다.

아두이노 프로그램
```
#include "DHT.h"  //DHT sensor library by Adafruit

#define DHTPIN 10       // DHT 센서가 연결된 핀 번호
#define DHTTYPE DHT11   // 사용하는 센서의 유형 (DHT11)

// DHT 센서 객체 생성
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);  // 시리얼 통신 초기화
  dht.begin();           // DHT 센서 초기화
  Serial.println("DHT11 센서를 이용한 온습도 측정 시작");
}

void loop() {
  float humidity = dht.readHumidity();     // 습도 값 읽기
  float temperature = dht.readTemperature(); // 온도 값 읽기

  // DHT 센서 데이터 유효성 확인
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("센서로부터 데이터를 읽을 수 없습니다!");
  } else {
    Serial.print("온도: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("습도: ");
    Serial.print(humidity);
    Serial.println(" %");
  }

  delay(2000); // 2초마다 측정
}
```

## 1.8 LM35 온도 센서 (4 번핀)

LM35 온도 센서는 아날로그 출력 타입의 온도 센서입니다. 온도를 섭씨(°C)로 측정하며, 출력 전압은 섭씨 1도당 10mV입니다. 이를 ESP32-S3의 4번 핀에 연결하여 값을 읽고 온도를 출력하는 프로그램은 다음과 같이 작성할 수 있습니다.

<img src="https://github.com/user-attachments/assets/8d3ef6cc-9df4-47de-a5eb-6bd3402c9eb4" alt="LM35 온도 센서" width="100">  LM35 온도 센서
```
LM35 센서가 4번핀에 연결되어 있어요 프로그램 해주세요
```
아두이노 프로그램
```
#define LM35_PIN 4  // LM35 센서의 출력 핀이 연결된 ESP32-S3의 핀 번호

void setup() {
  Serial.begin(115200);           // 시리얼 통신 초기화
  analogReadResolution(12);       // 12비트 ADC 해상도 설정 (0~4095)
  Serial.println("LM35 온도 센서 사용 예제");
}

void loop() {
  int adcValue = analogRead(LM35_PIN);    // LM35 센서에서 아날로그 값 읽기
  float voltage = (adcValue / 4095.0) * 3.3;  // ADC 값을 전압으로 변환 (ESP32-S3의 참조 전압: 3.3V)
  float temperature = voltage * 100.0;       // 전압을 온도로 변환 (1V = 100°C)

  Serial.print("ADC 값: ");
  Serial.print(adcValue);
  Serial.print(" | 전압: ");
  Serial.print(voltage, 2);
  Serial.print(" V | 온도: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

  delay(1000);  // 1초 대기
}
```

## 1.9 RGB Led 센서 (15,21,16 번핀 red,green,blue)

이 프로그램은 RGB LED의 각 핀에 대해 LED의 밝기를 제어합니다. 빨강, 초록, 파랑, 흰색의 조합을 통해 다양한 색상을 순차적으로 출력합니다.

<img src="https://github.com/user-attachments/assets/8d3ef6cc-9df4-47de-a5eb-6bd3402c9eb4" alt="RGB Led 센서" width="100">  RGB Led 센서
```
15,21,16 이 차례로 red, green, blue 가 연결되어 있습니다. led 프로그램 해줘
```
아두이노 프로그램
```
// Define the RGB LED pins
const int redPin = 15;
const int greenPin = 21;
const int bluePin = 16;

void setup() {
  // Initialize the RGB LED pins as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop() {
  // Red
  setColor(255, 0, 0); // Turn the RGB LED red
  delay(1000);         // Wait for a second
  // Green
  setColor(0, 255, 0); // Turn the RGB LED green
  delay(1000);         // Wait for a second
  // Blue
  setColor(0, 0, 255); // Turn the RGB LED blue
  delay(1000);         // Wait for a second
  // White
  setColor(255, 255, 255); // Turn the RGB LED white
  delay(1000);             // Wait for a second
  // Off
  setColor(0, 0, 0); // Turn the RGB LED off
  delay(1000);       // Wait for a second
}


// Function to set the color of the RGB LED
void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}
```

# Otto Ninja 로봇
배선연결 요약
|이름 | 핀번호 |
|-----|-------|
| LF | 7번 |
| LL | 6번 |
| RF | 5번 |
| RL | 4번 |

```
서보모터를 esp32 보드 i2r-05 에 연결합니다. LF 7번, LL 6번, RF 5번, RL 4번 핀에 연결 했습니다.
LF RF 는 180도 각도조절 서보모터이고 LL RL 은 360도 회전하는 서보모터 입니다.
ESP32Servo.h 를 이용하여 ninja otto 로봇을 아두이노 프로그램 하려고 합니다.
LF 7번 핀을 이용해서 회전 제어하는 예제 프로그램 만들어 주세요
```

✅ LF 서보모터 각도 테스트 프로그램 (핀 7번)
```
#include <ESP32Servo.h>

// LF 서보모터를 7번 핀에 연결
const int LF_PIN = 7;
Servo lfServo;

void setup() {
  Serial.begin(115200);

  // 180도 서보모터 초기화
  lfServo.setPeriodHertz(50);  // 서보 주파수 설정 (일반적으로 50Hz)
  lfServo.attach(LF_PIN, 500, 2400);  // 최소/최대 펄스 폭 (마이크로초 단위, 서보에 따라 다름)
  Serial.println("LF 서보모터 제어 시작");
}

void loop() {
  Serial.println("0도");
  lfServo.write(0);  // 0도 위치로 이동
  delay(1000);

  Serial.println("90도");
  lfServo.write(90);  // 90도 위치로 이동
  delay(1000);

  Serial.println("180도");
  lfServo.write(180);  // 180도 위치로 이동
  delay(1000);

  Serial.println("90도로 복귀");
  lfServo.write(90);  // 다시 중간 위치로
  delay(1000);
}

```

✅ LL 360 서보모터 전진 후진 테스트 프로그램 (핀 6번)
```
#include <ESP32Servo.h>

Servo servo360;

void setup() {
  Serial.begin(115200);
  servo360.setPeriodHertz(50);
  servo360.attach(6, 500, 2400);  // 핀 6번에 연결

  Serial.println("360도 서보모터 정지값 보정 테스트");
}

void loop() {
  // 전진
  servo360.write(120);
  Serial.println("▶ 전진");
  delay(2000);

  // 정지 - 보정된 마이크로초 사용
  servo360.writeMicroseconds(1500); // ← 필요시 1485~1515로 조정
  Serial.println("⏹ 정지");
  delay(2000);

  // 후진
  servo360.write(60);
  Serial.println("◀ 후진");
  delay(2000);

  // 정지
  servo360.writeMicroseconds(1500); // ← 다시 정지
  Serial.println("⏹ 정지");
  delay(2000);
}
```

✅ LL 360 서보모터 속도 조절 원리

|펄스(ms)	| 마이크로초(us) |	동작 |	속도 |
|---------|---------------|-----|--------|
|1.5ms	   | 1500          | 정지|  	0%   |
|   ↑     |	1600~2400	    | 전진|	느림 → 빠름|
|   ↓     |	1400~500      |	후진|	느림 → 빠름|


LL 서보모터 전진 속도제어 테스트 프로그램 (핀 6번)
속도제어는 1600 에서 2400까지 가능하나 저속에서 소리가만나고 회전하지 않을 때는 공급되는 전류가 약해서 그러니 건전지를 새것으로 교체하거나 별도의 외부 전원을 공급하면 정상 동작 합니다.
```
#include <ESP32Servo.h>

Servo servoLL;

void setup() {
  Serial.begin(115200);

  servoLL.setPeriodHertz(50);         // 서보모터용 50Hz 주파수
  servoLL.attach(6, 500, 2400);       // GPIO 6번, 500~2400us 펄스폭 설정

  Serial.println("LL 모터 전진 속도 증가 테스트 시작");
}

void loop() {
  // 1500us가 정지. 1520~1700까지 점점 빨라짐
  for (int pulse = 1700; pulse <= 1900; pulse += 10) {
    servoLL.writeMicroseconds(pulse);
    Serial.print("전진 PWM: ");
    Serial.print(pulse);
    Serial.println("us");
    delay(1000);  // 속도 변화 관찰을 위한 대기
  }

  // 정지
  servoLL.writeMicroseconds(1500);
  Serial.println("정지 (1500us)");
  delay(3000); // 다음 루프 전 잠시 대기
}
```
✅ MQTT 통신에 의한 전진 후진 통신 프로그램
chatGpt 에 다음과 같이 요구하세 
```
아두이노 esp32 s3로 프로그램 합니다.
와이파이 연결은 ssid i2r password 00000000 입니다.
mqtt 서버는 ai.doowon.ac.kr 포트는 1883 
intopic i2r/kdi6033@gmail.com/in  outtopic i2r/kdi6033/out
프로토콜은 order=1 이면 전진, order=2 이면 후진하게 해줘
ArduinoJson.h를 사용해서 프로그램 해줘
전후진 프로그램은 앞의 예제를 참조해서 작성해줘
```
#### ✅ LL 모터만 전후진 아두이노 프로그램
출력된 아두이노 프로그램
mqtt 통신으로 {"order":1} 을 보내면 전진 {"order":2} 를 보내면 추진 합니다.
이것은 react로 작성하세요. 와이파이 정보와 intopic, outtopic은 자신의 것으로 수정하세요
```
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// WiFi 정보
const char* ssid = "i2r";
const char* password = "00000000";

// MQTT 서버 정보
const char* mqtt_server = "ai.doowon.ac.kr";
const int mqtt_port = 1883;
const char* intopic = "i2r/kdi6033@gmail.com/in";
const char* outtopic = "i2r/kdi6033@gmail.com/out";

// 서보모터 관련
Servo servoLL;
const int servoPin = 6; // LL 모터 핀

WiFiClient espClient;
PubSubClient client(espClient);

// 서보 제어 함수
void controlServo(int order) {
  if (order == 1) {
    // 전진
    servoLL.writeMicroseconds(1800);
    Serial.println("전진 명령 - PWM: 1800us");
  } else if (order == 2) {
    // 후진
    servoLL.writeMicroseconds(1200);
    Serial.println("후진 명령 - PWM: 1200us");
  } else {
    // 정지
    servoLL.writeMicroseconds(1500);
    Serial.println("정지 명령 - PWM: 1500us");
  }
}

// MQTT 메시지 수신 콜백
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("수신 메시지: ");
  Serial.println(msg);

  // JSON 파싱
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, msg);
  if (error) {
    Serial.print("JSON 파싱 실패: ");
    Serial.println(error.c_str());
    return;
  }

  int order = doc["order"];
  Serial.print("수신된 order: ");
  Serial.println(order);
  controlServo(order);
}

// MQTT 연결 함수
void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT 연결 시도 중...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("MQTT 연결 성공");
      client.subscribe(intopic);
    } else {
      Serial.print("실패, 상태코드: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // 서보 초기화
  servoLL.setPeriodHertz(50);  // 서보 주파수 설정
  servoLL.attach(servoPin, 500, 2400); // 0도~180도에 해당하는 펄스 폭
  servoLL.writeMicroseconds(1500);  // 초기 상태 정지

  // WiFi 연결
  WiFi.begin(ssid, password);
  Serial.print("WiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi 연결 완료");

  // MQTT 설정
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

```

#### ✅ LL RL 모터 전후진 아두이노 프로그램
chatGPT에 다음과 같이 요구한다.
```
https://github.com/kdi6033/i2r-05/blob/main/README.md#otto-ninja-%EB%A1%9C%EB%B4%87
여기 참조해서 LL(6번핀) RL(4번핀) 두개의 모터가 같이 동작해서 전진 후진 동작하게, 그리고 order=0 는 정지 하게  다음 프로그램 수정해줘
여기에 앞에 프로그램응 복사해서 넣어 주세요
```
```
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// WiFi 정보
const char* ssid = "i2r";
const char* password = "00000000";

// MQTT 서버 정보
const char* mqtt_server = "ai.doowon.ac.kr";
const int mqtt_port = 1883;
const char* intopic = "i2r/kdi6033@gmail.com/in";
const char* outtopic = "i2r/kdi6033@gmail.com/out";

// 서보모터 관련
Servo servoLL;
const int servoPin = 6; // LL 모터 핀

WiFiClient espClient;
PubSubClient client(espClient);

// 서보 제어 함수
void controlServo(int order) {
  if (order == 1) {
    // 전진
    servoLL.writeMicroseconds(1800);
    Serial.println("전진 명령 - PWM: 1800us");
  } else if (order == 2) {
    // 후진
    servoLL.writeMicroseconds(1200);
    Serial.println("후진 명령 - PWM: 1200us");
  } else {
    // 정지
    servoLL.writeMicroseconds(1500);
    Serial.println("정지 명령 - PWM: 1500us");
  }
}

// MQTT 메시지 수신 콜백
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("수신 메시지: ");
  Serial.println(msg);

  // JSON 파싱
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, msg);
  if (error) {
    Serial.print("JSON 파싱 실패: ");
    Serial.println(error.c_str());
    return;
  }

  int order = doc["order"];
  Serial.print("수신된 order: ");
  Serial.println(order);
  controlServo(order);
}

// MQTT 연결 함수
void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT 연결 시도 중...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("MQTT 연결 성공");
      client.subscribe(intopic);
    } else {
      Serial.print("실패, 상태코드: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // 서보 초기화
  servoLL.setPeriodHertz(50);  // 서보 주파수 설정
  servoLL.attach(servoPin, 500, 2400); // 0도~180도에 해당하는 펄스 폭
  servoLL.writeMicroseconds(1500);  // 초기 상태 정지

  // WiFi 연결
  WiFi.begin(ssid, password);
  Serial.print("WiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi 연결 완료");

  // MQTT 설정
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
```

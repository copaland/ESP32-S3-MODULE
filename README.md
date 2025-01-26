# i2r-05
ESP32 S3 16M Flash, AI IoT 개발용 보드


![i2r-05](https://github.com/user-attachments/assets/ba88e650-298e-4400-9a06-2c0836869400)
![i2r-05 회로도](https://github.com/user-attachments/assets/ed673ea3-f054-4370-ae92-979a29e72c4d)
<img width="482" alt="i2r Shield V1" src="https://github.com/user-attachments/assets/fcc54b19-4a71-4e17-8461-90e9ae864907" />

| i2r-05 | name |
|--------|-------|
| 47 | LED1 D13 파랑 |
| 38 | LED2 D12 빨간 |
| 15 | RGB LED red |
| 16 | RGB LED green |
| 21 | RGB LED blue |
| 11 | Buzzer D5 |
| 10 | DHT11 D4 |
| 1 | Rotation A0 |
| 8 | SW1 D2 |
| 9 | SW2 D3 |
| 2 | Light A1 |
| 12 | IR Receiver D6 |
|    | LM35 A2 |

# 1. i2r-05 AI 보드 Arduino IDE 프로그램
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
}7,
```
## 1.2 Led 제어 (47, 38 번핀)

<img src="https://github.com/user-attachments/assets/8d3ef6cc-9df4-47de-a5eb-6bd3402c9eb4" alt="Led Control" width="100">  47, 38번 Led 제어    
```
```

#include <Wire.h>
#include <Adafruit_INA219.h>

// 점검할 센서 설정 (현재 사용 중인 주소들)
Adafruit_INA219 sensors[] = {Adafruit_INA219(0x41), Adafruit_INA219(0x44), Adafruit_INA219(0x40)};
const char* labels[] = {"Solar(0x41)", "Wind(0x44)", "Batt(0x40)"};

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Wire.begin();
  Wire.setWireTimeout(3000, true); // 노이즈로 인한 멈춤 방지
}

void loop() {
   Serial.println(F("========================================"));
  Serial.println(F("   INA219 센서 건강 상태 정밀 진단"));
  Serial.println(F("========================================"));

  for (int i = 0; i < 3; i++) {
    delay(1000);
    Serial.print(F("\n[")); Serial.print(labels[i]); Serial.println(F(" 점검 시작]"));

    // 1단계: I2C 통신 확인
    Wire.beginTransmission(0x40 + (i == 0 ? 1 : (i == 1 ? 4 : 0))); // 주소 매칭
    if (Wire.endTransmission() == 0) {
      Serial.println(F(" 1단계: I2C 물리적 연결 [성공]"));
    } else {
      Serial.println(F(" 1단계: I2C 연결 실패 [배선 확인 필요]"));
      continue;
    }

    // 2단계: 칩 초기화 확인
    if (sensors[i].begin()) {
      Serial.println(F(" 2단계: 소프트웨어 초기화 [성공]"));
      sensors[i].setCalibration_32V_2A();
    } else {
      Serial.println(F(" 2단계: 칩 응답 없음 [칩 손상 의심]"));
      continue;
    }

    // 3단계: 데이터 무결성 점검 (5회 반복)
    Serial.println(F(" 3단계: 측정값 안정성 테스트..."));
    for (int j = 1; j <= 5; j++) {
      float v = sensors[i].getBusVoltage_V();
      float c = sensors[i].getCurrent_mA();
      
      Serial.print(F("   (")); Serial.print(j); Serial.print(F("/5) 전압: "));
      Serial.print(v); Serial.print(F("V | 전류: "));
      Serial.print(c); Serial.println(F("mA"));
      
      // 아무것도 연결 안 했는데 전류가 높게 나오면 션트 저항 손상
      if (abs(c) > 50.0) {
        Serial.println(F("   !! 경고: 전류 오차가 큽니다. 션트 저항 손상 가능성 높음."));
      }
      delay(200);
    }
  }
  Serial.println(F("\n진단이 종료되었습니다."));
}


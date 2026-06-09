const int laserPin = 23; // 레이저 제어 (HIGH일 때 ON)
const int dotPin = 34;   // 짧은 신호 인터럽트 핀
const int dashPin = 35;  // 긴 신호 인터럽트 핀

// 인터럽트에서 변경되는 변수는 반드시 volatile 선언!
volatile bool dotDetected = false;
volatile bool dashDetected = false;

// --- ESP32 전용 초고속 인터럽트 서비스 루틴(ISR) ---
void IRAM_ATTR catchDot() {
  dotDetected = true;
}

void IRAM_ATTR catchDash() {
  dashDetected = true;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(laserPin, OUTPUT);
  pinMode(dotPin, INPUT);
  pinMode(dashPin, INPUT);
  
  // ESP32 인터럽트 연결: 신호가 0V -> 5V(3.3V)로 튀어오를 때(RISING) 즉시 함수 실행!
  attachInterrupt(dotPin, catchDot, RISING);
  attachInterrupt(dashPin, catchDash, RISING);
  
  digitalWrite(laserPin, LOW); // 대기 상태 (레이저 끔)
  Serial.println("=== ESP32 인터럽트 기반 1~40ms 정밀 스윕 가동 ===");
  delay(2000);
}

void loop() {
  // 1ms부터 40ms까지 1ms(1000us) 단위로 정밀 테스트
  for (int duration = 1; duration <= 40; duration += 1) {
    
    // 1. 인터럽트 깃발 초기화
    dotDetected = false;
    dashDetected = false;
    
    // 2. 레이저 ON! (신호 시작)
    digitalWrite(laserPin, HIGH);
    
    // micros()를 이용한 마이크로초 단위의 정밀한 레이저 유지
    unsigned long startMicros = micros();
    unsigned long targetMicros = (unsigned long)duration * 1000;
    while (micros() - startMicros < targetMicros) {
      // 텅 비워둡니다. 
      // 아두이노가 여기서 멍 때리고 있어도, 
      // 하드웨어 스파크가 튀면 인터럽트가 알아서 깃발(true)을 세웁니다!
    }
    
    // 3. 레이저 OFF! (이 순간 하드웨어 방전 트리거 발동)
    digitalWrite(laserPin, LOW);
    
    // 하드웨어 스파크가 튀고 인터럽트가 실행될 찰나의 여유를 줍니다.
    delay(50); 
    
    // 4. 결과 출력 (인터럽트가 깃발을 들었는지 확인만 하면 끝)
    Serial.print("테스트 시간: "); Serial.print(duration); Serial.print("ms \t---> ");
    
    // 긴 신호 우선 판독 (킬 스위치 동작 확인)
    if (dashDetected) {
      Serial.println("🔵 긴 신호 (Dash) 감지 완료!");
    } else if (dotDetected) {
      Serial.println("🟢 짧은 신호 (Dot) 감지 완료!");
    } else {
      Serial.println("❌ 무응답");
    }
    
    // 다음 스윕 전 완벽한 하드웨어 방전을 위한 1.5초 대기
    delay(1500); 
  }
  
  Serial.println("=== 1~40ms 1사이클 스윕 완료! 5초 후 재시작 ===");
  delay(5000);
}

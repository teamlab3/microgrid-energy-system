const int dotPin = 34;   // 짧은 신호 (Dot) 입력 핀
const int dashPin = 35;  // 긴 신호 (Dash) 입력 핀

// 인터럽트에서 값을 변경하는 플래그 변수
volatile bool dotDetected = false;
volatile bool dashDetected = false;

// 중복 감지(노이즈) 방지를 위한 시간 기록 변수
volatile unsigned long lastDotTime = 0;
volatile unsigned long lastDashTime = 0;
const unsigned long debounceDelay = 200; // 200ms 이내의 튀는 신호는 1번으로 무시

// --- ESP32 전용 인터럽트 서비스 루틴(ISR) ---
void IRAM_ATTR catchDot() {
  unsigned long currentTime = millis();
  // 마지막 감지 후 200ms가 지났을 때만 인정 (노이즈 필터링)
  if (currentTime - lastDotTime > debounceDelay) {
    dotDetected = true;
    lastDotTime = currentTime;
  }
}

void IRAM_ATTR catchDash() {
  unsigned long currentTime = millis();
  if (currentTime - lastDashTime > debounceDelay) {
    dashDetected = true;
    lastDashTime = currentTime;
  }
}

void setup() {
  Serial.begin(115200);
  
  // 34번, 35번 핀은 ESP32에서 내부 풀업/풀다운이 없는 순수 입력 핀입니다.
  // (질문자님의 회로에 이미 10k 풀업과 모스펫이 있으니 완벽합니다)
  pinMode(dotPin, INPUT);
  pinMode(dashPin, INPUT);
  
  // 신호가 LOW(0V)에서 HIGH(3.3V)로 튀어 오르는 순간(RISING) 인터럽트 발생!
  attachInterrupt(digitalPinToInterrupt(dotPin), catchDot, RISING);
  attachInterrupt(digitalPinToInterrupt(dashPin), catchDash, RISING);
  
  Serial.println("=========================================");
  Serial.println(" ESP32 모스 부호 인터럽트 수신 대기 중...");
  Serial.println("=========================================");
}

void loop() {
  // 인터럽트가 깃발을 세웠는지 상시 확인만 합니다.
  
  if (dotDetected) {
    Serial.println("🟢 [감지] 짧은 신호 (Dot) 들어옴!");
    dotDetected = false; // 깃발 내리기
  }

  if (dashDetected) {
    Serial.println("🔵 [감지] 긴 신호 (Dash) 들어옴!");
    dashDetected = false; // 깃발 내리기
  }
  
  // 이 loop 안에서는 나중에 웹서버 관제 코드를 돌리거나
  // 디스플레이를 띄우는 등 아무 딴짓을 해도 신호를 절대 놓치지 않습니다.
}

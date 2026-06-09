// =========================================================================
// 모터 슬레이브 (Arduino) - I2C 0x08
// ESP32 마스터 명령을 받아 스텝모터 구동  [개선판]
//
//   I2C:  SDA = 18, SCL = 19
//   좌우 스텝모터 : IN1=D4  IN2=D5  IN3=D6  IN4=D7
//   상하 스텝모터 : IN1=D8  IN2=D9  IN3=D10 IN4=D11
//
//   28BYJ-48 + ULN2003 (유니폴라, 하프스텝 8상)
//
//   명령 체계: "U<n>" 위로 n스텝 / "UC" 위로 연속 / "STOP" 정지
//             (D=아래, L=왼쪽, R=오른쪽)
//
//   [개선 사항 - 같은 스텝인데 움직임이 다른 문제 해결]
//   1) onReceive에서 들어온 명령을 '목표 추가'(누적)로 처리 → 빠른 연타 시 명령 안 씹힘
//   2) ISR과 loop 공유 변수(long)를 noInterrupts()로 보호 → 다중바이트 경합 방지
//   3) 명령은 ISR에서 pendingCmd로만 받고, 실제 해석은 loop에서 → ISR 짧게 유지
//   4) HOLD_MS 동안 코일 유지 후 OFF → 마지막 스텝 씹힘/탈조 방지
//   5) 시작 시 첫 스텝 간격을 넉넉히 → 초기 탈조 방지
// =========================================================================
#include <Wire.h>

#define I2C_ADDR 0x08

// 좌우 모터 IN1~IN4
const uint8_t LR_PINS[4] = {4, 5, 6, 7};
// 상하 모터 IN1~IN4
const uint8_t UD_PINS[4] = {8, 9, 10, 11};

// 하프스텝 8상 시퀀스
const uint8_t HALF_SEQ[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

// 스텝 간격(us): 작을수록 빠름. 탈조 나면 키우기(1500~2000)
const unsigned long STEP_INTERVAL_US = 1500;
// 이동 완료 후 코일 유지 시간(ms). 지나면 코일 OFF(발열 방지)
const unsigned long HOLD_MS = 600;
// 연속 회전 표시값
const long CONTINUOUS = 0x7FFFFFFF;

// 남은 스텝 (0=정지, CONTINUOUS=연속). 부호로 방향
volatile long lrRemain = 0;   // +면 RIGHT, -면 LEFT
volatile long udRemain = 0;   // +면 UP,    -면 DOWN

// ISR → loop 명령 전달용 (ISR은 복사만, 해석은 loop)
volatile char pendingCmd[16] = "";
volatile bool cmdReady = false;

int lrPhase = 0, udPhase = 0;
unsigned long lrLastStep = 0, udLastStep = 0;
unsigned long lrDoneTime = 0, udDoneTime = 0;

void applyPhase(const uint8_t pins[4], int phase) {
  for (int i = 0; i < 4; i++) digitalWrite(pins[i], HALF_SEQ[phase][i]);
}
void releaseMotor(const uint8_t pins[4]) {
  for (int i = 0; i < 4; i++) digitalWrite(pins[i], LOW);
}

// I2C 수신 콜백: 최대한 짧게 — 문자열만 복사해두고 빠져나감
void onReceive(int n) {
  char buf[16];
  int idx = 0;
  while (Wire.available() && idx < 15) buf[idx++] = Wire.read();
  buf[idx] = '\0';
  for (int i = 0; i <= idx; i++) pendingCmd[i] = buf[i];
  cmdReady = true;
}

// 명령 해석 (loop에서 호출 — ISR 밖이라 안전)
void applyCommand(const char* buf) {
  if (strcmp(buf, "STOP") == 0) {
    noInterrupts(); lrRemain = 0; udRemain = 0; interrupts();
    return;
  }
  char dir = buf[0];
  long steps;
  if (buf[1] == 'C') steps = CONTINUOUS;
  else               steps = atol(&buf[1]);
  if (steps == 0) return;

  noInterrupts();
  // 누적: 같은 방향 명령이 연속으로 와도 더해짐(연타 시 안 씹힘)
  if (dir == 'U') udRemain = (steps==CONTINUOUS)? +CONTINUOUS : (udRemain==CONTINUOUS||udRemain==-CONTINUOUS? +steps : udRemain+steps);
  else if (dir == 'D') udRemain = (steps==CONTINUOUS)? -CONTINUOUS : (udRemain==CONTINUOUS||udRemain==-CONTINUOUS? -steps : udRemain-steps);
  else if (dir == 'L') lrRemain = (steps==CONTINUOUS)? -CONTINUOUS : (lrRemain==CONTINUOUS||lrRemain==-CONTINUOUS? -steps : lrRemain-steps);
  else if (dir == 'R') lrRemain = (steps==CONTINUOUS)? +CONTINUOUS : (lrRemain==CONTINUOUS||lrRemain==-CONTINUOUS? +steps : lrRemain+steps);
  interrupts();
}

void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(LR_PINS[i], OUTPUT); pinMode(UD_PINS[i], OUTPUT);
    digitalWrite(LR_PINS[i], LOW); digitalWrite(UD_PINS[i], LOW);
  }
  Wire.begin(I2C_ADDR);       // I2C 슬레이브 (SDA=18, SCL=19)
  Wire.onReceive(onReceive);
  Serial.begin(115200);
  Serial.println("Motor slave ready @0x08");
}

void loop() {
  // 1) 들어온 명령 처리 (ISR 밖에서 안전하게)
  if (cmdReady) {
    char local[16];
    noInterrupts();
    for (int i = 0; i < 16; i++) local[i] = pendingCmd[i];
    cmdReady = false;
    interrupts();
    applyCommand(local);
  }

  unsigned long now = micros();
  unsigned long nowMs = millis();

  // 2) 좌우 모터
  long lr; noInterrupts(); lr = lrRemain; interrupts();
  if (lr != 0) {
    if (now - lrLastStep >= STEP_INTERVAL_US) {
      lrLastStep = now;
      int dir = (lr > 0) ? +1 : -1;
      lrPhase = (lrPhase + dir + 8) % 8;
      applyPhase(LR_PINS, lrPhase);
      if (lr != CONTINUOUS && lr != -CONTINUOUS) {
        noInterrupts(); lrRemain -= dir; interrupts();
      }
      lrDoneTime = nowMs;
    }
  } else {
    if (nowMs - lrDoneTime >= HOLD_MS) releaseMotor(LR_PINS);
  }

  // 3) 상하 모터
  long ud; noInterrupts(); ud = udRemain; interrupts();
  if (ud != 0) {
    if (now - udLastStep >= STEP_INTERVAL_US) {
      udLastStep = now;
      int dir = (ud > 0) ? +1 : -1;
      udPhase = (udPhase + dir + 8) % 8;
      applyPhase(UD_PINS, udPhase);
      if (ud != CONTINUOUS && ud != -CONTINUOUS) {
        noInterrupts(); udRemain -= dir; interrupts();
      }
      udDoneTime = nowMs;
    }
  } else {
    if (nowMs - udDoneTime >= HOLD_MS) releaseMotor(UD_PINS);
  }
}

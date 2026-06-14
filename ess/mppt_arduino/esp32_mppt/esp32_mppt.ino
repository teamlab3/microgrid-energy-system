/* =====================================================================
 *  하이브리드(태양광 + 풍력) MPPT 충전 컨트롤러  /  ESP32
 * =====================================================================
 *
 *  [한 줄 요약]
 *  태양광·풍력 두 입력을 SEPIC 컨버터로 받아 P&O MPPT로 최대 전력을
 *  추종하고, 18650 4셀을 안전하게 골라 충전하는 ESS 제어 펌웨어.
 *
 *  [하드웨어 구성]
 *   - 입력  : 태양광, 풍력 → 각각 SEPIC 컨버터 (PWM으로 스위치 구동)
 *   - 계측  : INA219 ×3 (solar / wind / battery), I2C
 *   - 배터리: 18650 단셀 ×4, 채널별 MOSFET으로 충전 경로 선택
 *   - 기타  : 홀센서(풍력 RPM), DHT11(온도)
 *
 *  [동작 개요 — loop() 단계]
 *   A. 배터리 관리 (어느 셀을 충전할지 선택)        ※ 플래그
 *   B. 선택된 셀의 충전 경로만 ON
 *   C. INA219 측정 + 센서 헬스체크/이상값 필터
 *   D. 전압 게이트 (입력이 약하면 채널 잠금)         ※ 플래그
 *   E. P&O MPPT (PERTURB→SETTLE→OBSERVE 2페이즈)     ※ 플래그
 *   F. PWM 출력 (안전 조건 충족 시에만)
 *   G. 풍력 RPM 계산
 *   H. 온도 측정
 *   I. 시리얼 출력 (Readable / CSV)                  ※ 플래그
 *   J. 워치독 리셋
 *
 *  [안전 장치]
 *   - 과충전 차단 : 셀 4.2V 도달 시 제외, 4.1V까지 재개 보류(히스테리시스)
 *   - 센서 보호   : 이상값 필터 + 연속 실패 시 재초기화,
 *                   배터리 센서 불량이면 PWM 강제 차단
 *   - 워치독      : loop가 멈추면 자동 리셋
 *
 *  [동작 모드 — 상단 플래그로 토글]
 *   ENABLE_BATT_MANAGER / ENABLE_VOLTAGE_GATE /
 *   ENABLE_PO_MPPT      / SERIAL_CSV_MODE
 *
 *  [환경] ESP32 Arduino Core 3.x (ledcAttach, esp_task_wdt 신 API 사용)
 * ===================================================================== */

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <DHT.h>
#include <esp_task_wdt.h>   // 워치독 타이머

// =============================================
// ESP32-CAM 전송용 UART2 (TX only)
// USB 디버그용 Serial(UART0)은 그대로 두고,
// GPIO 25를 별도 UART2 TX로 사용해 ESP32-CAM(GPIO 3)에 CSV 전송.
// → USB 시리얼 모니터 디버깅과 CAM 전송을 동시에 할 수 있음.
// 배선: ESP32 GPIO 25 → ESP32-CAM GPIO 3, GND 공유 필수.
// =============================================
#define CAM_TX_PIN  26
HardwareSerial CamSerial(2);

// ---------------------------------------------------------------------
//  함수 목록 (정의는 아래쪽). 파일 구성을 한눈에 보기 위한 선언.
// ---------------------------------------------------------------------
int  calcStartPwm(float battV);                 // SOC 기반 PWM 시작점
bool isVoltSane(float v);                        // 전압 sanity 검사
bool isCurrSane(float c);                        // 전류 sanity 검사
void resetMppt(int &pwm, float &prePow, int &dir, float battV);  // MPPT 초기화
bool updateGate(float v, float threshold, bool &on,
                unsigned long &hold, unsigned long now);          // 전압 게이트 판정

// =============================================
// 기능 플래그 (0 = 비활성화, 1 = 활성화)
// =============================================
#define ENABLE_BATT_MANAGER   0   // 배터리 순차 측정 및 충전 대상 선택 로직
#define ENABLE_VOLTAGE_GATE   0   // 전압 감시 윈도우 (final_on_s/w) 기반 PWM 잠금
#define ENABLE_PO_MPPT        1   // P&O MPPT 알고리즘
#define SERIAL_CSV_MODE       1   // 0 = Readable 출력, 1 = CSV 파싱 출력

// 디버그: 고정 PWM 강제 출력
// 1로 켜면 MPPT/시작점 계산을 무시하고 양 채널을 DEBUG_PWM_VALUE로 고정.
// 배터리 없이 "내가 지정한 듀티가 실제로 나가는지"(모스펫/게이트/전력경로)
// 확인할 때 사용. INA219 전류가 PWM 따라 변하면 경로 정상.
#define DEBUG_FIXED_PWM       0   // 0 = 정상 동작, 1 = 고정 PWM 강제
#define DEBUG_PWM_VALUE       40  // 강제 출력할 듀티 (PWM_MIN~PWM_MAX 범위 권장)

// =============================================
// 주기 설정 (ms)
// =============================================
#define SENSOR_INTERVAL   100     // INA219 읽기 주기
#define MPPT_PERTURB      1000    // P&O 한 스텝 전체 주기 (perturb 시작 간격)
#define MPPT_SETTLE       500     // PWM 변경 후 동작점 안정화 대기 시간
#define GATE_ON_TIME      2000    // 입력이 임계값 이상 이만큼 연속 유지되면 게이트 ON
#define GATE_OFF_TIME     3000    // 입력이 임계값 미만 이만큼 연속 유지되면 게이트 OFF
#define PRINT_INTERVAL    500     // 시리얼 출력 주기
#define DHT_INTERVAL      2000    // DHT11 읽기 주기 (datasheet 최소 1s, 여유 2s)
#define RPM_TIMEOUT       5000    // 홀센서 무신호 → RPM 0 처리

// =============================================
// 센서 헬스체크 / sanity 설정
// =============================================
#define WDT_TIMEOUT_S     5       // 워치독 타임아웃 (초). loop가 이만큼 멈추면 리셋
#define SENSOR_FAIL_LIMIT 5       // 연속 실패 이만큼이면 센서 재초기화 시도
#define VOLT_SANITY_MIN   -1.0f   // 유효 전압 하한 (V)
#define VOLT_SANITY_MAX   30.0f   // 유효 전압 상한 (V)
#define CURR_SANITY_ABS   5000.0f // 유효 전류 절대값 상한 (mA)

// =============================================
// 핀 정의
// =============================================
#define BLANK          999

#define SOLAR_PWM_PIN   32
#define WIND_PWM_PIN    27
const int BATT_PINS[4] = {16, 17, 18, 19};

#define HALL_PIN        14
#define DHT_PIN         13
#define DHT_TYPE        DHT11

// =============================================
// PWM (ledc) 설정
// =============================================
const int PWM_FREQ = 31400;
const int PWM_RES  = 8;
const int PWM_MIN  = 40;
const int PWM_MAX  = 150;

// =============================================
// 전압 게이트 임계값
// =============================================
const float SOLAR_GATE_V = 5.0f;
const float WIND_GATE_V  = 1.0f;

// =============================================
// INA219 센서 (I2C)
// =============================================
Adafruit_INA219 sun(0x44);
Adafruit_INA219 wind(0x41);
Adafruit_INA219 batt(0x40);

// =============================================
// DHT11
// =============================================
DHT dht(DHT_PIN, DHT_TYPE);

// =============================================
// 홀센서 (인터럽트 기반 RPM 측정)
// ISR 내 float 연산 제거 → interval만 저장, 임계영역 보호
// =============================================
volatile unsigned long hall_interval  = 0;
volatile bool          hall_triggered = false;
static unsigned long   hall_last_time = 0;   // ISR 전용, 일반 변수
portMUX_TYPE hallMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR hallISR() {
    unsigned long now = millis();
    if (hall_last_time != 0) {
        unsigned long interval = now - hall_last_time;
        if (interval > 10) {
            portENTER_CRITICAL_ISR(&hallMux);
            hall_interval = interval;
            portEXIT_CRITICAL_ISR(&hallMux);
        }
    }
    hall_last_time = now;
    hall_triggered = true;
}

// =============================================
// MPPT 제어 변수
// =============================================
int   PwmValue_s = PWM_MIN;
int   PwmValue_w = PWM_MIN;
float prePow_s   = 0;
float prePow_w   = 0;
int   dir_s      = 1;
int   dir_w      = 1;

// P&O 2페이즈 상태머신
// phase 0 = PERTURB(섭동 적용) 대기, phase 1 = SETTLE(안정화) 대기 후 OBSERVE
int           mppt_phase   = 0;
unsigned long settle_timer = 0;

bool final_on_s = false;
bool final_on_w = false;

// =============================================
// 센서 측정값 (전역: 여러 단계에서 공유)
// =============================================
float sv = 0, sa = 0, sw = 0;   // solar
float wv = 0, wa = 0, ww = 0;   // wind
float bv = 0, ba = 0, bw = 0;   // battery
float temp = -1.0f;

// =============================================
// 센서 헬스 상태
// *_ok    : 현재 센서가 신뢰 가능한가
// *_fail  : 연속 실패(무응답/이상값) 횟수
// batt_ok 가 false면 과충전 판단 불가 → 안전을 위해 PWM 강제 차단
// =============================================
bool sun_ok = false,  wind_ok = false,  batt_ok = false;
int  sun_fail = 0,    wind_fail = 0,     batt_fail = 0;

// =============================================
// 배터리 인덱스 / 셀 전압
// =============================================
int target_index = 0;
int b1v = 0, b2v = 0, b3v = 0, b4v = 0;

// =============================================
// 타이머 변수
// =============================================
unsigned long print_timer    = 0;
unsigned long rpm_zero_timer = 0;
unsigned long mppt_timer     = 0;
unsigned long sensor_timer   = 0;
unsigned long dht_timer      = 0;

// 전압 게이트: 채널별 "현재 조건이 유지되기 시작한 시각"
// (ON 후보 = 임계값 이상 유지, OFF 후보 = 임계값 미만 유지)
unsigned long gate_hold_s = 0;
unsigned long gate_hold_w = 0;

// =============================================
// 시리얼 송신 버퍼
// CamSerial(ESP32-CAM)로는 모드와 무관하게 항상 CSV를 보내므로 항상 선언.
// =============================================
char serialBuf[256];

// =============================================
// 18650 기반 PWM 시작점 계산
// =============================================
int calcStartPwm(float battV) {
    float soc = (battV - 3.0f) / (4.2f - 3.0f);
    soc = constrain(soc, 0.0f, 1.0f);
    float curve = 1.0f - (2.0f * soc - 1.0f) * (2.0f * soc - 1.0f);
    return (int)(PWM_MIN + curve * (PWM_MAX - PWM_MIN));
}

// =============================================
// [배터리 로직] ENABLE_BATT_MANAGER == 1 일 때만 컴파일
// =============================================
#if ENABLE_BATT_MANAGER

#define BATT_STAGE_1_MAX      3.4f
#define BATT_STAGE_2_MAX      3.7f
#define BATT_STAGE_3_MAX      4.0f
#define BATT_STAGE_4_MAX      4.2f
#define BATT_MIN_V            3.0f
#define BATT_MEASURE_WAIT     300
#define BATT_MEASURE_INTERVAL 30000

// 충전 안전 임계값
#define BATT_FULL_V           4.2f    // 이 전압 이상이면 충전 중단 (만충)
#define BATT_RESUME_V         4.1f    // 이 전압 아래로 떨어져야 충전 재개 (히스테리시스)

float batt_voltages[4]           = {0, 0, 0, 0};
bool  batt_full[4]               = {false, false, false, false};  // 셀별 만충 래치
unsigned long batt_measure_timer = 0;
bool batt_measured               = false;

int getBattStage(float v) {
    if (v < BATT_MIN_V)       return 0;
    if (v < BATT_STAGE_1_MAX) return 1;
    if (v < BATT_STAGE_2_MAX) return 2;
    if (v < BATT_STAGE_3_MAX) return 3;
    return 4;
}

// 측정 중 핀 제어는 이 함수가 단독으로 담당 (loop B단계와 충돌 방지)
bool measureAllBatteries() {
    static int           measure_idx   = 0;
    static int           measure_state = 0;
    static unsigned long state_timer   = 0;

    switch (measure_state) {
        case 0:
            ledcWrite(SOLAR_PWM_PIN, 0);
            ledcWrite(WIND_PWM_PIN, 0);
            measure_idx   = 0;
            measure_state = 1;
            return true;

        case 1:
            for (int i = 0; i < 4; i++) {
                digitalWrite(BATT_PINS[i], (i == measure_idx) ? HIGH : LOW);
            }
            state_timer   = millis();
            measure_state = 2;
            return true;

        case 2:
            if (millis() - state_timer < BATT_MEASURE_WAIT) return true;
            batt_voltages[measure_idx] = batt.getBusVoltage_V();

            // 만충 래치 갱신 (히스테리시스)
            // 4.2V 이상 → 만충 처리, 4.1V 아래로 내려가야 해제
            if (batt_voltages[measure_idx] >= BATT_FULL_V) {
                batt_full[measure_idx] = true;
            } else if (batt_voltages[measure_idx] < BATT_RESUME_V) {
                batt_full[measure_idx] = false;
            }

            measure_idx++;
            if (measure_idx < 4) {
                measure_state = 1;
            } else {
                measure_state = 0;
                batt_measured = true;
                return false;
            }
            return true;
    }
    return false;
}

// 충전할 셀을 선택. 만충 셀과 방전 셀(3.0V↓)은 제외.
// 충전 가능한 셀이 없으면 -1 반환 → 충전 중단 신호.
int selectChargingBattery() {
    if (!batt_measured) return 0;
    int   min_stage = 5;
    float min_v     = 9999.0f;
    int   selected  = -1;   // 기본값: 충전 대상 없음
    for (int i = 0; i < 4; i++) {
        if (batt_full[i]) continue;            // 만충 셀 제외 (과충전 차단)
        int stage = getBattStage(batt_voltages[i]);
        if (stage == 0) continue;              // 과방전 셀 제외
        if (stage < min_stage || (stage == min_stage && batt_voltages[i] < min_v)) {
            min_stage = stage;
            min_v     = batt_voltages[i];
            selected  = i;
        }
    }
    return selected;
}

bool updateBatteryManager() {
    static bool measuring = false;
    if (!batt_measured || millis() - batt_measure_timer >= BATT_MEASURE_INTERVAL) {
        measuring          = true;
        batt_measure_timer = millis();
    }
    if (measuring) {
        bool still_measuring = measureAllBatteries();
        if (!still_measuring) {
            measuring    = false;
            target_index = selectChargingBattery();
        }
        return true;
    }
    return false;
}

#endif // ENABLE_BATT_MANAGER

// =============================================
// 센서 헬스체크 헬퍼
// =============================================

// 전압/전류가 물리적으로 말이 되는 범위인지 검사
bool isVoltSane(float v) { return (v > VOLT_SANITY_MIN && v < VOLT_SANITY_MAX); }
bool isCurrSane(float c) { return (fabs(c) < CURR_SANITY_ABS); }

// INA219 한 개를 읽어서 sanity 통과 시 v/a/w 갱신.
// 통과하면 true(+fail 리셋), 실패하면 false(+fail 증가, 직전값 유지).
bool readSensor(Adafruit_INA219 &dev, float &v, float &a, float &w,
                int &fail, bool clampNeg) {
    float nv = dev.getBusVoltage_V();
    float na = dev.getCurrent_mA();

    if (!isVoltSane(nv) || !isCurrSane(na)) {
        fail++;
        return false;            // 이상값: 직전값 유지
    }
    if (clampNeg) na = max(na, 0.0f);
    v = nv;
    a = na;
    w = v * (a / 1000.0f);
    fail = 0;
    return true;
}

// 연속 실패가 한계를 넘으면 재초기화 시도
void recoverSensor(Adafruit_INA219 &dev, int &fail, bool &ok) {
    if (fail >= SENSOR_FAIL_LIMIT) {
        ok = dev.begin();
        if (ok) dev.setCalibration_32V_2A();
        fail = 0;                // 카운터 리셋(성공/실패 무관, 매 주기 재시도 방지)
    }
}

// =============================================
// MPPT 시작점 초기화 (게이트 진입 시 공통 사용)
// =============================================
void resetMppt(int &pwm, float &prePow, int &dir, float battV) {
    prePow = 0;
    dir    = 1;
    pwm    = calcStartPwm(battV);
    mppt_phase = 0;   // 섭동 페이즈부터 다시 시작
}

// =============================================
// 전압 게이트 판정 (연속 유지 + 히스테리시스)
//  - 현재 OFF: 입력이 임계값 이상으로 GATE_ON_TIME 연속 유지되면 ON
//  - 현재 ON : 입력이 임계값 미만으로 GATE_OFF_TIME 연속 유지되면 OFF
//  - 조건이 깨지면 hold 타이머를 리셋해 순간 스파이크/딥을 무시
// 반환: 게이트 상태가 OFF→ON으로 바뀌는 순간이면 true (MPPT 재시작 신호)
// =============================================
bool updateGate(float v, float threshold, bool &on,
                unsigned long &hold, unsigned long now) {
    bool just_turned_on = false;
    if (!on) {
        // OFF 상태: ON 조건(임계값 이상) 유지 시간 확인
        if (v >= threshold) {
            if (now - hold >= GATE_ON_TIME) {
                on = true;
                just_turned_on = true;
            }
        } else {
            hold = now;   // 조건 깨짐 → 타이머 리셋
        }
    } else {
        // ON 상태: OFF 조건(임계값 미만) 유지 시간 확인
        if (v < threshold) {
            if (now - hold >= GATE_OFF_TIME) {
                on = false;
            }
        } else {
            hold = now;   // 조건 깨짐 → 타이머 리셋
        }
    }
    return just_turned_on;
}

// =============================================
// setup
// =============================================
void setup() {
    Serial.begin(115200);

    // ESP32-CAM 전송용 UART2: GPIO 25 TX, RX 미사용(-1)
    CamSerial.begin(115200, SERIAL_8N1, -1, CAM_TX_PIN);

    Wire.begin();
    Wire.setClock(400000);   // INA219는 400kHz 지원, 배선 짧으면 OK

    ledcAttach(SOLAR_PWM_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(WIND_PWM_PIN,  PWM_FREQ, PWM_RES);
    ledcWrite(SOLAR_PWM_PIN, 0);
    ledcWrite(WIND_PWM_PIN,  0);

    sun_ok  = sun.begin();   if (sun_ok)  sun.setCalibration_32V_2A();
    wind_ok = wind.begin();  if (wind_ok) wind.setCalibration_32V_2A();
    batt_ok = batt.begin();  if (batt_ok) batt.setCalibration_32V_2A();

    for (int i = 0; i < 4; i++) pinMode(BATT_PINS[i], OUTPUT);
    for (int i = 0; i < 4; i++) {
        digitalWrite(BATT_PINS[i], (i == target_index) ? HIGH : LOW);
    }

    dht.begin();

    pinMode(HALL_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

#if DEBUG_FIXED_PWM
    // 디버그: 게이트 무시하고 양 채널을 고정 듀티로
    final_on_s = true;
    final_on_w = true;
    PwmValue_s = DEBUG_PWM_VALUE;
    PwmValue_w = DEBUG_PWM_VALUE;
#elif !ENABLE_VOLTAGE_GATE
    // 게이트 비활성화 시 시작점도 SOC 기반으로 (게이트 ON과 일관)
    final_on_s = true;
    final_on_w = true;
    float initBv = batt.getBusVoltage_V();
    if (!isVoltSane(initBv)) initBv = 3.6f;   // 센서 이상 시 18650 공칭값으로 폴백
    PwmValue_s = calcStartPwm(initBv);
    PwmValue_w = calcStartPwm(initBv);
#endif

    unsigned long now = millis();
    print_timer = rpm_zero_timer = now;
    mppt_timer  = sensor_timer = dht_timer = now;
    gate_hold_s = gate_hold_w = now;

    // 워치독: loop가 WDT_TIMEOUT_S초 이상 멈추면 자동 리셋
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    esp_task_wdt_init(&wdt_cfg);
    esp_task_wdt_add(NULL);   // 현재(loop) 태스크 감시 등록

    Serial.printf("[SYS] MPPT ESP32 초기화 완료 (sun:%d wind:%d batt:%d)\n",
        sun_ok, wind_ok, batt_ok);
}

// =============================================
// loop : A~J 단계를 매 주기 순서대로 실행
//   (각 단계는 자체 타이머로 필요한 주기에만 실제 동작)
// =============================================
void loop() {
    unsigned long now = millis();

    // --- A. 배터리 관리 ---
#if ENABLE_BATT_MANAGER
    bool batt_measuring = updateBatteryManager();
#else
    bool batt_measuring = false;
#endif

    // --- B. 배터리 핀 출력 (측정 중이 아닐 때만; 측정 중엔 measure 함수가 담당) ---
    // target_index == -1(전부 만충)이면 어떤 핀도 일치하지 않아 전부 LOW = 충전 차단
    if (!batt_measuring) {
        for (int i = 0; i < 4; i++) {
            digitalWrite(BATT_PINS[i], (i == target_index) ? HIGH : LOW);
        }
    }

    // --- C. INA219 데이터 읽기 (SENSOR_INTERVAL 주기) ---
    if (now - sensor_timer >= SENSOR_INTERVAL) {
        // solar/wind는 음수 전류 0 클램프, battery는 충/방전 방향 보존
        sun_ok  = readSensor(sun,  sv, sa, sw, sun_fail,  true);
        wind_ok = readSensor(wind, wv, wa, ww, wind_fail, true);
        batt_ok = readSensor(batt, bv, ba, bw, batt_fail, false);

        // 연속 실패 시 재초기화 시도
        recoverSensor(sun,  sun_fail,  sun_ok);
        recoverSensor(wind, wind_fail, wind_ok);
        recoverSensor(batt, batt_fail, batt_ok);

        sensor_timer = now;
    }

    // --- D. 전압 게이트 (연속 유지 + 히스테리시스) ---
    // 입력이 임계값 이상으로 일정 시간 유지돼야 ON, 미만으로 유지돼야 OFF.
    // 순간 스파이크/딥은 hold 타이머 리셋으로 무시 → 안정적으로 켜고 끔.
#if ENABLE_VOLTAGE_GATE
    bool turned_on_s = updateGate(sv, SOLAR_GATE_V, final_on_s, gate_hold_s, now);
    bool turned_on_w = updateGate(wv, WIND_GATE_V,  final_on_w, gate_hold_w, now);

    // 막 켜진 채널은 SOC 기반 시작점에서 MPPT 재시작
    if (turned_on_s) resetMppt(PwmValue_s, prePow_s, dir_s, bv);
    if (turned_on_w) resetMppt(PwmValue_w, prePow_w, dir_w, bv);

    // 꺼진 채널은 PWM 시작점으로 정리 (다음 ON 때 깨끗하게 시작)
    if (!final_on_s) { PwmValue_s = PWM_MIN; prePow_s = 0; dir_s = 1; }
    if (!final_on_w) { PwmValue_w = PWM_MIN; prePow_w = 0; dir_w = 1; }
#endif

    // --- E. P&O MPPT 로직 (2페이즈: PERTURB → SETTLE → OBSERVE) ---
    // DEBUG_FIXED_PWM이 켜져 있으면 섭동하지 않아 PWM이 고정값으로 유지됨
#if ENABLE_PO_MPPT && !DEBUG_FIXED_PWM
    if (!batt_measuring) {
        if (mppt_phase == 0) {
            // [PERTURB] MPPT_PERTURB 주기마다 PWM을 한 칸 섭동
            if (now - mppt_timer >= MPPT_PERTURB) {
                if (final_on_s) {
                    PwmValue_s = constrain(PwmValue_s + dir_s, PWM_MIN, PWM_MAX);
                }
                if (final_on_w) {
                    PwmValue_w = constrain(PwmValue_w + dir_w, PWM_MIN, PWM_MAX);
                }
                // 섭동을 가했으니 안정화 대기 페이즈로 전환
                settle_timer = now;
                mppt_phase   = 1;
            }
        } else {
            // [SETTLE → OBSERVE] 동작점 안정화 후 전력 비교 → 다음 방향 결정
            if (now - settle_timer >= MPPT_SETTLE) {
                if (final_on_s) {
                    // 섭동 후 전력이 줄었으면 방향 반전
                    if (sw < prePow_s) dir_s = -dir_s;
                    prePow_s = sw;   // 다음 비교 기준 갱신
                }
                if (final_on_w) {
                    if (ww < prePow_w) dir_w = -dir_w;
                    prePow_w = ww;
                }
                // 한 스텝 완료, 다음 PERTURB 주기 시작
                mppt_timer = now;
                mppt_phase = 0;
            }
        }
    }
#endif

    // --- F. PWM 출력 ---
    // PWM을 0으로 강제하는 조건:
    //  - batt_measuring : 배터리 측정 중
    //  - no_target      : 충전 가능한 셀 없음(전부 만충)
    //  - !batt_ok       : 배터리 센서 무응답 → 과충전 판단 불가 → 안전 차단
    bool no_target = false;
#if ENABLE_BATT_MANAGER
    no_target = (target_index < 0);
#endif

#if DEBUG_FIXED_PWM
    // 디버그 모드: 배터리/센서 없이 PWM 자체를 확인하는 게 목적이므로
    // 안전 차단을 우회하고 항상 고정 듀티를 출력한다.
    bool force_off = false;
#else
    bool force_off = (batt_measuring || no_target || !batt_ok);  // 공통 차단
#endif

    // 채널별 실제 PWM 차단 여부 (시리얼 출력의 BLANK 표시에 사용)
    bool solar_off = force_off || !final_on_s;
    bool wind_off  = force_off || !final_on_w;

    ledcWrite(SOLAR_PWM_PIN, solar_off ? 0 : PwmValue_s);
    ledcWrite(WIND_PWM_PIN,  wind_off  ? 0 : PwmValue_w);
    // 주의: 여기서 sv/sw 등 원본은 건드리지 않음.
    //       BLANK 표시는 시리얼 출력 직전(I단계)에서만 적용해
    //       MPPT 판단용 원본값이 오염되지 않게 한다.

    // --- G. RPM 계산 / 타임아웃 처리 ---
    unsigned long interval_snapshot;
    portENTER_CRITICAL(&hallMux);
    interval_snapshot = hall_interval;
    portEXIT_CRITICAL(&hallMux);

    float rpm = 0.0f;
    if (interval_snapshot > 0) {
        rpm = 60000.0f / (float)interval_snapshot;
    }

    if (hall_triggered) {
        rpm_zero_timer = now;
        hall_triggered = false;
    }
    if (now - rpm_zero_timer > RPM_TIMEOUT) {
        rpm = 0.0f;
        portENTER_CRITICAL(&hallMux);
        hall_interval = 0;
        portEXIT_CRITICAL(&hallMux);
        hall_last_time = 0;
    }

    // --- H. DHT11 읽기 (DHT_INTERVAL 주기) ---
    if (now - dht_timer >= DHT_INTERVAL) {
        float t = dht.readTemperature();
        if (!isnan(t)) temp = t;   // NaN이면 직전값 유지
        dht_timer = now;
    }

    // --- I. 시리얼 송신 (PRINT_INTERVAL 주기) ---
    if (now - print_timer >= PRINT_INTERVAL) {

        // 셀 전압 채우기
#if ENABLE_BATT_MANAGER
        b1v = (int)(batt_voltages[0] * 100);
        b2v = (int)(batt_voltages[1] * 100);
        b3v = (int)(batt_voltages[2] * 100);
        b4v = (int)(batt_voltages[3] * 100);
#else
        int bv_arr[4] = {0, 0, 0, 0};
        bv_arr[target_index] = (int)(bv * 100);
        b1v = bv_arr[0]; b2v = bv_arr[1]; b3v = bv_arr[2]; b4v = bv_arr[3];
#endif

        // 출력용 값 준비: PWM이 막힌 채널은 전압에 BLANK(999), 전류·전력 0.
        // 원본(sv/sw 등)은 MPPT 판단용으로 보존하고 여기서만 표시값을 만든다.
        float out_sv = solar_off ? (float)BLANK : sv;
        float out_sa = solar_off ? 0.0f : sa;
        float out_sw = solar_off ? 0.0f : sw;
        float out_wv = wind_off  ? (float)BLANK : wv;
        float out_wa = wind_off  ? 0.0f : wa;
        float out_ww = wind_off  ? 0.0f : ww;

        // CSV 문자열 생성 (모드와 무관하게 항상 생성)
        // CSV: solar_pwm, solar_v*100, solar_a, solar_w*100,
        //      wind_pwm, wind_v*100, wind_a, wind_w*100,
        //      batt_v*100, batt_a, batt_w*100,
        //      b1v, b2v, b3v, b4v, target, rpm*1000, temp*10
        //  ※ solar_v / wind_v 가 99900(BLANK*100)이면 해당 채널 PWM 차단 상태
        snprintf(serialBuf, sizeof(serialBuf),
            "%d,%d,%d,%d,"
            "%d,%d,%d,%d,"
            "%d,%d,%d,"
            "%d,%d,%d,%d,"
            "%d,%d,%d",
            PwmValue_s, (int)(out_sv * 100), (int)(out_sa), (int)(out_sw * 100),
            PwmValue_w, (int)(out_wv * 100), (int)(out_wa), (int)(out_ww * 100),
            (int)(bv * 100), (int)(ba), (int)(bw * 100),
            b1v, b2v, b3v, b4v,
            target_index + 1,
            (int)(rpm * 1000),
            (int)(temp * 10)
        );

        // ESP32-CAM으로는 항상 CSV 전송 (GPIO 25 → CAM GPIO 3)
        CamSerial.println(serialBuf);

#if SERIAL_CSV_MODE
        // USB 디버그도 CSV로 출력
        Serial.println(serialBuf);
#else
        // USB 디버그는 사람이 읽기 좋은 형식으로 출력
        Serial.println("--------------------");
#if DEBUG_FIXED_PWM
        Serial.printf("[DEBUG] 고정 PWM 모드 (값=%d) — 안전로직 우회 중\n", DEBUG_PWM_VALUE);
#endif
        if (solar_off) {
            Serial.printf("[Solar]  PWM:   0  V: BLANK  (차단)\n");
        } else {
            Serial.printf("[Solar]  PWM: %3d  V: %.2fV  A: %dmA  W: %.2fW\n",
                PwmValue_s, sv, (int)sa, sw);
        }
        if (wind_off) {
            Serial.printf("[Wind ]  PWM:   0  V: BLANK  (차단)\n");
        } else {
            Serial.printf("[Wind ]  PWM: %3d  V: %.2fV  A: %dmA  W: %.2fW\n",
                PwmValue_w, wv, (int)wa, ww);
        }
        if (target_index < 0) {
            Serial.printf("[Batt ]  V: %.2fV  A: %.0fmA  W: %.2fW  Target: CHG OFF (all full)\n",
                bv, ba, bw);
        } else {
            Serial.printf("[Batt ]  V: %.2fV  A: %.0fmA  W: %.2fW  Target: #%d\n",
                bv, ba, bw, target_index + 1);
        }
        Serial.printf("[Cell ]  B1: %.2fV  B2: %.2fV  B3: %.2fV  B4: %.2fV\n",
            b1v / 100.0f, b2v / 100.0f, b3v / 100.0f, b4v / 100.0f);
        Serial.printf("[Misc ]  RPM: %.1f  Temp: %.1f'C  Sensors[S/W/B]: %d/%d/%d\n",
            rpm, temp, sun_ok, wind_ok, batt_ok);
#endif

        print_timer = now;
    }

    // --- J. 워치독 리셋 (loop가 정상 진행 중임을 알림) ---
    esp_task_wdt_reset();
}

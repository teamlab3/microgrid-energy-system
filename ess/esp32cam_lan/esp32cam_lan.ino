#include <SPI.h>
#include <Ethernet.h>
#include "esp_camera.h"

// =============================================
// W5500 SPI 핀 설정 (HSPI 사용, 실제 사례 기반)
// =============================================
#define ETH_MISO 12
#define ETH_MOSI 13
#define ETH_SCK  14
#define ETH_CS    2

// =============================================
// 플래시 LED (GPIO 4)
// =============================================
#define LED_PIN   4

// =============================================
// 아두이노 시리얼 통신 (UART2, RX only)
// =============================================
#define ARD_RX 33
HardwareSerial ArduinoSerial(2);

// =============================================
// 이더넷 설정
// =============================================
byte mac[]        = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress serverIP(172, 30, 8, 140);
IPAddress ip      (172, 30, 8, 150);
EthernetClient client;

// =============================================
// 상태 플래그
// =============================================
bool eth_initialized = false;
bool cam_initialized = false;

// =============================================
// 센서 데이터 버퍼
// =============================================
char rxBuf[512];
int  rxIndex    = 0;
bool rxOverflow = false;
char last_sensor_data[512] = "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
unsigned long lastSend = 0;

// =============================================
// ESP32-CAM AI-Thinker 카메라 핀 정의
// =============================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// =============================================
// LED 깜빡임 함수
// =============================================
void blinkLED(int interval) {
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    if (millis() - lastBlink >= interval) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        lastBlink = millis();
    }
}

// =============================================
// 카메라 초기화
// =============================================
bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_UXGA;
    config.jpeg_quality = 2;
    config.fb_count     = 1;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("[CAM] 초기화 실패");
        return false;
    }

    Serial.println("[CAM] 워밍업 중...");
    for (int i = 0; i < 15; i++) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        delay(200);
    }
    Serial.println("[CAM] 초기화 완료");
    return true;
}

// =============================================
// 이더넷 초기화
// =============================================
bool initEthernet() {
    Serial.println("[ETH] W5500 초기화 시도...");

    pinMode(ETH_CS, OUTPUT);
    digitalWrite(ETH_CS, HIGH);
    delay(100);

    SPI.begin(ETH_SCK, ETH_MISO, ETH_MOSI, ETH_CS);
    Ethernet.init(ETH_CS);
    Ethernet.begin(mac, ip);

    Serial.println("[ETH] 링크 대기 중...");
    unsigned long ethTimer = millis();
    while (Ethernet.linkStatus() != LinkON) {
        if (millis() - ethTimer > 10000) {
            Serial.println("[ETH] W5500 인식 실패 - 링크 연결 안됨");
            return false;
        }
        blinkLED(2000);
        delay(100);
        Serial.print(".");
    }

    eth_initialized = true;
    Serial.print("\n[ETH] W5500 인식 성공 | IP: ");
    Serial.println(Ethernet.localIP());
    return true;
}

// =============================================
// 서버 재접속
// =============================================
void reconnectServer() {
    static unsigned long lastRetry = 0;
    static bool firstTry = true;

    if (!firstTry && millis() - lastRetry < 5000) return;
    firstTry  = false;
    lastRetry = millis();

    client.stop();
    Serial.println("[NET] 서버 재접속 시도...");
    if (client.connect(serverIP, 5000)) {
        Serial.println("[NET] 서버 연결 성공");
        digitalWrite(LED_PIN, LOW);
    } else {
        Serial.println("[NET] 서버 연결 실패, 5초 후 재시도");
    }
}

// =============================================
// 아두이노 시리얼 수신
// =============================================
void readArduinoSerial() {
    while (ArduinoSerial.available()) {
        char c = ArduinoSerial.read();

        if (c == '\r') continue;

        if (c == '\n') {
            if (!rxOverflow) {
                rxBuf[rxIndex] = '\0';
                if (rxIndex > 0) {
                    strncpy(last_sensor_data, rxBuf, sizeof(last_sensor_data) - 1);
                    last_sensor_data[sizeof(last_sensor_data) - 1] = '\0';
                    Serial.print("[ARD] 수신: ");
                    Serial.println(last_sensor_data);
                }
            } else {
                Serial.println("[ARD] 오버플로우 라인 폐기");
                rxOverflow = false;
            }
            rxIndex = 0;
        } else {
            if (rxIndex < (int)sizeof(rxBuf) - 1) {
                rxBuf[rxIndex++] = c;
            } else {
                rxOverflow = true;
                rxIndex    = 0;
                Serial.println("[ARD] 버퍼 오버플로우 감지");
            }
        }
    }
}

// =============================================
// 데이터 전송 (청크 방식으로 수정)
// =============================================
void sendData() {
    client.println(last_sensor_data);

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[CAM] 캡처 실패 - 센서 데이터만 전송");
        client.println("IMG_SIZE:0");
        return;
    }

    client.print("IMG_SIZE:");
    client.println(fb->len);

    // 1024바이트씩 나눠서 전송
    const size_t chunkSize = 1024;
    size_t totalWritten = 0;
    size_t remaining = fb->len;

    while (remaining > 0) {
        size_t toWrite = min(remaining, chunkSize);
        size_t written = client.write(fb->buf + totalWritten, toWrite);
        if (written == 0) {
            Serial.println("[NET] 전송 중 오류 → 재연결");
            esp_camera_fb_return(fb);
            client.stop();
            return;
        }
        totalWritten += written;
        remaining -= written;
    }
    client.flush();

    esp_camera_fb_return(fb);
    Serial.printf("[NET] 전송 완료 | %u bytes\n", fb->len);
}

// =============================================
// setup
// =============================================
void setup() {
    Serial.begin(115200);

    pinMode(ETH_CS, OUTPUT);
    digitalWrite(ETH_CS, HIGH);
    delay(100);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    ArduinoSerial.begin(115200, SERIAL_8N1, ARD_RX, -1);
    Serial.println("[SYS] 아두이노 시리얼 준비 (115200 baud)");

    // 카메라 먼저 초기화
    Serial.println("[CAM] 카메라 초기화 시도...");
    if (initCamera()) {
        cam_initialized = true;
    } else {
        Serial.println("[SYS] 카메라 초기화 실패");
    }

    // 이더넷 나중에 초기화
    if (!initEthernet()) {
        Serial.println("[SYS] W5500 인식 실패");
    }

    Serial.println("[SYS] 초기화 완료");
    Serial.printf("[SYS] W5500: %s | 카메라: %s\n",
        eth_initialized ? "OK" : "FAIL",
        cam_initialized ? "OK" : "FAIL"
    );
}

// =============================================
// loop
// =============================================
void loop() {
    readArduinoSerial();

    if (!eth_initialized) {
        blinkLED(2000);
        return;
    }

    if (!client.connected()) {
        blinkLED(1000);
        reconnectServer();
        return;
    }

    if (!cam_initialized) {
        blinkLED(500);
        return;
    }

    digitalWrite(LED_PIN, LOW);

    if (millis() - lastSend >= 500) {
        lastSend = millis();
        sendData();
    }
}

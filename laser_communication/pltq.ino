#include <Arduino.h>
#include <U8g2lib.h>
#include <Keypad.h>
#include <Wire.h>

// =========================================================================
// I2C 모터 슬레이브 주소
// =========================================================================
#define MOTOR_I2C_ADDR 0x08

// =========================================================================
// [기존] 하드웨어 핀 설정 및 객체 선언
// =========================================================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_raw(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_comb(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// 4x4 키패드 설정 (요청하신 새로운 키맵핑)
const byte ROWS = 4; 
const byte COLS = 4; 
char keys[ROWS][COLS] = {
     {'D','B','#','E'},
     {'3','6','9','A'},
     {'2','5','8','0'},
     {'1','4','7','M'}
};
// byte rowPins[ROWS] = {25, 26, 27, 14}; 
// byte colPins[COLS] = {16, 17, 5, 18};
//핀 매핑 수정_성민
byte rowPins[ROWS] = {18, 5, 17, 16};
byte colPins[COLS] = {14, 27, 26, 25};

Keypad customKeypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// =========================================================================
// [추가됨] 모스 통신 핀 및 상태 변수
// =========================================================================
#define LASER_PIN      23
#define RX_SIGNAL_PIN  34
#define RX_LENGTH_PIN  35

// [송신용 타이머 및 상태]
#define DOT_ON     200
#define DASH_ON    600
#define SYMBOL_GAP 200
#define JAMO_GAP   850
#define WORD_GAP   2000

enum TxState { TX_IDLE, TX_SIGNAL_ON, TX_SIGNAL_OFF, TX_JAMO_GAP, TX_WORD_GAP };
TxState txState = TX_IDLE;
String txMorseString = "";
int txPos = 0;
unsigned long txTimer = 0;

bool rxLastState = LOW;
unsigned long rxFallTime = 0;      // 마지막 짧은핀 하강 엣지 시각
String rxMorseBuffer = "";
String rxJamoArray[60];
int rxJamoCount = 0;
String rxLogBuffer = "";

bool rxLengthLastState = LOW;      // 긴핀 이전 상태 (미사용이지만 선언 유지)

// [DOT/DASH 판정] 34핀 꺼진 후 일정 시간 동안 35핀 감시
#define LONG_PIN_WATCH 350   // 35핀 감시 시간 (ms)
bool rxPendingSymbol = false;     // 심볼 판정 대기 중 플래그
unsigned long rxFallEdgeTime = 0; // 34핀 하강 엣지 시각
bool rxLongPinSeen = false;       // 감시 중 35핀 HIGH 감지 여부
// 수신 채팅 로그 (최대 3줄, 카톡식 위로 쌓기)
#define RX_LOG_MAX 3
String rxChatLog[RX_LOG_MAX] = {"", "", ""};
// 실시간 수신 중인 모스 기호 (하단 표시용, • -)
String rxLiveMorse = "";
// 모드: 0=타자, 1=모터, 2=모스부호(수동)
int currentMode = 0;

// [모스 수동 모드] 레이저 ON/OFF 타이머
#define MANUAL_DOT_MS  200
#define MANUAL_DASH_MS 600
bool manualLaserOn = false;
unsigned long manualLaserTimer = 0;

void updateDisplays(); // 전방 선언

// =========================================================================
// [기존] 천지인 데이터 테이블
// =========================================================================
const String CHOSUNG_LIST[] = {"ㄱ","ㄲ","ㄴ","ㄷ","ㄸ","ㄹ","ㅁ","ㅂ","ㅃ","ㅅ","ㅆ","ㅇ","ㅈ","ㅉ","ㅊ","ㅋ","ㅌ","ㅍ","ㅎ"};
const String JUNGSUNG_LIST[] = {"ㅏ","ㅐ","ㅑ","ㅒ","ㅓ","ㅔ","ㅕ","ㅖ","ㅗ","ㅘ","ㅙ","ㅚ","ㅛ","ㅜ","ㅝ","ㅞ","ㅟ","ㅠ","ㅡ","ㅢ","ㅣ"};
const String JONGSUNG_LIST[] = {"","ㄱ","ㄲ","ㄳ","ㄴ","ㄵ","ㄶ","ㄷ","ㄹ","ㄺ","ㄻ","ㄼ","ㄽ","ㄾ","ㄿ","ㅀ","ㅁ","ㅂ","ㅄ","ㅅ","ㅆ","ㅇ","ㅈ","ㅊ","ㅋ","ㅌ","ㅍ","ㅎ"};

const String MULTITAP_GROUPS[7][3] = {
  {"ㄱ", "ㅋ", "ㄲ"}, {"ㄴ", "ㄹ", ""}, {"ㄷ", "ㅌ", "ㄸ"}, 
  {"ㅂ", "ㅍ", "ㅃ"}, {"ㅅ", "ㅎ", "ㅆ"}, {"ㅈ", "ㅊ", "ㅉ"}, {"ㅇ", "ㅁ", ""}
};
const int MULTITAP_SIZES[] = {3, 2, 3, 3, 3, 3, 2};
const String PUNCT_GROUP = ".,?!";

struct DoubleConsonant { String b1; String b2; String result; };
DoubleConsonant DBL_CONS[] = {
  {"ㄱ","ㅅ","ㄳ"}, {"ㄴ","ㅈ","ㄵ"}, {"ㄴ","ㅎ","ㄶ"}, {"ㄹ","ㄱ","ㄺ"}, {"ㄹ","ㅁ","ㄻ"}, 
  {"ㄹ","ㅂ","ㄼ"}, {"ㄹ","ㅅ","ㄽ"}, {"ㄹ","ㅌ","ㄾ"}, {"ㄹ","ㅍ","ㄿ"}, {"ㄹ","ㅎ","ㅀ"}, {"ㅂ","ㅅ","ㅄ"}
};

// =========================================================================
// [기존] 천지인 글로벌 상태 변수
// =========================================================================
String typedText = "";       
String currentSyllable = ""; 
int choIdx = -1; int jungIdx = -1; int jongIdx = 0;             
String currentVowelStr = ""; int punctIdx = -1;           
unsigned long lastPressTime = 0;
char lastKey = '\0'; int tapCount = 0;
bool showDecomposedView = false;

// =========================================================================
// [추가됨] 헬퍼 함수: 벡터 기반 점(·) 출력 엔진 (디스플레이 폰트 깨짐 방지용)
// =========================================================================
// =========================================================================
// [추가됨] 송신 텍스트 스크롤: OLED 폭(128px)을 초과하면 앞 글자부터 잘라냄
// 6x10 폰트 기준 한글은 12px, ASCII는 6px 폭으로 계산
// =========================================================================
String getTxDisplayText(String src, int maxWidth) {
  // 뒤에서부터 글자를 쌓아서 maxWidth 이내의 표시 문자열을 반환
  int totalW = 0;
  int startByte = src.length(); // 표시 시작 바이트 위치 (뒤에서 찾음)

  int i = src.length();
  while (i > 0) {
    // UTF-8 역방향: 멀티바이트 시작 바이트 찾기
    int charStart = i - 1;
    while (charStart > 0 && (src.charAt(charStart) & 0xC0) == 0x80) charStart--;

    unsigned char c1 = src.charAt(charStart);
    int charW = 6; // ASCII 기본 폭
    if ((c1 & 0xF0) == 0xE0) charW = 13; // 한글(3바이트): 12px + 1 spacing
    else if ((c1 & 0xE0) == 0xC0) charW = 9; // 2바이트 UTF-8

    if (totalW + charW > maxWidth) break;
    totalW += charW;
    startByte = charStart;
    i = charStart;
  }
  return src.substring(startByte);
}

void drawSafeUTF8(U8G2 &u8g2, int x, int y, String str) {
  int curX = x;
  for (int i = 0; i < str.length(); ) {
    unsigned char c1 = str[i];
    
    // ASCII 기반 온점(.)도 벡터 원형으로 직접 출력
    if (c1 == '.') {
      u8g2.drawDisc(curX + 3, y - 4, 2);
      curX += 8; i++; continue;
    }
    if ((c1 & 0x80) == 0) {
      char buf[2] = {(char)c1, 0};
      u8g2.drawStr(curX, y, buf); curX += u8g2.getStrWidth(buf); i++;
    } else if ((c1 & 0xE0) == 0xC0) {
      // U+00B7 (C2 B7): 소스 파일에 저장된 아래아(·) 인코딩 → 벡터 원으로 직접 출력
      if (i + 1 < str.length() && c1 == 0xC2 && (unsigned char)str[i+1] == 0xB7) {
        u8g2.drawDisc(curX + 3, y - 4, 2);
        curX += 8; i += 2; continue;
      }
      // U+00B7 두 개 연속(··): 벡터 원 2개
      if (i + 3 < str.length() && c1 == 0xC2 && (unsigned char)str[i+1] == 0xB7
          && (unsigned char)str[i+2] == 0xC2 && (unsigned char)str[i+3] == 0xB7) {
        u8g2.drawDisc(curX + 3, y - 4, 2); u8g2.drawDisc(curX + 9, y - 4, 2);
        curX += 14; i += 4; continue;
      }
      String sub = str.substring(i, i + 2);
      u8g2.drawUTF8(curX, y, sub.c_str()); curX += u8g2.getUTF8Width(sub.c_str()); i += 2;
    } else if ((c1 & 0xF0) == 0xE0) {
      if (i + 2 < str.length()) {
        unsigned char c2 = str[i + 1], c3 = str[i + 2];
        // 🚨 U+318D 천지인 고유 점(·) 발견 시 -> 절대 폰트를 쓰지 않고 직접 원을 그림!
        if (c1 == 0xE3 && c2 == 0x86 && c3 == 0x8D) { 
          u8g2.drawDisc(curX + 3, y - 4, 2);
          curX += 8; i += 3; continue;
        }
        // U+318E 쌍점(··) 발견 시 -> 벡터 동그라미 2개 그림
        if (c1 == 0xE3 && c2 == 0x86 && c3 == 0x8E) { 
          u8g2.drawDisc(curX + 3, y - 4, 2); u8g2.drawDisc(curX + 9, y - 4, 2);
          curX += 14; i += 3; continue;
        }
      }
      String sub = str.substring(i, i + 3);
      u8g2.drawUTF8(curX, y, sub.c_str()); curX += u8g2.getUTF8Width(sub.c_str()); i += 3;
    } else { i++; }
  }
}

// =========================================================================
// [추가됨] 통신 및 모아쓰기 헬퍼 함수들 (help.ino 기반)
// =========================================================================
int getChoIdxBase(String s) { for (int i=0; i<19; i++) if (CHOSUNG_LIST[i] == s) return i; return -1; }
int getJungIdxBase(String s) { for (int i=0; i<21; i++) if (JUNGSUNG_LIST[i] == s) return i; return -1; }
int getJongIdxBase(String s) { for (int i=0; i<28; i++) if (JONGSUNG_LIST[i] == s) return i; return -1; }
bool isVowelBase(String s) { return getJungIdxBase(s) != -1; }
bool isConsonantBase(String s) { return getChoIdxBase(s) != -1 || getJongIdxBase(s) > 0; }

String getDoubleJongBase(String j1, String j2) {
  for(int i=0; i<11; i++) if(DBL_CONS[i].b1 == j1 && DBL_CONS[i].b2 == j2) return DBL_CONS[i].result;
  return "";
}

String getDoubleJungBase(String j1, String j2) {
  if (j1=="ㅗ"&&j2=="ㅏ") return "ㅘ"; if (j1=="ㅗ"&&j2=="ㅐ") return "ㅙ";
  if (j1=="ㅗ"&&j2=="ㅣ") return "ㅚ"; if (j1=="ㅜ"&&j2=="ㅓ") return "ㅝ";
  if (j1=="ㅜ"&&j2=="ㅔ") return "ㅞ"; if (j1=="ㅜ"&&j2=="ㅣ") return "ㅟ";
  if (j1=="ㅡ"&&j2=="ㅣ") return "ㅢ"; if (j1=="ㅑ"&&j2=="ㅣ") return "ㅒ"; 
  if (j1=="ㅕ"&&j2=="ㅣ") return "ㅖ"; if (j1=="ㅏ"&&j2=="ㅣ") return "ㅐ"; 
  if (j1=="ㅓ"&&j2=="ㅣ") return "ㅔ"; return "";
}

String composeHangulBase(String cho, String jung, String jong) {
  int ci = getChoIdxBase(cho), vi = getJungIdxBase(jung), ji = getJongIdxBase(jong);
  if (ci == -1 || vi == -1) return cho; if (ji == -1) ji = 0;
  uint32_t u = 0xAC00 + (ci * 21 * 28) + (vi * 28) + ji; String r = "";
  r += (char)(0xE0 | ((u >> 12) & 0x0F)); r += (char)(0x80 | ((u >>  6) & 0x3F)); r += (char)(0x80 | ( u & 0x3F));
  return r;
}

void splitDoubleJongBase(String j, String &l, String &r) {
  for(int i=0; i<11; i++) { if(DBL_CONS[i].result == j) { l = DBL_CONS[i].b1; r = DBL_CONS[i].b2; return; } }
  l = j; r = "";
}

String moasseugiBase(String arr[], int n) {
  String result = "", cho = "", jung = "", jong = "";
  for (int i = 0; i < n; i++) {
    String cur = arr[i];
    if (cur == " " || cur == "W") {
      if (cho != "") result += composeHangulBase(cho, jung, jong);
      result += " "; cho = ""; jung = ""; jong = ""; continue;
    }
    // ── 구두점: 오토마타 우회하여 바로 출력 ──
    if (cur == "," || cur == "." || cur == "?" || cur == "!") {
      if (cho != "") result += composeHangulBase(cho, jung, jong);
      result += cur; cho = ""; jung = ""; jong = ""; continue;
    }
    if (cho == "") {
      if (isConsonantBase(cur)) cho = cur;
      else if (isVowelBase(cur)) { cho = "ㅇ"; jung = cur; }
    } else if (jung == "") {
      if (isVowelBase(cur)) jung = cur; else { result += composeHangulBase(cho, "", ""); cho = cur; }
    } else if (jong == "") {
      if (isConsonantBase(cur)) {
        if (i + 1 < n && isVowelBase(arr[i + 1])) { result += composeHangulBase(cho, jung, ""); cho = cur; jung = ""; } 
        else jong = cur;
      } else if (isVowelBase(cur)) {
        String dblJung = getDoubleJungBase(jung, cur);
        if (dblJung != "") jung = dblJung; else { result += composeHangulBase(cho, jung, ""); cho = "ㅇ"; jung = cur; }
      }
    } else {
      if (isConsonantBase(cur)) {
        String dbl = getDoubleJongBase(jong, cur);
        if (dbl != "" && (i + 1 == n || !isVowelBase(arr[i + 1]))) jong = dbl;
        else { result += composeHangulBase(cho, jung, jong); cho = cur; jung = ""; jong = ""; }
      } else if (isVowelBase(cur)) {
        String lj = "", rj = ""; splitDoubleJongBase(jong, lj, rj);
        if (rj != "") { jong = lj; result += composeHangulBase(cho, jung, jong); cho = rj; }
        else { String nc = jong; jong = ""; result += composeHangulBase(cho, jung, jong); cho = nc; }
        jung = cur;
      }
    }
  }
  if (cho != "") result += composeHangulBase(cho, jung, jong);
  return result;
}

bool decomposeHangul(uint32_t uni, int &c, int &ju, int &jo); // 전방선언

int decomposeTextToJamoBase(String text, String out[]) {
  int count = 0; int len = text.length();
  for (int i = 0; i < len; ) {
    unsigned char c = (unsigned char)text[i];
    if (c == ' ') { out[count++] = " "; i++; } 
    else if (c < 0x80) {
      String ch = String((char)c);
      if (ch == "," || ch == "." || ch == "?" || ch == "!") out[count++] = ch;
      i++;
    } 
    else if ((c & 0xE0) == 0xE0) { 
      if (i + 2 >= len) break;
      uint32_t u = ((c & 0x0F) << 12) | (((unsigned char)text[i+1] & 0x3F) << 6) | (((unsigned char)text[i+2] & 0x3F));
      i += 3;
      if (u >= 0xAC00 && u <= 0xD7A3) {
        int cIdx, juIdx, joIdx;
        decomposeHangul(u, cIdx, juIdx, joIdx);
        out[count++] = CHOSUNG_LIST[cIdx];
        String v = JUNGSUNG_LIST[juIdx];
        if (v == "ㅘ") { out[count++] = "ㅗ"; out[count++] = "ㅏ"; } else if (v == "ㅙ") { out[count++] = "ㅗ"; out[count++] = "ㅐ"; }
        else if (v == "ㅚ") { out[count++] = "ㅗ"; out[count++] = "ㅣ"; } else if (v == "ㅝ") { out[count++] = "ㅜ"; out[count++] = "ㅓ"; }
        else if (v == "ㅞ") { out[count++] = "ㅜ"; out[count++] = "ㅔ"; } else if (v == "ㅟ") { out[count++] = "ㅜ"; out[count++] = "ㅣ"; }
        else if (v == "ㅢ") { out[count++] = "ㅡ"; out[count++] = "ㅣ"; } else if (v == "ㅒ") { out[count++] = "ㅑ"; out[count++] = "ㅣ"; }
        else if (v == "ㅖ") { out[count++] = "ㅕ"; out[count++] = "ㅣ"; } else { out[count++] = v; }
        
        if (joIdx > 0) {
          String j = JONGSUNG_LIST[joIdx]; String lj, rj;
          splitDoubleJongBase(j, lj, rj);
          out[count++] = lj; if(rj != "") out[count++] = rj;
        }
      }
    } else { i++; }
  }
  return count;
}

String jamoToMorse(String jamo) {
  if (jamo=="ㄱ") return ".-.."; if (jamo=="ㄴ") return "..-."; if (jamo=="ㄷ") return "-...";
  if (jamo=="ㄹ") return "...-"; if (jamo=="ㅁ") return "--";   if (jamo=="ㅂ") return ".--";
  if (jamo=="ㅅ") return "--.";  if (jamo=="ㅇ") return "-.-";  if (jamo=="ㅈ") return ".--.";
  if (jamo=="ㅊ") return "-.-."; if (jamo=="ㅋ") return "-..-"; if (jamo=="ㅌ") return "--..";
  if (jamo=="ㅍ") return "---";  if (jamo=="ㅎ") return ".-...";
  if (jamo=="ㄲ") return ".-...-.."; if (jamo=="ㄸ") return "-...-..."; if (jamo=="ㅃ") return ".--.--";
  if (jamo=="ㅆ") return "--.--.";   if (jamo=="ㅉ") return ".--..--.";
  if (jamo=="ㅏ") return ".";    if (jamo=="ㅑ") return "..";   if (jamo=="ㅓ") return "-";
  if (jamo=="ㅕ") return "...";  if (jamo=="ㅗ") return ".-";   if (jamo=="ㅛ") return "-.";
  if (jamo=="ㅜ") return "...."; if (jamo=="ㅠ") return ".-.";  if (jamo=="ㅡ") return "-..";
  if (jamo=="ㅣ") return "..-";  if (jamo=="ㅐ") return "--.-"; if (jamo=="ㅔ") return "-.--";
  if (jamo==" ")  return "W";
  if (jamo==",") return "--..--";
  if (jamo==".") return ".-.-.-";
  if (jamo=="?") return "..--..";
  if (jamo=="!") return "-.-.--";
  return "";
}

String morseToJamo(String morse) {
  if (morse==".-..") return "ㄱ"; if (morse=="..-.") return "ㄴ"; if (morse=="-...") return "ㄷ";
  if (morse=="...-") return "ㄹ"; if (morse=="--")   return "ㅁ"; if (morse==".--")  return "ㅂ";
  if (morse=="--.")  return "ㅅ"; if (morse=="-.-")  return "ㅇ"; if (morse==".--.") return "ㅈ";
  if (morse=="-.-.") return "ㅊ"; if (morse=="-..-") return "ㅋ"; if (morse=="--..") return "ㅌ";
  if (morse=="---")  return "ㅍ"; if (morse==".-...") return "ㅎ";
  if (morse==".-...-..") return "ㄲ"; if (morse=="-...-...") return "ㄸ"; if (morse==".--.--")   return "ㅃ";
  if (morse=="--.--.") return "ㅆ"; if (morse==".--..--.") return "ㅉ";
  if (morse==".")    return "ㅏ"; if (morse=="..")   return "ㅑ"; if (morse=="-")    return "ㅓ";
  if (morse=="...")  return "ㅕ"; if (morse==".-")   return "ㅗ"; if (morse=="-.")   return "ㅛ";
  if (morse=="....") return "ㅜ"; if (morse==".-.")  return "ㅠ"; if (morse=="-..")  return "ㅡ";
  if (morse=="..-")  return "ㅣ"; if (morse=="--.-") return "ㅐ"; if (morse=="-.--") return "ㅔ";
  if (morse=="--..--") return ",";
  if (morse==".-.-.-") return ".";
  if (morse=="..--..") return "?";
  if (morse=="-.-.--") return "!";
  return "?";
}

void startMorseTx(String arr[], int count) {
  txMorseString = "";
  for (int i = 0; i < count; i++) {
    String morse = jamoToMorse(arr[i]);
    if (morse != "") txMorseString += morse + "/";
  }
  txMorseString += "n";
  txPos = 0; txState = TX_SIGNAL_OFF;
  txTimer = millis();
  updateDisplays();
}

void updateTx() {
  if (txState == TX_IDLE) return;
  unsigned long now = millis();
  switch (txState) {
    case TX_SIGNAL_ON:
      if (now - txTimer >= (unsigned long)((txMorseString[txPos-1] == '-') ? DASH_ON : DOT_ON)) {
        digitalWrite(LASER_PIN, LOW);
        txState = TX_SIGNAL_OFF; txTimer = now;
      } break;
    case TX_SIGNAL_OFF:
      if (now - txTimer >= SYMBOL_GAP) {
        if (txPos >= (int)txMorseString.length()) { txState = TX_IDLE; updateDisplays(); return; }
        char c = txMorseString[txPos++];
        if (c == '.' || c == '-') { digitalWrite(LASER_PIN, HIGH); txState = TX_SIGNAL_ON; txTimer = now; }
        else if (c == '/') { txState = TX_JAMO_GAP; txTimer = now; }
        else               { txState = TX_WORD_GAP; txTimer = now; }
      } break;
    case TX_JAMO_GAP:
      if (now - txTimer >= JAMO_GAP) { txState = TX_SIGNAL_OFF; txTimer = now; } break;
    case TX_WORD_GAP:
      if (now - txTimer >= WORD_GAP) { txState = TX_IDLE; updateDisplays(); } break;
    default: break;
  }
}

// =========================================================================
// [추가됨] 모스 수신부 완전 모듈화 (3가지 함수 분리 적용)
// =========================================================================
void addMorseSymbol(char symbol) {
  rxMorseBuffer += symbol;
  // 심볼 사이 띄어쓰기로 구분 (같은 자모 내 심볼 갭)
  if (rxLiveMorse.length() > 0) {
    char last = rxLiveMorse.charAt(rxLiveMorse.length() - 1);
    if (last == '.' || last == '-') rxLiveMorse += ' ';
  }
  rxLiveMorse += symbol;
  updateDisplays();
}

void flushMorseBuffer() {
  if (rxMorseBuffer != "") {
    String jamo = morseToJamo(rxMorseBuffer);
    if (jamo != "" && rxJamoCount < 60) {
      rxJamoArray[rxJamoCount++] = jamo;
    }
    rxMorseBuffer = "";
    rxLiveMorse += '/'; // 자모 갭 구분
    updateDisplays();
  }
}

void processReceivedMessage() {
  String result = moasseugiBase(rxJamoArray, rxJamoCount);
  if (result != "") {
    rxLogBuffer = result;
    Serial.println("[수신] " + rxLogBuffer);
    // 카톡식: 위로 밀어올리기
    for (int i = 0; i < RX_LOG_MAX - 1; i++) rxChatLog[i] = rxChatLog[i + 1];
    rxChatLog[RX_LOG_MAX - 1] = result;
  }
  rxLiveMorse = ""; // 하단 모스 클리어
  rxJamoCount = 0;
  rxFallTime = 0;
  updateDisplays();
}

void updateRx() {
  bool shortPin = digitalRead(RX_SIGNAL_PIN);  // 짧은핀
  bool longPin  = digitalRead(RX_LENGTH_PIN);  // 긴핀
  unsigned long now = millis();

  // 짧은핀 상승 엣지 (레이저 켜짐): JAMO_GAP 지났으면 자모 확정
  if (shortPin == HIGH && rxLastState == LOW) {
    if (rxFallTime > 0 && (now - rxFallTime) >= JAMO_GAP && rxMorseBuffer != "") {
      flushMorseBuffer();
    }
  }

  // 짧은핀 하강 엣지 (레이저 꺼짐): 감시 시작 (중복 엣지 무시)
  if (shortPin == LOW && rxLastState == HIGH && !rxPendingSymbol) {
    rxPendingSymbol = true;
    rxFallEdgeTime = now;
    rxFallTime = now;
    rxLongPinSeen = false;
  }

  // 감시 중: 35핀 HIGH가 한 번이라도 오면 기록
  if (rxPendingSymbol && longPin == HIGH) {
    rxLongPinSeen = true;
  }

  // LONG_PIN_WATCH 시간이 지나면 판정 확정
  // 감시 중 35핀 HIGH가 있었으면 DASH, 없었으면 DOT
  if (rxPendingSymbol && (now - rxFallEdgeTime >= LONG_PIN_WATCH)) {
    addMorseSymbol(rxLongPinSeen ? '-' : '.');
    rxPendingSymbol = false;
    rxLongPinSeen = false;
  }

  // WORD_GAP 초과: 마지막 자모 확정 + 글자 완성
  if (rxMorseBuffer != "" && rxFallTime > 0 && (now - rxFallTime) >= WORD_GAP) {
    flushMorseBuffer();
    processReceivedMessage();
  }

  rxLastState = shortPin;
}

// =========================================================================
// [기존 Tlqkf Base] 천지인 파싱 및 오토마타 헬퍼 (수정 없음)
// =========================================================================
String popLastUtf8Char(String str) {
  if (str.length() == 0) return "";
  int len = str.length(); int i = len - 1;
  while (i > 0 && (str.charAt(i) & 0xC0) == 0x80) { i--; }
  return str.substring(0, i);
}

bool decomposeHangul(uint32_t uni, int &c, int &ju, int &jo) {
  if (uni >= 0xAC00 && uni <= 0xD7A3) {
    uint32_t idx = uni - 0xAC00;
    c = idx / 588; ju = (idx % 588) / 28; jo = idx % 28;
    return true;
  }
  return false;
}

String getNextVowel(String cur, char key) {
  if (cur == "") {
    if (key == '1') return "ㅣ";
    if (key == '2') return "·"; 
    if (key == '3') return "ㅡ";
  }
  if (key == '1') {
    if (cur == "·") return "ㅓ"; if (cur == "··") return "ㅕ"; if (cur == "ㅏ") return "ㅐ";
    if (cur == "ㅑ") return "ㅒ"; if (cur == "ㅓ") return "ㅔ"; if (cur == "ㅕ") return "ㅖ";
    if (cur == "ㅗ") return "ㅚ"; if (cur == "ㅜ") return "ㅟ"; if (cur == "ㅡ") return "ㅢ";
    if (cur == "ㅘ") return "ㅙ"; if (cur == "ㅝ") return "ㅞ";
    if (cur == "ㅠ") return "ㅝ"; // ㅜ→ㅠ→ㅝ 경로
  } 
  else if (key == '2') {
    if (cur == "ㅣ") return "ㅏ"; if (cur == "ㅏ") return "ㅑ"; if (cur == "ㅡ") return "ㅜ";
    if (cur == "ㅜ") return "ㅠ"; if (cur == "·") return "··"; if (cur == "ㅗ") return "ㅛ";
    if (cur == "ㅚ") return "ㅘ";
  } 
  else if (key == '3') {
    if (cur == "·") return "ㅗ"; if (cur == "··") return "ㅛ";
  }
  return "";
}

String assembleHangul() {
  if (choIdx == -1) {
    if (currentVowelStr != "") return currentVowelStr;
    if (jungIdx != -1) return JUNGSUNG_LIST[jungIdx];
    return "";
  }
  if (jungIdx == -1) return CHOSUNG_LIST[choIdx] + currentVowelStr;
  uint16_t uniCode = 0xAC00 + (choIdx * 588) + (jungIdx * 28) + jongIdx;
  String result = "";
  result += (char)(0xE0 | ((uniCode >> 12) & 0x0F));
  result += (char)(0x80 | ((uniCode >> 6) & 0x3F));
  result += (char)(0x80 | (uniCode & 0x3F));
  return result;
}

void flushBuffer() {
  bool hasAraeoaOnly = (currentVowelStr == "·" || currentVowelStr == "··") && jungIdx == -1;

  if (!hasAraeoaOnly && (choIdx != -1 || currentVowelStr != "")) {
    String assembled = assembleHangul();
    if (assembled.indexOf("\xC2\xB7") == -1 &&
        assembled.indexOf("\xE3\x86\x8D") == -1) {
      typedText += assembled;
    }
  } else if (!hasAraeoaOnly && currentSyllable != "") {
    typedText += currentSyllable;
  }

  // 아래아만 있고 초성이 있는 경우: 초성은 다음 글자로 넘겨야 하므로 보존
  int savedChoIdx = (hasAraeoaOnly && choIdx != -1) ? choIdx : -1;

  jungIdx = -1; jongIdx = 0; currentVowelStr = ""; currentSyllable = "";
  tapCount = 0; lastKey = '\0'; punctIdx = -1;

  choIdx = savedChoIdx; // -1이면 그냥 리셋, 아니면 초성 유지
}

// =========================================================================
// [기존 Tlqkf Base] 실시간 자소 드로잉 벡터 엔진 (수정 없음)
// =========================================================================
void drawJasoVector(U8G2 &u8g2, int x, int y, int w, int h, String jaso) {
  u8g2.setDrawColor(1);
  if (jaso == "ㄱ") { u8g2.drawHLine(x, y, w); u8g2.drawVLine(x + w - 1, y, h); }
  else if (jaso == "ㄴ") { u8g2.drawVLine(x, y, h); u8g2.drawHLine(x, y + h - 1, w); }
  else if (jaso == "ㄷ") { u8g2.drawHLine(x, y, w); u8g2.drawVLine(x, y, h); u8g2.drawHLine(x, y + h - 1, w); }
  else if (jaso == "ㄹ") { u8g2.drawHLine(x, y, w); u8g2.drawVLine(x + w - 1, y, h / 3 + 1); u8g2.drawHLine(x, y + h / 2, w); u8g2.drawVLine(x, y + h / 2, h / 2); u8g2.drawHLine(x, y + h - 1, w); }
  else if (jaso == "ㅁ") { u8g2.drawFrame(x, y, w, h); }
  else if (jaso == "ㅂ") { u8g2.drawVLine(x, y, h); u8g2.drawVLine(x + w - 1, y, h); u8g2.drawHLine(x, y + h / 2, w); u8g2.drawHLine(x, y + h - 1, w); }
  else if (jaso == "ㅅ") { u8g2.drawLine(x + w / 2, y, x, y + h - 1); u8g2.drawLine(x + w / 2, y, x + w - 1, y + h - 1); }
  else if (jaso == "ㅇ") { int r = (w < h ? w : h) / 3; if (r < 2) r = 2; u8g2.drawRFrame(x, y, w, h, r); } 
  else if (jaso == "ㅈ") { u8g2.drawHLine(x, y, w); u8g2.drawLine(x + w / 2, y, x, y + h - 1); u8g2.drawLine(x + w / 2, y, x + w - 1, y + h - 1); }
  else if (jaso == "ㅊ") { int topY = y + h / 6; u8g2.drawHLine(x + w / 4, y, w / 2); u8g2.drawHLine(x, topY, w); u8g2.drawLine(x + w / 2, topY, x, y + h - 1); u8g2.drawLine(x + w / 2, topY, x + w - 1, y + h - 1); }
  else if (jaso == "ㅋ") { u8g2.drawHLine(x, y, w); u8g2.drawVLine(x + w - 1, y, h); u8g2.drawHLine(x, y + h / 2, w); }
  else if (jaso == "ㅌ") { u8g2.drawHLine(x, y, w); u8g2.drawVLine(x, y, h); u8g2.drawHLine(x, y + h / 2, w); u8g2.drawHLine(x, y + h - 1, w); }
  else if (jaso == "ㅍ") { u8g2.drawHLine(x, y, w); u8g2.drawHLine(x, y + h - 1, w); u8g2.drawVLine(x + w / 3, y, h); u8g2.drawVLine(x + 2 * w / 3, y, h); }
  else if (jaso == "ㅎ") { int line1_y = y; int line2_y = y + h / 5; int circle_y = y + 2 * h / 5; int circle_h = h - (circle_y - y); u8g2.drawHLine(x + w / 4, line1_y, w / 2); u8g2.drawHLine(x, line2_y, w); u8g2.drawFrame(x + w / 4, circle_y, w / 2, circle_h > 2 ? circle_h : 2); }
  else if (jaso == "ㄲ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄱ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㄱ"); }
  else if (jaso == "ㄸ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄷ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㄷ"); }
  else if (jaso == "ㅃ") { drawJasoVector(u8g2, x, y, w/2, h, "ㅂ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅂ"); }
  else if (jaso == "ㅆ") { drawJasoVector(u8g2, x, y, w/2, h, "ㅅ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅅ"); }
  else if (jaso == "ㅉ") { drawJasoVector(u8g2, x, y, w/2, h, "ㅈ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅈ"); }
  else if (jaso == "ㄳ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄱ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅅ"); }
  else if (jaso == "ㄵ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄴ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅈ"); }
  else if (jaso == "ㄶ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄴ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅎ"); }
  else if (jaso == "ㄺ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄹ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㄱ"); }
  else if (jaso == "ㄻ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄹ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅁ"); }
  else if (jaso == "ㄼ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄹ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅂ"); }
  else if (jaso == "ㄽ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄹ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅅ"); }
  else if (jaso == "ㄾ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄹ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅌ"); }
  else if (jaso == "ㄿ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄹ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅍ"); }
  else if (jaso == "ㅀ") { drawJasoVector(u8g2, x, y, w/2, h, "ㄹ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅎ"); }
  else if (jaso == "ㅄ") { drawJasoVector(u8g2, x, y, w/2, h, "ㅂ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅅ"); }
  else if (jaso == "ㅣ") { u8g2.drawVLine(x + w / 2, y, h); }
  else if (jaso == "ㅡ") { u8g2.drawHLine(x, y + h / 2, w); }
  else if (jaso == "·") { int cx = x + w / 2; int cy = y + h / 2; u8g2.drawPixel(cx, cy); u8g2.drawPixel(cx - 1, cy); u8g2.drawPixel(cx + 1, cy); u8g2.drawPixel(cx, cy - 1); u8g2.drawPixel(cx, cy + 1); }
  else if (jaso == "··") { int cx1 = x + w / 3; int cx2 = x + 2 * w / 3; int cy = y + h / 2; u8g2.drawPixel(cx1, cy); u8g2.drawPixel(cx1 - 1, cy); u8g2.drawPixel(cx1 + 1, cy); u8g2.drawPixel(cx1, cy - 1); u8g2.drawPixel(cx1, cy + 1); u8g2.drawPixel(cx2, cy); u8g2.drawPixel(cx2 - 1, cy); u8g2.drawPixel(cx2 + 1, cy); u8g2.drawPixel(cx2, cy - 1); u8g2.drawPixel(cx2, cy + 1); }
  else if (jaso == "ㅏ") { u8g2.drawVLine(x + w / 3, y, h); u8g2.drawHLine(x + w / 3, y + h / 2, w / 2); }
  else if (jaso == "ㅑ") { u8g2.drawVLine(x + w / 4, y, h); u8g2.drawHLine(x + w / 4, y + h / 3, w / 2); u8g2.drawHLine(x + w / 4, y + 2 * h / 3, w / 2); }
  else if (jaso == "ㅓ") { u8g2.drawVLine(x + 2 * w / 3, y, h); u8g2.drawHLine(x + w / 3, y + h / 2, w / 3); }
  else if (jaso == "ㅕ") { u8g2.drawVLine(x + 3 * w / 4, y, h); u8g2.drawHLine(x + w / 4, y + h / 3, w / 2); u8g2.drawHLine(x + w / 4, y + 2 * h / 3, w / 2); }
  else if (jaso == "ㅗ") { u8g2.drawHLine(x, y + h - 1, w); u8g2.drawVLine(x + w / 2, y + h / 4, h / 2 + 1); }
  else if (jaso == "ㅛ") { u8g2.drawHLine(x, y + h - 1, w); u8g2.drawVLine(x + w / 3, y + h / 4, h / 2 + 1); u8g2.drawVLine(x + 2 * w / 3, y + h / 4, h / 2 + 1); }
  else if (jaso == "ㅜ") { u8g2.drawHLine(x, y, w); u8g2.drawVLine(x + w / 2, y, h / 2 + 1); }
  else if (jaso == "ㅠ") { u8g2.drawHLine(x, y, w); u8g2.drawVLine(x + w / 3, y, h / 2 + 1); u8g2.drawVLine(x + 2 * w / 3, y, h / 2 + 1); }
  else if (jaso == "ㅐ") { u8g2.drawVLine(x + w / 4, y, h); u8g2.drawHLine(x + w / 4, y + h / 2, w / 2); u8g2.drawVLine(x + 3 * w / 4, y, h); }
  else if (jaso == "ㅒ") { u8g2.drawVLine(x + w / 5, y, h); u8g2.drawHLine(x + w / 5, y + h / 3, 3 * w / 5); u8g2.drawHLine(x + w / 5, y + 2 * h / 3, 3 * w / 5); u8g2.drawVLine(x + 4 * w / 5, y, h); }
  else if (jaso == "ㅔ") { u8g2.drawVLine(x + w / 4, y, h); u8g2.drawHLine(x, y + h / 2, w / 4); u8g2.drawVLine(x + 3 * w / 4, y, h); }
  else if (jaso == "ㅖ") { u8g2.drawVLine(x + w / 4, y, h); u8g2.drawHLine(x, y + h / 3, w / 4); u8g2.drawHLine(x, y + 2 * h / 3, w / 4); u8g2.drawVLine(x + 3 * w / 4, y, h); }
  else if (jaso == "ㅚ") { drawJasoVector(u8g2, x, y + h/2 + 1, w/2, h - h/2 - 1, "ㅗ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅣ"); }
  else if (jaso == "ㅘ") { drawJasoVector(u8g2, x, y + h/2 + 1, w/2, h - h/2 - 1, "ㅗ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅏ"); }
  else if (jaso == "ㅙ") { drawJasoVector(u8g2, x, y + h/2 + 1, w/2, h - h/2 - 1, "ㅗ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅐ"); }
  else if (jaso == "ㅝ") { drawJasoVector(u8g2, x, y + h/2 + 1, w/2, h - h/2 - 1, "ㅜ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅓ"); }
  else if (jaso == "ㅞ") { drawJasoVector(u8g2, x, y + h/2 + 1, w/2, h - h/2 - 1, "ㅜ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅔ"); }
  else if (jaso == "ㅟ") { drawJasoVector(u8g2, x, y + h/2 + 1, w/2, h - h/2 - 1, "ㅜ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅣ"); }
  else if (jaso == "ㅢ") { drawJasoVector(u8g2, x, y + h/2 + 1, w/2, h - h/2 - 1, "ㅡ"); drawJasoVector(u8g2, x + w/2 + 1, y, w - w/2 - 1, h, "ㅣ"); }
}

void drawHangulSyllableBox(U8G2 &u8g2, int x, int y, int size, String cho, String jung, String jong) {
  if (cho == "" && jung == "" && jong == "") return;
  if (cho == "" && jung != "") { drawJasoVector(u8g2, x, y, size, size, jung); return; }
  if (cho != "" && jung == "") { drawJasoVector(u8g2, x, y, size, size, cho); return; }
  
  bool isVertical = (jung == "ㅣ" || jung == "ㅏ" || jung == "ㅑ" || jung == "ㅓ" || jung == "ㅕ" || jung == "ㅐ" || jung == "ㅒ" || jung == "ㅔ" || jung == "ㅖ");
  bool isHorizontal = (jung == "ㅡ" || jung == "ㅗ" || jung == "ㅛ" || jung == "ㅜ" || jung == "ㅠ");
  int gap = 1; int cW = size, cH = size; int jW = size, jH = size; int jgW = size, jgH = size;
  int cX = x, cY = y; int jX = x, jY = y; int jgX = x, jgY = y;

  if (jong != "") { 
    int topH = (size * 56) / 100;
    int botH = size - topH - gap; 
    jgX = x; jgY = y + topH + gap; jgW = size; jgH = botH;
    
    if (isVertical) {
      cW = (size * 45) / 100; cH = topH; 
      jX = x + cW + gap; jY = y; jW = size - cW - gap; jH = topH;
    } else if (isHorizontal) {
      cH = (size * 38) / 100; cW = size; 
      jX = x; jY = y + cH; jW = size; jH = topH - cH;
    } else {
      cW = (size * 45) / 100; cH = (size * 38) / 100; jX = x; jY = y; jW = size; jH = topH; 
    }
  } else {
    if (isVertical) {
      cW = (size * 45) / 100; cH = size; 
      jX = x + cW + gap; jY = y; jW = size - cW - gap; jH = size;
    } else if (isHorizontal) {
      cH = (size * 45) / 100; cW = size; 
      jX = x; jY = y + cH + gap; jW = size; jH = size - cH - gap;
    } else {
      cW = (size * 45) / 100; cH = (size * 45) / 100; jX = x; jY = y; jW = size; jH = size;
    }
  }
  
  if (cho != "") drawJasoVector(u8g2, cX, cY, cW, cH, cho);
  if (jung != "") drawJasoVector(u8g2, jX, jY, jW, jH, jung);
  if (jong != "") drawJasoVector(u8g2, jgX, jgY, jgW, jgH, jong);
}

void renderStringCustom(U8G2 &u8g2, int startX, int startY, String str, int boxSize, int spacing) {
  int curX = startX; int curY = startY;
  for (int i = 0; i < str.length(); ) {
    unsigned char c1 = str.charAt(i); uint32_t uni = 0; int len = 0;
    if ((c1 & 0x80) == 0) { uni = c1; len = 1; }
    else if ((c1 & 0xE0) == 0xC0) { uni = ((c1 & 0x1F) << 6) | (str.charAt(i+1) & 0x3F); len = 2; }
    else if ((c1 & 0xF0) == 0xE0) { uni = ((c1 & 0x0F) << 12) | ((str.charAt(i+1) & 0x3F) << 6) | (str.charAt(i+2) & 0x3F); len = 3; }
    if (len == 0) { i++; continue; }
    int startIdx = i; i += len;
    if (uni >= 0xAC00 && uni <= 0xD7A3) { 
      int c = 0, ju = 0, jo = 0; decomposeHangul(uni, c, ju, jo);
      drawHangulSyllableBox(u8g2, curX, curY, boxSize, CHOSUNG_LIST[c], JUNGSUNG_LIST[ju], JONGSUNG_LIST[jo]);
      curX += boxSize + spacing;
    } else if (uni >= 0x3131 && uni <= 0x318E) { 
      String logicStr = str.substring(startIdx, startIdx + len);
      drawHangulSyllableBox(u8g2, curX, curY, boxSize, logicStr, "", "");
      curX += boxSize + spacing;
    } else { 
      String logicStr = str.substring(startIdx, startIdx + len);
      if (logicStr == " ") { curX += boxSize / 2 + spacing; }
      else if (uni == 0x00B7 || uni == 0x318D) {
        // 아래아(·) — 벡터 원으로 직접 출력
        drawJasoVector(u8g2, curX, curY, boxSize, boxSize, "·");
        curX += boxSize + spacing;
      } else {
        u8g2.setFont(u8g2_font_6x10_tf); u8g2.drawStr(curX, curY + boxSize - 1, logicStr.c_str());
        curX += 6 + spacing;
      }
    }
    if (curX > 120) { curX = startX; curY += boxSize + 6; }
  }
}

// =========================================================================
// [추가됨] 수신부 하단 실시간 모스 벡터 렌더러
// rxLiveMorse는 '.', '-', ' '(자모 간 구분) 조합의 원시 문자열
// '.'  → 채워진 원(disc) 벡터
// '-'  → 가로 선(hline) 벡터
// ' '  → 자모 간 간격
// 앞 심볼과 뒤 심볼 사이에는 자동으로 심볼 간격(gap)을 줌
// =========================================================================
void drawRxLiveMorse(U8G2 &u8g2, int x, int y, String morse) {
  // 오른쪽 끝 기준으로 최대 표시 글자 수 제한 (바이트 기준 20자)
  if ((int)morse.length() > 20) morse = morse.substring(morse.length() - 20);

  const int DOT_R    = 2;   // 점 반지름
  const int DASH_W   = 7;   // 선 길이
  const int DASH_H   = 1;   // 선 두께 (1=1px, 2=2px)
  const int SYM_GAP  = 3;   // 심볼 간 간격
  const int JAMO_GAP_PX = 6; // 자모(공백) 간 간격
  const int CY       = y;   // 수직 중심

  int curX = x;
  bool prevWasSymbol = false; // 직전에 심볼(점/선)이 있었는지

  for (int i = 0; i < (int)morse.length(); i++) {
    char c = morse.charAt(i);
    if (c == ' ') {
      curX += JAMO_GAP_PX;
      prevWasSymbol = false;
    } else if (c == '.') {
      if (prevWasSymbol) curX += SYM_GAP;
      u8g2.drawDisc(curX + DOT_R, CY, DOT_R);
      curX += DOT_R * 2 + 1;
      prevWasSymbol = true;
    } else if (c == '/') {
      if (prevWasSymbol) curX += SYM_GAP;
      // 슬래시: 단어 구분 — 대각선으로 그리기
      u8g2.drawLine(curX + 1, CY + 3, curX + 4, CY - 3);
      curX += 7;
      prevWasSymbol = false;
    } else if (c == '-') {
      if (prevWasSymbol) curX += SYM_GAP;
      // 2px 두께 가로 선
      u8g2.drawHLine(curX, CY - 1, DASH_W);
      u8g2.drawHLine(curX, CY,     DASH_W);
      curX += DASH_W + 1;
      prevWasSymbol = true;
    }
    if (curX > 126) break; // 화면 넘침 방지
  }
}

// =========================================================================
// [기존 Tlqkf Base + 수정됨] 디스플레이 렌더러 (폰트 . 렌더링 무시 후 커스텀 적용)
// =========================================================================
void updateDisplays() {
  // 1번 OLED: 상하단 직관적 분할 레이아웃 및 텍스트/모스 송신 모니터
  u8g2_raw.clearBuffer();
  u8g2_raw.enableUTF8Print();
  u8g2_raw.setFont(u8g2_font_6x10_tf);

  // 우상단 모드 표시
  const char* modeLabel[] = {"[TYPING]", "[MOTOR]", "[MORSE]"};
  u8g2_raw.drawStr(128 - (strlen(modeLabel[currentMode]) * 6), 10, modeLabel[currentMode]);

  u8g2_raw.setFont(u8g2_font_unifont_t_korean2); 
  
  // 🚨 상단 영역: 기존 `u8g2_raw.print(typedText)`를 없애고 커스텀 벡터 함수 탑재
  // OLED 128px 폭을 넘으면 앞 글자를 잘라내어 항상 최신 텍스트가 보이도록 스크롤
  drawSafeUTF8(u8g2_raw, 0, 20, getTxDisplayText(typedText, 128));
  
  u8g2_raw.drawHLine(0, 36, 128);
  
  // 하단 영역: 고정식 벡터 조합 피드백 공간
  String activeSyllable = "";
  if (choIdx != -1 || currentVowelStr != "") activeSyllable = assembleHangul();
  if (punctIdx != -1) activeSyllable = String(PUNCT_GROUP.charAt(punctIdx));
  
  if (activeSyllable != "") {
    u8g2_raw.setFont(u8g2_font_6x10_tf);
    u8g2_raw.drawStr(4, 53, "Typing:");
    renderStringCustom(u8g2_raw, 52, 42, activeSyllable, 16, 2);
    u8g2_raw.drawFrame(49, 39, 22, 22);
  } else if (currentMode == 1) {
    // 모터 모드 안내
    u8g2_raw.setFont(u8g2_font_6x10_tf);
    u8g2_raw.drawStr(4, 46, "2:UP  8:DOWN");
    u8g2_raw.drawStr(4, 59, "4:L   6:R  (re=STOP)");
  } else if (currentMode == 2) {
    // 모스 수동 모드 안내
    u8g2_raw.setFont(u8g2_font_6x10_tf);
    u8g2_raw.drawStr(4, 46, "2(.) : DOT");
    u8g2_raw.drawStr(4, 59, "3(-) : DASH");
  } else {
    u8g2_raw.setFont(u8g2_font_6x10_tf);
    if (txState != TX_IDLE) u8g2_raw.drawStr(4, 53, "TX...");
    else u8g2_raw.drawStr(4, 53, "Ready...");
  }
  u8g2_raw.sendBuffer();

  // 2번 OLED: 수신 로그 독립 모니터 출력
  u8g2_comb.clearBuffer();
  if (showDecomposedView) {
    u8g2_comb.setFont(u8g2_font_6x10_tf);
    u8g2_comb.drawStr(4, 12, "[ DECOMPOSE DEBUG ]");
    u8g2_comb.drawHLine(0, 16, 128);
    u8g2_comb.drawStr(4, 31, "Chosung  :"); 
    if (choIdx != -1) u8g2_comb.drawStr(70, 31, CHOSUNG_LIST[choIdx].c_str());
    u8g2_comb.drawStr(4, 45, "Jungsung :"); 
    if (currentVowelStr != "") u8g2_comb.drawStr(70, 45, currentVowelStr.c_str());
    u8g2_comb.drawStr(4, 59, "Jongsung :"); 
    if (jongIdx > 0) u8g2_comb.drawStr(70, 59, JONGSUNG_LIST[jongIdx].c_str());
  } else {
    // 수신 OLED: 상단=카톡식 채팅 로그, 하단=실시간 모스 기호
    u8g2_comb.drawHLine(0, 44, 128); // 상하 구분선

    // ── 상단: 채팅 로그 (최대 3줄, 아래가 최신) ──────────────────────
    // 줄 높이 13px, 3줄 → y: 13, 26, 39
    for (int i = 0; i < RX_LOG_MAX; i++) {
      if (rxChatLog[i] == "") continue;
      int y = 13 + i * 13;
      u8g2_comb.setFont(u8g2_font_unifont_t_korean2);
      drawSafeUTF8(u8g2_comb, 0, y, rxChatLog[i]);
    }
    if (rxChatLog[0] == "" && rxChatLog[1] == "" && rxChatLog[2] == "") {
      u8g2_comb.setFont(u8g2_font_6x10_tf);
      u8g2_comb.drawStr(4, 20, "Waiting RX...");
    }

    // ── 하단: 실시간 모스 기호 벡터 렌더 (• -) ──────────────────────
    if (rxLiveMorse != "") {
      // 벡터 렌더러로 직접 점/선 그리기 (폰트 깨짐 없음)
      drawRxLiveMorse(u8g2_comb, 2, 57, rxLiveMorse);
    } else {
      u8g2_comb.setFont(u8g2_font_6x10_tf);
      u8g2_comb.drawStr(2, 61, "-- RX idle --");
    }
  }
  u8g2_comb.sendBuffer();
}

// =========================================================================
// [기존 Tlqkf Base] 오토마타 핵심 프로세서 (수정 없음, Loop에서 우회 관리)
// =========================================================================
void processKey(char key) {
  unsigned long now = millis();
  
  if (key == 'E' || key == 'D') { showDecomposedView = !showDecomposedView; return; }
  if (key == 'A' || key == 'C') { flushBuffer(); return; }
  if (key == '#') {
    if (currentSyllable != "" || choIdx != -1 || currentVowelStr != "") { flushBuffer(); }
    else { typedText += " "; }
    lastPressTime = now; return;
  }
  
  if (key == 'B') {
    if (choIdx != -1 || currentVowelStr != "" || jongIdx > 0 || punctIdx != -1) {
      if (lastKey == '*') { currentSyllable = ""; punctIdx = -1; lastKey = '\0'; }
      else {
        if (jongIdx > 0) {
          String jStr = JONGSUNG_LIST[jongIdx]; String keepJong = ""; bool isDouble = false;
          for (int i = 0; i < 11; i++) { if (DBL_CONS[i].result == jStr) { keepJong = DBL_CONS[i].b1; isDouble = true; break; } }
          if (isDouble) { for(int j=0; j<28; j++) { if(JONGSUNG_LIST[j] == keepJong) { jongIdx = j; break; } } } 
          else { jongIdx = 0; }
        } 
        else if (currentVowelStr != "") {
          if (currentVowelStr == "ㅙ") currentVowelStr = "ㅘ"; else if (currentVowelStr == "ㅘ") currentVowelStr = "ㅗ";
          else if (currentVowelStr == "ㅝ") currentVowelStr = "ㅜ"; else if (currentVowelStr == "ㅞ") currentVowelStr = "ㅝ";
          else if (currentVowelStr == "ㅚ") currentVowelStr = "ㅗ"; else if (currentVowelStr == "ㅟ") currentVowelStr = "ㅜ";
          else if (currentVowelStr == "ㅢ") currentVowelStr = "ㅡ"; else if (currentVowelStr == "ㅐ") currentVowelStr = "ㅏ";
          else if (currentVowelStr == "ㅔ") currentVowelStr = "ㅓ"; else if (currentVowelStr == "ㅒ") currentVowelStr = "ㅑ";
          else if (currentVowelStr == "ㅖ") currentVowelStr = "ㅕ"; else if (currentVowelStr == "ㅑ") currentVowelStr = "ㅏ";
          else if (currentVowelStr == "ㅕ") currentVowelStr = "ㅓ"; else if (currentVowelStr == "ㅛ") currentVowelStr = "ㅗ";
          else if (currentVowelStr == "ㅠ") currentVowelStr = "ㅜ"; else if (currentVowelStr == "ㅏ") currentVowelStr = "ㅣ";
          else if (currentVowelStr == "ㅓ") currentVowelStr = "·"; else if (currentVowelStr == "ㅗ") currentVowelStr = "·";
          else if (currentVowelStr == "ㅜ") currentVowelStr = "ㅡ"; else if (currentVowelStr == "··") currentVowelStr = "·";
          else { currentVowelStr = ""; }
          
          jungIdx = -1;
          if (currentVowelStr != "") {
            for (int i = 0; i < 21; i++) { if (JUNGSUNG_LIST[i] == currentVowelStr) { jungIdx = i; break; } }
          }
        } 
        else if (choIdx != -1) { choIdx = -1; }
        currentSyllable = (choIdx == -1 && currentVowelStr == "") ? "" : assembleHangul();
      }
    } else { typedText = popLastUtf8Char(typedText); }
    lastPressTime = now; return;
  }
  
  if (key == '*') {
    if (lastKey != '*' && currentSyllable != "") { flushBuffer(); }
    if (lastKey == '*' && (now - lastPressTime < 1000)) { punctIdx = (punctIdx + 1) % PUNCT_GROUP.length(); }
    else { flushBuffer(); punctIdx = 0; }
    currentSyllable = String(PUNCT_GROUP.charAt(punctIdx)); lastKey = key; lastPressTime = now; return;
  }
  if (lastKey == '*') { flushBuffer(); }

  if (key == '1' || key == '2' || key == '3') {
    if (choIdx != -1 && jungIdx != -1 && jongIdx > 0) {
      String jStr = JONGSUNG_LIST[jongIdx]; String keepJong = ""; String migrateCho = ""; bool isDouble = false;
      for (int i = 0; i < 11; i++) { if (DBL_CONS[i].result == jStr) { keepJong = DBL_CONS[i].b1; migrateCho = DBL_CONS[i].b2; isDouble = true; break; } }
      if (!isDouble) { migrateCho = jStr; keepJong = ""; }
      if (keepJong == "") jongIdx = 0;
      else { for(int j=0; j<28; j++) { if(JONGSUNG_LIST[j] == keepJong) { jongIdx = j; break; } } }
      typedText += assembleHangul(); choIdx = -1;      for(int i=0; i<19; i++) { if(CHOSUNG_LIST[i] == migrateCho) { choIdx = i; break; } }
      jungIdx = -1; jongIdx = 0; currentVowelStr = "";
    }
    
    String nextV = getNextVowel(currentVowelStr, key);
    if (nextV != "") {
      currentVowelStr = nextV; jungIdx = -1;
      for (int i = 0; i < 21; i++) { if (JUNGSUNG_LIST[i] == currentVowelStr) { jungIdx = i; break; } }
    } else {
      flushBuffer(); currentVowelStr = getNextVowel("", key);
      for (int i = 0; i < 21; i++) { if (JUNGSUNG_LIST[i] == currentVowelStr) { jungIdx = i; break; } }
    }
    currentSyllable = assembleHangul(); lastKey = key; lastPressTime = now; return;
  }

  int groupIdx = -1;
  if (key == '4') groupIdx = 0; else if (key == '5') groupIdx = 1; else if (key == '6') groupIdx = 2;
  else if (key == '7') groupIdx = 3; else if (key == '8') groupIdx = 4; else if (key == '9') groupIdx = 5;
  else if (key == '0') groupIdx = 6;
  
  if (groupIdx != -1) {
    int groupSize = MULTITAP_SIZES[groupIdx];
    if (choIdx == -1 && currentVowelStr != "") { flushBuffer(); }
    if (choIdx == -1) {
      tapCount = 0; lastKey = key; String con = MULTITAP_GROUPS[groupIdx][tapCount];
      for (int i = 0; i < 19; i++) { if (CHOSUNG_LIST[i] == con) { choIdx = i; break; } }
    }
    else if (choIdx != -1 && jungIdx == -1) {
      if (key == lastKey && (now - lastPressTime < 1000)) { tapCount = (tapCount + 1) % groupSize; }
      else { flushBuffer(); tapCount = 0; lastKey = key; }
      String con = MULTITAP_GROUPS[groupIdx][tapCount];
      choIdx = -1;
      for (int i = 0; i < 19; i++) { if (CHOSUNG_LIST[i] == con) { choIdx = i; break; } }
    }
    else if (choIdx != -1 && jungIdx != -1) {
      if (jongIdx == 0) {
        tapCount = 0; lastKey = key; String con = MULTITAP_GROUPS[groupIdx][tapCount];
        for (int i = 0; i < 28; i++) { if (JONGSUNG_LIST[i] == con) { jongIdx = i; break; } }
      }
      else {
        if (key == lastKey && (now - lastPressTime < 1000)) {
          String curJongStr = JONGSUNG_LIST[jongIdx]; bool insideGroup = false;
          for(int m=0; m<groupSize; m++) { if(MULTITAP_GROUPS[groupIdx][m] == curJongStr) { insideGroup = true; break; } }
          if (insideGroup) {
            // 단순 종성(ㄱ,ㄹ 등) 멀티탭 순환
            tapCount = (tapCount + 1) % groupSize; String con = MULTITAP_GROUPS[groupIdx][tapCount];
            for (int i = 0; i < 28; i++) { if (JONGSUNG_LIST[i] == con) { jongIdx = i; break; } }
          } else {
            // 겹받침 상태에서 같은 키 재입력 → 두 번째 자음을 멀티탭 순환
            // ex) ㄽ(ㄹ+ㅅ) 상태에서 8키 재입력 → ㄹ+ㅎ=ㅀ, 한 번 더 → ㄹ+ㅆ=없으면 flushBuffer
            bool foundDouble = false;
            for (int i = 0; i < 11; i++) {
              if (DBL_CONS[i].result == curJongStr) {
                // 현재 겹받침의 b2가 같은 그룹인지 확인
                int b2GroupIdx = -1;
                for(int g=0; g<7; g++) for(int m=0; m<MULTITAP_SIZES[g]; m++)
                  if(MULTITAP_GROUPS[g][m] == DBL_CONS[i].b2) { b2GroupIdx = g; break; }
                if (b2GroupIdx == groupIdx) {
                  // b2를 다음 탭으로 올려서 새 겹받침 시도
                  int curTap = 0;
                  for(int m=0; m<MULTITAP_SIZES[groupIdx]; m++)
                    if(MULTITAP_GROUPS[groupIdx][m] == DBL_CONS[i].b2) { curTap = m; break; }
                  int nextTap = (curTap + 1) % MULTITAP_SIZES[groupIdx];
                  String nextB2 = MULTITAP_GROUPS[groupIdx][nextTap];
                  bool nextDoubleFound = false;
                  for(int k=0; k<11; k++) {
                    if(DBL_CONS[k].b1 == DBL_CONS[i].b1 && DBL_CONS[k].b2 == nextB2) {
                      for(int j=0; j<28; j++) { if(JONGSUNG_LIST[j]==DBL_CONS[k].result) { jongIdx=j; break; } }
                      tapCount = nextTap; nextDoubleFound = true; foundDouble = true; break;
                    }
                  }
                  if (!nextDoubleFound) {
                    // 다음 탭으로 겹받침 안 되면 flushBuffer 후 새 초성
                    flushBuffer(); tapCount = 0; lastKey = key;
                    String con = MULTITAP_GROUPS[groupIdx][tapCount];
                    for(int ii=0; ii<19; ii++) { if(CHOSUNG_LIST[ii]==con) { choIdx=ii; break; } }
                    foundDouble = true;
                  }
                  break;
                }
              }
            }
            if (!foundDouble) {
              flushBuffer(); tapCount = 0; lastKey = key; String con = MULTITAP_GROUPS[groupIdx][tapCount];
              for (int i = 0; i < 19; i++) { if (CHOSUNG_LIST[i] == con) { choIdx = i; break; } }
            }
          }
        } else {
          String curJongStr = JONGSUNG_LIST[jongIdx]; bool isDoubleFormed = false;
          // 새 키의 모든 탭 후보를 시도해 겹받침 여부 확인
          // 단, 이미 겹받침인 경우 동일한 결과로 다시 매칭되는 탭은 건너뜀
          for (int t = 0; t < MULTITAP_SIZES[groupIdx] && !isDoubleFormed; t++) {
            String newJongStr = MULTITAP_GROUPS[groupIdx][t];
            if (newJongStr == "") continue;
            for (int i = 0; i < 11; i++) {
              if (DBL_CONS[i].b1 == curJongStr && DBL_CONS[i].b2 == newJongStr) {
                // 이미 현재 종성과 동일한 결과면 스킵 (ex: ㄽ 상태에서 ㅅ→ㄽ 재매칭 방지)
                if (DBL_CONS[i].result == curJongStr) break;
                for(int j=0; j<28; j++) { if(JONGSUNG_LIST[j] == DBL_CONS[i].result) { jongIdx = j; break; } }
                tapCount = t; lastKey = key;
                isDoubleFormed = true; break;
              }
            }
          }
          if (isDoubleFormed) { tapCount = 0; lastKey = key; }
          else {
            flushBuffer(); tapCount = 0; lastKey = key; String con = MULTITAP_GROUPS[groupIdx][tapCount];
            for (int i = 0; i < 19; i++) { if (CHOSUNG_LIST[i] == con) { choIdx = i; break; } }
          }
        }
      }
    }
    currentSyllable = assembleHangul(); lastPressTime = now;
  }
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);

  // I2C 마스터 초기화
  Wire.begin();          // OLED용 (SDA=21, SCL=22)
  Wire1.begin(32, 33);   // 아두이노 모터 슬레이브용 (SDA=32, SCL=33)

  // 모스부호용 통신 핀 활성화
  pinMode(LASER_PIN, OUTPUT);
  pinMode(RX_SIGNAL_PIN, INPUT);
  pinMode(RX_LENGTH_PIN, INPUT);
  digitalWrite(LASER_PIN, LOW);

  u8g2_raw.setI2CAddress(0x78); u8g2_raw.begin();
  u8g2_comb.setI2CAddress(0x7A); u8g2_comb.begin();
  updateDisplays();
}

// =========================================================================
// I2C 모터 명령 전송 헬퍼
// =========================================================================
void sendMotorCmd(const char* cmd) {
  Wire1.beginTransmission(MOTOR_I2C_ADDR);
  Wire1.write((const uint8_t*)cmd, strlen(cmd));
  Wire1.endTransmission();
}

// =========================================================================
// 모스 수동 모드: 레이저 타이머 처리
// =========================================================================
void updateManualLaser() {
  if (manualLaserOn && (millis() - manualLaserTimer >= (unsigned long)(manualLaserOn ? 1 : 0))) {
    // 실제 OFF는 아래 로직에서 처리
  }
}

// =========================================================================
// LOOP: 모드별 분기 처리
// =========================================================================
void loop() {
  unsigned long now = millis();

  // 수동 레이저 타임아웃 처리 (모스 수동 모드)
  if (manualLaserOn && (now - manualLaserTimer >= (manualLaserOn ? manualLaserOn : 1))) {
    // 타이머는 키 누름 시 설정, 여기서는 상태만 감시
  }

  updateTx();
  updateRx();

  // ── 모드 공통: M키는 항상 모드 전환 ──────────────────────────────────

  char customKey = customKeypad.getKey();

  if (customKey == 'M') {
    if (currentMode == 1) { sendMotorCmd("STOP"); digitalWrite(LASER_PIN, LOW); }
    if (currentMode == 2) { digitalWrite(LASER_PIN, LOW); manualLaserOn = false; }
    currentMode = (currentMode + 1) % 3;
    if (currentMode == 1) digitalWrite(LASER_PIN, HIGH); // 모터 모드 진입 시 레이저 ON
    updateDisplays();
    return;
  }

  // 타이핑 모드(0)에서는 송신 텍스트까지 함께 클리어
  // 모터(1) / 모스 수동(2) 모드에서는 수신 버퍼만 클리어
  if (customKey == 'D') {
    // 수신 관련 버퍼 (모든 모드 공통 클리어)
    rxMorseBuffer = ""; rxJamoCount = 0; rxFallTime = 0;
    rxLiveMorse = ""; rxPendingSymbol = false; rxLongPinSeen = false;
    for (int i = 0; i < RX_LOG_MAX; i++) rxChatLog[i] = "";
    rxLogBuffer = "";

    // 타이핑 모드(0)일 때만 송신 텍스트 및 오토마타 상태도 클리어
    if (currentMode == 0) {
      choIdx = -1; jungIdx = -1; jongIdx = 0; currentVowelStr = ""; currentSyllable = "";
      tapCount = 0; lastKey = '\0'; punctIdx = -1;
      typedText = "";
    }

    lastPressTime = now;
    updateDisplays();
    return;  // 이후 모드별 처리로 내려가지 않도록 차단
  }

  // ── 모드 0: 타자 모드 ────────────────────────────────────────────────
  if (currentMode == 0) {
    if (txState != TX_IDLE) return;

    if (customKey) {
      if (customKey == 'E') {
        flushBuffer();
        if (typedText.length() > 0) {
          String txJamo[100];
          int txCount = decomposeTextToJamoBase(typedText, txJamo);
          startMorseTx(txJamo, txCount);
          typedText = "";
        }
        lastPressTime = now;
      }
      else if (customKey == 'A') { flushBuffer(); lastPressTime = now; }
      else if (customKey == '#') { processKey('*'); lastPressTime = now; }
      else { processKey(customKey); lastPressTime = now; }
      updateDisplays();
    }

    // 시리얼 디버깅
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c != '\n' && c != '\r' && c != ' ') {
        if      (c == 'D') { choIdx=-1; jungIdx=-1; jongIdx=0; currentVowelStr=""; currentSyllable=""; tapCount=0; lastKey='\0'; punctIdx=-1; typedText=""; rxMorseBuffer=""; rxJamoCount=0; rxFallTime=0; rxLiveMorse=""; rxPendingSymbol=false; rxLongPinSeen=false; for(int i=0;i<RX_LOG_MAX;i++) rxChatLog[i]=""; rxLogBuffer=""; lastPressTime=now; }
        else if (c == 'E') { flushBuffer(); if(typedText.length()>0){ String j[100]; int n=decomposeTextToJamoBase(typedText,j); startMorseTx(j,n); typedText=""; } lastPressTime=now; }
        else if (c == 'A') { flushBuffer(); lastPressTime=now; }
        else if (c == '#') { processKey('*'); lastPressTime=now; }
        else { processKey(c); lastPressTime=now; }
        updateDisplays();
      }
    }

    if (currentSyllable != "" && (now - lastPressTime > 4000)) {
      flushBuffer(); updateDisplays();
    }
  }

  // ── 모드 1: 모터 제어 모드 ───────────────────────────────────────────
  // 키 → I2C 명령 매핑
  // 2(아래아 버튼=·): 위로  /  8: 아래로  /  4(ㄱ): 왼쪽  /  6(ㄷ): 오른쪽
  // 같은 버튼 재입력 → STOP  /  다른 방향 버튼 → 즉시 전환
  else if (currentMode == 1) {
    static char lastMotorKey = '\0';
    static bool motorRunning = false;

    if (customKey) {
      // 이동 버튼 목록
      bool isMove = (customKey=='2'||customKey=='8'||customKey=='4'||customKey=='6');
      if (isMove) {
        const char* cmd = "";
        if      (customKey == '2') cmd = "UP";
        else if (customKey == '8') cmd = "DOWN";
        else if (customKey == '4') cmd = "LEFT";
        else if (customKey == '6') cmd = "RIGHT";

        if (motorRunning && customKey == lastMotorKey) {
          // 같은 키 재입력 → 정지
          sendMotorCmd("STOP");
          motorRunning = false; lastMotorKey = '\0';
        } else {
          // 새 방향 or 다른 방향 → 즉시 이동
          sendMotorCmd(cmd);
          motorRunning = true; lastMotorKey = customKey;
        }
        updateDisplays();
      }
      // 나머지 키는 모터 모드에서 무시
    }
  }

  // ── 모드 2: 모스 수동 송신 모드 ─────────────────────────────────────
  // 2(아래아): 짧은 레이저(DOT)  /  3(ㅡ): 긴 레이저(DASH)
  // 키를 누르는 동안 ON, 떼면 OFF (Keypad 라이브러리 PRESSED/RELEASED 활용)
  else if (currentMode == 2) {
    // getKey()는 PRESSED만 반환하므로, 여기서는 고정 시간 펄스 방식 사용
    // 누르면 정해진 시간 레이저 ON → 자동 OFF
    if (!manualLaserOn) {
      if (customKey == '2') {
        digitalWrite(LASER_PIN, HIGH);
        manualLaserOn = true;
        manualLaserTimer = now + MANUAL_DOT_MS;
      } else if (customKey == '3') {
        digitalWrite(LASER_PIN, HIGH);
        manualLaserOn = true;
        manualLaserTimer = now + MANUAL_DASH_MS;
      }
    }
    // 타이머 만료 시 레이저 OFF
    if (manualLaserOn && now >= manualLaserTimer) {
      digitalWrite(LASER_PIN, LOW);
      manualLaserOn = false;
    }
  }
}

// =========================================================================
// 한글 모스부호 광통신 — FreeRTOS 리팩토링
//
// [변경 사항]
//   - hw_timer_t / timerBegin / timerAttachInterrupt / onTxTimer ISR → 완전 제거
//   - morseTxTask (Core 1, Priority 10): vTaskDelay로 타이밍, LASER_PIN 제어
//   - rxProcessTask (Core 0, Priority 6): ISR → rxSymQueue → 모스 디코딩
//   - displayTask (Core 0, Priority 3): OLED sendBuffer (블로킹 허용, 낮은 우선순위)
//   - keypadTask (Core 1, Priority 5): customKeypad 폴링 → keyQueue
//   - volatile flag + 단방향 쓰기로 공유 변수 보호 (Mutex 없음)
//   - Wire1(모터) 활성화
// =========================================================================

#include <Arduino.h>
#include <U8g2lib.h>
#include <Keypad.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// =========================================================================
// I2C 모터 슬레이브 주소
// =========================================================================
#define MOTOR_I2C_ADDR 0x08

// =========================================================================
// 하드웨어 핀
// =========================================================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_raw (U8G2_R0, U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_comb(U8G2_R0, U8X8_PIN_NONE);

const byte ROWS = 4, COLS = 4;
char keys[ROWS][COLS] = {
  {'D','B','#','E'},
  {'3','6','9','A'},
  {'2','5','8','0'},
  {'1','4','7','M'}
};
byte rowPins[ROWS] = {25, 26, 27, 14};
byte colPins[COLS] = {4, 16, 17, 5};
Keypad customKeypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

#define LASER_PIN      23
#define RX_SIGNAL_PIN  34   // DOT
#define RX_LENGTH_PIN  35   // DASH

// =========================================================================
// 타이밍 파라미터 (디버그 모드에서 런타임 변경)
// =========================================================================
volatile int DBG_DOT_ON   = 10;
volatile int DBG_DASH_ON  = 30;
volatile int DBG_SYM_GAP  = 30;
volatile int DBG_JAMO_GAP = 70;
volatile int DBG_WORD_GAP = 120;

// =========================================================================
// FreeRTOS Queue / Task 핸들
// =========================================================================
// 수신 심볼 큐: ISR → rxProcessTask ('.' 또는 '-')
static QueueHandle_t rxSymQueue  = nullptr;
// 키패드 큐: keypadTask → 메인 처리 (loop 대신 keypadTask가 push)
static QueueHandle_t keyQueue    = nullptr;
// 송신 트리거 큐: morseTxTask 깨우기용 (더미 uint8_t)
static QueueHandle_t txTrigQueue = nullptr;

static TaskHandle_t hMorseTxTask    = nullptr;
static TaskHandle_t hDisplayTask    = nullptr;
static TaskHandle_t hRxProcessTask  = nullptr;
static TaskHandle_t hKeypadTask     = nullptr;

// =========================================================================
// 송신 상태 (morseTxTask 단독 쓰기, 다른 Task는 읽기만)
// =========================================================================
#define TX_MORSE_MAX 512
static char  txMorseString[TX_MORSE_MAX] = "";
static int   txMorseLen  = 0;
volatile bool txBusy     = false;     // true = 송신 중
String txCurrentText = "";            // 표시용 모아쓰기 원문

// 종단 마커: '.' '-' '/' 이외 문자 → WORD_GAP 후 종료
#define TX_END_MARKER 'W'

// =========================================================================
// 수신 상태 (rxProcessTask + ISR 공유)
// =========================================================================
#define DEBOUNCE_DOT  8000  // us
#define DEBOUNCE_DASH 30000  // us
volatile unsigned long rxLastDotTime  = 0;
volatile unsigned long rxLastDashTime = 0;
volatile unsigned long rxLastSymbolTime = 0;  // ms
// 인터럽트 → rxProcessTask 폴링용 플래그 (큐 대신)
volatile bool rxGotDot  = false;
volatile bool rxGotDash = false;

String rxMorseBuffer = "";
String rxJamoArray[60];
int    rxJamoCount   = 0;
String rxLogBuffer   = "";

#define RX_LOG_MAX 3
String rxChatLog[RX_LOG_MAX]   = {"", "", ""};
int    rxChatScroll[RX_LOG_MAX]= {0, 0, 0};
unsigned long rxChatScrollTimer = 0;
#define RX_CHAT_SCROLL_SPEED_MS  60
#define RX_CHAT_SCROLL_PAUSE_MS 1500

String rxLiveMorse          = "";
bool   rxLiveMorseCompleted = false;
int    rxLiveMorseScroll    = 0;
unsigned long rxLiveMorseScrollTimer = 0;

// =========================================================================
// 디스플레이 갱신 플래그 (displayTask가 읽고 클리어)
// =========================================================================
volatile bool displayDirty = false;
volatile bool rxDirty      = false;

// =========================================================================
// 모드: 0=타자, 1=모터, 2=모스수동, 3=디버그
// =========================================================================
int currentMode = 0;
// 모터 서브모드: 0=16스텝, 1=10스텝, 2=토글(연속)
int motorSub = 0;
char motorRunningDir = '\0';  // 토글 모드에서 현재 도는 방향
bool motorFound = true;        // 모터 슬레이브 I2C 인식 여부 (false=인식 실패)

// 수동 모스 (모드 2)
bool          manualLaserOn    = false;
unsigned long manualLaserTimer = 0;

// =========================================================================
// 디버그 모드 전역
// =========================================================================
volatile int* dbgParams[5];
const char*   dbgParamNames[5] = {"DOT_ON","DASH_ON","SYM_GAP","JAM_GAP","WRD_GAP"};
int  dbgCursor  = 0;
bool dbgEditing = false;
bool dbgBlink   = false;
unsigned long dbgBlinkTimer = 0;
#define DBG_BLINK_MS 400

String dbgRxMorse   = "";
String dbgRxHangul  = "";
String dbgStatus    = "";
bool   dbgSuccess   = false;
int    dbgMorseScroll  = 0;
int    dbgHangulScroll = 0;
unsigned long dbgScrollTimer = 0;
#define DBG_SCROLL_SPEED_MS  50
#define DBG_SCROLL_PAUSE_MS 1200

const char* DBG_SENTENCES[5] = {
  "\xED\x99\x8D\xEC\xB0\xBD\xEA\xB8\xB0\xEC\x8C\xA4\xEA\xB0\x90\xEC\x82\xAC\xED\x95\xA9\xEB\x8B\x88\xEB\x8B\xA4",
  "\xEA\xB9\x80\xEC\x86\x94\xEC\x8C\xA4\xEB\xB3\xB4\xEA\xB3\xA0\xEC\x8B\xB6\xEC\x96\xB4\xEC\x9A\x94",
  "\xEC\xB5\x9C\xEC\x98\x81\xEC\x95\x84\xEC\x8C\xA4\xEC\xB5\x9C\xEA\xB3\xA0\xEC\x98\x88\xEC\x9A\x94",
  "\xED\x8C\x80\xEB\x9E\xA9\xEC\x93\xB0\xEB\xA6\xAC\xED\x99\x94\xEC\x9D\xB4\xED\x8C\x85\x21",
  "\xEA\xB3\xBC\xED\x95\x99\xEC\x8B\xA4\xEC\x82\xBC\xED\x99\x94\xEC\x9D\xB4\xED\x8C\x85"
};
String dbgPendingExpected = "";

// =========================================================================
// 천지인 데이터 테이블
// =========================================================================
const String CHOSUNG_LIST[]  = {"ㄱ","ㄲ","ㄴ","ㄷ","ㄸ","ㄹ","ㅁ","ㅂ","ㅃ","ㅅ","ㅆ","ㅇ","ㅈ","ㅉ","ㅊ","ㅋ","ㅌ","ㅍ","ㅎ"};
const String JUNGSUNG_LIST[] = {"ㅏ","ㅐ","ㅑ","ㅒ","ㅓ","ㅔ","ㅕ","ㅖ","ㅗ","ㅘ","ㅙ","ㅚ","ㅛ","ㅜ","ㅝ","ㅞ","ㅟ","ㅠ","ㅡ","ㅢ","ㅣ"};
const String JONGSUNG_LIST[] = {"","ㄱ","ㄲ","ㄳ","ㄴ","ㄵ","ㄶ","ㄷ","ㄹ","ㄺ","ㄻ","ㄼ","ㄽ","ㄾ","ㄿ","ㅀ","ㅁ","ㅂ","ㅄ","ㅅ","ㅆ","ㅇ","ㅈ","ㅊ","ㅋ","ㅌ","ㅍ","ㅎ"};

const String MULTITAP_GROUPS[7][3] = {
  {"ㄱ","ㅋ","ㄲ"}, {"ㄴ","ㄹ",""}, {"ㄷ","ㅌ","ㄸ"},
  {"ㅂ","ㅍ","ㅃ"}, {"ㅅ","ㅎ","ㅆ"}, {"ㅈ","ㅊ","ㅉ"}, {"ㅇ","ㅁ",""}
};
const int MULTITAP_SIZES[] = {3, 2, 3, 3, 3, 3, 2};
const String PUNCT_GROUP = ".,?!";

struct DoubleConsonant { String b1, b2, result; };
DoubleConsonant DBL_CONS[] = {
  {"ㄱ","ㅅ","ㄳ"}, {"ㄴ","ㅈ","ㄵ"}, {"ㄴ","ㅎ","ㄶ"}, {"ㄹ","ㄱ","ㄺ"}, {"ㄹ","ㅁ","ㄻ"},
  {"ㄹ","ㅂ","ㄼ"}, {"ㄹ","ㅅ","ㄽ"}, {"ㄹ","ㅌ","ㄾ"}, {"ㄹ","ㅍ","ㄿ"}, {"ㄹ","ㅎ","ㅀ"}, {"ㅂ","ㅅ","ㅄ"}
};

// =========================================================================
// 천지인 오토마타 상태
// =========================================================================
String typedText      = "";
String currentSyllable= "";
int    choIdx  = -1, jungIdx = -1, jongIdx = 0;
String currentVowelStr = "";
int    punctIdx = -1;
unsigned long lastPressTime = 0;
char   lastKey = '\0';
int    tapCount = 0;
bool   showDecomposedView = false;
int    typedTextScroll = 0;
unsigned long typedTextScrollTimer = 0;
#define SCROLL_SPEED_MS   60
#define SCROLL_PAUSE_MS 1500

// =========================================================================
// 전방 선언
// =========================================================================
void updateDisplays();
void dbgDrawLeft();
void dbgDrawRight();
String assembleHangul();
String popLastUtf8Char(String str);
String getNextVowel(String cur, char key);

// =========================================================================
// 유틸: 송신 표시 텍스트 잘라내기
// =========================================================================
String getTxDisplayText(String src, int maxWidth) {
  int totalW = 0, startByte = src.length();
  int i = src.length();
  while (i > 0) {
    int charStart = i - 1;
    while (charStart > 0 && (src.charAt(charStart) & 0xC0) == 0x80) charStart--;
    unsigned char c1 = src.charAt(charStart);
    int charW = ((c1 & 0xF0) == 0xE0) ? 13 : ((c1 & 0xE0) == 0xC0) ? 9 : 6;
    if (totalW + charW > maxWidth) break;
    totalW += charW; startByte = charStart; i = charStart;
  }
  return src.substring(startByte);
}

// =========================================================================
// 유틸: 점(·) 포함 UTF-8 안전 drawStr
// =========================================================================
void drawSafeUTF8(U8G2 &u8g2, int x, int y, String str) {
  int curX = x;
  for (int i = 0; i < (int)str.length(); ) {
    unsigned char c1 = str[i];
    if (c1 == '.') {
      u8g2.drawDisc(curX + 3, y - 4, 2); curX += 8; i++; continue;
    }
    if ((c1 & 0x80) == 0) {
      char buf[2] = {(char)c1, 0};
      u8g2.drawStr(curX, y, buf); curX += u8g2.getStrWidth(buf); i++;
    } else if ((c1 & 0xE0) == 0xC0) {
      if (i+1 < (int)str.length() && c1 == 0xC2 && (unsigned char)str[i+1] == 0xB7) {
        u8g2.drawDisc(curX+3, y-4, 2); curX += 8; i += 2; continue;
      }
      String sub = str.substring(i, i+2);
      u8g2.drawUTF8(curX, y, sub.c_str()); curX += u8g2.getUTF8Width(sub.c_str()); i += 2;
    } else if ((c1 & 0xF0) == 0xE0) {
      if (i+2 < (int)str.length()) {
        unsigned char c2 = str[i+1], c3 = str[i+2];
        if (c1==0xE3 && c2==0x86 && c3==0x8D) { u8g2.drawDisc(curX+3,y-4,2); curX+=8; i+=3; continue; }
        if (c1==0xE3 && c2==0x86 && c3==0x8E) { u8g2.drawDisc(curX+3,y-4,2); u8g2.drawDisc(curX+9,y-4,2); curX+=14; i+=3; continue; }
      }
      String sub = str.substring(i, i+3);
      u8g2.drawUTF8(curX, y, sub.c_str()); curX += u8g2.getUTF8Width(sub.c_str()); i += 3;
    } else { i++; }
  }
}

// =========================================================================
// 한글 자모 ↔ 모스 변환
// =========================================================================
int getChoIdxBase(String s)  { for(int i=0;i<19;i++) if(CHOSUNG_LIST[i]==s)  return i; return -1; }
int getJungIdxBase(String s) { for(int i=0;i<21;i++) if(JUNGSUNG_LIST[i]==s) return i; return -1; }
int getJongIdxBase(String s) { for(int i=0;i<28;i++) if(JONGSUNG_LIST[i]==s) return i; return -1; }
bool isVowelBase(String s)     { return getJungIdxBase(s) != -1; }
bool isConsonantBase(String s) { return getChoIdxBase(s) != -1 || getJongIdxBase(s) > 0; }

String getDoubleJongBase(String j1, String j2) {
  for(int i=0;i<11;i++) if(DBL_CONS[i].b1==j1 && DBL_CONS[i].b2==j2) return DBL_CONS[i].result;
  return "";
}
String getDoubleJungBase(String j1, String j2) {
  if(j1=="ㅗ"&&j2=="ㅏ") return "ㅘ"; if(j1=="ㅗ"&&j2=="ㅐ") return "ㅙ";
  if(j1=="ㅗ"&&j2=="ㅣ") return "ㅚ"; if(j1=="ㅜ"&&j2=="ㅓ") return "ㅝ";
  if(j1=="ㅜ"&&j2=="ㅔ") return "ㅞ"; if(j1=="ㅜ"&&j2=="ㅣ") return "ㅟ";
  if(j1=="ㅡ"&&j2=="ㅣ") return "ㅢ"; if(j1=="ㅑ"&&j2=="ㅣ") return "ㅒ";
  if(j1=="ㅕ"&&j2=="ㅣ") return "ㅖ"; if(j1=="ㅏ"&&j2=="ㅣ") return "ㅐ";
  if(j1=="ㅓ"&&j2=="ㅣ") return "ㅔ"; return "";
}
String composeHangulBase(String cho, String jung, String jong) {
  int ci=getChoIdxBase(cho), vi=getJungIdxBase(jung), ji=getJongIdxBase(jong);
  if(ci==-1||vi==-1) return cho; if(ji==-1) ji=0;
  uint32_t u=0xAC00+(ci*21*28)+(vi*28)+ji; String r="";
  r+=(char)(0xE0|((u>>12)&0x0F)); r+=(char)(0x80|((u>>6)&0x3F)); r+=(char)(0x80|(u&0x3F));
  return r;
}
void splitDoubleJongBase(String j, String &l, String &r) {
  for(int i=0;i<11;i++) if(DBL_CONS[i].result==j){ l=DBL_CONS[i].b1; r=DBL_CONS[i].b2; return; }
  l=j; r="";
}
String moasseugiBase(String arr[], int n) {
  String result="", cho="", jung="", jong="";
  for(int i=0;i<n;i++) {
    String cur=arr[i];
    if(cur==" "||cur=="W") { if(cho!="") result+=composeHangulBase(cho,jung,jong); result+=" "; cho=""; jung=""; jong=""; continue; }
    if(cur==","||cur=="."||cur=="?"||cur=="!") { if(cho!="") result+=composeHangulBase(cho,jung,jong); result+=cur; cho=""; jung=""; jong=""; continue; }
    if(cho=="") { if(isConsonantBase(cur)) cho=cur; else if(isVowelBase(cur)) { cho="ㅇ"; jung=cur; } }
    else if(jung=="") { if(isVowelBase(cur)) jung=cur; else { result+=composeHangulBase(cho,"",""); cho=cur; } }
    else if(jong=="") {
      if(isConsonantBase(cur)) { if(i+1<n&&isVowelBase(arr[i+1])) { result+=composeHangulBase(cho,jung,""); cho=cur; jung=""; } else jong=cur; }
      else if(isVowelBase(cur)) { String dj=getDoubleJungBase(jung,cur); if(dj!="") jung=dj; else { result+=composeHangulBase(cho,jung,""); cho="ㅇ"; jung=cur; } }
    } else {
      if(isConsonantBase(cur)) { String dbl=getDoubleJongBase(jong,cur); if(dbl!=""&&(i+1==n||!isVowelBase(arr[i+1]))) jong=dbl; else { result+=composeHangulBase(cho,jung,jong); cho=cur; jung=""; jong=""; } }
      else if(isVowelBase(cur)) { String lj="",rj=""; splitDoubleJongBase(jong,lj,rj); if(rj!="") { jong=lj; result+=composeHangulBase(cho,jung,jong); cho=rj; } else { String nc=jong; jong=""; result+=composeHangulBase(cho,jung,jong); cho=nc; } jung=cur; }
    }
  }
  if(cho!="") result+=composeHangulBase(cho,jung,jong);
  return result;
}

bool decomposeHangul(uint32_t uni, int &c, int &ju, int &jo) {
  if(uni>=0xAC00&&uni<=0xD7A3) { uint32_t idx=uni-0xAC00; c=idx/588; ju=(idx%588)/28; jo=idx%28; return true; }
  return false;
}

int decomposeTextToJamoBase(String text, String out[]) {
  int count=0, len=text.length();
  for(int i=0;i<len;) {
    unsigned char c=(unsigned char)text[i];
    if(c==' ') { out[count++]=" "; i++; }
    else if(c<0x80) { String ch=String((char)c); if(ch==","||ch=="."||ch=="?"||ch=="!") out[count++]=ch; i++; }
    else if((c&0xE0)==0xE0) {
      if(i+2>=len) break;
      uint32_t u=((c&0x0F)<<12)|(((unsigned char)text[i+1]&0x3F)<<6)|(((unsigned char)text[i+2]&0x3F)); i+=3;
      if(u>=0xAC00&&u<=0xD7A3) {
        int cIdx,juIdx,joIdx; decomposeHangul(u,cIdx,juIdx,joIdx);
        out[count++]=CHOSUNG_LIST[cIdx];
        String v=JUNGSUNG_LIST[juIdx];
        if(v=="ㅘ"){out[count++]="ㅗ";out[count++]="ㅏ";} else if(v=="ㅙ"){out[count++]="ㅗ";out[count++]="ㅐ";}
        else if(v=="ㅚ"){out[count++]="ㅗ";out[count++]="ㅣ";} else if(v=="ㅝ"){out[count++]="ㅜ";out[count++]="ㅓ";}
        else if(v=="ㅞ"){out[count++]="ㅜ";out[count++]="ㅔ";} else if(v=="ㅟ"){out[count++]="ㅜ";out[count++]="ㅣ";}
        else if(v=="ㅢ"){out[count++]="ㅡ";out[count++]="ㅣ";} else if(v=="ㅒ"){out[count++]="ㅑ";out[count++]="ㅣ";}
        else if(v=="ㅖ"){out[count++]="ㅕ";out[count++]="ㅣ";} else out[count++]=v;
        if(joIdx>0) { String j=JONGSUNG_LIST[joIdx]; String lj,rj; splitDoubleJongBase(j,lj,rj); out[count++]=lj; if(rj!="") out[count++]=rj; }
      }
    } else { i++; }
  }
  return count;
}

String jamoToMorse(String jamo) {
  if(jamo=="ㄱ") return ".-.."; if(jamo=="ㄴ") return "..-." ; if(jamo=="ㄷ") return "-...";
  if(jamo=="ㄹ") return "...-"; if(jamo=="ㅁ") return "--"   ; if(jamo=="ㅂ") return ".--";
  if(jamo=="ㅅ") return "--."  ; if(jamo=="ㅇ") return "-.-"  ; if(jamo=="ㅈ") return ".--.";
  if(jamo=="ㅊ") return "-.-." ; if(jamo=="ㅋ") return "-..-" ; if(jamo=="ㅌ") return "--..";
  if(jamo=="ㅍ") return "---"  ; if(jamo=="ㅎ") return ".-...";
  if(jamo=="ㄲ") return ".-...-.."; if(jamo=="ㄸ") return "-...-..."; if(jamo=="ㅃ") return ".--.--";
  if(jamo=="ㅆ") return "--.--.";   if(jamo=="ㅉ") return ".--..--.";
  if(jamo=="ㅏ") return "."   ; if(jamo=="ㅑ") return ".."   ; if(jamo=="ㅓ") return "-";
  if(jamo=="ㅕ") return "..."  ; if(jamo=="ㅗ") return ".-"   ; if(jamo=="ㅛ") return "-.";
  if(jamo=="ㅜ") return "...."; if(jamo=="ㅠ") return ".-."   ; if(jamo=="ㅡ") return "-..";
  if(jamo=="ㅣ") return "..-" ; if(jamo=="ㅐ") return "--.-"  ; if(jamo=="ㅔ") return "-.--";
  if(jamo==" ")  return "W";
  if(jamo==",") return "--..--"; if(jamo==".") return ".-.-.-";
  if(jamo=="?") return "..--.."; if(jamo=="!") return "-.-.--";
  return "";
}
String morseToJamo(String morse) {
  if(morse==".-..") return "ㄱ"; if(morse=="..-.") return "ㄴ"; if(morse=="-...") return "ㄷ";
  if(morse=="...-") return "ㄹ"; if(morse=="--")   return "ㅁ"; if(morse==".--")  return "ㅂ";
  if(morse=="--.")  return "ㅅ"; if(morse=="-.-")  return "ㅇ"; if(morse==".--.") return "ㅈ";
  if(morse=="-.-.") return "ㅊ"; if(morse=="-..-") return "ㅋ"; if(morse=="--..") return "ㅌ";
  if(morse=="---")  return "ㅍ"; if(morse==".-...") return "ㅎ";
  if(morse==".-...-..") return "ㄲ"; if(morse=="-...-...") return "ㄸ"; if(morse==".--.--") return "ㅃ";
  if(morse=="--.--.") return "ㅆ"; if(morse==".--..--.") return "ㅉ";
  if(morse==".")    return "ㅏ"; if(morse=="..")   return "ㅑ"; if(morse=="-")    return "ㅓ";
  if(morse=="...")  return "ㅕ"; if(morse==".-")   return "ㅗ"; if(morse=="-.")   return "ㅛ";
  if(morse=="....") return "ㅜ"; if(morse==".-.")  return "ㅠ"; if(morse=="-..")  return "ㅡ";
  if(morse=="..-")  return "ㅣ"; if(morse=="--.-") return "ㅐ"; if(morse=="-.--") return "ㅔ";
  if(morse=="--..--") return ","; if(morse==".-.-.-") return ".";
  if(morse=="..--..") return "?"; if(morse=="-.-.--") return "!";
  return "?";
}

// =========================================================================
// ISR: RISING 엣지에서 플래그만 세팅 (rxProcessTask가 폴링)
// =========================================================================
void IRAM_ATTR onDotSignal() {
  unsigned long now=(unsigned long)esp_timer_get_time();
  if(now-rxLastDotTime < DEBOUNCE_DOT) return;
  rxLastDotTime = now;
  rxLastSymbolTime = now/1000;
  rxGotDot = true;
}
void IRAM_ATTR onDashSignal() {
  unsigned long now=(unsigned long)esp_timer_get_time();
  if(now-rxLastDashTime < DEBOUNCE_DASH) return;
  rxLastDashTime = now;
  rxLastSymbolTime = now/1000;
  rxGotDash = true;
}

// =========================================================================
// 수신 처리 헬퍼 (rxProcessTask에서만 호출)
// =========================================================================
void addMorseSymbol(char symbol) {
  if(rxLiveMorseCompleted) { rxLiveMorse=""; rxLiveMorseCompleted=false; rxLiveMorseScroll=0; }
  rxMorseBuffer += symbol;
  if(rxLiveMorse.length()>0) {
    char last = rxLiveMorse.charAt(rxLiveMorse.length()-1);
    if(last=='.'||last=='-') rxLiveMorse += ' ';
  }
  rxLiveMorse += symbol;
  if(currentMode==3) {
    if(dbgRxMorse.length()>0) { char last=dbgRxMorse.charAt(dbgRxMorse.length()-1); if(last=='.'||last=='-') dbgRxMorse+=' '; }
    dbgRxMorse += symbol;
  }
  rxDirty = true;
}

void flushMorseBuffer() {
  if(rxMorseBuffer!="") {
    String jamo = morseToJamo(rxMorseBuffer);
    if(jamo!=""&&rxJamoCount<60) rxJamoArray[rxJamoCount++]=jamo;
    rxMorseBuffer = "";
    rxLiveMorse += '/';
    if(currentMode==3) dbgRxMorse += '/';
    rxDirty = true;
  }
}

void processReceivedMessage() {
  String result = moasseugiBase(rxJamoArray, rxJamoCount);
  if(result!="") {
    rxLogBuffer = result;
    Serial.println("[수신] " + rxLogBuffer);
    for(int i=0;i<RX_LOG_MAX-1;i++) { rxChatLog[i]=rxChatLog[i+1]; rxChatScroll[i]=rxChatScroll[i+1]; }
    rxChatLog[RX_LOG_MAX-1]=result; rxChatScroll[RX_LOG_MAX-1]=0;
  }
  rxMorseBuffer=""; rxLiveMorse=""; rxLiveMorseCompleted=true; rxLiveMorseScroll=0;
  rxJamoCount=0; rxLastSymbolTime=0; rxLastDotTime=0; rxLastDashTime=0;
  if(currentMode==3) {
    dbgRxHangul = result!="" ? result : rxLogBuffer;
    dbgHangulScroll=0; dbgMorseScroll=0;
    if(dbgPendingExpected!="") {
      if(dbgRxHangul==dbgPendingExpected) { dbgStatus=">> SUCCESS <<"; dbgSuccess=true; }
      else {
        dbgSuccess=false;
        int expLen=0,rcvLen=0;
        for(int i=0;i<(int)dbgPendingExpected.length();) { unsigned char c=(unsigned char)dbgPendingExpected[i]; if((c&0x80)==0)i+=1; else if((c&0xE0)==0xC0)i+=2; else if((c&0xF0)==0xE0)i+=3; else i+=1; expLen++; }
        for(int i=0;i<(int)dbgRxHangul.length();)    { unsigned char c=(unsigned char)dbgRxHangul[i];   if((c&0x80)==0)i+=1; else if((c&0xE0)==0xC0)i+=2; else if((c&0xF0)==0xE0)i+=3; else i+=1; rcvLen++; }
        String errCode="ER"; bool first=true;
        auto appendErr=[&](int param,char dir,int amount){ if(!first) errCode+="N"; errCode+=String(param)+dir+String(amount); first=false; };
        if(rcvLen==0)            { appendErr(5,'U',100); appendErr(1,'U',5); }
        else if(rcvLen>expLen*2) { appendErr(3,'U',10); }
        else if(rcvLen<expLen/2) { appendErr(4,'U',20); appendErr(3,'D',5); }
        else if(rcvLen>expLen)   { appendErr(4,'U',10); }
        else if(rcvLen<expLen)   { appendErr(5,'U',50); appendErr(1,'U',3); }
        else                     { appendErr(2,'U',10); appendErr(1,'D',3); }
        dbgStatus=errCode;
      }
      dbgPendingExpected="";
    } else { dbgStatus="RX OK"; }
  }
  rxDirty=true;
}

// =========================================================================
// 송신 시작 (keypadTask 또는 loop에서 호출)
// =========================================================================
void startMorseTx(String arr[], int count) {
  if(txBusy) return;  // 이미 송신 중이면 무시

  txMorseLen=0;
  memset(txMorseString, 0, TX_MORSE_MAX);
  for(int i=0;i<count;i++) {
    String morse=jamoToMorse(arr[i]);
    if(morse!="") {
      for(int j=0;j<(int)morse.length()&&txMorseLen<TX_MORSE_MAX-2;j++)
        txMorseString[txMorseLen++]=morse[j];
      if(txMorseLen<TX_MORSE_MAX-2) txMorseString[txMorseLen++]='/';
    }
  }
  if(txMorseLen<TX_MORSE_MAX-1) txMorseString[txMorseLen++]=TX_END_MARKER;
  txMorseString[txMorseLen]='\0';
  txCurrentText = moasseugiBase(arr, count);

  // OLED 즉시 업데이트 (송신 중엔 displayTask가 TX화면 유지)
  displayDirty = true;

  // morseTxTask 깨우기
  uint8_t dummy = 1;
  xQueueSend(txTrigQueue, &dummy, 0);
}

// =========================================================================
// ─── FreeRTOS Tasks ───────────────────────────────────────────────────────
// =========================================================================

// ── morseTxTask: Core 1, Priority 10 ─────────────────────────────────────
// vTaskDelay로 타이밍, LASER_PIN 직접 제어
// txTrigQueue에 신호가 들어오면 송신 시작
void morseTxTask(void* param) {
  uint8_t dummy;
  for(;;) {
    // 트리거 대기 (무기한)
    if(xQueueReceive(txTrigQueue, &dummy, portMAX_DELAY) == pdTRUE) {
      txBusy = true;
      int pos = 0;
      while(pos < txMorseLen) {
        char c = txMorseString[pos++];
        if(c == '.') {
          digitalWrite(LASER_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(DBG_DOT_ON));
          digitalWrite(LASER_PIN, LOW);
          vTaskDelay(pdMS_TO_TICKS(DBG_SYM_GAP));
        } else if(c == '-') {
          digitalWrite(LASER_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(DBG_DASH_ON));
          digitalWrite(LASER_PIN, LOW);
          vTaskDelay(pdMS_TO_TICKS(DBG_SYM_GAP));
        } else if(c == '/') {
          // JAMO_GAP은 SYM_GAP을 이미 썼으므로 차액만 추가
          int extra = DBG_JAMO_GAP - DBG_SYM_GAP;
          if(extra > 0) vTaskDelay(pdMS_TO_TICKS(extra));
        } else {
          // TX_END_MARKER 'W' 또는 기타 → WORD_GAP
          int extra = DBG_WORD_GAP - DBG_SYM_GAP;
          if(extra > 0) vTaskDelay(pdMS_TO_TICKS(extra));
        }
      }
      digitalWrite(LASER_PIN, LOW);  // 안전 확인
      txBusy = false;
      displayDirty = true;  // 송신 완료 → 화면 갱신
    }
  }
}

// ── rxProcessTask: Core 0, Priority 6 ────────────────────────────────────
// rxSymQueue에서 심볼을 꺼내 모스 → 자모 → 한글 변환
// WORD_GAP 타임아웃으로 문장 완성 판정
void rxProcessTask(void* param) {
  unsigned long lastSymTime = 0;
  bool hasPending  = false;
  bool jamoFlushed = false;   // JAMO_GAP 경계 flush 1회 제한

  for(;;) {
    // 모터 모드(1)에서는 수신 처리 안 함 (레이저가 모터 신호와 무관하게 켜질 수 있어 차단)
    if(currentMode==1) {
      rxGotDot = false; rxGotDash = false;
      hasPending = false; lastSymTime = 0; jamoFlushed = false;
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // 인터럽트가 세팅한 플래그를 폴링 (큐 대신)
    bool gotDot  = rxGotDot;
    bool gotDash = rxGotDash;
    rxGotDot = false; rxGotDash = false;

    unsigned long now = millis();

    if(gotDot || gotDash) {
      char sym = gotDot ? '.' : '-';
      // 이전 심볼로부터 JAMO_GAP 지났으면 자모 경계
      if(hasPending && (now - lastSymTime) >= (unsigned long)DBG_JAMO_GAP) {
        flushMorseBuffer();
      }
      lastSymTime = now;
      hasPending  = true;
      jamoFlushed = false;
      addMorseSymbol(sym);

    } else if(hasPending) {
      unsigned long elapsed = now - lastSymTime;

      if(elapsed >= (unsigned long)DBG_WORD_GAP) {
        // WORD_GAP 초과 → 문장 확정
        if(rxMorseBuffer != "") flushMorseBuffer();
        if(rxJamoCount   >  0) processReceivedMessage();
        else                   rxLastSymbolTime = 0;
        lastSymTime = 0;
        hasPending  = false;
        jamoFlushed = false;
      } else if(elapsed >= (unsigned long)DBG_JAMO_GAP && !jamoFlushed) {
        // JAMO_GAP 초과 → 자모 경계만 (1회)
        flushMorseBuffer();
        jamoFlushed = true;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // 5ms 주기 폴링
  }
}

// ── keypadTask: Core 1, Priority 5 ───────────────────────────────────────
// 키패드 폴링 → keyQueue에 push
void keypadTask(void* param) {
  for(;;) {
    char k = customKeypad.getKey();
    if(k) xQueueSend(keyQueue, &k, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ── displayTask: Core 0, Priority 3 ──────────────────────────────────────
// dirty flag가 세팅되면 OLED 갱신
// sendBuffer() 블로킹(~25ms×2)은 낮은 우선순위로 다른 Task에 영향 없음
void displayTask(void* param) {
  for(;;) {
    bool doUpdate = displayDirty || rxDirty;
    if(doUpdate) {
      displayDirty = false;
      rxDirty      = false;
      updateDisplays();
    }
    // 스크롤 업데이트 (별도 dirty 세팅)
    unsigned long now = millis();

    // rxChatLog 스크롤
    if(now - rxChatScrollTimer >= RX_CHAT_SCROLL_SPEED_MS) {
      rxChatScrollTimer = now;
      for(int i=0;i<RX_LOG_MAX;i++) {
        if(rxChatLog[i]=="") continue;
        int w=0;
        for(int j=0;j<(int)rxChatLog[i].length();) { unsigned char c=(unsigned char)rxChatLog[i][j]; if((c&0xF0)==0xE0){w+=13;j+=3;} else if((c&0xE0)==0xC0){w+=9;j+=2;} else{w+=6;j++;} }
        int overflow=max(0,w-128);
        if(overflow>0) { rxChatScroll[i]++; int pause=RX_CHAT_SCROLL_PAUSE_MS/RX_CHAT_SCROLL_SPEED_MS; if(rxChatScroll[i]>overflow+pause) rxChatScroll[i]=0; rxDirty=true; }
      }
    }
    // rxLiveMorse 스크롤
    if(now - rxLiveMorseScrollTimer >= SCROLL_SPEED_MS) {
      rxLiveMorseScrollTimer = now;
      int w=0;
      for(int i=0;i<(int)rxLiveMorse.length();i++) { char c=rxLiveMorse.charAt(i); if(c=='.') w+=8; else if(c=='-') w+=11; else if(c==' ') w+=6; else if(c=='/') w+=7; }
      int overflow=max(0,w-126);
      if(overflow>0) { rxLiveMorseScroll++; int pause=SCROLL_PAUSE_MS/SCROLL_SPEED_MS; if(rxLiveMorseScroll>overflow+pause) rxLiveMorseScroll=0; rxDirty=true; }
    }
    // typedText 스크롤
    if(now - typedTextScrollTimer >= SCROLL_SPEED_MS) {
      typedTextScrollTimer = now;
      int w=0;
      for(int i=0;i<(int)typedText.length();) { unsigned char c=(unsigned char)typedText[i]; if((c&0xF0)==0xE0){w+=13;i+=3;} else if((c&0xE0)==0xC0){w+=9;i+=2;} else{w+=6;i++;} }
      int overflow=max(0,w-128);
      if(overflow>0) { typedTextScroll++; int pause=SCROLL_PAUSE_MS/SCROLL_SPEED_MS; if(typedTextScroll>overflow+pause) typedTextScroll=0; displayDirty=true; }
    }
    // 디버그 스크롤
    if(currentMode==3) {
      if(now-dbgScrollTimer>=DBG_SCROLL_SPEED_MS) {
        dbgScrollTimer=now;
        int morseW=max(0,(int)dbgRxMorse.length()*6-128);
        if(morseW>0){dbgMorseScroll++;if(dbgMorseScroll>morseW+DBG_SCROLL_PAUSE_MS/DBG_SCROLL_SPEED_MS)dbgMorseScroll=0;}
        int hW=0; for(int i=0;i<(int)dbgRxHangul.length();){unsigned char c=(unsigned char)dbgRxHangul[i];if((c&0xF0)==0xE0){hW+=13;i+=3;}else if((c&0xE0)==0xC0){hW+=9;i+=2;}else{hW+=6;i++;}} int ho=max(0,hW-128);
        if(ho>0){dbgHangulScroll++;if(dbgHangulScroll>ho+DBG_SCROLL_PAUSE_MS/DBG_SCROLL_SPEED_MS)dbgHangulScroll=0;}
        rxDirty=true;
      }
      if(dbgEditing && now-dbgBlinkTimer>=DBG_BLINK_MS){ dbgBlink=!dbgBlink; dbgBlinkTimer=now; displayDirty=true; }
    }

    vTaskDelay(pdMS_TO_TICKS(5));  // 5ms 주기로 dirty 확인
  }
}

// =========================================================================
// 천지인 오토마타 헬퍼
// =========================================================================
String popLastUtf8Char(String str) {
  if(str.length()==0) return "";
  int i=str.length()-1;
  while(i>0&&(str.charAt(i)&0xC0)==0x80) i--;
  return str.substring(0,i);
}
String getNextVowel(String cur, char key) {
  if(cur=="") { if(key=='1') return "ㅣ"; if(key=='2') return "·"; if(key=='3') return "ㅡ"; }
  if(key=='1') { if(cur=="·") return "ㅓ"; if(cur=="··") return "ㅕ"; if(cur=="ㅏ") return "ㅐ"; if(cur=="ㅑ") return "ㅒ"; if(cur=="ㅓ") return "ㅔ"; if(cur=="ㅕ") return "ㅖ"; if(cur=="ㅗ") return "ㅚ"; if(cur=="ㅜ") return "ㅟ"; if(cur=="ㅡ") return "ㅢ"; if(cur=="ㅘ") return "ㅙ"; if(cur=="ㅝ") return "ㅞ"; if(cur=="ㅠ") return "ㅝ"; }
  else if(key=='2') { if(cur=="ㅣ") return "ㅏ"; if(cur=="ㅏ") return "ㅑ"; if(cur=="ㅡ") return "ㅜ"; if(cur=="ㅜ") return "ㅠ"; if(cur=="·") return "··"; if(cur=="ㅗ") return "ㅛ"; if(cur=="ㅚ") return "ㅘ"; }
  else if(key=='3') { if(cur=="·") return "ㅗ"; if(cur=="··") return "ㅛ"; }
  return "";
}
String assembleHangul() {
  if(choIdx==-1) { if(currentVowelStr!="") return currentVowelStr; if(jungIdx!=-1) return JUNGSUNG_LIST[jungIdx]; return ""; }
  if(jungIdx==-1) return CHOSUNG_LIST[choIdx]+currentVowelStr;
  uint16_t uniCode=0xAC00+(choIdx*588)+(jungIdx*28)+jongIdx;
  String result="";
  result+=(char)(0xE0|((uniCode>>12)&0x0F));
  result+=(char)(0x80|((uniCode>>6)&0x3F));
  result+=(char)(0x80|(uniCode&0x3F));
  return result;
}
void flushBuffer() {
  bool hasAraeoaOnly=(currentVowelStr=="·"||currentVowelStr=="··")&&jungIdx==-1;
  if(!hasAraeoaOnly&&(choIdx!=-1||currentVowelStr!="")) {
    String assembled=assembleHangul();
    if(assembled.indexOf("\xC2\xB7")==-1&&assembled.indexOf("\xE3\x86\x8D")==-1) typedText+=assembled;
  } else if(!hasAraeoaOnly&&currentSyllable!="") { typedText+=currentSyllable; }
  int savedChoIdx=(hasAraeoaOnly&&choIdx!=-1)?choIdx:-1;
  jungIdx=-1; jongIdx=0; currentVowelStr=""; currentSyllable=""; tapCount=0; lastKey='\0'; punctIdx=-1;
  choIdx=savedChoIdx;
}

// =========================================================================
// 키 처리 (keypadTask에서 꺼낸 키를 loop에서 처리)
// =========================================================================
void processKey(char key) {
  unsigned long now = millis();
  if(key=='E'||key=='D') { showDecomposedView=!showDecomposedView; return; }
  if(key=='A'||key=='C') { flushBuffer(); return; }
  if(key=='#') {
    if(currentSyllable!=""||choIdx!=-1||currentVowelStr!="") flushBuffer();
    else typedText+=" ";
    lastPressTime=now; return;
  }
  if(key=='B') {
    if(choIdx!=-1||currentVowelStr!=""||jongIdx>0||punctIdx!=-1) {
      if(lastKey=='*') { currentSyllable=""; punctIdx=-1; lastKey='\0'; }
      else {
        if(jongIdx>0) { String jStr=JONGSUNG_LIST[jongIdx]; String keepJong=""; bool isDouble=false; for(int i=0;i<11;i++) { if(DBL_CONS[i].result==jStr) { keepJong=DBL_CONS[i].b1; isDouble=true; break; } } if(isDouble) { for(int j=0;j<28;j++) { if(JONGSUNG_LIST[j]==keepJong){ jongIdx=j; break; } } } else jongIdx=0; }
        else if(currentVowelStr!="") {
          if(currentVowelStr=="ㅙ") currentVowelStr="ㅘ"; else if(currentVowelStr=="ㅘ") currentVowelStr="ㅗ";
          else if(currentVowelStr=="ㅝ") currentVowelStr="ㅜ"; else if(currentVowelStr=="ㅞ") currentVowelStr="ㅝ";
          else if(currentVowelStr=="ㅚ") currentVowelStr="ㅗ"; else if(currentVowelStr=="ㅟ") currentVowelStr="ㅜ";
          else if(currentVowelStr=="ㅢ") currentVowelStr="ㅡ"; else if(currentVowelStr=="ㅐ") currentVowelStr="ㅏ";
          else if(currentVowelStr=="ㅔ") currentVowelStr="ㅓ"; else if(currentVowelStr=="ㅒ") currentVowelStr="ㅑ";
          else if(currentVowelStr=="ㅖ") currentVowelStr="ㅕ"; else if(currentVowelStr=="ㅑ") currentVowelStr="ㅏ";
          else if(currentVowelStr=="ㅕ") currentVowelStr="ㅓ"; else if(currentVowelStr=="ㅛ") currentVowelStr="ㅗ";
          else if(currentVowelStr=="ㅠ") currentVowelStr="ㅜ"; else if(currentVowelStr=="ㅏ") currentVowelStr="ㅣ";
          else if(currentVowelStr=="ㅓ") currentVowelStr="·"; else if(currentVowelStr=="ㅗ") currentVowelStr="·";
          else if(currentVowelStr=="ㅜ") currentVowelStr="ㅡ"; else if(currentVowelStr=="··") currentVowelStr="·";
          else currentVowelStr="";
          jungIdx=-1; if(currentVowelStr!="") { for(int i=0;i<21;i++) { if(JUNGSUNG_LIST[i]==currentVowelStr){ jungIdx=i; break; } } }
        } else if(choIdx!=-1) choIdx=-1;
        currentSyllable=(choIdx==-1&&currentVowelStr=="")?"":assembleHangul();
      }
    } else typedText=popLastUtf8Char(typedText);
    lastPressTime=now; return;
  }
  if(key=='*') {
    if(lastKey!='*'&&currentSyllable!="") flushBuffer();
    if(lastKey=='*'&&(now-lastPressTime<1000)) punctIdx=(punctIdx+1)%PUNCT_GROUP.length();
    else { flushBuffer(); punctIdx=0; }
    currentSyllable=String(PUNCT_GROUP.charAt(punctIdx)); lastKey=key; lastPressTime=now; return;
  }
  if(lastKey=='*') flushBuffer();
  if(key=='1'||key=='2'||key=='3') {
    if(choIdx!=-1&&jungIdx!=-1&&jongIdx>0) {
      String jStr=JONGSUNG_LIST[jongIdx]; String keepJong="",migrateCho=""; bool isDouble=false;
      for(int i=0;i<11;i++) { if(DBL_CONS[i].result==jStr){ keepJong=DBL_CONS[i].b1; migrateCho=DBL_CONS[i].b2; isDouble=true; break; } }
      if(!isDouble){ migrateCho=jStr; keepJong=""; }
      if(keepJong=="") jongIdx=0; else { for(int j=0;j<28;j++) { if(JONGSUNG_LIST[j]==keepJong){ jongIdx=j; break; } } }
      typedText+=assembleHangul(); choIdx=-1; for(int i=0;i<19;i++) { if(CHOSUNG_LIST[i]==migrateCho){ choIdx=i; break; } }
      jungIdx=-1; jongIdx=0; currentVowelStr="";
    }
    String nextV=getNextVowel(currentVowelStr,key);
    if(nextV!="") { currentVowelStr=nextV; jungIdx=-1; for(int i=0;i<21;i++) { if(JUNGSUNG_LIST[i]==currentVowelStr){ jungIdx=i; break; } } }
    else { flushBuffer(); currentVowelStr=getNextVowel("",key); for(int i=0;i<21;i++) { if(JUNGSUNG_LIST[i]==currentVowelStr){ jungIdx=i; break; } } }
    currentSyllable=assembleHangul(); lastKey=key; lastPressTime=now; return;
  }
  int groupIdx=-1;
  if(key=='4') groupIdx=0; else if(key=='5') groupIdx=1; else if(key=='6') groupIdx=2;
  else if(key=='7') groupIdx=3; else if(key=='8') groupIdx=4; else if(key=='9') groupIdx=5; else if(key=='0') groupIdx=6;
  if(groupIdx!=-1) {
    int groupSize=MULTITAP_SIZES[groupIdx];
    if(choIdx==-1&&currentVowelStr!="") flushBuffer();
    if(choIdx==-1) { tapCount=0; lastKey=key; String con=MULTITAP_GROUPS[groupIdx][tapCount]; for(int i=0;i<19;i++) { if(CHOSUNG_LIST[i]==con){ choIdx=i; break; } } }
    else if(choIdx!=-1&&jungIdx==-1) {
      if(key==lastKey&&(now-lastPressTime<1000)) tapCount=(tapCount+1)%groupSize;
      else { flushBuffer(); tapCount=0; lastKey=key; }
      String con=MULTITAP_GROUPS[groupIdx][tapCount]; choIdx=-1; for(int i=0;i<19;i++) { if(CHOSUNG_LIST[i]==con){ choIdx=i; break; } }
    } else if(choIdx!=-1&&jungIdx!=-1) {
      if(jongIdx==0) { tapCount=0; lastKey=key; String con=MULTITAP_GROUPS[groupIdx][tapCount]; for(int i=0;i<28;i++) { if(JONGSUNG_LIST[i]==con){ jongIdx=i; break; } } }
      else {
        if(key==lastKey&&(now-lastPressTime<1000)) {
          String curJongStr=JONGSUNG_LIST[jongIdx]; bool insideGroup=false;
          for(int m=0;m<groupSize;m++) { if(MULTITAP_GROUPS[groupIdx][m]==curJongStr){ insideGroup=true; break; } }
          if(insideGroup) { tapCount=(tapCount+1)%groupSize; String con=MULTITAP_GROUPS[groupIdx][tapCount]; for(int i=0;i<28;i++) { if(JONGSUNG_LIST[i]==con){ jongIdx=i; break; } } }
          else {
            bool foundDouble=false;
            for(int i=0;i<11;i++) {
              if(DBL_CONS[i].result==curJongStr) {
                int b2GroupIdx=-1; for(int g=0;g<7;g++) for(int m=0;m<MULTITAP_SIZES[g];m++) if(MULTITAP_GROUPS[g][m]==DBL_CONS[i].b2){ b2GroupIdx=g; break; }
                if(b2GroupIdx==groupIdx) {
                  int curTap=0; for(int m=0;m<MULTITAP_SIZES[groupIdx];m++) if(MULTITAP_GROUPS[groupIdx][m]==DBL_CONS[i].b2){ curTap=m; break; }
                  int nextTap=(curTap+1)%MULTITAP_SIZES[groupIdx]; String nextB2=MULTITAP_GROUPS[groupIdx][nextTap]; bool ndf=false;
                  for(int k=0;k<11;k++) { if(DBL_CONS[k].b1==DBL_CONS[i].b1&&DBL_CONS[k].b2==nextB2){ for(int j=0;j<28;j++) { if(JONGSUNG_LIST[j]==DBL_CONS[k].result){ jongIdx=j; break; } } tapCount=nextTap; ndf=true; foundDouble=true; break; } }
                  if(!ndf) { flushBuffer(); tapCount=0; lastKey=key; String con=MULTITAP_GROUPS[groupIdx][tapCount]; for(int ii=0;ii<19;ii++) { if(CHOSUNG_LIST[ii]==con){ choIdx=ii; break; } } foundDouble=true; }
                  break;
                }
              }
            }
            if(!foundDouble) { flushBuffer(); tapCount=0; lastKey=key; String con=MULTITAP_GROUPS[groupIdx][tapCount]; for(int i=0;i<19;i++) { if(CHOSUNG_LIST[i]==con){ choIdx=i; break; } } }
          }
        } else {
          String curJongStr=JONGSUNG_LIST[jongIdx]; bool isDoubleFormed=false;
          for(int t=0;t<MULTITAP_SIZES[groupIdx]&&!isDoubleFormed;t++) {
            String newJongStr=MULTITAP_GROUPS[groupIdx][t]; if(newJongStr=="") continue;
            for(int i=0;i<11;i++) { if(DBL_CONS[i].b1==curJongStr&&DBL_CONS[i].b2==newJongStr) { if(DBL_CONS[i].result==curJongStr) break; for(int j=0;j<28;j++) { if(JONGSUNG_LIST[j]==DBL_CONS[i].result){ jongIdx=j; break; } } tapCount=t; lastKey=key; isDoubleFormed=true; break; } }
          }
          if(isDoubleFormed) { tapCount=0; lastKey=key; }
          else { flushBuffer(); tapCount=0; lastKey=key; String con=MULTITAP_GROUPS[groupIdx][tapCount]; for(int i=0;i<19;i++) { if(CHOSUNG_LIST[i]==con){ choIdx=i; break; } } }
        }
      }
    }
    currentSyllable=assembleHangul(); lastPressTime=now;
  }
}

// =========================================================================
// 디스플레이 렌더러 (displayTask에서만 호출)
// =========================================================================
void drawJasoVector(U8G2 &u8g2, int x, int y, int w, int h, String jaso) {
  u8g2.setDrawColor(1);
  if(jaso=="ㄱ"){u8g2.drawHLine(x,y,w);u8g2.drawVLine(x+w-1,y,h);}
  else if(jaso=="ㄴ"){u8g2.drawVLine(x,y,h);u8g2.drawHLine(x,y+h-1,w);}
  else if(jaso=="ㄷ"){u8g2.drawHLine(x,y,w);u8g2.drawVLine(x,y,h);u8g2.drawHLine(x,y+h-1,w);}
  else if(jaso=="ㄹ"){u8g2.drawHLine(x,y,w);u8g2.drawVLine(x+w-1,y,h/3+1);u8g2.drawHLine(x,y+h/2,w);u8g2.drawVLine(x,y+h/2,h/2);u8g2.drawHLine(x,y+h-1,w);}
  else if(jaso=="ㅁ"){u8g2.drawFrame(x,y,w,h);}
  else if(jaso=="ㅂ"){u8g2.drawVLine(x,y,h);u8g2.drawVLine(x+w-1,y,h);u8g2.drawHLine(x,y+h/2,w);u8g2.drawHLine(x,y+h-1,w);}
  else if(jaso=="ㅅ"){u8g2.drawLine(x+w/2,y,x,y+h-1);u8g2.drawLine(x+w/2,y,x+w-1,y+h-1);}
  else if(jaso=="ㅇ"){int r=(w<h?w:h)/3;if(r<2)r=2;u8g2.drawRFrame(x,y,w,h,r);}
  else if(jaso=="ㅈ"){u8g2.drawHLine(x,y,w);u8g2.drawLine(x+w/2,y,x,y+h-1);u8g2.drawLine(x+w/2,y,x+w-1,y+h-1);}
  else if(jaso=="ㅊ"){int ty=y+h/6;u8g2.drawHLine(x+w/4,y,w/2);u8g2.drawHLine(x,ty,w);u8g2.drawLine(x+w/2,ty,x,y+h-1);u8g2.drawLine(x+w/2,ty,x+w-1,y+h-1);}
  else if(jaso=="ㅋ"){u8g2.drawHLine(x,y,w);u8g2.drawVLine(x+w-1,y,h);u8g2.drawHLine(x,y+h/2,w);}
  else if(jaso=="ㅌ"){u8g2.drawHLine(x,y,w);u8g2.drawVLine(x,y,h);u8g2.drawHLine(x,y+h/2,w);u8g2.drawHLine(x,y+h-1,w);}
  else if(jaso=="ㅍ"){u8g2.drawHLine(x,y,w);u8g2.drawHLine(x,y+h-1,w);u8g2.drawVLine(x+w/3,y,h);u8g2.drawVLine(x+2*w/3,y,h);}
  else if(jaso=="ㅎ"){u8g2.drawHLine(x+w/4,y,w/2);u8g2.drawHLine(x,y+h/5,w);u8g2.drawFrame(x+w/4,y+2*h/5,w/2,h-2*h/5>2?h-2*h/5:2);}
  else if(jaso=="ㄲ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄱ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㄱ");}
  else if(jaso=="ㄸ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄷ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㄷ");}
  else if(jaso=="ㅃ"){drawJasoVector(u8g2,x,y,w/2,h,"ㅂ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅂ");}
  else if(jaso=="ㅆ"){drawJasoVector(u8g2,x,y,w/2,h,"ㅅ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅅ");}
  else if(jaso=="ㅉ"){drawJasoVector(u8g2,x,y,w/2,h,"ㅈ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅈ");}
  else if(jaso=="ㄳ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄱ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅅ");}
  else if(jaso=="ㄵ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄴ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅈ");}
  else if(jaso=="ㄶ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄴ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅎ");}
  else if(jaso=="ㄺ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄹ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㄱ");}
  else if(jaso=="ㄻ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄹ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅁ");}
  else if(jaso=="ㄼ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄹ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅂ");}
  else if(jaso=="ㄽ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄹ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅅ");}
  else if(jaso=="ㄾ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄹ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅌ");}
  else if(jaso=="ㄿ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄹ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅍ");}
  else if(jaso=="ㅀ"){drawJasoVector(u8g2,x,y,w/2,h,"ㄹ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅎ");}
  else if(jaso=="ㅄ"){drawJasoVector(u8g2,x,y,w/2,h,"ㅂ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅅ");}
  else if(jaso=="ㅣ"){u8g2.drawVLine(x+w/2,y,h);}
  else if(jaso=="ㅡ"){u8g2.drawHLine(x,y+h/2,w);}
  else if(jaso=="·"){int cx=x+w/2,cy=y+h/2;u8g2.drawPixel(cx,cy);u8g2.drawPixel(cx-1,cy);u8g2.drawPixel(cx+1,cy);u8g2.drawPixel(cx,cy-1);u8g2.drawPixel(cx,cy+1);}
  else if(jaso=="··"){int cx1=x+w/3,cx2=x+2*w/3,cy=y+h/2;u8g2.drawPixel(cx1,cy);u8g2.drawPixel(cx1-1,cy);u8g2.drawPixel(cx1+1,cy);u8g2.drawPixel(cx1,cy-1);u8g2.drawPixel(cx1,cy+1);u8g2.drawPixel(cx2,cy);u8g2.drawPixel(cx2-1,cy);u8g2.drawPixel(cx2+1,cy);u8g2.drawPixel(cx2,cy-1);u8g2.drawPixel(cx2,cy+1);}
  else if(jaso=="ㅏ"){u8g2.drawVLine(x+w/3,y,h);u8g2.drawHLine(x+w/3,y+h/2,w/2);}
  else if(jaso=="ㅑ"){u8g2.drawVLine(x+w/4,y,h);u8g2.drawHLine(x+w/4,y+h/3,w/2);u8g2.drawHLine(x+w/4,y+2*h/3,w/2);}
  else if(jaso=="ㅓ"){u8g2.drawVLine(x+2*w/3,y,h);u8g2.drawHLine(x+w/3,y+h/2,w/3);}
  else if(jaso=="ㅕ"){u8g2.drawVLine(x+3*w/4,y,h);u8g2.drawHLine(x+w/4,y+h/3,w/2);u8g2.drawHLine(x+w/4,y+2*h/3,w/2);}
  else if(jaso=="ㅗ"){u8g2.drawHLine(x,y+h-1,w);u8g2.drawVLine(x+w/2,y+h/4,h/2+1);}
  else if(jaso=="ㅛ"){u8g2.drawHLine(x,y+h-1,w);u8g2.drawVLine(x+w/3,y+h/4,h/2+1);u8g2.drawVLine(x+2*w/3,y+h/4,h/2+1);}
  else if(jaso=="ㅜ"){u8g2.drawHLine(x,y,w);u8g2.drawVLine(x+w/2,y,h/2+1);}
  else if(jaso=="ㅠ"){u8g2.drawHLine(x,y,w);u8g2.drawVLine(x+w/3,y,h/2+1);u8g2.drawVLine(x+2*w/3,y,h/2+1);}
  else if(jaso=="ㅐ"){u8g2.drawVLine(x+w/4,y,h);u8g2.drawHLine(x+w/4,y+h/2,w/2);u8g2.drawVLine(x+3*w/4,y,h);}
  else if(jaso=="ㅒ"){u8g2.drawVLine(x+w/5,y,h);u8g2.drawHLine(x+w/5,y+h/3,3*w/5);u8g2.drawHLine(x+w/5,y+2*h/3,3*w/5);u8g2.drawVLine(x+4*w/5,y,h);}
  else if(jaso=="ㅔ"){u8g2.drawVLine(x+w/4,y,h);u8g2.drawHLine(x,y+h/2,w/4);u8g2.drawVLine(x+3*w/4,y,h);}
  else if(jaso=="ㅖ"){u8g2.drawVLine(x+w/4,y,h);u8g2.drawHLine(x,y+h/3,w/4);u8g2.drawHLine(x,y+2*h/3,w/4);u8g2.drawVLine(x+3*w/4,y,h);}
  else if(jaso=="ㅚ"){drawJasoVector(u8g2,x,y+h/2+1,w/2,h-h/2-1,"ㅗ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅣ");}
  else if(jaso=="ㅘ"){drawJasoVector(u8g2,x,y+h/2+1,w/2,h-h/2-1,"ㅗ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅏ");}
  else if(jaso=="ㅙ"){drawJasoVector(u8g2,x,y+h/2+1,w/2,h-h/2-1,"ㅗ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅐ");}
  else if(jaso=="ㅝ"){drawJasoVector(u8g2,x,y+h/2+1,w/2,h-h/2-1,"ㅜ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅓ");}
  else if(jaso=="ㅞ"){drawJasoVector(u8g2,x,y+h/2+1,w/2,h-h/2-1,"ㅜ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅔ");}
  else if(jaso=="ㅟ"){drawJasoVector(u8g2,x,y+h/2+1,w/2,h-h/2-1,"ㅜ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅣ");}
  else if(jaso=="ㅢ"){drawJasoVector(u8g2,x,y+h/2+1,w/2,h-h/2-1,"ㅡ");drawJasoVector(u8g2,x+w/2+1,y,w-w/2-1,h,"ㅣ");}
}

void drawHangulSyllableBox(U8G2 &u8g2, int x, int y, int size, String cho, String jung, String jong) {
  if(cho==""&&jung==""&&jong=="") return;
  if(cho==""&&jung!="") { drawJasoVector(u8g2,x,y,size,size,jung); return; }
  if(cho!=""&&jung=="") { drawJasoVector(u8g2,x,y,size,size,cho); return; }
  bool isVertical=(jung=="ㅣ"||jung=="ㅏ"||jung=="ㅑ"||jung=="ㅓ"||jung=="ㅕ"||jung=="ㅐ"||jung=="ㅒ"||jung=="ㅔ"||jung=="ㅖ");
  bool isHorizontal=(jung=="ㅡ"||jung=="ㅗ"||jung=="ㅛ"||jung=="ㅜ"||jung=="ㅠ");
  int gap=1,cW=size,cH=size,jW=size,jH=size,jgW=size,jgH=size;
  int cX=x,cY=y,jX=x,jY=y,jgX=x,jgY=y;
  if(jong!="") {
    int topH=(size*56)/100, botH=size-topH-gap; jgX=x; jgY=y+topH+gap; jgW=size; jgH=botH;
    if(isVertical){cW=(size*45)/100;cH=topH;jX=x+cW+gap;jY=y;jW=size-cW-gap;jH=topH;}
    else if(isHorizontal){cH=(size*38)/100;cW=size;jX=x;jY=y+cH;jW=size;jH=topH-cH;}
    else{cW=(size*45)/100;cH=(size*38)/100;jX=x;jY=y;jW=size;jH=topH;}
  } else {
    if(isVertical){cW=(size*45)/100;cH=size;jX=x+cW+gap;jY=y;jW=size-cW-gap;jH=size;}
    else if(isHorizontal){cH=(size*45)/100;cW=size;jX=x;jY=y+cH+gap;jW=size;jH=size-cH-gap;}
    else{cW=(size*45)/100;cH=(size*45)/100;jX=x;jY=y;jW=size;jH=size;}
  }
  if(cho!="") drawJasoVector(u8g2,cX,cY,cW,cH,cho);
  if(jung!="") drawJasoVector(u8g2,jX,jY,jW,jH,jung);
  if(jong!="") drawJasoVector(u8g2,jgX,jgY,jgW,jgH,jong);
}

void renderStringCustom(U8G2 &u8g2, int startX, int startY, String str, int boxSize, int spacing) {
  int curX=startX, curY=startY;
  for(int i=0;i<(int)str.length();) {
    unsigned char c1=str.charAt(i); uint32_t uni=0; int len=0;
    if((c1&0x80)==0){uni=c1;len=1;} else if((c1&0xE0)==0xC0){uni=((c1&0x1F)<<6)|(str.charAt(i+1)&0x3F);len=2;} else if((c1&0xF0)==0xE0){uni=((c1&0x0F)<<12)|((str.charAt(i+1)&0x3F)<<6)|(str.charAt(i+2)&0x3F);len=3;}
    if(len==0){i++;continue;} int startIdx=i; i+=len;
    if(uni>=0xAC00&&uni<=0xD7A3){ int c=0,ju=0,jo=0; decomposeHangul(uni,c,ju,jo); drawHangulSyllableBox(u8g2,curX,curY,boxSize,CHOSUNG_LIST[c],JUNGSUNG_LIST[ju],JONGSUNG_LIST[jo]); curX+=boxSize+spacing; }
    else if(uni>=0x3131&&uni<=0x318E){ String ls=str.substring(startIdx,startIdx+len); drawHangulSyllableBox(u8g2,curX,curY,boxSize,ls,"",""); curX+=boxSize+spacing; }
    else { String ls=str.substring(startIdx,startIdx+len); if(ls==" ") curX+=boxSize/2+spacing; else if(uni==0x00B7||uni==0x318D){drawJasoVector(u8g2,curX,curY,boxSize,boxSize,"·");curX+=boxSize+spacing;} else{u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(curX,curY+boxSize-1,ls.c_str());curX+=6+spacing;} }
    if(curX>120){curX=startX;curY+=boxSize+6;}
  }
}

void drawRxLiveMorse(U8G2 &u8g2, int x, int y, String morse) {
  if((int)morse.length()>20) morse=morse.substring(morse.length()-20);
  const int DOT_R=2, DASH_W=7, SYM_GAP_PX=3, JAMO_GAP_PX=6, CY=y;
  int curX=x; bool prevWasSymbol=false;
  for(int i=0;i<(int)morse.length();i++) {
    char c=morse.charAt(i);
    if(c==' '){curX+=JAMO_GAP_PX;prevWasSymbol=false;}
    else if(c=='.'){if(prevWasSymbol)curX+=SYM_GAP_PX;u8g2.drawDisc(curX+DOT_R,CY,DOT_R);curX+=DOT_R*2+1;prevWasSymbol=true;}
    else if(c=='/'){if(prevWasSymbol)curX+=SYM_GAP_PX;u8g2.drawLine(curX+1,CY+3,curX+4,CY-3);curX+=7;prevWasSymbol=false;}
    else if(c=='-'){if(prevWasSymbol)curX+=SYM_GAP_PX;u8g2.drawHLine(curX,CY-1,DASH_W);u8g2.drawHLine(curX,CY,DASH_W);curX+=DASH_W+1;prevWasSymbol=true;}
    if(curX>126) break;
  }
}

void updateDisplays() {
  if(currentMode==3){ dbgDrawLeft(); dbgDrawRight(); return; }

  // ── u8g2_raw: 타이핑 + 상태 ──
  u8g2_raw.clearBuffer();
  u8g2_raw.enableUTF8Print();
  u8g2_raw.setFont(u8g2_font_6x10_tf);
  const char* modeLabel[]={"[TYPING]","[MOTOR]","[MORSE]","[DEBUG]"};
  u8g2_raw.drawStr(128-(strlen(modeLabel[currentMode])*6),10,modeLabel[currentMode]);
  u8g2_raw.setFont(u8g2_font_unifont_t_korean2);
  u8g2_raw.setClipWindow(0,8,128,22);
  drawSafeUTF8(u8g2_raw, 0-typedTextScroll, 20, typedText);
  u8g2_raw.setMaxClipWindow();
  u8g2_raw.drawHLine(0,36,128);
  String activeSyllable="";
  if(choIdx!=-1||currentVowelStr!="") activeSyllable=assembleHangul();
  if(punctIdx!=-1) activeSyllable=String(PUNCT_GROUP.charAt(punctIdx));
  if(activeSyllable!="") {
    u8g2_raw.setFont(u8g2_font_6x10_tf);
    u8g2_raw.drawStr(4,53,"Typing:");
    renderStringCustom(u8g2_raw,52,42,activeSyllable,16,2);
    u8g2_raw.drawFrame(49,39,22,22);
  } else if(currentMode==1) {
    u8g2_raw.setFont(u8g2_font_6x10_tf);
    const char* subName[4] = {"5 STEP","10 STEP","15 STEP","TOGGLE"};
    char line[24];
    if(motorSub==3 && motorRunningDir!='\0')
      snprintf(line, sizeof(line), "MODE: TOGGLE(%c)", motorRunningDir);
    else
      snprintf(line, sizeof(line), "MODE: %s", subName[motorSub]);
    u8g2_raw.drawStr(4,46,line);
    if(!motorFound) {
      u8g2_raw.setFont(u8g2_font_unifont_t_korean2);
      drawSafeUTF8(u8g2_raw, 4, 59, "\xEB\xAA\xA8\xED\x84\xB0 \xEC\x9D\xB8\xEC\x8B\x9D \xEC\x8B\xA4\xED\x8C\xA8!");
    } else {
      u8g2_raw.drawStr(4,59,"5:mode 2/8/4/6:move");
    }
  } else if(currentMode==2) {
    u8g2_raw.setFont(u8g2_font_6x10_tf);
    u8g2_raw.drawStr(4,46,"2(.) : DOT");
    u8g2_raw.drawStr(4,59,"3(-) : DASH");
  } else {
    u8g2_raw.setFont(u8g2_font_6x10_tf);
    u8g2_raw.drawStr(4,53,txBusy?"TX...":"Ready...");
  }
  u8g2_raw.sendBuffer();

  // ── u8g2_comb: 수신 로그 ──
  u8g2_comb.clearBuffer();
  if(showDecomposedView) {
    u8g2_comb.setFont(u8g2_font_6x10_tf);
    u8g2_comb.drawStr(4,12,"[ DECOMPOSE DEBUG ]");
    u8g2_comb.drawHLine(0,16,128);
    u8g2_comb.drawStr(4,31,"Chosung  :"); if(choIdx!=-1) u8g2_comb.drawStr(70,31,CHOSUNG_LIST[choIdx].c_str());
    u8g2_comb.drawStr(4,45,"Jungsung:"); if(currentVowelStr!="") u8g2_comb.drawStr(70,45,currentVowelStr.c_str());
    u8g2_comb.drawStr(4,59,"Jongsung:"); if(jongIdx>0) u8g2_comb.drawStr(70,59,JONGSUNG_LIST[jongIdx].c_str());
  } else {
    u8g2_comb.drawHLine(0,46,128);
    for(int i=0;i<RX_LOG_MAX;i++) {
      if(rxChatLog[i]=="") continue;
      int y=13+i*14;   // 줄 간격 14px (한글 16px 안 겹치게)
      u8g2_comb.setFont(u8g2_font_unifont_t_korean2);
      u8g2_comb.setClipWindow(0,y-13,128,y+2);  // 글자 전체 포함, 구분선(46) 위까지
      drawSafeUTF8(u8g2_comb,0-rxChatScroll[i],y,rxChatLog[i]);
      u8g2_comb.setMaxClipWindow();
    }
    if(rxChatLog[0]==""&&rxChatLog[1]==""&&rxChatLog[2]=="") { u8g2_comb.setFont(u8g2_font_6x10_tf); u8g2_comb.drawStr(4,20,"Waiting RX..."); }
    if(rxLiveMorse!="") { u8g2_comb.setClipWindow(0,50,128,64); drawRxLiveMorse(u8g2_comb,2-rxLiveMorseScroll,57,rxLiveMorse); u8g2_comb.setMaxClipWindow(); }
    else { u8g2_comb.setFont(u8g2_font_6x10_tf); u8g2_comb.drawStr(2,61,"-- RX idle --"); }
  }
  u8g2_comb.sendBuffer();
}

void dbgDrawLeft() {
  u8g2_raw.clearBuffer(); u8g2_raw.setFont(u8g2_font_6x10_tf);
  u8g2_raw.drawStr(0,10,"[3:DEBUG TIMING]"); u8g2_raw.drawHLine(0,13,128);
  for(int i=0;i<5;i++) {
    int y=23+i*10; bool selected=(i==dbgCursor);
    if(selected){u8g2_raw.drawBox(0,y-9,128,11);u8g2_raw.setDrawColor(0);}
    u8g2_raw.drawStr(2,y,dbgParamNames[i]);
    bool showVal=!(selected&&dbgEditing&&!dbgBlink);
    if(showVal){String v=String(*dbgParams[i]);u8g2_raw.drawStr(75,y,v.c_str());}
    if(selected){u8g2_raw.drawStr(104,y,dbgEditing?"ED!":"<7>");u8g2_raw.setDrawColor(1);}
  }
  u8g2_raw.sendBuffer();
}
void dbgDrawRight() {
  u8g2_comb.clearBuffer(); u8g2_comb.setFont(u8g2_font_6x10_tf);
  u8g2_comb.drawStr(0,10,"RAW:");
  if(dbgRxMorse!=""){u8g2_comb.setClipWindow(0,11,128,22);u8g2_comb.drawStr(0-(int)dbgMorseScroll,21,dbgRxMorse.c_str());u8g2_comb.setMaxClipWindow();}
  else u8g2_comb.drawStr(35,21,"-- no rx --");
  u8g2_comb.drawHLine(0,23,128); u8g2_comb.drawStr(0,33,"KOR:");
  if(dbgRxHangul!=""){u8g2_comb.setClipWindow(0,34,128,48);u8g2_comb.setFont(u8g2_font_unifont_t_korean2);drawSafeUTF8(u8g2_comb,0-(int)dbgHangulScroll,45,dbgRxHangul);u8g2_comb.setMaxClipWindow();u8g2_comb.setFont(u8g2_font_6x10_tf);}
  else u8g2_comb.drawStr(35,45,"-- no rx --");
  u8g2_comb.drawHLine(0,50,128);
  if(dbgStatus.startsWith(">> SUCCESS")){u8g2_comb.drawBox(0,52,128,12);u8g2_comb.setDrawColor(0);u8g2_comb.drawStr(8,62,">> SUCCESS <<");u8g2_comb.setDrawColor(1);}
  else if(dbgStatus.length()>0) u8g2_comb.drawStr(1,62,dbgStatus.substring(0,20).c_str());
  u8g2_comb.sendBuffer();
}

// =========================================================================
// 디버그 모드 헬퍼
// =========================================================================
void dbgApplyConstraints() {
  for(int i=0;i<5;i++) if(*dbgParams[i]<1) *dbgParams[i]=1;
  if(DBG_SYM_GAP<DBG_DOT_ON)     DBG_SYM_GAP=DBG_DOT_ON;
  if(DBG_JAMO_GAP<=DBG_SYM_GAP)  DBG_JAMO_GAP=DBG_SYM_GAP+1;
  if(DBG_WORD_GAP<=DBG_JAMO_GAP) DBG_WORD_GAP=DBG_JAMO_GAP+1;
  if(DBG_DASH_ON<DBG_DOT_ON)     DBG_DASH_ON=DBG_DOT_ON;
}
void dbgSendSentence(int idx) {
  String sentence=String(DBG_SENTENCES[idx]);
  String jamoArr[200]; int jamoCount=decomposeTextToJamoBase(sentence,jamoArr);
  dbgRxMorse=""; dbgRxHangul=""; dbgStatus="TX..."; dbgMorseScroll=0; dbgHangulScroll=0;
  dbgPendingExpected=sentence;
  startMorseTx(jamoArr,jamoCount);
  dbgDrawRight();
}

// =========================================================================
// 모터 명령 (Wire1 → 아두이노 슬레이브 0x08)
// =========================================================================
void sendMotorCmd(const char* cmd) {
  Wire1.beginTransmission(MOTOR_I2C_ADDR);
  Wire1.write((const uint8_t*)cmd, strlen(cmd));
  uint8_t result = Wire1.endTransmission();
  motorFound = (result == 0);  // 0=ACK(인식 성공), 그 외=실패
}

// 모터 슬레이브 존재 확인 (데이터 없이 주소만 ping)
void pingMotor() {
  Wire1.beginTransmission(MOTOR_I2C_ADDR);
  uint8_t result = Wire1.endTransmission();
  motorFound = (result == 0);
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();  // OLED용 (SDA=21, SCL=22)
  Wire1.begin(18, 19);  // 모터 슬레이브용 (SDA=18, SCL=19)

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  pinMode(RX_SIGNAL_PIN, INPUT_PULLDOWN);
  pinMode(RX_LENGTH_PIN, INPUT_PULLDOWN);

  // Queue 생성
  // rxSymQueue 제거: 인터럽트 플래그(rxGotDot/rxGotDash) 폴링 방식으로 변경
  keyQueue    = xQueueCreate(16, sizeof(char));   // keypadTask → loop
  txTrigQueue = xQueueCreate(1,  sizeof(uint8_t));// loop → morseTxTask

  // ISR
  attachInterrupt(digitalPinToInterrupt(RX_SIGNAL_PIN), onDotSignal,  RISING);
  attachInterrupt(digitalPinToInterrupt(RX_LENGTH_PIN),  onDashSignal, RISING);

  // 디버그 파라미터 포인터
  dbgParams[0]=&DBG_DOT_ON; dbgParams[1]=&DBG_DASH_ON;
  dbgParams[2]=&DBG_SYM_GAP; dbgParams[3]=&DBG_JAMO_GAP; dbgParams[4]=&DBG_WORD_GAP;

  // OLED 초기화
  u8g2_raw.setI2CAddress(0x78);  u8g2_raw.begin();
  u8g2_comb.setI2CAddress(0x7A); u8g2_comb.begin();

  // Task 생성
  // morseTxTask: Core 1, Priority 10 (LASER 타이밍 최우선)
  xTaskCreatePinnedToCore(morseTxTask,   "morseTx",   4096, nullptr, 10, &hMorseTxTask,   1);
  // rxProcessTask: Core 0, Priority 6
  xTaskCreatePinnedToCore(rxProcessTask, "rxProc",     4096, nullptr,  6, &hRxProcessTask, 0);
  // keypadTask: Core 1, Priority 5
  xTaskCreatePinnedToCore(keypadTask,    "keypad",     2048, nullptr,  5, &hKeypadTask,    1);
  // displayTask: Core 0, Priority 3 (OLED 블로킹 낮은 우선순위 허용)
  xTaskCreatePinnedToCore(displayTask,   "display",    8192, nullptr,  3, &hDisplayTask,   0);

  updateDisplays();
}

// =========================================================================
// LOOP: 키 처리만 담당 (디스플레이·수신·송신은 모두 Task로 이전)
// =========================================================================
void loop() {
  unsigned long now = millis();
  char customKey = '\0';
  xQueueReceive(keyQueue, &customKey, 0);  // non-blocking

  // ── M키: 모드 전환 (공통) ──
  if(customKey=='M') {
    if(currentMode==1){ sendMotorCmd("STOP"); digitalWrite(LASER_PIN,LOW); }
    if(currentMode==2){ digitalWrite(LASER_PIN,LOW); manualLaserOn=false; }
    // 비확정(조합 중) 글자 버퍼 비우기 (확정된 typedText는 유지)
    choIdx=-1; jungIdx=-1; jongIdx=0; currentVowelStr=""; currentSyllable="";
    tapCount=0; lastKey='\0'; punctIdx=-1;
    currentMode=(currentMode+1)%4;
    if(currentMode==1) { digitalWrite(LASER_PIN,HIGH); pingMotor(); }  // 모터 모드 진입 시 인식 확인
    displayDirty=true;
    return;
  }

  // ── D키: 전체 클리어 (공통) ──
  if(customKey=='D') {
    rxMorseBuffer=""; rxJamoCount=0; rxLastSymbolTime=0;
    rxLiveMorse=""; rxLastDotTime=0; rxLastDashTime=0;
    // 인터럽트 플래그 클리어
    rxGotDot=false; rxGotDash=false;
    for(int i=0;i<RX_LOG_MAX;i++){rxChatLog[i]="";rxChatScroll[i]=0;}
    rxLogBuffer=""; dbgRxMorse=""; dbgRxHangul=""; dbgStatus=""; dbgMorseScroll=0; dbgHangulScroll=0;
    choIdx=-1; jungIdx=-1; jongIdx=0; currentVowelStr=""; currentSyllable="";
    tapCount=0; lastKey='\0'; punctIdx=-1; typedText=""; typedTextScroll=0;
    lastPressTime=now; displayDirty=true; rxDirty=true;
    return;
  }

  // ── 모드 0: 타자 ──
  if(currentMode==0) {
    if(!txBusy) {
      if(customKey) {
        if(customKey=='E') {
          flushBuffer();
          if(typedText.length()>0) {
            String txJamo[100]; int txCount=decomposeTextToJamoBase(typedText,txJamo);
            startMorseTx(txJamo,txCount); typedText="";
          }
          lastPressTime=now;
        } else if(customKey=='A') { flushBuffer(); lastPressTime=now; }
        else if(customKey=='#') { processKey('*'); lastPressTime=now; }
        else { processKey(customKey); lastPressTime=now; }
        displayDirty=true;
      }
      if(Serial.available()>0) {
        char c=Serial.read();
        if(c!='\n'&&c!='\r'&&c!=' ') {
          if(c=='E'){ flushBuffer(); if(typedText.length()>0){ String j[100]; int n=decomposeTextToJamoBase(typedText,j); startMorseTx(j,n); typedText=""; } lastPressTime=now; }
          else if(c=='A'){ flushBuffer(); lastPressTime=now; }
          else if(c=='#'){ processKey('*'); lastPressTime=now; }
          else { processKey(c); lastPressTime=now; }
          displayDirty=true;
        }
      }
      if(currentSyllable!=""&&(now-lastPressTime>4000)){ flushBuffer(); displayDirty=true; }
    }
  }

  // ── 모드 1: 모터 ──
  else if(currentMode==1) {
    // 1초마다 모터 연결 상태 확인 (키 입력 없어도 감지)
    static unsigned long lastPing=0;
    if(now-lastPing>=1000) {
      lastPing=now;
      bool prev=motorFound;
      pingMotor();
      if(prev!=motorFound) displayDirty=true;  // 상태 바뀌면 화면 갱신
    }
    if(customKey) {
      // 가운데(5) = 서브모드 순환
      if(customKey=='5') {
        if(motorRunningDir!='\0') { sendMotorCmd("STOP"); motorRunningDir='\0'; } // 토글 중이면 먼저 정지
        motorSub = (motorSub+1)%4;
        displayDirty=true;
      } else {
        // 방향키 → 방향 문자
        char dir='\0';
        if(customKey=='2')dir='U'; else if(customKey=='8')dir='D';
        else if(customKey=='4')dir='L'; else if(customKey=='6')dir='R';

        if(dir!='\0') {
          char cmd[8];
          if(motorSub==3) {
            // 토글: 같은 방향 다시 누르면 정지, 아니면 연속 회전
            if(motorRunningDir==dir) { sendMotorCmd("STOP"); motorRunningDir='\0'; }
            else { cmd[0]=dir; cmd[1]='C'; cmd[2]='\0'; sendMotorCmd(cmd); motorRunningDir=dir; }
          } else {
            // 스텝 모드: 16 또는 10스텝
            int steps = (motorSub==0) ? 5 : (motorSub==1) ? 10 : 15;
            snprintf(cmd, sizeof(cmd), "%c%d", dir, steps);
            sendMotorCmd(cmd);
          }
          displayDirty=true;
        }
      }
    }
  }

  // ── 모드 2: 모스 수동 송신 ──
  else if(currentMode==2) {
    static unsigned long manualLaserDuration=0;
    if(!manualLaserOn) {
      if(customKey=='2'){ digitalWrite(LASER_PIN,HIGH); manualLaserOn=true; manualLaserTimer=now; manualLaserDuration=(unsigned long)DBG_DOT_ON; }
      else if(customKey=='3'){ digitalWrite(LASER_PIN,HIGH); manualLaserOn=true; manualLaserTimer=now; manualLaserDuration=(unsigned long)DBG_DASH_ON; }
    }
    if(manualLaserOn&&(now-manualLaserTimer)>=manualLaserDuration){ digitalWrite(LASER_PIN,LOW); manualLaserOn=false; displayDirty=true; }
  }

  // ── 모드 3: 디버그 ──
  else if(currentMode==3) {
    if(customKey&&!txBusy) {
      if(customKey=='1'){dbgCursor=(dbgCursor+4)%5;displayDirty=true;}
      else if(customKey=='4'){dbgCursor=(dbgCursor+1)%5;displayDirty=true;}
      else if(customKey=='7'){
        dbgEditing=!dbgEditing; dbgBlink=true; dbgBlinkTimer=millis();
        if(!dbgEditing){dbgApplyConstraints();dbgStatus="SAVED!";}
        displayDirty=true;
      } else if(dbgEditing) {
        int delta=0;
        if(customKey=='2')delta=+1; else if(customKey=='5')delta=-1;
        else if(customKey=='3')delta=+10; else if(customKey=='6')delta=-10;
        if(delta!=0){*dbgParams[dbgCursor]+=delta;if(*dbgParams[dbgCursor]<1)*dbgParams[dbgCursor]=1;displayDirty=true;}
      } else {
        if(customKey=='8'){
          dbgRxMorse="";dbgRxHangul="";dbgStatus="DOT TX";dbgPendingExpected="";dbgDrawRight();
          // 단발 DOT: morseTxTask를 통하지 않고 직접 (블로킹 최소: DOT_ON ms)
          digitalWrite(LASER_PIN,HIGH); vTaskDelay(pdMS_TO_TICKS(DBG_DOT_ON)); digitalWrite(LASER_PIN,LOW);
          dbgStatus="DOT SENT";displayDirty=true;
        } else if(customKey=='9'){
          dbgRxMorse="";dbgRxHangul="";dbgStatus="DASH TX";dbgPendingExpected="";dbgDrawRight();
          digitalWrite(LASER_PIN,HIGH); vTaskDelay(pdMS_TO_TICKS(DBG_DASH_ON)); digitalWrite(LASER_PIN,LOW);
          dbgStatus="DASH SENT";displayDirty=true;
        }
        else if(customKey=='0'){dbgSendSentence(0);displayDirty=true;}
        else if(customKey=='A'){dbgSendSentence(1);displayDirty=true;}
        else if(customKey=='E'){dbgSendSentence(2);displayDirty=true;}
        else if(customKey=='#'){dbgSendSentence(3);displayDirty=true;}
        else if(customKey=='B'){
          // 레이저 5V 토글: 누르면 계속 ON, 다시 누르면 OFF
          static bool dbgLaserOn=false;
          dbgLaserOn=!dbgLaserOn;
          digitalWrite(LASER_PIN, dbgLaserOn?HIGH:LOW);
          dbgRxMorse="";dbgRxHangul="";dbgPendingExpected="";
          dbgStatus=dbgLaserOn?"LASER ON":"LASER OFF";
          displayDirty=true;
        }
      }
    }
  }

  // loop()는 매우 빠르게 반환 — 실제 무거운 작업은 모두 Task에서 처리됨
  vTaskDelay(pdMS_TO_TICKS(1));
}

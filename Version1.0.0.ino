#include <Arduino.h>
#include <TFT_eSPI.h>
#include "driver/twai.h"
#include <Preferences.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>

// ==========================================
// 1. PIN DEFINITIONS & CONSTANTS
// ==========================================
#define CAN_TX_PIN    GPIO_NUM_27
#define CAN_RX_PIN    GPIO_NUM_22
#define LDR_PIN       34          
#define BACKLIGHT_PIN 21          

// CYD Touch Pins (XPT2046)
#define XPT_CLK  25
#define XPT_CS   33
#define XPT_MOSI 32
#define XPT_MISO 39
#define XPT_IRQ  36

// Backlight & AutoDim Settings
#define BACKLIGHT_CHANNEL 0
#define PWM_FREQ          12000    
#define PWM_RES           8        

// --- ตั้งค่า Wi-Fi และ Private GitHub OTA ---
const char* UPDATE_SSID = "YOUR_WIFI_SSID";             // ใส่ชื่อ Wi-Fi ของคุณ
const char* UPDATE_PASSWORD = "YOUR_WIFI_PASSWORD";     // ใส่รหัสผ่าน Wi-Fi
const String CURRENT_VERSION = "v1.0.0";                // เวอร์ชันปัจจุบันของเฟิร์มแวร์

// สำหรับ Private Repo ให้ใช้ Raw URL และ Personal Access Token (PAT)
const String GITHUB_BIN_URL = "https://raw.githubusercontent.com/YOUR_USERNAME/YOUR_REPO/main/firmware.bin"; 
const String GITHUB_PAT = "YOUR_GITHUB_PERSONAL_ACCESS_TOKEN"; // Personal Access Token (PAT) จาก GitHub

// ==========================================
// 2. GLOBAL OBJECTS & VARIABLES
// ==========================================
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);
Preferences preferences;          
XPT2046_Touchscreen ts(XPT_CS);   

unsigned long lastCANReqTime = 0;
unsigned long lastUIRefresh = 0;
unsigned long lastO2RxTime = 0;     

unsigned long o2TouchStartTime = 0;
bool isO2Touching = false;
bool o2ResetMenuShown = false; 

unsigned long updateTouchStartTime = 0;
bool isUpdateTouching = false;
bool updateMenuShown = false;

String dtcCodes[5];
int dtcCount = 0;
bool showDtcPopup = false;
bool hasCheckedDtc = false;
unsigned long startupTimer = 0;
unsigned long popupStartTime = 0;

struct OBDData {
  float speed = 0;
  float rpm = 0;
  float coolant = 0;
  float battery = 0;
  float stft = 0;
  float ltft = 0;
  float lambda = 0;
  uint8_t loopStatus = 0;
} obd;

const uint8_t PID_LOOP_STATUS = 0x03;
const uint8_t PID_COOLANT     = 0x05;
const uint8_t PID_STFT        = 0x06;
const uint8_t PID_LTFT        = 0x07;
const uint8_t PID_RPM         = 0x0C;
const uint8_t PID_SPEED       = 0x0D;
const uint8_t PID_BATTERY     = 0x42;

const uint8_t o2PidList[] = {0x24, 0x34, 0x14}; 
const int totalO2Pids = sizeof(o2PidList) / sizeof(o2PidList[0]);
int currentO2PidIdx = 0;
uint8_t activeO2Pid = o2PidList[0];

uint8_t pidCycle[] = {
  PID_SPEED, PID_RPM, PID_COOLANT, PID_BATTERY, 
  PID_STFT, PID_LTFT, 0x24, PID_LOOP_STATUS 
};
uint8_t currentPidIdx = 0;

// ==========================================
// 3. FUNCTION PROTOTYPES
// ==========================================
void setBacklight(uint8_t brightness);
void autoDim();
void renderDashboard();
void updateGridCard(int x, int y, int w, int h, const char* label, String valStr, const char* unit, uint16_t accentColor, uint16_t valColor = TFT_WHITE);
void updateO2WideCard(int x, int y, int w, int h, float lambdaVal);
void initTWAI();
void sendOBDRequest(uint8_t pid);
void readCANResponse();
void checkTouchO2Reset();
void checkTouchUpdateMenu();
void sendReadDtcRequest();
void decodeDtc(uint8_t b1, uint8_t b2);
void renderDtcPopup();
void checkTouchToDismiss();
void performPrivateGitHubOTA();
void renderWifiConnectedMenu();

// ==========================================
// 4. SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- CYD OBD2 Universal Dash Starting ---");

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  SPI.begin(XPT_CLK, XPT_MISO, XPT_MOSI, XPT_CS);
  ts.begin();
  ts.setRotation(1);

  analogSetAttenuation(ADC_0db);
  pinMode(LDR_PIN, INPUT);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(BACKLIGHT_PIN, PWM_FREQ, PWM_RES);
#else
  ledcSetup(BACKLIGHT_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(BACKLIGHT_PIN, BACKLIGHT_CHANNEL);
#endif

  setBacklight(255);
  initTWAI();
  tft.fillScreen(TFT_BLACK);

  preferences.begin("obd_config", false);
  activeO2Pid = preferences.getUChar("saved_o2", o2PidList[0]);
  
  for (int i = 0; i < totalO2Pids; i++) {
    if (o2PidList[i] == activeO2Pid) {
      currentO2PidIdx = i;
      break;
    }
  }
  Serial.printf("Loaded Saved O2 PID: 0x%02X\n", activeO2Pid);

  lastO2RxTime = millis();
}

void loop() {
  unsigned long now = millis();

  if (!hasCheckedDtc && now > 5000) {
    dtcCount = 0;
    sendReadDtcRequest();
    hasCheckedDtc = true;
    startupTimer = now;
  }

  if (showDtcPopup) {
    if (now - popupStartTime >= 15000) {
      showDtcPopup = false; 
      tft.fillScreen(TFT_BLACK); 
    }
    checkTouchToDismiss(); 
    renderDtcPopup();
  } 
  else {
    checkTouchUpdateMenu(); 
    checkTouchO2Reset();    

    if (now - lastO2RxTime > 1500) {
      currentO2PidIdx = (currentO2PidIdx + 1) % totalO2Pids;
      activeO2Pid = o2PidList[currentO2PidIdx];
      lastO2RxTime = now; 
      Serial.printf("Switching to alternative O2 PID: 0x%02X\n", activeO2Pid);
    }
    pidCycle[6] = activeO2Pid;

    if (now - lastCANReqTime >= 40) {
      lastCANReqTime = now;
      sendOBDRequest(pidCycle[currentPidIdx]);
      
      currentPidIdx++;
      if (currentPidIdx >= (sizeof(pidCycle) / sizeof(pidCycle[0]))) {
        currentPidIdx = 0;
        autoDim();
      }
    }

    readCANResponse();

    if (now - lastUIRefresh >= 200) {
      lastUIRefresh = now;
      renderDashboard();
    }
  }
}

// ==========================================
// 5. HELPER FUNCTIONS & LOGIC
// ==========================================
void setBacklight(uint8_t brightness) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(BACKLIGHT_PIN, brightness);
#else
  ledcWrite(BACKLIGHT_CHANNEL, brightness);
#endif
}

void autoDim() {
  long ldrSum = 0;
  for (int i = 0; i < 16; i++) {
    ldrSum += analogRead(LDR_PIN);
    delayMicroseconds(50);
  }
  int ldrValue = ldrSum / 16;
  int targetBrightness = map(ldrValue, 0, 1023, 255, 5);
  targetBrightness = constrain(targetBrightness, 5, 255);

  static float currentBrightness = 255.0;
  if (abs(targetBrightness - currentBrightness) > 2.0) {
    currentBrightness += (targetBrightness - currentBrightness) * 0.15;
    setBacklight((uint8_t)currentBrightness);
  }
}

void decodeDtc(uint8_t b1, uint8_t b2) {
  if (dtcCount >= 5) return;
  char prefix;
  switch (b1 >> 6) {
    case 0: prefix = 'P'; break;
    case 1: prefix = 'C'; break;
    case 2: prefix = 'B'; break;
    case 3: prefix = 'U'; break;
  }
  int code = ((b1 & 0x3F) << 8) | b2;
  char buffer[8];
  sprintf(buffer, "%c%04X", prefix, code);
  dtcCodes[dtcCount++] = String(buffer);
}

void sendReadDtcRequest() {
  twai_message_t message;
  message.identifier = 0x7DF;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 8;
  message.data[0] = 0x01; 
  message.data[1] = 0x03; 
  for (int i = 2; i < 8; i++) message.data[i] = 0x55;
  twai_transmit(&message, pdMS_TO_TICKS(10));
}

void checkTouchToDismiss() {
  if (ts.tirqTouched() && ts.touched()) {
    unsigned long now = millis();
    if (!isO2Touching) {
      isO2Touching = true;
      o2TouchStartTime = now;
    } else if (now - o2TouchStartTime >= 2000) { 
      showDtcPopup = false; 
      isO2Touching = false;
      tft.fillScreen(TFT_BLACK); 
    }
  } else {
    isO2Touching = false;
  }
}

void checkTouchUpdateMenu() {
  if (o2ResetMenuShown) return;

  if (ts.tirqTouched() && ts.touched()) {
    TS_Point p = ts.getPoint();
    int touchX = map(p.x, 200, 3700, 0, 320);
    int touchY = map(p.y, 200, 3800, 0, 240);

    bool touchInRpmCard = (touchX >= 3 && touchX <= 103 && touchY >= 3 && touchY <= 77);

    if (!isUpdateTouching) {
      if (touchInRpmCard) {
        isUpdateTouching = true;
        updateTouchStartTime = millis();
      }
    } else {
      unsigned long now = millis();
      unsigned long elapsed = now - updateTouchStartTime;
      int remainingSeconds = 3 - (elapsed / 1000);
      if (remainingSeconds < 0) remainingSeconds = 0;

      updateMenuShown = true;

      spr.createSprite(220, 90);
      spr.fillSprite(TFT_BLACK);
      spr.drawRoundRect(0, 0, 220, 90, 8, TFT_CYAN);
      
      spr.setTextColor(TFT_CYAN, TFT_BLACK);
      spr.setTextDatum(TC_DATUM);
      spr.drawString("SYSTEM UPDATE?", 110, 10, 2);
      
      spr.setTextColor(TFT_WHITE, TFT_BLACK);
      spr.setTextDatum(MC_DATUM);
      spr.drawString(String(remainingSeconds) + " SECONDS", 110, 52, 4);
      
      spr.pushSprite(50, 75);
      spr.deleteSprite();

      if (elapsed >= 3000) {
        Serial.println("System Update Menu triggered! Connecting to Wi-Fi with 30s timeout...");
        
        WiFi.begin(UPDATE_SSID, UPDATE_PASSWORD);
        unsigned long wifiStartTime = millis();
        bool connected = false;

        while (millis() - wifiStartTime < 30000) {
          unsigned long wifiElapsed = millis() - wifiStartTime;
          int wifiRemainingSec = 30 - (wifiElapsed / 1000);
          if (wifiRemainingSec < 0) wifiRemainingSec = 0;

          spr.createSprite(260, 130);
          spr.fillSprite(TFT_BLACK);
          spr.drawRoundRect(0, 0, 260, 130, 8, TFT_CYAN);
          
          spr.setTextColor(TFT_CYAN, TFT_BLACK);
          spr.setTextDatum(TC_DATUM);
          spr.drawString("CONNECTING WIFI...", 130, 15, 2);
          
          spr.setTextColor(TFT_WHITE, TFT_BLACK);
          spr.setTextDatum(MC_DATUM);
          spr.drawString(String(UPDATE_SSID), 130, 52, 2);
          
          spr.setTextColor(TFT_YELLOW, TFT_BLACK);
          spr.drawString("TIMEOUT IN: " + String(wifiRemainingSec) + "s", 130, 90, 2);
          
          spr.pushSprite(30, 55);
          spr.deleteSprite();

          if (WiFi.status() == WL_CONNECTED) {
            connected = true;
            break;
          }
          delay(500);
        }

        if (connected) {
          Serial.println("Wi-Fi Connected successfully!");
          renderWifiConnectedMenu();
        } else {
          Serial.println("Wi-Fi connection timeout! Returning to dashboard...");
          WiFi.disconnect(true);
          
          tft.fillScreen(TFT_BLACK);
          tft.setTextColor(TFT_RED, TFT_BLACK);
          tft.setTextDatum(MC_DATUM);
          tft.drawString("WIFI TIMEOUT!", tft.width() / 2, tft.height() / 2 - 10, 4);
          tft.setTextColor(TFT_WHITE, TFT_BLACK);
          tft.drawString("RETURNING TO DASHBOARD...", tft.width() / 2, tft.height() / 2 + 20, 2);
          delay(2000);
          
          updateMenuShown = false;
          isUpdateTouching = false;
          tft.fillScreen(TFT_BLACK);
        }
      }
    }
  } else {
    if (isUpdateTouching) {
      isUpdateTouching = false;
      if (updateMenuShown) {
        updateMenuShown = false;
        tft.fillScreen(TFT_BLACK); 
      }
    }
  }
}

// แสดงเมนู 2 ปุ่มเมื่อเชื่อมต่อ Wi-Fi สำเร็จ
void renderWifiConnectedMenu() {
  tft.fillScreen(TFT_BLACK);
  
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("FIRMWARE UPDATE", tft.width() / 2, 10, 2);
  
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Current: " + CURRENT_VERSION, tft.width() / 2, 32, 2);

  // ปุ่มที่ 1: อัปเดตจาก Private GitHub
  tft.drawRoundRect(20, 65, 280, 65, 8, TFT_GREEN);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("UPDATE FROM PRIVATE GITHUB", 160, 78, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Tap with PAT Auth", 160, 102, 1);

  // ปุ่มที่ 2: กลับหน้าแดชบอร์ด
  tft.drawRoundRect(20, 145, 280, 65, 8, TFT_RED);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("BACK TO DASHBOARD", 160, 158, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Disconnect & Return", 160, 182, 1);

  while (true) {
    if (ts.tirqTouched() && ts.touched()) {
      TS_Point p = ts.getPoint();
      int touchX = map(p.x, 200, 3700, 0, 320);
      int touchY = map(p.y, 200, 3800, 0, 240);

      // เช็คปุ่มที่ 1: อัปเดต
      if (touchX >= 20 && touchX <= 300 && touchY >= 65 && touchY <= 130) {
        delay(300);
        performPrivateGitHubOTA();
        break;
      }
      // เช็คปุ่มที่ 2: กลับแดชบอร์ด
      else if (touchX >= 20 && touchX <= 300 && touchY >= 145 && touchY <= 210) {
        delay(300);
        WiFi.disconnect(true);
        tft.fillScreen(TFT_BLACK);
        updateMenuShown = false;
        isUpdateTouching = false;
        break;
      }
    }
    delay(50);
  }
}

// ฟังก์ชันดาวน์โหลดเฟิร์มแวร์จาก Private GitHub ด้วย Token ยืนยันตัวตน
void performPrivateGitHubOTA() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("CONNECTING PRIVATE REPO...", tft.width() / 2, tft.height() / 2, 2);

  WiFiClientSecure client;
  client.setInsecure(); // ข้ามการเช็ค SSL Certificate เพื่อความเสถียร

  HTTPClient https;
  https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS); // รองรับการ Redirect ไปยัง CDN ของ GitHub

  if (https.begin(client, GITHUB_BIN_URL)) {
    // แนบ Token สำหรับ Private Repository
    https.addHeader("Authorization", "Bearer " + GITHUB_PAT);
    
    int httpCode = https.GET();
    if (httpCode == HTTP_CODE_OK) {
      int contentLength = https.getSize();
      bool canBegin = Update.begin(contentLength);
      
      if (canBegin) {
        WiFiClient *stream = https.getStreamPtr();
        size_t written = Update.writeStream(*stream);

        if (written == contentLength) {
          if (Update.end()) {
            if (Update.isFinished()) {
              tft.fillScreen(TFT_BLACK);
              tft.setTextColor(TFT_GREEN, TFT_BLACK);
              tft.setTextDatum(MC_DATUM);
              tft.drawString("UPDATE SUCCESS!", tft.width() / 2, tft.height() / 2 - 10, 4);
              tft.drawString("REBOOTING...", tft.width() / 2, tft.height() / 2 + 20, 2);
              delay(2000);
              ESP.restart();
            }
          }
        }
      }
    } else {
      Serial.printf("HTTP Error code: %d\n", httpCode);
    }
    https.end();
  }

  // กรณีอัปเดตไม่สำเร็จ
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("UPDATE FAILED!", tft.width() / 2, tft.height() / 2 - 10, 4);
  tft.drawString("RETURNING...", tft.width() / 2, tft.height() / 2 + 20, 2);
  delay(2000);
  
  updateMenuShown = false;
  isUpdateTouching = false;
  tft.fillScreen(TFT_BLACK);
}

void checkTouchO2Reset() {
  if (updateMenuShown) return;

  if (ts.tirqTouched() && ts.touched()) {
    TS_Point p = ts.getPoint();
    int touchX = map(p.x, 200, 3700, 0, 320);
    int touchY = map(p.y, 200, 3800, 0, 240);

    bool touchInO2Card = (touchX >= 3 && touchX <= 210 && touchY >= 159 && touchY <= 233);

    if (!isO2Touching) {
      if (touchInO2Card) {
        isO2Touching = true;
        o2TouchStartTime = millis();
      }
    } else {
      unsigned long now = millis();
      unsigned long elapsed = now - o2TouchStartTime;
      int remainingSeconds = 3 - (elapsed / 1000);
      if (remainingSeconds < 0) remainingSeconds = 0;

      o2ResetMenuShown = true;

      spr.createSprite(220, 90);
      spr.fillSprite(TFT_BLACK);
      spr.drawRoundRect(0, 0, 220, 90, 8, TFT_YELLOW);
      
      spr.setTextColor(TFT_YELLOW, TFT_BLACK);
      spr.setTextDatum(TC_DATUM);
      spr.drawString("RESET O2 SENSOR?", 110, 10, 2);
      
      spr.setTextColor(TFT_WHITE, TFT_BLACK);
      spr.setTextDatum(MC_DATUM);
      spr.drawString(String(remainingSeconds) + " SECONDS", 110, 52, 4);
      
      spr.pushSprite(50, 75);
      spr.deleteSprite();

      if (elapsed >= 3000) {
        Serial.println("O2 Sensor reset triggered! Clearing saved O2 config and restarting...");
        preferences.begin("obd_config", false);
        preferences.remove("saved_o2"); 
        preferences.end();

        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("O2 RESET SUCCESS!", tft.width() / 2, tft.height() / 2 - 10, 4);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("RESTARTING...", tft.width() / 2, tft.height() / 2 + 20, 2);
        
        delay(2000);
        ESP.restart(); 
      }
    }
  } else {
    if (isO2Touching) {
      isO2Touching = false;
      if (o2ResetMenuShown) {
        o2ResetMenuShown = false;
        tft.fillScreen(TFT_BLACK); 
      }
    }
  }
}

// ==========================================
// RENDER INTERFACES
// ==========================================
void renderDashboard() {
  if (isO2Touching || isUpdateTouching || showDtcPopup) return;

  int w = 100;
  int h = 74;

  bool isClosedLoop = (obd.loopStatus == 2);
  uint16_t trimColor = isClosedLoop ? TFT_GREEN : TFT_RED;

  String stftValStr = isClosedLoop ? String((int)obd.stft) : "--";
  String ltftValStr = isClosedLoop ? String((int)obd.ltft) : "--";

  float afrVal = obd.lambda * 14.7;
  String afrStr = (obd.lambda > 0) ? String(afrVal, 1) : "--";
  
  uint16_t afrValColor = TFT_GREEN; 
  if (obd.lambda > 0 && obd.lambda < 0.95) {
    afrValColor = TFT_CYAN;     
  } else if (obd.lambda > 1.05) {
    afrValColor = TFT_ORANGE;   
  }
  if (obd.lambda == 0) {
    afrValColor = TFT_WHITE;
  }

  updateGridCard(3, 3, w, h, "RPM", String((int)obd.rpm), "RPM", TFT_RED);
  updateGridCard(110, 3, w, h, "SPEED", String((int)obd.speed), "KM/H", TFT_SKYBLUE);
  updateGridCard(217, 3, w, h, "COOLANT", String((int)obd.coolant), "C", TFT_ORANGE);

  updateGridCard(3, 81, w, h, "STFT", stftValStr, "%", trimColor);
  updateGridCard(110, 81, w, h, "LTFT", ltftValStr, "%", trimColor);
  updateGridCard(217, 81, w, h, "BATTERY", String(obd.battery, 1), "V", TFT_MAGENTA);

  updateO2WideCard(3, 159, 207, h, obd.lambda);
  updateGridCard(217, 159, w, h, "AFR", afrStr, "RATIO", TFT_CYAN, afrValColor);
}

void renderDtcPopup() {
  unsigned long now = millis();
  unsigned long elapsed = now - popupStartTime;
  int remainingSec = 15 - (elapsed / 1000);
  if (remainingSec < 0) remainingSec = 0;
  
  int barWidth = map(15000 - elapsed, 0, 15000, 0, 220);

  spr.createSprite(260, 150);
  spr.fillSprite(TFT_BLACK);
  spr.drawRoundRect(0, 0, 260, 150, 10, TFT_RED); 
  
  spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  spr.setTextDatum(TC_DATUM);
  spr.drawString("CHECK ENGINE!", 130, 10, 4);
  
  for (int i = 0; i < dtcCount && i < 2; i++) {
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    spr.drawString(dtcCodes[i], 130, 40 + (i * 20), 4);
  }
  
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.drawString("HOLD 2S TO DISMISS (" + String(remainingSec) + "s)", 130, 95, 1);
  
  spr.drawRect(20, 125, 220, 10, TFT_DARKGREY);
  spr.fillRect(20, 125, barWidth, 10, TFT_RED);

  spr.pushSprite(30, 45);
  spr.deleteSprite();
}

void updateGridCard(int x, int y, int w, int h, const char* label, String valStr, const char* unit, uint16_t accentColor, uint16_t valColor) {
  spr.createSprite(w, h);
  spr.fillSprite(TFT_BLACK);

  spr.drawRoundRect(0, 0, w, h, 6, accentColor);

  spr.setTextColor(TFT_SILVER, TFT_BLACK);
  spr.setTextDatum(TC_DATUM);
  spr.drawString(label, w / 2, 4, 2);

  spr.setTextColor(valColor, TFT_BLACK);
  spr.setTextDatum(MC_DATUM);
  spr.drawString(valStr, w / 2, h / 2 + 2, 4);

  spr.setTextColor(accentColor, TFT_BLACK);
  spr.setTextDatum(BC_DATUM);
  spr.drawString(unit, w / 2, h - 4, 2);

  spr.pushSprite(x, y);
  spr.deleteSprite();
}

void updateO2WideCard(int x, int y, int w, int h, float lambdaVal) {
  spr.createSprite(w, h);
  spr.fillSprite(TFT_BLACK);

  uint16_t statusColor = TFT_GREEN;
  String statusText = "IDEAL";
  
  if (lambdaVal > 0 && lambdaVal < 0.95) {
    statusColor = TFT_CYAN;     
    statusText  = "RICH";
  } else if (lambdaVal > 1.05) {
    statusColor = TFT_ORANGE;   
    statusText  = "LEAN";
  }

  spr.drawRoundRect(0, 0, w, h, 6, statusColor);

  spr.setTextColor(TFT_SILVER, TFT_BLACK);
  spr.setTextDatum(TC_DATUM);
  spr.drawString("O2 LAMBDA STATUS", w / 2, 4, 2);

  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.setTextDatum(MC_DATUM);
  spr.drawString(String(lambdaVal, 2), 52, h / 2 + 6, 4);

  spr.setTextColor(statusColor, TFT_BLACK);
  spr.setTextDatum(MC_DATUM);
  spr.drawString(statusText, 155, h / 2 + 6, 4);

  spr.pushSprite(x, y);
  spr.deleteSprite();
}

void initTWAI() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
}

void sendOBDRequest(uint8_t pid) {
  twai_message_t message;
  message.identifier = 0x7DF;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 8;
  message.data[0] = 0x02;
  message.data[1] = 0x01;
  message.data[2] = pid;
  for (int i = 3; i < 8; i++) message.data[i] = 0x55;
  twai_transmit(&message, pdMS_TO_TICKS(10));
}

void readCANResponse() {
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(5)) == ESP_OK) {
    if (message.identifier == 0x7E8 && message.data[1] == 0x41) {
      uint8_t pid = message.data[2];
      uint8_t A = message.data[3];
      uint8_t B = message.data[4];

      switch (pid) {
        case PID_LOOP_STATUS: obd.loopStatus = A; break;
        case PID_COOLANT:     obd.coolant = A - 40; break;
        case PID_RPM:         obd.rpm = ((A * 256.0) + B) / 4.0; break;
        case PID_SPEED:       obd.speed = A; break;
        case PID_BATTERY:     obd.battery = ((A * 256.0) + B) / 1000.0; break;
        case PID_STFT:        obd.stft = (A - 128) * 100.0 / 128.0; break;
        case PID_LTFT:        obd.ltft = (A - 128) * 100.0 / 128.0; break;
        
        case 0x24:
        case 0x34:
          if (pid == activeO2Pid) {
            obd.lambda = ((A * 256.0) + B) / 32768.0;
            lastO2RxTime = millis(); 
            
            static uint8_t lastSavedPid = 0;
            if (lastSavedPid != activeO2Pid) {
              preferences.begin("obd_config", false);
              preferences.putUChar("saved_o2", activeO2Pid);
              preferences.end();
              lastSavedPid = activeO2Pid;
              Serial.printf("Auto-saved working O2 PID 0x%02X to Flash memory!\n", activeO2Pid);
            }
          }
          break;

        case 0x14:
          if (pid == activeO2Pid) {
            float voltage = (A / 200.0);
            obd.lambda = voltage; 
            lastO2RxTime = millis();
            
            static uint8_t lastSavedPid = 0;
            if (lastSavedPid != activeO2Pid) {
              preferences.begin("obd_config", false);
              preferences.putUChar("saved_o2", activeO2Pid);
              preferences.end();
              lastSavedPid = activeO2Pid;
              Serial.printf("Auto-saved working O2 PID 0x%02X to Flash memory!\n", activeO2Pid);
            }
          }
          break;
      }
    }
    else if (message.identifier == 0x7E8 && message.data[1] == 0x43) {
      int numBytes = message.data[0] - 1;
      int numDtcs = numBytes / 2;
      dtcCount = 0;
      
      for (int i = 0; i < numDtcs && i < 5; i++) {
        decodeDtc(message.data[2 + (i * 2)], message.data[3 + (i * 2)]);
      }
      
      if (dtcCount > 0) {
        showDtcPopup = true;
        popupStartTime = millis();
        Serial.printf("DTC Detected! Found %d codes.\n", dtcCount);
      }
    }
  }
}
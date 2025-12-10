#include <FS.h>
#include <HTTPClient.h>
#include <WiFi.h>
#define DEST_FS_USES_SPIFFS
#include <Arduino.h>
#include <ArduinoJson.h>
#include <GxEPD2_7C.h>
#include <MQTT.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <esp32fota.h>
#include <qrcode.h>
#include <rom/rtc.h>

#include "Adafruit_GFX.h"
#include "EEPROM.h"
#include "SPIFFS.h"
#include "Ticker.h"
#include "driver/rtc_io.h"
#include "kxtj3-1057.h"
#include "secrets.h"

#define DEBUG 1

#if DEBUG
#define PRINTS(s)         \
   do {                   \
      Serial.print(F(s)); \
   } while (false)
#define PRINT(s, v)       \
   do {                   \
      Serial.print(F(s)); \
      Serial.print(v);    \
   } while (false)
#else
#define PRINTS(s)
#define PRINT(s, v)
#endif

#ifdef ARDUHAL_LOG_LEVEL
#undef ARDUHAL_LOG_LEVEL
#define ARDUHAL_LOG_LEVEL ARDUHAL_LOG_LEVEL_NONE
#endif
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex

// E-Paper Pins
#define BUSY_PIN 18
#define RST_PIN 1
#define DC_PIN 19
#define CS_EPD_PIN 20

// flash
#define CS_FLASH_PIN 21

// SPI Pins
#define SCK_PIN 15
#define MOSI_PIN 4  // DIN
#define MISO_PIN 5  // DIN

// I2C
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7
#define INT_PIN 0
#define ACC_ADDR 0x0E
#define USBC_ADDR 0x21

// Pins Periphery
#define LED_PIN 14
#define BAT_VOLT_SENSE_PIN 2
#define BAT_VOLT_EN_PIN 3
#define BUTTON_PIN 9
#define CHG_EN_PIN 22
#define CHG_STAT_PIN 23

// AWS IoT certs
const char* keyFileCons;
const char* crtFileCons;

#define EPD_WIDTH 800
#define EPD_HEIGHT 480
#define OFFSET_BITMAP 4  // how many bytes store the picture information (resolution)

#define uS_TO_S_FACTOR 1000000ULL    /* Conversion factor for micro seconds to seconds */
#define LENGTH(x) (strlen(x) + 1)    // length of char string
#define EEPROM_SIZE 1024             // EEPROM size
#define EEPROM_SETTINGS_ADR 500      // start address to store settings
#define SOFTWARE_VERSION "0.0.0"     // Software version
#define EPD_TYPE_IDENTIFIER "epd7-"  // Type of device (screen type)
#define OTA_URL ENV_OTA_URL
#define OTA_URL_DEV ENV_OTA_URL_DEV
#define DEFAULT_WIFI_PW ENV_WIFI_PW_DEPLOY
#define DEFAULT_WIFI_SSID ENV_WIFI_SSID_DEPLOY
#define DEFAULT_SLEEP 3600          // Default time how long to sleep after update
#define WIFI_INIT_TIME 300          // Time how long to wait for ESPTouch to finish
#define APPCONNECT_INIT_TIME 300    // Time how long to wait for Onboarding to finish
#define FAILSAVE_TIMER 180          // Time to shutdown the device if no reaction or hangup
#define MAX_RECONNECTS 10           // (default 10) maximum wifi reconnect loops with wifi configured until 24h deep sleep
#define RECONNECT_LOOP_TIME 60      // (default 60) how many seconds the reconnect is active to set new wifi password
#define BAT_LOW_VALUE 4200          // value to show battery low
#define BAT_OFF_VALUE 4000          // value to not boot anymore
#define VDD_CORRECTION_FACTOR 2.30  // factor to get real VDD voltage from measured value

#if DEBUG
const bool DEBUG_FLAG = true;
#else
const bool DEBUG_FLAG = false;
#endif

#define FONT_MAIN u8g2_font_helvB24_tf  // Font for main text
#define FONT_BIG u8g2_font_helvB14_tf   // Font for big text
#define FONT_NORMAL u8g2_font_helvB12_tf
#define FONT_SMALL u8g2_font_helvR08_tf
#define FONT_INFO u8g2_font_7x14_tf
#define FONT_VERSION u8g2_font_tom_thumb_4x6_tf

struct wifiSettings {
   String bleSSID;  // Seconds to Sleep after update done
   String blePASS;  // Color Settings for EPD
   String ssid;     // string variable to store ssid
   String pss;      // string variable to store password
   int wifiQuality;
   uint8_t wifiRetries;
   bool wifiIsConnected;
   bool wifiOnboardingFailed;
   bool wifiConfig;  // Set true if there was a wifi config done
   bool bleInitOk;
   bool isDeployWifi;
   String clientId;
} wifiSettings = {
    .bleSSID = "",
    .blePASS = "",
    .ssid = "",
    .pss = "",
    .wifiQuality = 0,
    .wifiRetries = 0,
    .wifiIsConnected = false,
    .wifiOnboardingFailed = false,
    .wifiConfig = false,
    .bleInitOk = false,
    .isDeployWifi = false,
    .clientId = "",
};

// display extra infos on top of all screens
struct displayInfo {
   bool version;
   bool batteryInfo;
   bool batteryLowBig;
   bool wifiSignal;
   bool deviceInfoString;
   bool wifiOfflineBig;
} displayInfos = {
    .version = false,
    .batteryInfo = false,
    .batteryLowBig = false,
    .wifiSignal = false,
    .deviceInfoString = false,
    .wifiOfflineBig = false};

struct displaySettings {
   uint8_t rotationText;
   uint8_t rotationPicture;
   bool quickRefresh;
   int displayQuickRefreshTime;
} displaySettings = {
    .rotationText = 3,
    .rotationPicture = 2,
    .quickRefresh = true,
    .displayQuickRefreshTime = 960};

// settings set via MQTT/IOT
struct settings {
   int timeout;       // Seconds to Sleep after update done
   String lut;        // Color Settings for EPD
   bool clearscreen;  // if clear screen before update
   bool showBatteryWarning;
   bool showWifiWarning;
} settings = {.timeout = 3600, .lut = "default", .clearscreen = true, .showBatteryWarning = true, .showWifiWarning = true};

// system read data
struct systemData {
   int vddValue;
   bool usbConnected;
   int deviceOrientation;
} systemData = {
    .vddValue = 5000,
    .usbConnected = true,
    .deviceOrientation = 0};

esp_sleep_wakeup_cause_t wakeup_reason;
using DisplayType = GxEPD2_7C<GxEPD2_730c_GDEP073E01, GxEPD2_730c_GDEP073E01::HEIGHT / 4>;
DisplayType display(GxEPD2_730c_GDEP073E01(/*CS=*/CS_EPD_PIN, /*DC=*/DC_PIN, /*RST=*/RST_PIN, /*BUSY=*/BUSY_PIN));  // Waveshare 5.65" 7-color
KXTJ3 myIMU(ACC_ADDR);                                                                                              // Address can be 0x0E or 0x0F
esp32FOTA myEsp32FOTA("esp32-fota-http", SOFTWARE_VERSION);
QRCode QR;
WiFiClientSecure net = WiFiClientSecure();
MQTTClient client(256);  // Buffer to 256 Byte
Preferences preferences;
U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;
Ticker onceTicker;
Ticker perdiodicLed;
Ticker onceDisplay;
SerialFlashFile saveFile;

NimBLECharacteristic* wifiConnectedCharacteristic;
NimBLEAdvertising* pAdvertising;
static NimBLEServer* pServer;

struct dataLayout {
   int integer;
   char byte[4];
};

char HOST_ADDRESS[] = ENV_AWS_IOT_ENDPOINT;
const uint8_t QR_VERSION = 3;     // QR Code Version
const uint8_t QR_QUIET_ZONE = 4;  // quiet zone all around

uint32_t freeHeap = 0;
int newVersionSave = 0;
int httpFileSize = 0;
int StartCounter = 0;

char CLIENT_ID[20];
char CLIENT_KEY[30];
char TOPIC_RECEIVE[64];
char DL_URL[50];

bool downloadStart = true;
bool deviceActivated = false;
bool deviceActivationNotStarted = false;
bool deviceActivationReset = false;
bool isNewVersion = false;
bool isDlUrl = false;
bool periodicLedIsOn = false;
bool epaperIsUpdating = false;
bool isTestMode = false;  // enabled if the device is in deployment and tests are running
bool buttonWake = false;

bool awsConnect(bool connect);
bool getVersionUpdate(void);
int startupCounter(bool reset);
bool resetAll(bool resetActivation, bool resetWifi);
bool wifiConnectionLostStore(bool setNoWifi = false, bool reset = false);
bool getActivatedFromMem(void);
void debugFS(void);
bool BleInit(String deviceId, bool enable);
bool setUpdateState(String state);
int storeSleepTimeMem(int updateTime = 0);
void gotToDeepSleep(int seconds, bool showScreen = true, bool motionWake = true);
bool updateDisplayAsync(String functionName);
void displayOverlays(DisplayType& display, displayInfo displayData, bool invertColors, bool fullcolor = false);
int setImageFromFS(String fileName);
void displayDebugInfo();
void displayWipe(bool quick);
void displaySetText(String info, bool blackBoard, bool quickRefresh = true);
void displaySetBlankTest(int offsetVar, bool doQuickRefresh);
void displayNoPicture();
void displayTurnOn();
void displayUpdatePicture();
void displayWifiActivate(bool wifiProvisioningDone);
bool waitDisplayComplete(bool quick);
bool accIntSet(int sensity);
bool chargeMode(bool enable);
bool usbInit();
bool usbCheckConnect();
void debugCheck();

void WiFiEvent(WiFiEvent_t event) {
   if (DEBUG_FLAG)
      // Serial.printf("[WiFi-event] event: %d\n", event);

      switch (event) {
         case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            if (DEBUG_FLAG) Serial.println("[NETWORK] WiFi connected");
            Serial.printf("[NETWORK] IP address: ");
            Serial.println(WiFi.localIP());

            if (wifiSettings.wifiIsConnected != true) {
               wifiSettings.wifiIsConnected = true;
               if (wifiSettings.bleInitOk) wifiConnectedCharacteristic->setValue(wifiSettings.wifiIsConnected);
            }

            break;
         case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("[NETWORK] WiFi lost connection");
            wifiSettings.wifiOnboardingFailed = true;
            if (wifiSettings.wifiIsConnected != false) {
               wifiSettings.wifiIsConnected = false;
               if (wifiSettings.bleInitOk) wifiConnectedCharacteristic->setValue(wifiSettings.wifiIsConnected);
            }
            break;
      }
}

void timeoutFailsave(int time) {
   Serial.println("printing in once function.");
   if (getActivatedFromMem()) {
      if (settings.timeout > 0) {
         gotToDeepSleep(settings.timeout);  // Disable version Check here

      } else {
         gotToDeepSleep(DEFAULT_SLEEP);
      }
   } else {
      gotToDeepSleep(0, true, false);  // Disable version Check here
   }
}

void iotReceiveHandler(String& topic, String& payload) {
   if (DEBUG_FLAG) Serial.print("[IOT RX] Message Received: ");
   if (DEBUG_FLAG) Serial.println(payload);
   JsonDocument doc;
   deserializeJson(doc, payload);

   if (doc["url"].is<JsonString>()) {
      Serial.println("[IOT RX] URL Message");
      const char* dlurl = doc["url"];
      sprintf(DL_URL, "%s%s", ENV_DOWNLOAD_URL, dlurl);
      isDlUrl = true;
   }

   if (doc["t"].is<long>()) {
      newVersionSave = doc["t"].as<long>();
      isNewVersion = true;
   }

   if (doc["act"].is<JsonString>()) {
      const char* activated;
      activated = doc["act"];
      if (strcmp(activated, "activated") == 0) {
         Serial.println("[IOT RX] Device is activated");
         if (doc["key"].is<JsonString>()) {
            const char* devicekey = doc["key"];
            sprintf(CLIENT_KEY, "%s", devicekey);
            // TODO: Maybe use this for activation
         }
         if (doc["lut"].is<JsonString>()) {
            const char* lut = doc["lut"];
            settings.lut = lut;
         }
         if (doc["timeout"].is<int>()) {
            settings.timeout = doc["timeout"].as<int>();
         }
         if (doc["clearscreen"].is<bool>()) {
            settings.clearscreen = doc["clearscreen"].as<bool>();
         }
         if (doc["overlay"].is<bool>()) {
            bool showOverlay = doc["overlay"].as<bool>();
            if (showOverlay) {
               settings.showWifiWarning = true;
               settings.showBatteryWarning = true;
            } else {
               settings.showWifiWarning = false;
               settings.showBatteryWarning = false;
            }
         }
         Serial.printf("[IOT RX] SETTINGS - Sleep: %d Lut: %s Overlay: %d-%d \n", settings.timeout, settings.lut, settings.showWifiWarning, settings.showBatteryWarning);
         deviceActivated = true;
      }
      if (strcmp(activated, "not_started") == 0) {
         Serial.println("[IOT RX] Device activation not started");
         deviceActivationNotStarted = true;
      }
      if (strcmp(activated, "reset") == 0) {
         deviceActivationReset = true;
      }
   }
}

void ledBlinkFunction() {
   periodicLedIsOn = !periodicLedIsOn;
   digitalWrite(LED_PIN, periodicLedIsOn);
}

// set blink with timeout in ms
void ledBlink(int timeout, bool on) {
   if (on) {
      perdiodicLed.detach();
      if (timeout <= 0) {
         delay(timeout + 100);
         digitalWrite(LED_PIN, HIGH);
         return;
      }
      perdiodicLed.attach_ms(timeout, ledBlinkFunction);
   } else {
      perdiodicLed.detach();
      delay(timeout + 100);
      digitalWrite(LED_PIN, LOW);
   }
}

// inputs: nopicture,turnon,updatepicture,wifiactivate,deviceactivate
void updateDisplayAsyncFunction(int functionNumber) {
   epaperIsUpdating = true;
   if (functionNumber == 1) {
      displayNoPicture();
   }
   if (functionNumber == 2) {
      displayTurnOn();
   }
   if (functionNumber == 3) {
      displayUpdatePicture();
   }
   if (functionNumber == 4) {
      displayWifiActivate(false);
   }
   if (functionNumber == 5) {
      displayWifiActivate(true);
   }

   epaperIsUpdating = false;
   return;
}

// set blink with timeout in ms
// inputs: nopicture,turnon,updatepicture,wifiactivate,deviceactivate
bool updateDisplayAsync(String functionName) {
   int functionNumber = 0;
   if (functionName == "nopicture") {
      functionNumber = 1;
   }
   if (functionName == "turnon") {
      functionNumber = 2;
   }
   if (functionName == "updatepicture") {
      functionNumber = 3;
   }
   if (functionName == "wifiactivate") {
      functionNumber = 4;
   }
   if (functionName == "deviceactivate") {
      functionNumber = 5;
   }

   if (epaperIsUpdating || functionNumber < 1) {
      return false;
   }
   onceDisplay.once_ms(100, updateDisplayAsyncFunction, functionNumber);
   return true;
}

String mac2String(const uint64_t mac) {
   byte ar[6];
   uint8_t* p = ar;

   for (int8_t i = 0; i <= 5; i++) {
      *p++ = mac >> (CHAR_BIT * i);
   }

   String s;
   for (int8_t i = 0; i < 6; i++) {
      char buf[2];
      sprintf(buf, "%02X", ar[i]);
      s += buf;
      if (i < 5)
         s += ':';
   }
   s += '\0';
   return s;
}

int readVDD(bool singleReading) {
   pinMode(BAT_VOLT_EN_PIN, OUTPUT);
   pinMode(BAT_VOLT_SENSE_PIN, INPUT);
   digitalWrite(BAT_VOLT_EN_PIN, LOW);
   delay(3);
   int retries = 1;
   if (!singleReading) {
      retries = 10;
   }
   float rawValue = 0;
   int count = 0;
   for (size_t i = 0; i < retries; i++) {
      float readValue = analogRead(BAT_VOLT_SENSE_PIN);
      delayMicroseconds(18000);
      rawValue += readValue;
      count++;
   }
   if (count > 0) {
      rawValue = rawValue / count;
   }
   pinMode(BAT_VOLT_EN_PIN, INPUT);
   return int(round(rawValue * VDD_CORRECTION_FACTOR));
}

bool EepromInit(int size) {
   if (!EEPROM.begin(size)) {  // Init EEPROM
      Serial.println("[MEM] failed to init EEPROM");
      delay(1000);
      return false;
   } else {
      Serial.println("[MEM] EEPROM init OK");
      return true;
   }
}
bool EepromClear() {
   char resetValue = 0;
   for (int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(i, resetValue);
   }
   Serial.println("[MEM] EEPROM clear OK");
   return true;
}

void writeStringToFlash(const char* toStore, int startAddr) {
   int i = 0;
   for (; i < LENGTH(toStore); i++) {
      EEPROM.write(startAddr + i, toStore[i]);
   }
   EEPROM.write(startAddr + i, '\0');
   EEPROM.commit();
}

String readStringFromFlash(int startAddr) {
   char in[128];  // char array of size 128 for reading the stored data
   int i = 0;
   for (; i < 128; i++) {
      in[i] = EEPROM.read(startAddr + i);
   }
   return String(in);
}

int readIntFromFlash(int addr) {
   dataLayout data3;
   EEPROM.get(addr, data3);
   return data3.integer;
}

void writeIntToFlash(int value, int startAddr) {
   dataLayout data2;
   data2.integer = value;
   EEPROM.put(startAddr, data2);
   EEPROM.commit();
}

void saveSettingsToFlash(int startAddr) {
   writeIntToFlash(settings.clearscreen, startAddr);
   writeIntToFlash(settings.showBatteryWarning, startAddr + 5);
   writeIntToFlash(settings.showWifiWarning, startAddr + 10);
   Serial.println("[MEM] Settings saved to EEPROM");
}

void restoreSettingsToFlash(int startAddr) {
   settings.clearscreen = readIntFromFlash(startAddr);
   settings.showBatteryWarning = readIntFromFlash(startAddr + 5);
   settings.showWifiWarning = readIntFromFlash(startAddr + 10);
   Serial.println("[MEM] Settings restored from EEPROM");
   if (DEBUG_FLAG) {
      Serial.printf("[MEM] Settings - ClearScreen: %d BatteryWarning: %d WifiWarning: %d \n", settings.clearscreen, settings.showBatteryWarning, settings.showWifiWarning);
   }
}

// charger active, refresh screen, scan wifi, ble adv
void emvTest(int counter) {
   bool isUsb = false;
   bool isCharging = false;
   if (usbCheckConnect()) {
      isCharging = chargeMode(true);
      isUsb = true;
   } else {
      isCharging = chargeMode(false);
      isUsb = false;
   }
   char softwareStringDebug[256];
   sprintf(softwareStringDebug, "Device Test (%s) USB:%d Charger:%d Loop: %d", SOFTWARE_VERSION, isUsb, isCharging, counter);
   displaySetText(softwareStringDebug, false);
   WiFi.mode(WIFI_STA);
   delay(100);
   Serial.println("[NETWORK] scan start");
   int n = WiFi.scanNetworks();
   if (n == 0) {
      Serial.println("no networks found");
   } else {
      Serial.print(n);
      Serial.println(" networks found");

      for (int i = 0; i < n; ++i) {
         Serial.print(i);
         Serial.print(": ");
         Serial.print(WiFi.SSID(i));
         Serial.print(" -> ");
         Serial.println(WiFi.RSSI(i));
      }
   }
}

bool wifiSmart() {
   bool isWifi = true;
   bool isReconnect = false;
   uint8_t reconnectCount = readIntFromFlash(140);
   if (reconnectCount > MAX_RECONNECTS + 1) {
      writeIntToFlash(0, 140);
      Serial.printf("[NETWORK] Fix Reconnect Counter Value and save to MEM \n");
   }

   wifiSettings.ssid = readStringFromFlash(0);  // Read SSID stored at address 0
   wifiSettings.pss = readStringFromFlash(40);  // Read Password stored at address 40
   if (wifiSettings.ssid.length() > 40 || wifiSettings.ssid.length() == 0) {
      wifiSettings.ssid = DEFAULT_WIFI_SSID;
      isWifi = false;
   }
   if (wifiSettings.pss.length() > 66 || wifiSettings.pss.length() == 0) {
      wifiSettings.pss = DEFAULT_WIFI_PW;
      isWifi = false;
   }

#if DEBUG
   Serial.printf("[NETWORK] SSID: %s (%d) PW: %s (%d)\n", wifiSettings.ssid.c_str(), wifiSettings.ssid.length(), wifiSettings.pss.c_str(), wifiSettings.pss.length());

#endif

   bool doReset = false;
   if (StartCounter >= 5) {
      doReset = true;
   }
   if (isWifi == false && isTestMode == false && doReset == false) updateDisplayAsync("wifiactivate");
   // try to connect to current wifi
   wifiSettings.wifiRetries = 0;
   for (int i = 0; i < 5; i++) {
      wifiSettings.wifiRetries++;
      Serial.printf("[NETWORK] connect try cont %d / %d\n", i + 1, 5);
      if (doReset && i >= 3) {
         break;  // leave wifi search quick on reset
      }
      if (waitDisplayComplete(true) && i >= 3 && isWifi == false) {
         BleInit(CLIENT_ID, true);  // needs to wait for display, otherwise scan no results
      }
      if (wifiSettings.ssid.length() < 2) {
         Serial.println("[NETWORK] Stop connect because no wifi set");
         break;
      }
      // if (i == 0) WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.pss.c_str());
      WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.pss.c_str());
      WiFi.waitForConnectResult();
      if (WiFi.status() == WL_CONNECTED) {
         break;
      }
      WiFi.disconnect(true);
      delay(1000);
      // WiFi.setSleep(true);
      /*
      IPAddress primaryDNS(8, 8, 8, 8);    // optional
      IPAddress secondaryDNS(1, 1, 1, 1);  // optional
      if (!WiFi.config("", "", "", primaryDNS, secondaryDNS)) {
         Serial.println("STA Failed to configure");
      }
      // WiFi.disconnect(true);*/
      // WiFi.disconnect(true);
      // delay(1000);
      // WiFi.reconnect();
      if (!isWifi && i >= 3 && waitDisplayComplete(true)) {
         Serial.println("[NETWORK] stop search because default wifi");  // skip the intense connect if default wifi
         break;
      }
   }
   if (isTestMode) return true;

   wifiSettings.wifiQuality = WiFi.RSSI();

   if (doReset && buttonWake) resetAll(true, true);

   // Serial.printf("[NETWORK] Wifi Connect done : %d\n", (WiFi.status() == WL_CONNECTED));
   //  check if reset is triggered via button
   // if wifi was connected in the past (device is activated but no wifi)
   // restart bluetooth to set new wifi
   bool testActivation = getActivatedFromMem();
   Serial.printf("[DEBUG] Activation Test: %d Button: %d\n", testActivation, buttonWake);

   if (WiFi.status() != WL_CONNECTED) startupCounter(true);  // only reset startup counter if no wifi. otherwise wait longer
   if (WiFi.status() != WL_CONNECTED && getActivatedFromMem() && buttonWake == true) {
      Serial.println("[NETWORK] BLE reprovisioning for wifi reconnect");
      BleInit(CLIENT_ID, true);
      int loopCount = 0;
      // loop to check bluetooth wifi change
      while (WiFi.status() != WL_CONNECTED) {
         loopCount++;
         delay(1000);
         Serial.print("o");
         if (loopCount > RECONNECT_LOOP_TIME) {
            Serial.printf("\n[NETWORK] BLE reprovisioning is over, sleep: %d s\n", settings.timeout);
            break;
         }
         if (wifiSettings.bleSSID.length() > 1 && wifiSettings.blePASS.length() > 1) {
            Serial.println("[NETWORK] reprovisioning got all BLE, try");
            writeIntToFlash(0, 140);  // reset reconnect counter
            isReconnect = true;
            break;
         }
      }
   }
   if (isReconnect || (WiFi.status() != WL_CONNECTED && !getActivatedFromMem()))  // if WiFi is not connected
   {
      wifiSettings.wifiOnboardingFailed = false;
      Serial.println("[NETWORK] wait for wifi via ble...");
      int countAttempt = 0;
      int failedCounter = 0;
      while (true) {
         countAttempt++;
         Serial.print(".");
         delay(250);
         if (countAttempt % (4 * 10) == 0) {
            // this gets triggered every 10 seconds on provisioning
            Serial.printf("[NETWORK] Provisioning attempt %d/%d \n", countAttempt, (WIFI_INIT_TIME * 4));
         }
         // if nothing gets set stop on some point
         if (countAttempt > (WIFI_INIT_TIME * 4)) {
            // break if wifi config fails
            BleInit(CLIENT_ID, false);  // end BLE
            gotToDeepSleep(0, true, false);
            return false;
         }
         if (wifiSettings.wifiOnboardingFailed) {
            failedCounter++;
            if (failedCounter > 10) {
               displaySetText("Bitte die WLAN Daten überprüfen!", false);
               return false;
            }
         }
         if (WiFi.status() == WL_CONNECTED) {
            Serial.println("[NETWORK] Wifi got IP (softAP)");
            break;
         }
         // try to connect wifi if ble data is set
         if (wifiSettings.bleSSID.length() > 1 && wifiSettings.blePASS.length() > 1) {
            Serial.println("[NETWORK] got all BLE, try to connect with data");
            WiFi.begin(wifiSettings.bleSSID.c_str(), wifiSettings.blePASS.c_str());
            WiFi.waitForConnectResult();
            break;
         }
      }
      Serial.println("");

      Serial.println("[NETWORK] Waiting for WiFi");
      countAttempt = 0;
      int wifiRetryCount = 0;
      while (WiFi.status() != WL_CONNECTED) {
         countAttempt++;
         wifiRetryCount++;
         if (countAttempt >= 45) {
            Serial.println("[NETWORK] Wifi config failed");
            displaySetText("Bitte die WLAN Daten überprüfen!", false);
            return false;
         }
         Serial.print("*");
         delay(500);
         if (wifiRetryCount > 20) {
            wifiRetryCount = 0;
            if (wifiSettings.bleSSID.length() > 1 && wifiSettings.blePASS.length() > 1) {
               WiFi.begin(wifiSettings.bleSSID.c_str(), wifiSettings.blePASS.c_str());
            } else {
               WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.pss.c_str());
            }
            WiFi.waitForConnectResult();
            if (WiFi.status() == WL_CONNECTED) {
               break;
            }
         }
      }
      Serial.println("[NETWORK] WiFi Connected.");
      wifiSettings.ssid = WiFi.SSID();
      wifiSettings.pss = WiFi.psk();
      int wifiNameLength = wifiSettings.ssid.length();
      int wifiPssLength = wifiSettings.pss.length();
#if DEBUG
      Serial.printf("[NETWORK] SSID: \"%s\" PW: \"%s\"\n", wifiSettings.ssid.c_str(), wifiSettings.pss.c_str());
      Serial.printf("[NETWORK] SSID Length: %d PW Length: %d\n", wifiNameLength, wifiPssLength);
#endif
      if (wifiNameLength <= 35 && wifiPssLength <= 65) {
         Serial.println("[MEM] Store SSID & PSS in Flash");
         writeStringToFlash(wifiSettings.ssid.c_str(), 0);
         writeStringToFlash(wifiSettings.pss.c_str(), 40);
      }

#if DEBUG
      String testSSID = readStringFromFlash(0);
      String testPASS = readStringFromFlash(40);
      Serial.printf("[NETWORK] From Flash: SSID: \"%s\" PW: \"%s\"\n", testSSID.c_str(), testPASS.c_str());
#endif
      startupCounter(true);
      BleInit(CLIENT_ID, false);
      if (!isReconnect) {
         resetAll(false, false);          // dont reset the device if the device was there and was just reconnected
         wifiSettings.wifiConfig = true;  // only set this on fresh config, not on reconnect (device will be deacivated if true)
      }
      writeIntToFlash(0, 170);  // Reset activation counter
      return true;
   } else if (WiFi.status() == WL_CONNECTED) {
      Serial.println("[NETWORK] WiFi Connected and activated");
      if (reconnectCount > 0) {
         Serial.println("[NETWORK] Reset reconnect Counter");
         writeIntToFlash(0, 140);
      }
      if (!isWifi && WiFi.SSID() == DEFAULT_WIFI_SSID) {
         wifiSettings.isDeployWifi = true;
      }
      if (wifiConnectionLostStore()) {
         Serial.println("[NETWORK] Clear wifi connection lost flag and reload image");
         resetAll(false, false);  // reset to reload image
      }
      BleInit(CLIENT_ID, false);
      return true;
   }
   Serial.println("[NETWORK] WiFi Connect failed");
   restoreSettingsToFlash(EEPROM_SETTINGS_ADR);
   settings.timeout = storeSleepTimeMem();
   if (getActivatedFromMem()) {
      if (reconnectCount < MAX_RECONNECTS) {
         reconnectCount++;
         writeIntToFlash(reconnectCount, 140);
         Serial.printf("[NETWORK] Activated but Wifi connect failed(%d/%d) - going to sleep %d \n", reconnectCount, MAX_RECONNECTS, settings.timeout);
         gotToDeepSleep(settings.timeout, false, true);

      } else {
         Serial.printf("[NETWORK] Activated but Wifi connect failed ended - sleep 12h \n");
         writeIntToFlash(0, 140);
         displayInfos.wifiOfflineBig = true;
         setImageFromFS("tmp.gz");  // reload the last image and show no wifi overlay
         wifiConnectionLostStore(true);
         delay(2000);
         gotToDeepSleep(43000, false, false);
      }
   } else {
      gotToDeepSleep(0, true, false);
   }
   return false;
}

String getRedirect(String url) {
   HTTPClient http;
   http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
   if (url.indexOf("https://") > 0) {
      http.begin(url, cert);
   } else {
      http.begin(url);
   }
   const char* headerkeys[] = {"Location"};
   size_t headerkeyssize = sizeof(headerkeys) / sizeof(char*);
   http.collectHeaders(headerkeys, headerkeyssize);
   int httpCode = http.GET();

   if (httpCode > 0) {
      if (httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
         String urlNew = http.header("Location").c_str();
         Serial.println("[NETWORK] Redirect with 301");
         http.end();
         return urlNew;
      }
   }
   return url;
}

class ServerCallbacks : public NimBLEServerCallbacks {
   void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
      Serial.printf("\n[BLE]Client address: %s\n", connInfo.getAddress().toString().c_str());
      pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
      if (wifiSettings.bleInitOk) {
         NimBLEDevice::startAdvertising();
      }
   }
   void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
      Serial.printf("\n[BLE] Client disconnected");
      if (wifiSettings.bleInitOk) {
         Serial.printf("\n[BLE] restart advertising...\n[");
         NimBLEDevice::startAdvertising();
      }
   };
   void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
      Serial.printf("\n[BLE] MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
   }

} serverCallbacks;

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
   void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
      Serial.printf("%s : onRead(), value: %s\n",
                    pCharacteristic->getUUID().toString().c_str(),
                    pCharacteristic->getValue().c_str());
   };

   void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
      char charType[128];
      char charValue[128];
      sprintf(charType, "%s", pCharacteristic->getUUID().toString().c_str());
      sprintf(charValue, "%s", pCharacteristic->getValue().c_str());
      Serial.printf("%s : onWrite(), value: %s\n",
                    pCharacteristic->getUUID().toString().c_str(),
                    pCharacteristic->getValue().c_str());

      if (strcmp("a62eed84-7b0d-11ed-a1eb-0242ac120002", charType) != 0) {
         Serial.printf("[BLE] Set wifi SSID: %s\n", charValue);
         wifiSettings.bleSSID = pCharacteristic->getValue();
      }
      if (strcmp("090b0ef2-7b0d-11ed-a1eb-0242ac120002", charType) != 0) {
         Serial.printf("[BLE] Set wifi PASS: %s\n", charValue);
         wifiSettings.blePASS = pCharacteristic->getValue();
      }
   };

   void onStatus(NimBLECharacteristic* pCharacteristic, int code) override {
      Serial.printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
   }
} chrCallbacks;

/** Handler class for descriptor actions */
class DescriptorCallbacks : public NimBLEDescriptorCallbacks {
   void onWrite(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) override {
      std::string dscVal = pDescriptor->getValue();
      Serial.printf("[BLE] Descriptor written value: %s\n", dscVal.c_str());
   }

   void onRead(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) override {
      Serial.printf("[BLE] %s Descriptor read\n", pDescriptor->getUUID().toString().c_str());
   }
} dscCallbacks;

bool BleInit(String deviceId, bool enable) {
   if (!enable) {
      if (wifiSettings.bleInitOk) {
         ledBlink(500, true);
         wifiSettings.bleInitOk = false;
         pAdvertising->stop();
         esp_bt_controller_enable(ESP_BT_MODE_BLE);
         esp_bt_controller_disable();
         esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
         // NimBLEDevice::deinit(false); //crashes with panic
         Serial.println("[BLE] BLE stopped");
      }
      return true;
   }
   if (wifiSettings.bleInitOk) {
      Serial.println("[BLE] already initialized, skip...");
      return true;
   }
   ledBlink(200, true);
   wifiSettings.bleSSID = "";
   wifiSettings.blePASS = "";
   String wifiSsidScan;
   WiFi.mode(WIFI_STA);
   delay(100);
   Serial.println("[NETWORK] scan start");
   int n = WiFi.scanNetworks();
   if (n == 0) {
      Serial.println("no networks found");
   } else {
      Serial.print(n);
      Serial.println(" networks found");
#if DEBUG
      for (int i = 0; i < n; ++i) {
         Serial.print(i);
         Serial.print(": ");
         Serial.print(WiFi.SSID(i));
         Serial.print(" -> ");
         Serial.println(WiFi.RSSI(i));
      }
#endif
      for (int i = 0; i < n; ++i) {
         wifiSsidScan = wifiSsidScan + WiFi.SSID(i) + "´";
         wifiSsidScan = wifiSsidScan + WiFi.RSSI(i) + "´´";

         if (wifiSsidScan.length() > 460) {
            wifiSsidScan = wifiSsidScan + "´...´...´";
            break;
         }
      }
   }

   NimBLEDevice::init(deviceId.c_str());
   // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
   NimBLEDevice::setSecurityAuth(false, false, true);

#ifdef ESP_PLATFORM
   NimBLEDevice::setPower(ESP_PWR_LVL_P9);
#else
   NimBLEDevice::setPower(9);
#endif

   pServer = NimBLEDevice::createServer();
   pServer->setCallbacks(&serverCallbacks);

   NimBLEService* deviceDataService = pServer->createService("7f74170e-7b0e-11ed-a1eb-0242ac120002");
   wifiConnectedCharacteristic = deviceDataService->createCharacteristic("4c578d4c-7b0e-11ed-a1eb-0242ac120002", NIMBLE_PROPERTY::READ);
   NimBLECharacteristic* wifiScanCharacteristic = deviceDataService->createCharacteristic("5131a3fc-7b0e-11ed-a1eb-0242ac120002", NIMBLE_PROPERTY::READ);

   NimBLEService* wifiDataService = pServer->createService("0515c086-7b0c-11ed-a1eb-0242ac120002");
   NimBLECharacteristic* wifiSsidCharacteristic = wifiDataService->createCharacteristic("090b0ef2-7b0d-11ed-a1eb-0242ac120002", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
   NimBLECharacteristic* wifiPwCharacteristic = wifiDataService->createCharacteristic("a62eed84-7b0d-11ed-a1eb-0242ac120002", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

   wifiSsidCharacteristic->setCallbacks(&chrCallbacks);
   wifiPwCharacteristic->setCallbacks(&chrCallbacks);
   wifiConnectedCharacteristic->setCallbacks(&chrCallbacks);
   wifiScanCharacteristic->setCallbacks(&chrCallbacks);

   NimBLE2904* wifiSsidDescriptor = (NimBLE2904*)wifiSsidCharacteristic->createDescriptor("2904");
   NimBLE2904* wifiPwDescriptor = (NimBLE2904*)wifiPwCharacteristic->createDescriptor("2904");
   NimBLE2904* wifiScanDescriptor = (NimBLE2904*)wifiScanCharacteristic->createDescriptor("2904");

   wifiSsidDescriptor->setDescription(0x00);
   wifiSsidDescriptor->setNamespace(0x00);
   wifiSsidDescriptor->setFormat(NimBLE2904::FORMAT_UTF8);
   wifiSsidDescriptor->setCallbacks(&dscCallbacks);

   wifiPwDescriptor->setDescription(0x00);
   wifiPwDescriptor->setNamespace(0x00);
   wifiPwDescriptor->setFormat(NimBLE2904::FORMAT_UTF8);
   wifiPwDescriptor->setCallbacks(&dscCallbacks);

   wifiScanDescriptor->setDescription(0x00);
   wifiScanDescriptor->setNamespace(0x00);
   wifiScanDescriptor->setFormat(NimBLE2904::FORMAT_UTF8);
   wifiScanDescriptor->setCallbacks(&dscCallbacks);

   wifiDataService->start();
   deviceDataService->start();

   wifiSsidCharacteristic->setValue("wifi-ssid");
   wifiPwCharacteristic->setValue("wifi-pw");
   wifiConnectedCharacteristic->setValue(wifiSettings.wifiIsConnected);
   wifiScanCharacteristic->setValue(wifiSsidScan);

   pAdvertising = NimBLEDevice::getAdvertising();
   pAdvertising->setName(deviceId.c_str());
   pAdvertising->addServiceUUID(wifiDataService->getUUID());
   pAdvertising->addServiceUUID(deviceDataService->getUUID());
   pAdvertising->enableScanResponse(true);
   pAdvertising->start();

   Serial.printf("[BLE] BLE Advertising started: %s \n", deviceId.c_str());
   wifiSettings.bleInitOk = true;
   return true;
}
// https://forum.arduino.cc/index.php?topic=565603.0
int downloadAndSaveFile(String fileName, String url) {
   bool success = 0;
   int systemFileSize = 0;
   HTTPClient http;
   http.setTimeout(10000);

   if (url.indexOf("https://") > 0) {
      http.begin(url, cert);
   } else {
      http.begin(url);
   }

   int httpCode = http.GET();

   if (httpCode > 0) {
      // file found at server
      if (httpCode == HTTP_CODE_OK) {
         int len = http.getSize();
         httpFileSize = len;

         if (SerialFlash.exists(fileName.c_str())) {
            Serial.println("[FLASH] Delete File");
            SerialFlashFile file = SerialFlash.open(fileName.c_str());
            file.erase();
            saveFile.close();
         }

         SerialFlash.createErasable(fileName.c_str(), httpFileSize);
         saveFile = SerialFlash.open(fileName.c_str());

         Serial.print("[DL] Download Size: ");
         Serial.println(len);
         int buff_size = 8128;
         unsigned char* buff = (unsigned char*)malloc(buff_size);
         unsigned char* file = (unsigned char*)malloc(httpFileSize);

         WiFiClient* stream = http.getStreamPtr();
         size_t downloaded_data_size = 0;
         while (http.connected() && (len > 0 || len == -1)) {
            // Available limited to 16328 bytes. Might be TLS segementation.
            size_t size = stream->available();
            if (size) {
               int c = stream->readBytes(buff, ((size > buff_size) ? buff_size : size));
               saveFile.write(buff, c);
               if (len > 0) {
                  len -= c;
               }
               downloaded_data_size += size;
            }
            delay(1);
            if (WiFi.status() != WL_CONNECTED) {
               http.end();
               saveFile.close();
               return -4;
            }
         }
         systemFileSize = saveFile.size();
         Serial.print("[FLASH] File size: ");
         Serial.println(systemFileSize);
         Serial.print("[DL] File Diff: ");
         int dif = systemFileSize - httpFileSize;
         Serial.println(dif);
         int maxDif = (httpFileSize / 80) * -1;
         if (maxDif > -2000) {
            maxDif = -5000;
         }
         Serial.print("[DL] MAX Diff: ");
         Serial.println(maxDif);
         free(buff);
         if (dif < maxDif) {
            success = -8;
         }
         saveFile.close();
      }
   } else {
      Serial.println("[DL] Error on HTTP request");
      success = -2;
   }
   http.end();
   return success;
}

uint16_t getColor(uint8_t color) {
   switch (color) {
      case 0:
         return GxEPD_BLACK;
         break;
      case 1:
         return GxEPD_BLUE;
         break;
      case 2:
         return GxEPD_GREEN;
         break;
      case 3:
         return GxEPD_RED;
         break;
      case 4:
         return GxEPD_RED;  // TODO: GxEPD_RED is used for orange in epd7 (but needs to be proper dithering)
         break;
      case 5:
         return GxEPD_YELLOW;
         break;
      case 6:
         return GxEPD_WHITE;
         break;
      default:
         return GxEPD_WHITE;
         break;
   }
}

bool waitDisplayComplete(bool quick) {
   int counter = 0;
   while (counter < 20) {
      counter++;
      if (quick) counter = 21;
      bool pinState = digitalRead(BUSY_PIN);
      if (pinState) {
         return true;
         break;
      }
#if DEBUG
      Serial.println("[EPD] Wait for Display...");
#endif
      delay(500);
   }
   return false;
}

void printQRBlock(uint16_t x, uint16_t y, uint8_t size, uint16_t col)
// Draw a square block of size pixels. Drawing individual pixels as this is faster
// that line segments and much faster that a filled rectangle.
{
   for (uint8_t i = 0; i < size; i++)
      for (uint8_t j = 0; j < size; j++)
         display.drawPixel(x + i, y + j, col);
}

// https://github.com/zenmanenergy/ESP8266-Arduino-Examples/blob/master/helloWorld_urlencoded/urlencode.ino
unsigned char h2int(char c) {
   if (c >= '0' && c <= '9') {
      return ((unsigned char)c - '0');
   }
   if (c >= 'a' && c <= 'f') {
      return ((unsigned char)c - 'a' + 10);
   }
   if (c >= 'A' && c <= 'F') {
      return ((unsigned char)c - 'A' + 10);
   }
   return (0);
}

String urldecode(String str) {
   String encodedString = "";
   char c;
   char code0;
   char code1;
   for (int i = 0; i < str.length(); i++) {
      c = str.charAt(i);
      if (c == '+') {
         encodedString += ' ';
      } else if (c == '%') {
         i++;
         code0 = str.charAt(i);
         i++;
         code1 = str.charAt(i);
         c = (h2int(code0) << 4) | h2int(code1);
         encodedString += c;
      } else {
         encodedString += c;
      }
      yield();
   }
   return encodedString;
}
// get the state of the wifi connection lost state and also set it
bool wifiConnectionLostStore(bool setNoWifi, bool reset) {
   int actualState = readIntFromFlash(200);
   if (DEBUG_FLAG) Serial.printf("[MEM] Wifi Connection lost state: %d\n", actualState);
   if (reset) {
      if (actualState != 0) writeIntToFlash(0, 200);
      return false;
   }
   if (setNoWifi) {
      if (actualState != 1) writeIntToFlash(1, 200);
      return true;
   }
   bool returnstate = false;
   if (actualState > 0) returnstate = true;
   return actualState;
}

// returns true if called first time and device is activated
// returns false if device is device is not activated or was called before
bool setActivatedToMem(bool isActivated) {
   uint8_t readActivated = readIntFromFlash(160);
   if (readActivated != isActivated) {
      writeIntToFlash(isActivated, 160);
      return true;
   }
   return false;
}
// get if the device was activated before
bool getActivatedFromMem() {
   int readActivated = readIntFromFlash(160);
   if (readActivated != 1) {
      readActivated = 0;
   }
   if (readActivated) return true;
   return false;
}

int storeSleepTimeMem(int updateTime) {
   int returnSleepTime = readIntFromFlash(210);
   if (updateTime > 0) {
      if (returnSleepTime != updateTime) {
         if (DEBUG_FLAG) Serial.printf("[MEM] Update Sleep Time to %d\n", updateTime);
         writeIntToFlash(updateTime, 210);
      }
      returnSleepTime = updateTime;
   } else {
      if (DEBUG_FLAG) Serial.printf("[MEM] Get Sleep Time: %d\n", returnSleepTime);
   }
   return returnSleepTime;
}

void debugFS() {
#if DEBUG
   Serial.printf("[MAIN] SPIFFS usage %d/%d heap free: %d/%d\n", SPIFFS.usedBytes(), SPIFFS.totalBytes(), ESP.getFreeHeap(), ESP.getHeapSize());
#endif
}

void displayOverlays(DisplayType& display, displayInfo displayData, bool invertColors, bool fullcolor) {
   int16_t tw = 0;
   int foreGround = GxEPD_WHITE_I;
   int backGround = GxEPD_BLACK_I;
   if (fullcolor) {
      foreGround = GxEPD_WHITE;
      backGround = GxEPD_BLACK;
   }
   if (invertColors) {
      // invert again
      int tempStore = foreGround;
      foreGround = backGround;
      backGround = tempStore;
   }
   char charBuffer[128];
   char charBuffer2[128];
   int pos = 0;

   if (displayData.deviceInfoString) {
      int deviceIdPos = 20;

      u8g2_for_adafruit_gfx.setForegroundColor(backGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(foreGround);  // apply Adafruit GFX color

      sprintf(charBuffer, "ID: %s", CLIENT_ID);
      String wifiSSID = WiFi.SSID();
      if (wifiSSID.length() > 1) {
         sprintf(charBuffer2, "%s WiFi: %s", charBuffer, wifiSSID.c_str());
         sprintf(charBuffer, "%s", charBuffer2);

      } else {
         sprintf(charBuffer2, "%s WiFi: NOT SET", charBuffer);
         sprintf(charBuffer, "%s", charBuffer2);
      }

      u8g2_for_adafruit_gfx.setFont(FONT_INFO);                                         // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer);                              // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, EPD_WIDTH - deviceIdPos);  // start writing at this position
      u8g2_for_adafruit_gfx.print(charBuffer);
   }

   if (displayData.batteryLowBig) {
      int batteryLowPos = 500;

      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFont(FONT_BIG);               // extended font
      u8g2_for_adafruit_gfx.setFontMode(1);                  // use u8g2 transparent mode (this is default)

      sprintf(charBuffer, "BATTERY LOW");
      int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer);                                   // text box width
      int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();                                            // positive
      int16_t td = u8g2_for_adafruit_gfx.getFontDescent();                                           // negative; in mathematicians view
      int16_t th = ta - td;                                                                          // text box height
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, (EPD_WIDTH + batteryLowPos - th) / 2);  // start writing at this position
      display.fillRect((EPD_HEIGHT - tw) / 2 - 2, (EPD_WIDTH + batteryLowPos - th) / 2 - 20, tw + 5, 25, backGround);
      u8g2_for_adafruit_gfx.print(charBuffer);
   }

   // Version Display bottom right
   if (displayData.version) {
      u8g2_for_adafruit_gfx.setFont(FONT_VERSION);           // extended font
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color

      sprintf(charBuffer, "%s", SOFTWARE_VERSION);
#if DEBUG
      sprintf(charBuffer2, "DEV: %s", charBuffer);
      sprintf(charBuffer, "%s", charBuffer2);
#endif
      tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer);  // text box width
      display.fillRect(EPD_HEIGHT - tw - 2 - pos, EPD_WIDTH - 7, tw + 3, 8, backGround);
      u8g2_for_adafruit_gfx.setCursor(EPD_HEIGHT - tw - 1 - pos, EPD_WIDTH - 1);
      u8g2_for_adafruit_gfx.print(charBuffer);
      pos = pos + 60;
   }
   if (displayData.batteryInfo) {
      u8g2_for_adafruit_gfx.setFont(FONT_VERSION);           // extended font
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      systemData.vddValue = readVDD(false);

      sprintf(charBuffer, "Bat: %dV", systemData.vddValue);

      tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer);  // text box width
      display.fillRect(EPD_HEIGHT - tw - 2 - pos, EPD_WIDTH - 7, tw + 3, 8, backGround);
      u8g2_for_adafruit_gfx.setCursor(EPD_HEIGHT - tw - 1 - pos, EPD_WIDTH - 1);
      u8g2_for_adafruit_gfx.print(charBuffer);
      pos = pos + 60;
   }

   if (displayData.wifiSignal) {
      u8g2_for_adafruit_gfx.setFont(FONT_VERSION);           // extended font
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      int wifiSignal = WiFi.RSSI();

      sprintf(charBuffer, "WiFi Sig: %d", wifiSignal);
      tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer);  // text box width
      display.fillRect(EPD_HEIGHT - tw - 2 - pos, EPD_WIDTH - 7, tw + 3, 8, backGround);
      u8g2_for_adafruit_gfx.setCursor(EPD_HEIGHT - tw - 1 - pos, EPD_WIDTH - 1);
      u8g2_for_adafruit_gfx.print(charBuffer);
      pos = pos + 60;
   }
}

int loadImageFromWeb(String url) {
   if (url.length() < 1)
      return -2;
   Serial.print("[DL] Download Image file: ");
   Serial.println(url);
   int setImage = 0;
   int downloadOk = 0;
   String newUrl = getRedirect(url);
   for (int i = 0; i <= 5; i++) {
      debugFS();
      downloadOk = downloadAndSaveFile("tmp.gz", newUrl);
      if (downloadOk == 0) {
         break;
      } else {
         if (WiFi.status() != WL_CONNECTED) {
            WiFi.disconnect(true);
            WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.pss.c_str());
            delay(3000);
         }
      }
   }
   debugFS();
   if (downloadOk == 0) {
      setUpdateState("download_ok");  // also connects aws
      Serial.println("[DL] Done");
      waitDisplayComplete(false);
      delay(50);
      setImage = setImageFromFS("tmp.gz");
   }
   debugFS();

   if (downloadOk == 0 && setImage == 0)
      return 0;
   else
      return -1;
}

bool getVersionUpdate(void) {
   bool isUpdate = false;
   if (!isNewVersion) {
      int counter = 0;
      while (counter < 50 && !isNewVersion) {
         delay(100);
      }
   }
   if (isNewVersion) {
      int oldVersion = readIntFromFlash(150);
      Serial.print("[MAIN] Flash File Version (OLD|NEW): ");
      Serial.print(oldVersion);
      Serial.print(" | ");
      Serial.println(newVersionSave);

      if (newVersionSave > oldVersion) {
         Serial.println("[MAIN] Device will update Image");
         isUpdate = true;
      } else {
         Serial.println("[MAIN] No new Image Version");
         isUpdate = false;
      }
   }
   return isUpdate;
}

int setImageFromFS(String fileName) {
   saveFile = SerialFlash.open(fileName.c_str());
   if (!saveFile) {
      Serial.println("File missing");
   }
   saveFile.seek(0);

   unsigned char buffer2[4];
   saveFile.read(buffer2, 4);

   uint16_t width = (buffer2[1] << 8) | (buffer2[0] & 0xff);
   uint16_t height = (buffer2[3] << 8) | (buffer2[2] & 0xff);

   Serial.printf("[BMP] Image H: %d W: %d \n", height, width);

   if (width > EPD_WIDTH || height > EPD_HEIGHT) {
      Serial.printf("[BMP] Image too wide or tall!");
      return 1;
   }

   // wipe if this setting is on
   if (settings.clearscreen && settings.lut != "default") {
      displayWipe(false);
      delay(2000);
   }

   // auto center image
   int img_x0 = (EPD_WIDTH - width) / 2;
   int img_y0 = (EPD_HEIGHT - height) / 2;
   int bufferSize = (width / 2);
   unsigned char buffer[bufferSize + 2];

   display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
   display.init(115200);
   display.setRotation(displaySettings.rotationPicture);
   display.setFullWindow();
   Serial.print("[EPD] Update Display... \n");
   display.firstPage();
   do {
      bool skiplast = false;
      int counter = OFFSET_BITMAP;
      saveFile.seek(counter);
      // runonce = true;
      display.fillRect(0, 0, EPD_WIDTH, EPD_HEIGHT, GxEPD_WHITE);
      for (int i = 0; i < height; i++) {
         saveFile.read(buffer, bufferSize);
         if (width % 2) {
            if (!skiplast) {
               counter = counter + bufferSize + 2;
            } else {
               counter = counter + bufferSize - 1;
            }
         } else {
            counter = counter + bufferSize;
         }
         saveFile.seek(counter);

         for (int j = 0; j < width; j++) {
            int x = j;
            int y = i;
            int pos = 0;
            if (width % 2) {
               pos = floor(j / 2);
               if (!skiplast && x >= 3) {
                  x = x - 3;
               }
            } else {
               pos = j / 2;
            }

            if (j % 2) {
               uint8_t low = buffer[pos] & 0x0F;
               display.drawPixel(x + img_x0, y + img_y0, getColor(low));

            } else {
               uint8_t high = buffer[pos] >> 4;
               display.drawPixel(x + img_x0, y + img_y0, getColor(high));
            }

            if (width % 2 && j == width - 1 && skiplast == false) {
               uint8_t low = buffer[pos] & 0x0F;
               display.drawPixel(0 + img_x0, y + img_y0 + 1, getColor(low));
               skiplast = true;
            } else if (width % 2 && skiplast && j == width - 1) {
               skiplast = false;
            }
         }
      }

      char charBuffer[128];
      u8g2_for_adafruit_gfx.setFontDirection(1);  // left to right (this is default)
      u8g2_for_adafruit_gfx.setFontMode(1);       // use u8g2 transparent mode (this is default)

      if (displayInfos.batteryLowBig && settings.showBatteryWarning) {
         int batteryLowPos = -500;
         u8g2_for_adafruit_gfx.setForegroundColor(GxEPD_WHITE);  // apply Adafruit GFX color
         u8g2_for_adafruit_gfx.setBackgroundColor(GxEPD_BLACK);  // apply Adafruit GFX color
         u8g2_for_adafruit_gfx.setFont(FONT_BIG);                // extended font
         sprintf(charBuffer, "BATTERY LOW");
         int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer);                                   // text box width
         int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();                                            // positive
         int16_t td = u8g2_for_adafruit_gfx.getFontDescent();                                           // negative; in mathematicians view
         int16_t th = ta - td;                                                                          // text box height
         u8g2_for_adafruit_gfx.setCursor((EPD_WIDTH + batteryLowPos - th) / 2, (EPD_HEIGHT - tw) / 2);  // start writing at this position
         display.fillRect((EPD_WIDTH + batteryLowPos - th) / 2 - 4, (EPD_HEIGHT - tw) / 2 - 2, 25, tw + 7, GxEPD_BLACK);
         u8g2_for_adafruit_gfx.print(charBuffer);

         // sprintf(charBuffer, "BATTERY LOW");
         // int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer);  // text box width
         // display.fillRect(0, (EPD_HEIGHT - tw) / 2, 9, tw + 5, GxEPD_RED);
         // u8g2_for_adafruit_gfx.setCursor(1, (EPD_HEIGHT - tw) / 2);
         // u8g2_for_adafruit_gfx.print(charBuffer);
      }
      if (displayInfos.wifiOfflineBig && settings.showWifiWarning) {
         int noWifiPos = -560;
         u8g2_for_adafruit_gfx.setForegroundColor(GxEPD_WHITE);  // apply Adafruit GFX color
         u8g2_for_adafruit_gfx.setBackgroundColor(GxEPD_BLACK);  // apply Adafruit GFX color
         u8g2_for_adafruit_gfx.setFont(FONT_BIG);                // extended font
         sprintf(charBuffer, "NO WIFI CONNECTION");
         int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer);                               // text box width
         int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();                                        // positive
         int16_t td = u8g2_for_adafruit_gfx.getFontDescent();                                       // negative; in mathematicians view
         int16_t th = ta - td;                                                                      // text box height
         u8g2_for_adafruit_gfx.setCursor((EPD_WIDTH + noWifiPos - th) / 2, (EPD_HEIGHT - tw) / 2);  // start writing at this position
         display.fillRect((EPD_WIDTH + noWifiPos - th) / 2 - 4, (EPD_HEIGHT - tw) / 2 - 2, 25, tw + 7, GxEPD_BLACK);
         u8g2_for_adafruit_gfx.print(charBuffer);
      }
#if DEBUG
      u8g2_for_adafruit_gfx.setFont(FONT_VERSION);            // extended font
      u8g2_for_adafruit_gfx.setForegroundColor(GxEPD_WHITE);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(GxEPD_RED);    // apply Adafruit GFX color
      sprintf(charBuffer, "DEV: %s", SOFTWARE_VERSION);
      int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer);  // text box width
      display.fillRect(0, EPD_HEIGHT - tw - 3, 9, tw + 5, GxEPD_RED);
      u8g2_for_adafruit_gfx.setCursor(1, EPD_HEIGHT - tw - 1);
      u8g2_for_adafruit_gfx.print(charBuffer);
#endif
#if DEBUG
/*
      u8g2_for_adafruit_gfx.setFontDirection(0);  // left to right (this is default)
      u8g2_for_adafruit_gfx.setFontMode(1);       // use u8g2 transparent mode (this is default)

      displayInfo displayInfoPicture = displayInfos;
      displayInfoPicture.version = true;
      displayInfoPicture.batteryInfo = true;
      displayInfoPicture.wifiSignal = true;
      displayInfoPicture.deviceInfoString = true;

      displayOverlays(display, displayInfoPicture, false, true);
      */
#endif
   } while (display.nextPage());
   Serial.println("[EPD] End Draw...");
   return 0;
}
void displayDebugInfo() {
   int textLines = 11;
   String info[textLines];
   if (deviceActivated || deviceActivationNotStarted || deviceActivationReset) {
      info[0] = "AWS Connection: OK";
   } else {
      info[0] = "AWS Connection: FAIL";
   }
   String ssidRead = readStringFromFlash(0);
   String pwRead = readStringFromFlash(40);
   int wifiSignal = WiFi.RSSI();
   systemData.vddValue = readVDD(false);
   bool isUsb = usbCheckConnect();

   info[1] = "";

   info[2] = "Device Status:";
   info[3] = "Activated: " + String(deviceActivated) + " NotStarted: " + String(deviceActivationNotStarted) + " Reset: " + String(deviceActivationReset);
   info[4] = "OTA URL: " + String(OTA_URL);
   info[5] = "DEV OTA URL: " + String(OTA_URL_DEV);
   info[6] = "";

   info[7] = "Flash:";
   info[8] = "Activated: " + String(getActivatedFromMem()) + " ActCount: " + String(readIntFromFlash(170)) + " FileVers: " + String(readIntFromFlash(150));
   info[9] = "SSID: " + ssidRead + " PW: " + pwRead;
   info[10] = "LINK: " + String(wifiSignal) + " BAT: " + String(systemData.vddValue) + "V USB: " + String(isUsb);
   Serial.println("[TEST] START");
   for (int i = 0; i < textLines; i++) {
      Serial.println(info[i]);
   }
   Serial.println("[TEST] END");

   String info1 = "Ich schlafe ...";
   String info2 = "Drücke die Taste auf der Rückseite";
   String info3 = "um mich zu wecken.";

   char msg[128];
   sprintf(msg, "%s%s%s%s", "https://paperlesspaper.de/b?d=", CLIENT_ID, "&w=99");

   uint8_t QRData[qrcode_getBufferSize(QR_VERSION)];
   uint8_t blockSize;
   uint8_t page = 0;
   qrcode_initText(&QR, QRData, QR_VERSION, ECC_LOW, msg);
   blockSize = 2;
   uint16_t x0 = (EPD_HEIGHT - 30 * blockSize) / 2;
   uint16_t y0 = 680;

   int foreGround = GxEPD_WHITE_I;
   int backGround = GxEPD_BLACK_I;
   bool fullColor = false;

   Serial.print(F("\n[EPD] Press to turn on Screen Loading - "));
   if (displaySettings.quickRefresh) {
      display.init(115200);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
   } else {
      display.enableQuickRefresh(0, false);
      display.init(115200);
      foreGround = GxEPD_WHITE;
      backGround = GxEPD_BLACK;
      fullColor = true;
   }
   display.setRotation(displaySettings.rotationText);
   display.firstPage();
   // Display 600*448
   do {
      int16_t tw = 0;
      display.fillRect(0, 0, EPD_HEIGHT, EPD_WIDTH, backGround);
      display.fillRect(2, 2, EPD_HEIGHT - 4, EPD_WIDTH - 4, foreGround);
      display.fillRect(4, 4, EPD_HEIGHT - 8, EPD_WIDTH - 8, backGround);
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFontDirection(0);             // left to right (this is default)
      u8g2_for_adafruit_gfx.setFontMode(1);                  // use u8g2 transparent mode (this is default)

      display.fillRect(x0 + 2, y0 + 2, QR.size * blockSize + QR_QUIET_ZONE + blockSize - 2, QR.size * blockSize + QR_QUIET_ZONE + blockSize - 2, foreGround);

      // For each vertical module
      for (uint8_t y = 0; y < QR.size; y++) {
         // Eor each horizontal module
         for (uint8_t x = 0; x < QR.size; x++) {
            if (qrcode_getModule(&QR, x, y))
               printQRBlock(x0 + (x * blockSize) + QR_QUIET_ZONE,
                            y0 + (y * blockSize) + QR_QUIET_ZONE,
                            blockSize,
                            (qrcode_getModule(&QR, x, y)) ? backGround : foreGround);
         }
      }

      u8g2_for_adafruit_gfx.setFont(FONT_MAIN);  // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info1.c_str());
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 300);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info1);

      u8g2_for_adafruit_gfx.setFont(FONT_BIG);                      // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info2.c_str());       // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 370);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info2);                           // UTF-8 string: "<" 550 448 664 ">"

      tw = u8g2_for_adafruit_gfx.getUTF8Width(info3.c_str());       // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 395);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info3);

      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);                   // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("I am sleeping...");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 560);  // start writing at this position
      u8g2_for_adafruit_gfx.print("I am sleeping...");

      u8g2_for_adafruit_gfx.setFont(FONT_SMALL);                                               // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Press the button on the back to wake me up.");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 590);                             // start writing at this position
      u8g2_for_adafruit_gfx.print("Press the button on the back to wake me up.");

   } while (display.nextPage());
}
/*
void displayDebugInfo_alt() {
   int textLines = 11;
   String info[textLines];
   if (deviceActivated || deviceActivationNotStarted || deviceActivationReset) {
      info[0] = "AWS Connection: OK";
   } else {
      info[0] = "AWS Connection: FAIL";
   }
   String ssidRead = readStringFromFlash(0);
   String pwRead = readStringFromFlash(40);
   int wifiSignal = WiFi.RSSI();
   systemData.vddValue = readVDD(false);
   bool isUsb = usbCheckConnect();

   info[1] = "";

   info[2] = "Device Status:";
   info[3] = "Activated: " + String(deviceActivated) + " NotStarted: " + String(deviceActivationNotStarted) + " Reset: " + String(deviceActivationReset);
   info[4] = "OTA URL: " + String(OTA_URL);
   info[5] = "DEV OTA URL: " + String(OTA_URL_DEV);
   info[6] = "";

   info[7] = "Flash:";
   info[8] = "Activated: " + String(getActivatedFromMem()) + " ActCount: " + String(readIntFromFlash(170)) + " FileVers: " + String(readIntFromFlash(150));
   info[9] = "SSID: " + ssidRead + " PW: " + pwRead;
   info[10] = "LINK: " + String(wifiSignal) + " BAT: " + String(systemData.vddValue) + "V USB: " + String(isUsb);
   Serial.println("[TEST] START");
   for (int i = 0; i < textLines; i++) {
      Serial.println(info[i]);
   }
   Serial.println("[TEST] END");
   int foreGround = GxEPD_BLACK_I;
   int backGround = GxEPD_WHITE_I;
   display.init(115200);
   display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
   display.setRotation(displaySettings.rotationText);
   display.firstPage();
   do {
      display.fillRect(0, 0, EPD_HEIGHT, EPD_WIDTH, GxEPD_BLACK_I);
      display.fillRect(2, 2, EPD_HEIGHT - 4, EPD_WIDTH - 4, GxEPD_WHITE_I);
      display.fillRect(4, 4, EPD_HEIGHT - 8, EPD_WIDTH - 8, GxEPD_BLACK_I);
      display.fillRect(8, 8, EPD_HEIGHT - 16, EPD_WIDTH - 16, GxEPD_WHITE_I);
      int16_t tw = 0;

      //        display.fillRect(EPD_HEIGHT - tw - 2, 592 + 1, tw + 3, 8, GxEPD_RED_I);

      u8g2_for_adafruit_gfx.setFontDirection(0);             // left to right (this is default)
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color

      u8g2_for_adafruit_gfx.setFont(FONT_MAIN);                    // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Deployment");       // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 50);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Deployment");

      u8g2_for_adafruit_gfx.setFontMode(1);  // use u8g2 transparent mode (this is default)

      int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();   // positive
      int16_t td = u8g2_for_adafruit_gfx.getFontDescent();  // negative; in mathematicians view
      int16_t th = ta - td;                                 // text box height
      for (int i = 0; i < textLines; i++) {
         switch (i) {
            case 0:
               u8g2_for_adafruit_gfx.setFont(FONT_BIG);
               break;
            case 4:
               u8g2_for_adafruit_gfx.setFont(FONT_INFO);
               break;
            case 5:
               u8g2_for_adafruit_gfx.setFont(FONT_INFO);
               break;
            default:
               u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);  // extended font
         }
         tw = u8g2_for_adafruit_gfx.getUTF8Width(info[i].c_str());                // text box width
         u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 120 + (i * 25));  // start writing at this position
         u8g2_for_adafruit_gfx.print(info[i]);
      }

      displayInfo displayInfoDebug;

      displayInfoDebug.version = true;
      displayInfoDebug.batteryInfo = true;
      displayInfoDebug.batteryLowBig = false;
      displayInfoDebug.wifiSignal = true;
      displayInfoDebug.deviceInfoString = true;

      displayOverlays(display, displayInfoDebug, false);

   } while (display.nextPage());
}
*/
void displaySetText(String info, bool isBlackboard, bool quickRefresh) {
   int foreGround = GxEPD_BLACK_I;
   int backGround = GxEPD_WHITE_I;
   int fill = GxEPD_WHITE_I;
   bool invert = false;
   bool fullcolor = false;
   if (isBlackboard) {
      invert = true;
      foreGround = GxEPD_WHITE_I;
      backGround = GxEPD_BLACK_I;
      fill = GxEPD_BLACK_I;
   }
   if (!displaySettings.quickRefresh) {
      quickRefresh = false;
   }
   if (quickRefresh) {
      fullcolor = false;
      display.init(115200);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
   } else {
      if (isBlackboard) {
         foreGround = GxEPD_WHITE;
         backGround = GxEPD_BLACK;
         fill = GxEPD_BLACK;
      } else {
         foreGround = GxEPD_BLACK;
         backGround = GxEPD_WHITE;
         fill = GxEPD_WHITE;
      }
      fullcolor = true;
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
      display.init(115200);
   }

   display.setRotation(displaySettings.rotationText);
   display.firstPage();
   do {
      display.fillRect(0, 0, EPD_HEIGHT, EPD_WIDTH, fill);
      u8g2_for_adafruit_gfx.setFontDirection(0);             // left to right (this is default)
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color

      u8g2_for_adafruit_gfx.setFont(FONT_BIG);  // extended font
      u8g2_for_adafruit_gfx.setFontMode(1);     // use u8g2 transparent mode (this is default)

      int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width(info.c_str());                 // text box width
      int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();                            // positive
      int16_t td = u8g2_for_adafruit_gfx.getFontDescent();                           // negative; in mathematicians view
      int16_t th = ta - td;                                                          // text box height
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, (EPD_WIDTH - th) / 2);  // start writing at this position

      u8g2_for_adafruit_gfx.print(info);
      if (info.length() > 1) {
         displayInfos.deviceInfoString = true;
      }

      displayOverlays(display, displayInfos, invert, fullcolor);

   } while (display.nextPage());
}

void displayNoPicture() {
   String info = "Bitte aktualisiere ein Bild in der App.";
   String info2 = "Aktivierung";
   String info3 = "abgeschlossen";

   Serial.print(F("[EPD] No Picture Screen Loading - \n"));

   int foreGround = GxEPD_BLACK_I;
   int backGround = GxEPD_WHITE_I;
   bool fullColor = false;

   if (displaySettings.quickRefresh) {
      display.init(115200);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
   } else {
      display.enableQuickRefresh(0, false);
      display.init(115200);
      foreGround = GxEPD_BLACK;
      backGround = GxEPD_WHITE;
      fullColor = true;
   }
   display.setRotation(displaySettings.rotationText);
   display.firstPage();
   // Display 600*448
   do {
      u8g2_for_adafruit_gfx.setFontDirection(0);  // left to right (this is default)
      display.fillRect(0, 0, EPD_HEIGHT, EPD_WIDTH, backGround);
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFontMode(1);                  // use u8g2 transparent mode (this is default)

      u8g2_for_adafruit_gfx.setFont(FONT_MAIN);  // extended font
      int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width(info2.c_str());
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 150);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info2);

      tw = u8g2_for_adafruit_gfx.getUTF8Width(info3.c_str());
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 190);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info3);

      u8g2_for_adafruit_gfx.setFont(FONT_BIG);                                       // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info.c_str());                         // text box width
      int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();                            // positive
      int16_t td = u8g2_for_adafruit_gfx.getFontDescent();                           // negative; in mathematicians view
      int16_t th = ta - td;                                                          // text box height
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, (EPD_WIDTH - th) / 2);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info);

      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);                        // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Activation Successful");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 560);       // start writing at this position
      u8g2_for_adafruit_gfx.print("Activation Successful");

      u8g2_for_adafruit_gfx.setFont(FONT_SMALL);                                        // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Please update a picture in your App.");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 590);                      // start writing at this position
      u8g2_for_adafruit_gfx.print("Please update a picture in your App.");              // UTF-8 string: "<" 550 448 664 ">"

      displayOverlays(display, displayInfos, false, fullColor);

   } while (display.nextPage());
}

void displayTurnOn() {
   String info = "Ich schlafe ...";
   String info2 = "Drücke die Taste auf der Rückseite";
   String info3 = "um mich zu wecken.";

   char msg[128];
   sprintf(msg, "%s%s%s%s", "https://paperlesspaper.de/b?d=", CLIENT_ID, "&w=99");

   uint8_t QRData[qrcode_getBufferSize(QR_VERSION)];
   uint8_t blockSize;
   uint8_t page = 0;
   qrcode_initText(&QR, QRData, QR_VERSION, ECC_LOW, msg);
   blockSize = 2;
   uint16_t x0 = (EPD_HEIGHT - 30 * blockSize) / 2;
   uint16_t y0 = 680;

   int foreGround = GxEPD_WHITE_I;
   int backGround = GxEPD_BLACK_I;
   bool fullColor = false;

   Serial.print(F("\n[EPD] Press to turn on Screen Loading - "));
   if (displaySettings.quickRefresh) {
      display.init(115200);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
   } else {
      display.enableQuickRefresh(0, false);
      display.init(115200);
      foreGround = GxEPD_WHITE;
      backGround = GxEPD_BLACK;
      fullColor = true;
   }
   display.setRotation(displaySettings.rotationText);
   display.firstPage();
   // Display 600*448
   do {
      int16_t tw = 0;
      display.fillRect(0, 0, EPD_HEIGHT, EPD_WIDTH, backGround);
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFontDirection(0);             // left to right (this is default)
      u8g2_for_adafruit_gfx.setFontMode(1);                  // use u8g2 transparent mode (this is default)

      display.fillRect(x0 + 2, y0 + 2, QR.size * blockSize + QR_QUIET_ZONE + blockSize - 2, QR.size * blockSize + QR_QUIET_ZONE + blockSize - 2, foreGround);

      // For each vertical module
      for (uint8_t y = 0; y < QR.size; y++) {
         // Eor each horizontal module
         for (uint8_t x = 0; x < QR.size; x++) {
            if (qrcode_getModule(&QR, x, y))
               printQRBlock(x0 + (x * blockSize) + QR_QUIET_ZONE,
                            y0 + (y * blockSize) + QR_QUIET_ZONE,
                            blockSize,
                            (qrcode_getModule(&QR, x, y)) ? backGround : foreGround);
         }
      }

      u8g2_for_adafruit_gfx.setFont(FONT_MAIN);  // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info.c_str());
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 300);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info);

      u8g2_for_adafruit_gfx.setFont(FONT_BIG);                      // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info2.c_str());       // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 370);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info2);                           // UTF-8 string: "<" 550 448 664 ">"

      tw = u8g2_for_adafruit_gfx.getUTF8Width(info3.c_str());       // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 395);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info3);

      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);                   // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("I am sleeping...");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 560);  // start writing at this position
      u8g2_for_adafruit_gfx.print("I am sleeping...");

      u8g2_for_adafruit_gfx.setFont(FONT_SMALL);                                               // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Press the button on the back to wake me up.");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 590);                             // start writing at this position
      u8g2_for_adafruit_gfx.print("Press the button on the back to wake me up.");

      displayOverlays(display, displayInfos, true, fullColor);

   } while (display.nextPage());
}

void displayUpdatePicture() {
   String info = "Bild wird aktualisiert";
   String info2 = "bitte warten...";

   Serial.print(F("[EPD] update pending Screen Loading\n"));
   display.init(115200);
   display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
   display.setRotation(displaySettings.rotationText);
   display.firstPage();
   // Display 600*448
   do {
      u8g2_for_adafruit_gfx.setFontDirection(0);  // left to right (this is default)
      display.fillRect(0, 0, EPD_HEIGHT, EPD_WIDTH, GxEPD_BLACK_I);
      u8g2_for_adafruit_gfx.setForegroundColor(GxEPD_WHITE_I);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(GxEPD_BLACK_I);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFontMode(1);                     // use u8g2 transparent mode (this is default)

      u8g2_for_adafruit_gfx.setFont(FONT_MAIN);                       // extended font
      int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width(info.c_str());  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 300);    // start writing at this position
      u8g2_for_adafruit_gfx.print(info);                              // UTF-8 string: "<" 550 448 664 ">"

      u8g2_for_adafruit_gfx.setFont(FONT_BIG);                      // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info2.c_str());       // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 370);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info2);

      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);                   // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Updating Picture");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 560);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Updating Picture");

      u8g2_for_adafruit_gfx.setFont(FONT_SMALL);                     // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Please be patient");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 590);   // start writing at this position
      u8g2_for_adafruit_gfx.print("Please be patient...");           // UTF-8 string: "<" 550 448 664 ">"

      displayOverlays(display, displayInfos, true);
   } while (display.nextPage());
}

void displayWifiActivate(bool wifiProvisioningDone) {
   // Paged display of the QR code with top left corner at (x0,y0)
   // on the display

   char msg[128];
   sprintf(msg, "%s%s%s%s", "https://paperlesspaper.de/b?d=", CLIENT_ID, "&w=", wifiProvisioningDone ? "1" : "0");
   // sprintf(msg, "%s%s", "https://paperlesspaper.de/b?d=", CLIENT_ID);

   uint8_t QRData[qrcode_getBufferSize(QR_VERSION)];
   uint8_t blockSize;
   uint8_t page = 0;
   // QR Code 5*30 = 150px

   qrcode_initText(&QR, QRData, QR_VERSION, ECC_LOW, msg);
   // blockSize = (display.height() - (2 * QR_QUIET_ZONE)) / QR.size;
   blockSize = 8;

   uint16_t x0 = (EPD_HEIGHT - 30 * blockSize) / 2;
   uint16_t y0 = 50;

   int foreGround = GxEPD_BLACK_I;
   int backGround = GxEPD_WHITE_I;
   bool fullColor = false;

   if (DEBUG_FLAG) Serial.printf("[EPD] Wifi Activate Function: %d\n", wifiProvisioningDone);
   Serial.printf("[EPD] QR Size: %d | Locate: %d,%d | DispH: %d \n", QR.size, x0, y0, display.height());
   Serial.printf("[EPD] QR Block: %d | Data: %s \n", blockSize, msg);

   if (displaySettings.quickRefresh) {
      display.init(115200);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
   } else {
      display.enableQuickRefresh(0, false);
      display.init(115200);
      foreGround = GxEPD_BLACK;
      backGround = GxEPD_WHITE;
      fullColor = true;
   }
   display.setRotation(displaySettings.rotationText);
   display.firstPage();
   // Display 600*448
   do {
      int16_t tw = 0;
      display.fillRect(0, 0, EPD_HEIGHT, EPD_WIDTH, backGround);
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      display.fillRect(x0, y0, QR.size * blockSize + QR_QUIET_ZONE + blockSize, QR.size * blockSize + QR_QUIET_ZONE + blockSize, backGround);

      // For each vertical module
      for (uint8_t y = 0; y < QR.size; y++) {
         // Eor each horizontal module
         for (uint8_t x = 0; x < QR.size; x++) {
            if (qrcode_getModule(&QR, x, y))
               printQRBlock(x0 + (x * blockSize) + QR_QUIET_ZONE,
                            y0 + (y * blockSize) + QR_QUIET_ZONE,
                            blockSize,
                            (qrcode_getModule(&QR, x, y)) ? foreGround : backGround);
         }
      }
      u8g2_for_adafruit_gfx.setFontMode(1);  // use u8g2 transparent mode (this is default)

      if (wifiProvisioningDone) {
         u8g2_for_adafruit_gfx.setFont(FONT_MAIN);                     // extended font
         tw = u8g2_for_adafruit_gfx.getUTF8Width("Gerät aktivieren");  // text box width
         u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 360);  // start writing at this position
         u8g2_for_adafruit_gfx.print("Gerät aktivieren");
      } else {
         u8g2_for_adafruit_gfx.setFont(FONT_MAIN);                     // extended font
         tw = u8g2_for_adafruit_gfx.getUTF8Width("WLAN verbinden");    // text box width
         u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 360);  // start writing at this position
         u8g2_for_adafruit_gfx.print("WLAN verbinden");
      }
      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);                                                   // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("1. Scanne den QR Code um die App zu installieren");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 400);                                  // start writing at this position
      u8g2_for_adafruit_gfx.print("1. Scanne den QR Code um die App zu installieren");

      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);                                    // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("2. Folge den Schritten in der App");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 420);                   // start writing at this position
      u8g2_for_adafruit_gfx.print("2. Folge den Schritten in der App");

      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);                   // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Connecting WiFi");   // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 560);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Connecting WiFi");

      u8g2_for_adafruit_gfx.setFont(FONT_SMALL);                                     // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Scan the QR code or open the App.");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 590);                   // start writing at this position
      u8g2_for_adafruit_gfx.print("Scan the QR code or open the App.");

      displayOverlays(display, displayInfos, false, fullColor);

   } while (display.nextPage());

   // Serial.print(F("\nEnd"));
}

void displayWipe(bool quick) {
   int fill = GxEPD_WHITE;

   if (quick) {
      display.init(115200);
      display.enableQuickRefresh(5200, true);
      fill = GxEPD_WHITE_I;
   } else {
      display.init(115200);
      display.enableQuickRefresh(5200, false);
      fill = GxEPD_WHITE;
   }
   display.setRotation(1);
   display.firstPage();
   do {
      display.fillRect(0, 0, EPD_HEIGHT, EPD_WIDTH, fill);
   } while (display.nextPage());
}

void displaySetBlankTest(int offsetVar, bool doQuickRefresh) {
   Serial.println("[EPD] Set Display to Empty");
   display.init(115200);
   if (doQuickRefresh) {
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
   } else {
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
   }

   u8g2_for_adafruit_gfx.setFontDirection(0);  // left to right (this is default)
   display.fillRect(0, 0, EPD_HEIGHT, EPD_WIDTH, GxEPD_BLACK);
   u8g2_for_adafruit_gfx.setForegroundColor(GxEPD_BLACK_I);  // apply Adafruit GFX color
   u8g2_for_adafruit_gfx.setBackgroundColor(GxEPD_WHITE_I);  // apply Adafruit GFX color
   u8g2_for_adafruit_gfx.setFont(FONT_BIG);                  // extended font
   u8g2_for_adafruit_gfx.setFontMode(1);                     // use u8g2 transparent mode (this is default)
   display.setRotation(1);

   int offset = offsetVar * 40;
   int offset2 = offsetVar * 20;

   display.firstPage();
   do {
      display.fillRect(0, 0, EPD_HEIGHT, EPD_WIDTH, GxEPD_BLACK);
      display.fillRect(2, 2, EPD_HEIGHT - 4, EPD_WIDTH - 4, GxEPD_WHITE);
      display.fillRect(5, 5, EPD_HEIGHT - 10, EPD_WIDTH - 10, GxEPD_BLACK);
      display.fillRect(8, 8, EPD_HEIGHT - 16, EPD_WIDTH - 16, GxEPD_WHITE);
      //  Space all around
      //  display.fillRect(0, 0, EPD_WIDTH, EPD_HEIGHT, GxEPD_WHITE);
      display.fillRect(40, offset, 40, 40, GxEPD_BLACK);
      display.fillRect(80, offset, 40, 40, GxEPD_BLUE);
      display.fillRect(120, offset, 40, 40, GxEPD_GREEN);
      display.fillRect(160, offset, 40, 40, GxEPD_RED);
      display.fillRect(200, offset, 40, 40, GxEPD_ORANGE);
      display.fillRect(240, offset, 40, 40, GxEPD_YELLOW);

      display.fillRect(300, offset2, 20, 20, GxEPD_BLACK);
      display.fillRect(320, offset2, 20, 20, GxEPD_BLUE);
      display.fillRect(340, offset2, 20, 20, GxEPD_GREEN);
      display.fillRect(360, offset2, 20, 20, GxEPD_RED);
      display.fillRect(380, offset2, 20, 20, GxEPD_ORANGE);
      display.fillRect(400, offset2, 20, 20, GxEPD_YELLOW);
      display.fillRect(421, offset2, 20, 20, GxEPD_YELLOW);

      // display.setCursor(220, 100 + offset2);
      // display.println(info);
   } while (display.nextPage());
}

String setLeadingZero(String input) {
   String result = "";
   if (input.length() == 1) {
      result = "0" + input;
   } else {
      result = input;
   }

   return result;
}

void setDeviceUid() {
   byte mac[6];
   WiFi.mode(WIFI_STA);
   WiFi.macAddress(mac);
   if (DEBUG_FLAG) {
      Serial.print("[WIFI] MAC: ");
      for (int i = 0; i < 6; i++) {
         Serial.printf("%02X", mac[i]);
         if (i < 5) {
            Serial.print(":");
         }
      }
      Serial.println();
   }
   String uniq = setLeadingZero(String(mac[0], HEX)) + setLeadingZero(String(mac[1], HEX)) + setLeadingZero(String(mac[2], HEX)) + setLeadingZero(String(mac[3], HEX)) + setLeadingZero(String(mac[4], HEX)) + setLeadingZero(String(mac[5], HEX));
   const uint8_t MSG_BUF_SIZE = 100;
   char mesg[MSG_BUF_SIZE] = {'\0'};  // Serial input message buffer
   uniq.toCharArray(mesg, MSG_BUF_SIZE);
   sprintf(CLIENT_ID, "%s%s", EPD_TYPE_IDENTIFIER, mesg);
   Serial.print("[MAIN] UID: ");
   Serial.println(CLIENT_ID);
}

bool awsConnect(bool connect) {
   int counter = 0;
   if (!connect) {
      Serial.println("[AWS] DISCONNECTED");
      client.disconnect();
      return false;
   }
   if (client.connected()) {
      if (DEBUG_FLAG) Serial.println("[AWS] is still connected...");
      return true;
   }
   while (WiFi.status() != WL_CONNECTED) {
      counter++;
      WiFi.disconnect(true);
      WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.pss.c_str());
      delay(1500);
      Serial.print(".");
      if (counter > 10) {
         Serial.println("[AWS] Not connecting - NO WIFI");
         return false;
      }
   }

   char keyFileUri[100];
   sprintf(keyFileUri, "/%s.key", CLIENT_ID);

   File myFile = SPIFFS.open(keyFileUri);
   String keyString;
   while (myFile.available()) {
      String buffer = myFile.readStringUntil('\n');
      keyString.concat(buffer);
      keyString.concat("\n");
   }
   keyFileCons = keyString.c_str();
   myFile.close();

   char crtFileUri[100];
   sprintf(crtFileUri, "/%s.crt", CLIENT_ID);
   File myFile2 = SPIFFS.open(crtFileUri);
   String crtString;
   while (myFile2.available()) {
      String buffer = myFile2.readStringUntil('\n');
      crtString.concat(buffer);
      crtString.concat("\n");
   }
   crtFileCons = crtString.c_str();
   myFile2.close();

   if (keyString.length() < 10 || crtString.length() < 10) {
      Serial.println("[IOT] Error: Certs not found");
      displaySetText("Error: Certificates not found", false);
      return false;
   }
   net.setCACert(cert);
   net.setCertificate(crtFileCons);
   net.setPrivateKey(keyFileCons);

   Serial.println("[AWS] connecting...");

   client.begin(HOST_ADDRESS, 8883, net);
   client.onMessage(iotReceiveHandler);
   counter = 0;
   while (!client.connect(CLIENT_ID)) {
      counter++;
      Serial.print("*");
      delay(500);
      if (counter > 8) {
         return false;
      }
   }
   sprintf(TOPIC_RECEIVE, "$aws/things/%s/epaper/receive", CLIENT_ID);
   client.subscribe(TOPIC_RECEIVE);
   Serial.println("[AWS] CONNECTED");
   return true;
}

bool setUpdateState(String state) {
   char TOPIC_SEND[64];
   sprintf(TOPIC_SEND, "$aws/things/%s/epaper/updatestate", CLIENT_ID);
   char payload[256];
   sprintf(payload, "{\"state\": \"%s\"}", state.c_str());  // Create the payload for publishing
   int counter = 0;
   while (counter < 5) {
      awsConnect(true);
      if (client.publish(TOPIC_SEND, payload, false, 1))  // Publish the message(Temp and humidity)
      {
         Serial.printf("[IOT] Set State to: %s \n", state.c_str());
         return true;
      } else {
         Serial.println("[IOT] Set State Error!");
      }
      counter++;
      delay(1000);
   }
   return true;
}

bool deactivateDevice() {
   if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[IOT] Deactivate failed: WiFi not connected");
      return false;
   }
   char TOPIC_ACTIVATE[64];
   char payload[512];
   sprintf(TOPIC_ACTIVATE, "$aws/things/%s/activateepaper", CLIENT_ID);

   bool isConnected = awsConnect(true);
   if (isConnected) {
      delay(200);
   } else {
      return false;
   }

   sprintf(payload, "{\"act\": 0}");  // Create the payload for publishing
   int counter = 0;
   while (counter < 5 && deviceActivationReset == false) {
      awsConnect(true);
      if (client.publish(TOPIC_ACTIVATE, payload, false, 1))  // Publish the message(Temp and humidity)
      {
         Serial.println("[IOT] Request Remove Device");
      } else {
         Serial.println("[IOT] Request Remove Device FAILED");
      }
      int counter2 = 0;
      while (counter2 < 50 && deviceActivationReset == false) {
         client.loop();
         counter2++;
         vTaskDelay(100);
      }
      counter++;
   }
   if (deviceActivationReset) {
      return true;
   } else {
      return false;
   }
}

bool deployDevice() {
#if DEBUG
   File root = SPIFFS.open("/");
   File file = root.openNextFile();
   int fileCount = 0;
   while (file) {
      fileCount++;
      Serial.print("[MAIN] FILE: ");
      Serial.println(file.name());

      file = root.openNextFile();
   }
   root.close();
   file.close();
   if (fileCount < 2) {
      Serial.println("[IOT] Error: Certs not found");
      displaySetText("Error: Certificates not found", false);
      return false;
   }
#endif

   bool isConnected = awsConnect(true);
   if (isConnected) {
      delay(100);
   } else {
      return false;
   }

   char TOPIC_ACTIVATE[64];
   char payload[256];

   sprintf(TOPIC_ACTIVATE, "$aws/things/%s/activateepaper", CLIENT_ID);
   // if the wifi is fresh configured, reset the activation
   if (wifiSettings.wifiConfig) {
      sprintf(payload, "{\"act\": 0}");  // Create the payload for publishing
      int counter = 0;
      while (counter < 5 && deviceActivationReset == false) {
         awsConnect(true);
         if (client.publish(TOPIC_ACTIVATE, payload, false, 1)) {
            Serial.println("[IOT] Request Remove Device");
         } else {
            Serial.println("[IOT] Request Remove Device FAILED");
         }
         int counter2 = 0;
         while (counter2 < 50 && deviceActivationReset == false) {
            client.loop();
            counter2++;
            vTaskDelay(100);
         }
         counter++;
      }
   }
   int oldVersion = readIntFromFlash(150);
   int sleepTime = storeSleepTimeMem();  // right now only used for api. could be used to use set sleep time on other places

   sprintf(payload, "{\"act\": 1,\"v\": \"%s\",\"file\": \"%d\",\"bat\": \"%d\",\"wake\": \"%d, %d\",\"wifi\": \"%d, %d\",\"usb\": \"%d\",\"orient\": \"%d\",\"timeout\": \"%d\"}", SOFTWARE_VERSION, oldVersion, systemData.vddValue, rtc_get_reset_reason(0), rtc_get_reset_reason(1), wifiSettings.wifiRetries, wifiSettings.wifiQuality, systemData.usbConnected, systemData.deviceOrientation, sleepTime);  // Create the payload for publishing
   int counter = 0;
   while (counter < 5 && deviceActivated == false && deviceActivationNotStarted == false) {
      awsConnect(true);
      if (client.publish(TOPIC_ACTIVATE, payload, false, 1)) {
         Serial.println("[IOT] Request Activation");
         client.loop();
      } else {
         Serial.println("[IOT] Request Activation FAILED");
         client.loop();
      }
      int counter2 = 0;
      while (counter2 < 100 && deviceActivated == false && deviceActivationNotStarted == false && !isDlUrl) {
         counter2++;
         client.loop();
         vTaskDelay(50);
      }
      counter++;
   }

   if (deviceActivated) {
      return true;
   } else {
      return false;
   }
   return true;
}

bool getImageUrl(bool reset) {
   if (reset) {
      isDlUrl = false;
   }
   client.loop();
   if (!isDlUrl) {
      char payload[512];
      char func[32];

      sprintf(func, "%s", "request");
      sprintf(payload, "{\"func\": \"%s\"}", func);  // Create the payload for publishing

      char TOPIC_SEND[64];
      sprintf(TOPIC_SEND, "$aws/things/%s/epaper/getimageurl", CLIENT_ID);

      int counter = 0;

      while (counter < 50) {
         client.loop();
         counter++;
         delay(100);
         if (isDlUrl) {
            return true;
         }
      }
      counter = 0;
      while (counter < 5 && !isDlUrl) {
         awsConnect(true);
         if (client.publish(TOPIC_SEND, payload, false, 0))  // Publish the message(Temp and humidity)
         {
            Serial.println("[IOT] Request Image URL");
         } else {
            Serial.println("[IOT] Request Image URL FAILED");
         }
         int counter2 = 0;
         while (counter2 < 15 && !isDlUrl) {
            client.loop();
            counter2++;
            vTaskDelay(500);
         }
         counter++;
      }
   }
   if (isDlUrl) {
      return true;
   } else {
      return false;
   }
}
// sleep x seconds
void gotToDeepSleep(int wakeuptimeout, bool showScreen, bool motionWake) {
   Serial.printf("[MAIN] Going to Sleep for %d seconds (MotionWake: %d)\n", wakeuptimeout, motionWake);
   WiFi.disconnect(true);
   accIntSet(50);  // Set acc int wakeup
   waitDisplayComplete(false);
   if (wakeuptimeout <= 0 && showScreen) {
      displayTurnOn();  // Ich schlafe screen
      waitDisplayComplete(true);
   }
   ledBlink(0, false);
   delay(5);
   display.powerOff();
   delay(5);
   display.hibernate();
   delay(5);
   digitalWrite(RST_PIN, 0);
   pinMode(LED_PIN, INPUT);
   Serial.flush();
   delay(10);
   setCpuFrequencyMhz(10);
   delay(5);
   esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
   if (wakeuptimeout > 0) {
      esp_sleep_enable_timer_wakeup(wakeuptimeout * uS_TO_S_FACTOR);
   } else {
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
   }

   pinMode(RST_PIN, INPUT);
   pinMode(CS_EPD_PIN, INPUT);
   pinMode(DC_PIN, INPUT);
   pinMode(LED_PIN, INPUT);
   pinMode(CS_FLASH_PIN, INPUT);
   pinMode(SCK_PIN, INPUT);
   pinMode(MOSI_PIN, INPUT);
   pinMode(MISO_PIN, INPUT);
   pinMode(I2C_SDA_PIN, INPUT);
   pinMode(I2C_SCL_PIN, INPUT);
   pinMode(BAT_VOLT_EN_PIN, INPUT);
   pinMode(CHG_EN_PIN, INPUT);
   // esp_sleep_cpu_retention_init();
   // gpio_wakeup_enable(GPIO_NUM_0, GPIO_INTR_LOW_LEVEL);
   // gpio_pullup_dis(GPIO_NUM_0);
   // gpio_pulldown_en(GPIO_NUM_0);

   // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
   // esp_sleep_disable_ext1_wakeup_io(0);
   // esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);  // 1 = High, 0 = Low
   // esp_sleep_enable_gpio_wakeup();
   //    TODO: Use light sleep for short sleep periods to keep the wakeup time shorter (measure this)
   // setCpuFrequencyMhz(40);
   // esp_light_sleep_start();
   if (motionWake) {
      esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(GPIO_NUM_0), ESP_EXT1_WAKEUP_ANY_HIGH);
      rtc_gpio_pulldown_en(GPIO_NUM_0);
      rtc_gpio_pullup_dis(GPIO_NUM_0);
   }

   esp_deep_sleep_start();
   //   setCpuFrequencyMhz(240);
   wakeup_reason = esp_sleep_get_wakeup_cause();
   Serial.printf("[MAIN] Wakeup Reason from Sleep: %d\n", wakeup_reason);
   ESP.restart();
}

int startupCounter(bool reset) {
   preferences.begin("my-app", false);
   unsigned int counter = preferences.getUInt("counter", 0);
   counter++;
   if (reset || counter > 16) {
      StartCounter = 0;
      counter = 0;
      Serial.println("[MAIN] Startup Counter RESET");
   }
#if DEBUG
   // 1 14 = button wakeup, // 5 14 = timed sleep wakeup,
   Serial.printf("[MAIN] Reset Reason 0: %d\n", rtc_get_reset_reason(0));
   Serial.printf("[MAIN] Reset Reason 1: %d\n", rtc_get_reset_reason(1));
#endif
   // internal reset
   if (rtc_get_reset_reason(0) != 1 || rtc_get_reset_reason(1) != 1) {
      StartCounter = 0;
      counter = 0;
   }
   if (rtc_get_reset_reason(0) == 1 && rtc_get_reset_reason(1) == 1) {
      Serial.printf("[MAIN] Wake via Button \n");
      buttonWake = true;
   }

   preferences.putUInt("counter", counter);
   preferences.end();
   return counter;
}

bool resetAll(bool resetActivation, bool resetWifi) {
   Serial.printf("[MAIN] Reset - ACT %d | WIFI %d \n", resetActivation, resetWifi);

   writeIntToFlash(0, 150);               // Reset picture version after ota to init update
   writeIntToFlash(0, 140);               // reset reconnect count
   setActivatedToMem(false);              // reset activation writeIntToFlash(0, 160);
   wifiConnectionLostStore(false, true);  // reset wifi lost flag

   if (resetWifi) {
      writeStringToFlash("", 0);   // storing ssid at address 0
      writeStringToFlash("", 40);  // storing pss at address 40
      wifiSettings.ssid = "";
      wifiSettings.pss = "";
      wifiSettings.bleSSID = "";
      wifiSettings.blePASS = "";
      wifiSettings.wifiRetries = 0;
   }

   if (resetActivation) {
      startupCounter(true);
      writeIntToFlash(0, 170);  // reset activation counter
      bool success = deactivateDevice();
      if (success) {
         displaySetText("Gerät wurde zurückgesetzt", false);
         displayWipe(false);
      } else {
         displaySetText("Kein Netzwerk - Gerät wurde zurückgesetzt", false);
         displayWipe(false);
      }
      // Send disable cloud querry
      gotToDeepSleep(0, true, false);
   }
   return true;
}

void debugCheck() {
   if (wifiSettings.isDeployWifi || isTestMode) {
      // if button wake
      if (rtc_get_reset_reason(0) == 1 && rtc_get_reset_reason(1) == 1) {
         Serial.println("[DEBUG] Deploy State");
         resetAll(false, true);
         waitDisplayComplete(false);
         displaySettings.quickRefresh = true;
         displayDebugInfo();
         delay(2000);
         gotToDeepSleep(300, false, false);  // dont show the deep sleep screen stay with debug info
      } else {
         Serial.println("[DEBUG] Deploy Sleep");
         gotToDeepSleep(0, false, false);
      }
      return;
   }
}

// test if a byte is set via serial to enter deploy mode
void testModeCheck() {
   if (getActivatedFromMem()) return;  // skip test scan in real activated device
   delay(15);
   Serial.printf("%s", "[MAIN] test mode check...\n");  // important, this triggers test start
   delay(40);
   int incomingByte = Serial.read();  // read the incoming byte:
   if (incomingByte == 84) {
      isTestMode = true;
      Serial.printf("%s", "[MAIN] test mode set!\n");
      EepromClear();
   } else {
      Serial.printf("%s", "[MAIN] test mode skipped.\n");
   }
}

void flashTest() {
   SerialFlash.eraseAll();

   while (SerialFlash.ready() == false) {
      // wait, 30 seconds to 2 minutes for most chips
   }
   // testFlash();
}

bool usbInit() {
   Wire.beginTransmission(USBC_ADDR);
   Wire.write(0x01);
   Wire.endTransmission(true);  // end write operation, as we just wanted to select the starting register

   Wire.requestFrom(USBC_ADDR, 1);  // request 6 bytes from peripheral device #8
   char c = Wire.read();            // receive a byte as character
   if (c == 0b00010000) {
      Serial.println("[USB] init OK");
      return true;
   } else {
      Serial.println("[USB] init FAIL");
      return false;
   }
}

bool usbCheckConnect() {
   Wire.beginTransmission(USBC_ADDR);
   Wire.write(0x11);
   Wire.endTransmission(true);  // end write operation, as we just wanted to select the starting register

   Wire.requestFrom(USBC_ADDR, 1);  // request 6 bytes from peripheral device #8
   char c = Wire.read();            // receive a byte as character
   char usbPower = (c >> 3) & 0x01;
   char usbPowerLow = (c >> 6) & 0x01;
   if (usbPower && !usbPowerLow) {
      return true;
   } else {
      return false;
   }
}

int accInit() {
   int orientation = 0;
   if (myIMU.begin(6.25, 2, false) == IMU_SUCCESS) {
      if (DEBUG_FLAG) Serial.println("[ACC] initialized.");
   } else {
      Serial.println("[ACC] Failed to initialize.");
      return -1;
      // while (true);  // stop running sketch if failed
   }

   uint8_t readData = 0;

// Get the ID:
#if DEBUG
   if (myIMU.readRegister(&readData, KXTJ3_WHO_AM_I) ==
       IMU_SUCCESS) {
      Serial.print("[ACC] Who am I? 0x");
      Serial.println(readData, HEX);
   } else {
      Serial.println("[ACC] Communication error, stopping.");
      // while (true);  // stop running sketch if failed
   }
#endif

   myIMU.standby(false);

   uint8_t dataLowRes = 0;
   int acc_x = 0;
   int acc_y = 0;
   int acc_z = 0;

   if (myIMU.readRegister(&dataLowRes, KXTJ3_XOUT_H) ==
       IMU_SUCCESS) {
      // Read accelerometer data in mg as Float
      float acc_x_loc = myIMU.axisAccel(X);
      acc_x = round((int)(acc_x_loc * 10));
   }

   if (myIMU.readRegister(&dataLowRes, KXTJ3_YOUT_H) ==
       IMU_SUCCESS) {
      // Read accelerometer data in mg as Float
      float acc_y_loc = myIMU.axisAccel(Y);
      acc_y = round((int)(acc_y_loc * 10));
   }

   if (myIMU.readRegister(&dataLowRes, KXTJ3_ZOUT_H) ==
       IMU_SUCCESS) {
      // Read accelerometer data in mg as Float
      float acc_z_loc = myIMU.axisAccel(Z);
      acc_z = round((int)(acc_z_loc * 10));
   }

   if (acc_x > 5 && acc_y < 5 && acc_y > -5) {
      orientation = 0;
   }
   if (acc_x < 5 && acc_x > -5 && acc_y > 5) {
      orientation = 1;
   }
   if (acc_x < -5 && acc_y < 5 && acc_y > -5) {
      orientation = 2;
   }
   if (acc_x < 5 && acc_x > -5 && acc_y < -5) {
      orientation = 3;
   }

   if (DEBUG_FLAG) Serial.printf("[ACC] Values: X:%d Y:%d Z:%d Orient: %d \n", acc_x, acc_y, acc_z, orientation);

   // Put IMU back into standby
   myIMU.resetInterrupt();
   myIMU.standby(true);
   return orientation;
}

bool accIntSet(int sensity) {
   myIMU.resetInterrupt();
   delay(1);
   myIMU.intConf(sensity, 3, 10, HIGH, -1, true, false, true, false, true);
   delay(1);
   return true;
}

bool chargeMode(bool enable) {
   // charge pin high disables the charger (GND enabled)
   bool chargeState = false;
   bool isCharging = false;
   pinMode(CHG_STAT_PIN, INPUT);
   delay(1);
   if (enable) {
      pinMode(CHG_EN_PIN, INPUT);
      delay(2);
      chargeState = digitalRead(CHG_STAT_PIN);
      if (chargeState == LOW) {
         Serial.println("[CHARGE] on - Charging");
         isCharging = true;
         ledBlink(0, true);
      } else {
         Serial.println("[CHARGE] on - Charge Done");
         digitalWrite(CHG_EN_PIN, LOW);
         ledBlink(500, true);
      }
   } else {
      pinMode(CHG_EN_PIN, OUTPUT);
      digitalWrite(CHG_EN_PIN, HIGH);
      delay(2);
      chargeState = digitalRead(CHG_STAT_PIN);
      if (chargeState == LOW) {
         Serial.println("[CHARGE] off - Charging");
         isCharging = true;
         ledBlink(0, true);
      } else {
         Serial.println("[CHARGE] off - Charge Done");
         digitalWrite(CHG_EN_PIN, LOW);
         ledBlink(500, true);
      }
   }
   return isCharging;
}

// write revision in set to set it in mem. it will restored on every boot
void checkDeviceBatch(int set = 0) {
   if (set > 0) {
      Serial.printf("[SETUP] write device revision to: %d\n", set);
      writeIntToFlash(set, 195);
   }
   int revNumber = readIntFromFlash(195);
   if (revNumber > 10) {
      Serial.printf("[SETUP] Reset revision number from: %d to default\n", revNumber);
      writeIntToFlash(0, 195);
      return;
   }
   if (DEBUG_FLAG) Serial.printf("[SETUP] set device revision to: %d\n", revNumber);
   switch (revNumber) {
      case 1:
         displaySettings.displayQuickRefreshTime = 4450;
         break;
      default:
         Serial.printf("[SETUP] set device revision to default\n");
   }
}

void test() {
   // adcAttachPin(14);  // Any pin that is ADC capable
   ledBlink(0, false);
   char charBuffer[128];
   pinMode(INT_PIN, OUTPUT);
   digitalWrite(INT_PIN, LOW);
   Serial.println("[DEBUG] Test Function");
   startupCounter(true);
   // displaySettings.displayQuickRefreshTime = 2900;//works cold

   restoreSettingsToFlash(EEPROM_SETTINGS_ADR);

   settings.clearscreen = true;
   settings.showBatteryWarning = false;
   settings.showWifiWarning = false;
   Serial.printf("[MEM] Settings - ClearScreen: %d BatteryWarning: %d WifiWarning: %d \n", settings.clearscreen, settings.showBatteryWarning, settings.showWifiWarning);

   saveSettingsToFlash(EEPROM_SETTINGS_ADR);

   settings.clearscreen = true;
   settings.showBatteryWarning = true;
   settings.showWifiWarning = true;
   Serial.printf("[MEM] Settings - ClearScreen: %d BatteryWarning: %d WifiWarning: %d \n", settings.clearscreen, settings.showBatteryWarning, settings.showWifiWarning);

   restoreSettingsToFlash(EEPROM_SETTINGS_ADR);

   while (true) {
      delay(5000);
   }

   while (true) {
      float temperature = temperatureRead();
      Serial.printf("Temp onBoard = %.2f °C\n", temperature);
      bool testCharge = chargeMode(true);
      systemData.vddValue = readVDD(false);
      Serial.printf("VDD: %d mV\n", systemData.vddValue);
      delay(5000);
   };

   // display.powerOff();
   //  BleInit(CLIENT_ID, true);
}

void setup() {
   Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
   pinMode(LED_PIN, OUTPUT);
   digitalWrite(LED_PIN, LOW);
   systemData.vddValue = readVDD(false);
   systemData.usbConnected = usbCheckConnect();
   if (!systemData.usbConnected && systemData.vddValue < 4000) {
      digitalWrite(LED_PIN, HIGH);
      gotToDeepSleep(86000, false, false);
   }

   StartCounter = startupCounter(false);
   EepromInit(EEPROM_SIZE);

   pinMode(12, INPUT);  // USB TEST PINS
   pinMode(13, INPUT);  // USB TEST PINS
   pinMode(INT_PIN, INPUT);
   pinMode(RST_PIN, OUTPUT);
   pinMode(CS_EPD_PIN, OUTPUT);
   pinMode(DC_PIN, OUTPUT);
   analogReadResolution(12);

#if DEBUG
   displayInfos.version = true;
   displayInfos.batteryInfo = true;
   displayInfos.wifiSignal = true;
   displayInfos.deviceInfoString = true;
   myEsp32FOTA.setManifestURL(OTA_URL_DEV);
#else
   displayInfos.deviceInfoString = true;
   myEsp32FOTA.setManifestURL(OTA_URL);
#endif
   char firmwareVersion[] = SOFTWARE_VERSION;

   ledBlink(500, true);
   Serial.begin(115200);
   sleep(1);

   Serial.printf("[MAIN] INIT Device V: %s\n", firmwareVersion);
   if (DEBUG_FLAG) Serial.printf("[MAIN] Current counter value: %u VDD: %d\n", StartCounter, systemData.vddValue);
   if (!SPIFFS.begin(true)) {
      Serial.println("[MEM] SPIFFS initialisation failed!");
   }
   debugFS();
   if (!systemData.usbConnected && systemData.vddValue < BAT_LOW_VALUE) displayInfos.batteryLowBig = true;  // enable bat low display if needed
   WiFi.onEvent(WiFiEvent);
   SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);  // SCK(), MISO(),MOSI(), SS()
   SerialFlash.begin(CS_FLASH_PIN);         // proceed even if begin() fails
   u8g2_for_adafruit_gfx.begin(display);

   setDeviceUid();

   systemData.deviceOrientation = accInit();
   // default is rotationText = 3 and rotationPicture = 2
   if (systemData.deviceOrientation == 2 || systemData.deviceOrientation == 3) {
      displaySettings.rotationText = 1;
      displaySettings.rotationPicture = 0;
   }
   chargeMode(false);  // enable charge mode
   float temperature = temperatureRead();
   if (DEBUG_FLAG) Serial.printf("[MAIN] Temp Main: %.2f °C\n", temperature);
   if (temperature < 21.0) {
      Serial.printf("[MAIN] Low Temp detected: %.2f °C - disable quick refresh\n", temperature);
      displaySettings.quickRefresh = false;
   }

#if DEBUG
   checkDeviceBatch();  // set settings for specific versions (1=old revision)
   // test();              //-----------------test---------please remove
#endif
   onceTicker.once_ms((FAILSAVE_TIMER * 1000) + (WIFI_INIT_TIME * 1000), timeoutFailsave, 0);
   testModeCheck();                   // check if needs to enter deploy state
   bool wifiConnected = wifiSmart();  // Replaces wifi begin

   ledBlink(500, true);
   onceTicker.detach();
   onceTicker.once_ms(FAILSAVE_TIMER * 1000, timeoutFailsave, 0);

#if DEBUG
   if (StartCounter > 2) {
      bool updatedNeeded = myEsp32FOTA.execHTTPcheck();
      if (updatedNeeded) {
         displaySettings.quickRefresh = true;
         waitDisplayComplete(false);
         delay(200);
         displaySetText("DEV OTA Update...", true, true);
         ledBlink(2000, true);
         Serial.println("[OTA] Dev OTA Started.....");
         writeIntToFlash(0, 170);
         resetAll(false, false);
         myEsp32FOTA.execOTA();
         delay(2000);
      } else {
         Serial.println("[OTA] Dev no OTA needed");
      }
   }
#else
   bool updatedNeeded = myEsp32FOTA.execHTTPcheck();
   if (updatedNeeded) {
      displaySettings.quickRefresh = true;
      waitDisplayComplete(false);
      delay(200);
      displaySetText("Updating...", true, true);
      ledBlink(2000, true);
      Serial.println("[MAIN] OTA Started.....");
      writeIntToFlash(0, 170);  // Reset activation counter in case activation is in ota proccess
      resetAll(false, false);
      myEsp32FOTA.execOTA();
      delay(2000);
   } else {
      Serial.println("[MAIN] no OTA needed");
   }
#endif
   startupCounter(true);
   bool activationDone = deployDevice();
   saveSettingsToFlash(EEPROM_SETTINGS_ADR);
   if (!getActivatedFromMem()) {
      debugCheck();
      //  TODO reset if device is still found activated on first wifi connect and not pending
   }
   if (!activationDone) {
      // only enter here if server activation not done or activation on server is confirmed reset state
      Serial.printf("[MAIN] ACT MEM: %d\n", getActivatedFromMem());
      Serial.printf("[MAIN] deviceActivationNotStarted : %d\n", deviceActivationNotStarted);
      Serial.printf("[MAIN] ACT WLCONNEDTED: %d\n", WL_CONNECTED);
      Serial.printf("[MAIN] wifi config: %d \n", wifiSettings.wifiConfig);

      // TODO: this needs improvement also on app side to hanlde activated devices that get reactivated
      if (getActivatedFromMem() && WiFi.status() == WL_CONNECTED && deviceActivationNotStarted) {
         // Reset if server activation state is disabled and device still has wifi, skip on deploy
         if (!wifiSettings.wifiConfig) {
            Serial.printf("[MAIN] Reset Device becaus still wifi - Activation deleted in database\n");
            resetAll(true, true);
         }
         return;
      }
      if (!client.connected()) {
         Serial.println("[MAIN] AWS not connected - Going to sleep");
         gotToDeepSleep(settings.timeout, false, false);
      }
      if (!getActivatedFromMem()) {
         int actCounter = readIntFromFlash(170);
         int maxCounter = round(APPCONNECT_INIT_TIME / 30);
         Serial.printf("[MAIN] Activation Counter: %d/%d\n", actCounter, maxCounter);
         if (actCounter > maxCounter + 2) {
            actCounter = 0;
         }
         if (actCounter % 3 == 0) {
            displayWifiActivate(true);
         }
         if (actCounter < maxCounter) {
            actCounter++;
            writeIntToFlash(actCounter, 170);
            gotToDeepSleep(20);
         } else {
            writeIntToFlash(0, 170);
            gotToDeepSleep(0, true, false);
         }
      }
   }
   writeIntToFlash(0, 170);  // reset activation sleep counter if activated
   setActivatedToMem(true);
   storeSleepTimeMem(settings.timeout);
   // TODO: Force Update after wifi screen if just one connection loss and then back
   // Set EEPROM on wifi screen and read here. If true and picture version is not 0 then update
   bool isUpdate = getVersionUpdate();
   if (!isUpdate) {
      if (newVersionSave <= 0) {
         setUpdateState("update_checked_nopicture");
         displayNoPicture();
      } else {
         setUpdateState("update_checked_noupdate");
      }
      delay(10);
      // WiFi.disconnect(true);
      //  WiFi.setSleep(true);
      if (settings.timeout > 0) {
         gotToDeepSleep(settings.timeout);  // Disable version Check here

      } else {
         gotToDeepSleep(DEFAULT_SLEEP);
      }
   }
   updateDisplayAsync("updatepicture");  // TODO: maybe remove

   if (SPIFFS.usedBytes() > 10000) {
      Serial.println("[MEM] SPIFFS seems to full ...");
   }

   bool success = getImageUrl(false);
   awsConnect(false);
   esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
   delay(10);
}

void loop() {
   if (downloadStart) {
      if (WiFi.status() == WL_CONNECTED) {
         downloadStart = false;
         delay(200);
         int success = loadImageFromWeb(DL_URL);
         if (success == 0) {
            setUpdateState("update_ok");
            writeIntToFlash(newVersionSave, 150);
         } else {
            setUpdateState("update_failed");
            displaySetText("Error: Picture download failed, please try again", false);
         }
      }
      delay(500);
   }
   if (WiFi.status() == WL_CONNECTED) {
      ledBlink(500, true);

   } else {
      ledBlink(200, true);
      if (DEBUG_FLAG)
         Serial.println("[NETWORK] WiFi connection lost ... reconnecting... ");

      WiFi.disconnect(true);
      WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.pss.c_str());
      int WLcount = 0;
      while (WiFi.status() != WL_CONNECTED && WLcount < 200) {
         delay(100);
         ++WLcount;
      }
   }

   if (!downloadStart) {
      // Picture Update Done
      Serial.println("[MAIN] End of Update");
      if (settings.timeout > 0) {
         gotToDeepSleep(settings.timeout);
      } else {
         gotToDeepSleep(DEFAULT_SLEEP);
      }
   }
}
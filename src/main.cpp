// #include <FS.h>
#include "epaper_display.h"
#include "types.h"
#include <HTTPClient.h>
#include <WiFi.h>

#define DEST_FS_USES_SPIFFS
#include <Arduino.h>
#include <ArduinoJson.h>
#include <MQTT.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <esp32fota.h>
#include <rom/crc.h>
#include <rom/rtc.h>

#include "EEPROM.h"
#include "SPIFFS.h"
#include "SdFat.h"
#include "Ticker.h"
#include "driver/rtc_io.h"
#include "kxtj3-1057.h"
#include "secrets.h"

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

#ifdef EPD_TYPE_13INCH
#define OTA_URL ENV_OTA_URL_13
#define OTA_URL_DEV ENV_OTA_URL_DEV_13
#else
#define OTA_URL ENV_OTA_URL
#define OTA_URL_DEV ENV_OTA_URL_DEV
#endif

// E-Paper Pins
#define DISP_POWER 12

// flash
#define CS_FLASH_PIN 21
#define CS_SD_PIN 13

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

#ifdef EPD_TYPE_13INCH
#define EPD_TYPE_IDENTIFIER "epd13-"
#define VDD_CORRECTION_FACTOR 6.68
#else
#define EPD_TYPE_IDENTIFIER "epd7-"
#define VDD_CORRECTION_FACTOR 2.28
#endif

#define BAT_LOW_VALUE 4300
#define BAT_OFF_VALUE 4200
#define OFFSET_BITMAP 4
#define uS_TO_S_FACTOR 1000000ULL
#define LENGTH(x) (strlen(x) + 1)
#define EEPROM_SIZE 1024
#define EEPROM_SETTINGS_ADR 500
#define DEFAULT_WIFI_PW ENV_WIFI_PW_DEPLOY
#define DEFAULT_WIFI_SSID ENV_WIFI_SSID_DEPLOY
#define DEFAULT_SLEEP 3600
#define WIFI_INIT_TIME 300
#define APPCONNECT_INIT_TIME 300
#define FAILSAVE_TIMER 180
#define MAX_RECONNECTS 10
#define RECONNECT_LOOP_TIME 60
#define SLEEP_RECALCULATION_PERIOD_SECONDS 30
#define LED_DIM_VALUE 5

#if DEBUG
const bool DEBUG_FLAG = true;
#else
const bool DEBUG_FLAG = false;
#endif

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

// settings set via MQTT/IOT
Settings settings = {
    .timeout = DEFAULT_SLEEP,
    .lut = "default",
    .clearscreen = true,
    .showBatteryWarning = true,
    .showWifiWarning = true,
    .sleepDisabled = false};

// system read data
SystemData systemData = {
    .wakeupCause = SYSTEM_RESET,
    .vddValue = 5000,
    .ledDimValue = 100,
    .sleepPrediction = DEFAULT_SLEEP,
    .newSleepTimeSet = false,
    .usbConnected = true,
    .deviceOrientation = 0,
    .displayPowerOn = false,
    .sdIsInit = false};

esp_sleep_wakeup_cause_t wakeup_reason;
KXTJ3 myIMU(ACC_ADDR);  // Address can be 0x0E or 0x0F
esp32FOTA myEsp32FOTA("esp32-fota-http", SOFTWARE_VERSION);
WiFiClientSecure net = WiFiClientSecure();
MQTTClient client(512);  // Buffer to 256 Byte
Preferences preferences;
Ticker tickerFailsave;
Ticker perdiodicLed;
Ticker perdiodicLedOff;
Ticker periodicAccCheck;
Ticker tickerStatupCounter;
SerialFlashFile saveFile;
SdFs sd;
RTC_DATA_ATTR timeval previousWakeup = timeval{.tv_sec = 0, .tv_usec = 0};

NimBLECharacteristic* wifiConnectedCharacteristic;
NimBLEAdvertising* pAdvertising;
static NimBLEServer* pServer;

char HOST_ADDRESS[] = ENV_AWS_IOT_ENDPOINT;

uint32_t freeHeap = 0;
int newVersionSave = 0;
int httpFileSize = 0;
int StartCounter = 0;

char CLIENT_ID[20];
char CLIENT_KEY[30];
char TOPIC_RECEIVE[64];
char DL_URL[50];

#define BLE_BUFFER_SIZE 19200
uint16_t bleWriteBufferPos = 0;
bool fwUpdateInProgress = false;
uint8_t* bleWriteBuffer = nullptr;
uint32_t calcCRC32(const uint8_t* data, size_t len) {
   return crc32_le(0, data, len);
}

bool downloadStart = true;
bool deviceActivated = false;
bool deviceActivationNotStarted = false;
bool deviceActivationReset = false;
bool isNewVersion = false;
bool isDlUrl = false;
bool periodicLedIsOn = false;
int periodicLedTimeout = 0;
bool isTestMode = false;  // enabled if the device is in deployment and tests are running
bool buttonWake = false;  // true if wakeup via reset button
bool stopAccRecheck = false;
bool isOrientUpdate = false;
bool isBleClientConnected = false;

void ledBlink(int timeout, bool on, int dimValue = 100);
bool awsConnect(bool connect);
bool getVersionUpdate(void);
void startupCounter(int reset);
bool resetAll(bool resetActivation, bool resetWifi);
bool wifiConnectionLostStore(bool setNoWifi = false, bool reset = false);
bool getActivatedFromMem(void);
void debugFS(void);
bool BleInit(String deviceId, bool enable);
bool setUpdateState(String state);
void writeIntToFlash(int value, int startAddr);
int storeSleepTimeMem(int updateTime = 0);
void gotToDeepSleep(int seconds, bool showScreen = true, bool motionWake = true);
bool accIntSet(int sensity);
void checkOrientationInBackground(int setOrientValue, bool isRunning = true);
bool chargeMode(bool enable);
bool usbInit();
bool usbCheckConnect();
void debugCheck();
int calculateSleepDuration(int defaultTimeout, bool forceReset, bool getDataOnly = false);
bool powerSupplyDisplay(bool enable);
bool sdInit(bool forceFormat = false);
void sdTest(bool doLog = false);
bool downloadBMPToFlash(const char* url, const char* filename, bool forceDownload = false);
wakeup_reason_t getWakeupReason();

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
   Serial.println("[MAIN] Failsave Timeout - Sleep Device");
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
   if (DEBUG_FLAG) Serial.print("[AWS RX] Message Received: ");
   if (DEBUG_FLAG) Serial.println(payload);
   JsonDocument doc;
   deserializeJson(doc, payload);

   if (doc["url"].is<JsonString>()) {
      Serial.println("[AWS RX] Picture URL Message");
      const char* dlurl = doc["url"];
      sprintf(DL_URL, "%s%s", ENV_DOWNLOAD_URL, dlurl);
      isDlUrl = true;
   }

   if (doc["ota"].is<JsonString>()) {
      const char* otaUrlTemp = doc["ota"];
      size_t otaUrlLength = strlen(otaUrlTemp);
      Serial.printf("[AWS RX] OTA URL received: '%s' (%d) \n", otaUrlTemp, otaUrlLength);
      if (otaUrlLength > 5) {
         myEsp32FOTA.setManifestURL(otaUrlTemp);
         displaySetQuickRefresh(true);
         waitDisplayComplete(false);
         delay(200);
         displaySetText("Updating Software...", true, true);
         ledBlink(2000, true);
         Serial.println("[OTA] OTA via MQTT Started.....");
         writeIntToFlash(0, 170);  // Reset activation counter in case activation is in ota proccess
         resetAll(false, false);
         myEsp32FOTA.execOTA();
         delay(2000);
      }
   }

   if (doc["t"].is<long>()) {
      newVersionSave = doc["t"].as<long>();
      isNewVersion = true;
   }

   if (doc["act"].is<JsonString>()) {
      const char* activated;
      activated = doc["act"];
      if (strcmp(activated, "activated") == 0) {
         Serial.println("[AWS RX] Device is activated");
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
            if (settings.timeout < 60) {
               settings.timeout = 60;  // minimum timeout 60 seconds TODO: add trigger to keep device always on
            }
         }
         if (doc["clearscreen"].is<bool>()) {
            settings.clearscreen = doc["clearscreen"].as<bool>();
         }
         if (doc["sleepdisabled"].is<bool>()) {
            settings.sleepDisabled = doc["sleepdisabled"].as<bool>();
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
         Serial.printf("[AWS RX] SETTINGS - Sleep: %d (disable:%d) Lut: %s Overlay: %d-%d \n", settings.timeout, settings.sleepDisabled, settings.lut, settings.showWifiWarning, settings.showBatteryWarning);
         deviceActivated = true;
      }
      if (strcmp(activated, "not_started") == 0) {
         Serial.println("[AWS RX] Device activation not started");
         deviceActivationNotStarted = true;
      }
      if (strcmp(activated, "reset") == 0) {
         Serial.println("[AWS RX] Device activation reset");
         deviceActivationReset = true;
      }
   }
}
void ledBlinkFunctionOff() {
   analogWrite(LED_PIN, 0);
   perdiodicLedOff.detach();
}

void ledBlinkFunction() {
   analogWrite(LED_PIN, systemData.ledDimValue);
   perdiodicLedOff.attach_ms(periodicLedTimeout * 0.2, ledBlinkFunctionOff);
}

// set blink with timeout in ms
void ledBlink(int timeout, bool on, int dimValue) {
   pinMode(LED_PIN, OUTPUT);
   if (dimValue > 100) dimValue = 100;
   if (dimValue < 0) dimValue = 0;
   systemData.ledDimValue = (dimValue * 255) / 100;
   if (DEBUG_FLAG) Serial.printf("[LED] Set DIM: %d Set Time: %d On: %d (dimInput:%d)\n", systemData.ledDimValue, timeout, on, dimValue);

   if (on) {
      if (timeout <= 0) {
         perdiodicLed.detach();
         perdiodicLedOff.detach();
         analogWrite(LED_PIN, systemData.ledDimValue);  // Dim brightness (0-255)
         return;
      }
      periodicLedIsOn = true;
      periodicLedTimeout = timeout * 2;
      perdiodicLed.attach_ms(periodicLedTimeout, ledBlinkFunction);

   } else {
      perdiodicLed.detach();
      perdiodicLedOff.detach();
      analogWrite(LED_PIN, 0);
      delay(periodicLedTimeout * 0.3);
      analogWrite(LED_PIN, 0);
      periodicLedIsOn = false;
   }
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
#ifdef EPD_TYPE_13INCH
   digitalWrite(BAT_VOLT_EN_PIN, HIGH);
#else
   digitalWrite(BAT_VOLT_EN_PIN, LOW);
#endif
   delay(3);
   int retries = 1;
   if (!singleReading) {
      retries = 5;
   }
   float rawValue = 0;
   int count = 0;
   for (size_t i = 0; i < retries; i++) {
      float readValue = analogRead(BAT_VOLT_SENSE_PIN);
      if (DEBUG_FLAG) Serial.printf("Voltage Read(%d) : %.1fmV corrected: %.1fmV \n", i, readValue, (readValue * VDD_CORRECTION_FACTOR));
      delayMicroseconds(20000);
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
   DataLayout data3;
   EEPROM.get(addr, data3);
   return data3.integer;
}

void writeIntToFlash(int value, int startAddr) {
   DataLayout data2;
   data2.integer = value;
   EEPROM.put(startAddr, data2);
   EEPROM.commit();
}

void saveSettingsToFlash(int startAddr) {
   writeIntToFlash(settings.clearscreen, startAddr);
   writeIntToFlash(settings.showBatteryWarning, startAddr + 5);
   writeIntToFlash(settings.showWifiWarning, startAddr + 10);
   writeIntToFlash(settings.sleepDisabled, startAddr + 15);
   Serial.println("[MEM] Settings saved to EEPROM");
}

void restoreSettingsToFlash(int startAddr) {
   settings.clearscreen = readIntFromFlash(startAddr);
   settings.showBatteryWarning = readIntFromFlash(startAddr + 5);
   settings.showWifiWarning = readIntFromFlash(startAddr + 10);
   settings.sleepDisabled = readIntFromFlash(startAddr + 15);
   Serial.println("[MEM] Settings restored from EEPROM");
   if (DEBUG_FLAG) {
      Serial.printf("[MEM] Settings - ClearScreen: %d BatteryWarning: %d WifiWarning: %d SleepDisabled: %d\n", settings.clearscreen, settings.showBatteryWarning, settings.showWifiWarning, settings.sleepDisabled);
   }
}

// charger active, refresh screen, scan wifi, ble adv
void emvTest(int counter) {
   bool isUsb = false;
   bool isCharging = false;
   systemData.deviceOrientation = 0;
   displaySetRotation(1);
   if (usbCheckConnect()) {
      isCharging = chargeMode(true);
      isUsb = true;
   } else {
      isCharging = chargeMode(false);
      isUsb = false;
   }
   sdInit();
   sdTest();
   char softwareStringDebug[256];
   sprintf(softwareStringDebug, "Device Test (%s) USB:%d Charger:%d Loop: %d", SOFTWARE_VERSION, isUsb, isCharging, counter);
   displaySetText(softwareStringDebug, false, false);
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
   BleInit(CLIENT_ID, true);
}

bool wifiSmart() {
   bool isWifi = true;
   bool isReconnect = false;
   uint8_t reconnectCount = readIntFromFlash(140);
   bool doReset = false;
   int wifiTimeout = 10000UL;
   if (StartCounter >= 5) {
      doReset = true;
      wifiTimeout = 3000UL;
   }
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

   if (isWifi == false && isTestMode == false && doReset == false) {
      // BleInit(CLIENT_ID, true);
      powerSupplyDisplay(true);
      wifiTimeout = 5000UL;
      updateDisplayAsync("wifiactivate");
   }
   // try to connect to current wifi
   wifiSettings.wifiRetries = 0;
   for (int i = 0; i < 5; i++) {
      wifiSettings.wifiRetries++;
      Serial.printf("[NETWORK] connect try cont %d / %d\n", i + 1, 5);
      if (doReset && i >= 1) {
         break;  // leave wifi search quick on reset
      }
      if (wifiSettings.ssid.length() < 2) {
         Serial.println("[NETWORK] Stop connect because no wifi set");
         break;
      }
      WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.pss.c_str());
      WiFi.waitForConnectResult(wifiTimeout);
      if (WiFi.status() == WL_CONNECTED) {
         break;
      }
      WiFi.disconnect(true);

      delay(1000);
      // if (!isWifi && i >= 1 && waitDisplayComplete(true)) {
      if (!isWifi && i >= 1) {
         BleInit(CLIENT_ID, true);
         Serial.println("[NETWORK] stop search because default wifi");  // skip the intense connect if default wifi
         break;
      }
   }

   if (isTestMode) return true;  // skip wifi activation in deployment

   wifiSettings.wifiQuality = WiFi.RSSI();

   if (doReset && buttonWake) resetAll(true, true);

   // Serial.printf("[NETWORK] Wifi Connect done : %d\n", (WiFi.status() == WL_CONNECTED));
   //  check if reset is triggered via button
   // if wifi was connected in the past (device is activated but no wifi)
   // restart bluetooth to set new wifi

   if (WiFi.status() != WL_CONNECTED && getActivatedFromMem() && buttonWake == true) {
      Serial.println("[NETWORK] BLE reprovisioning for wifi reconnect");
      BleInit(CLIENT_ID, true);
      int loopCount = 0;
      // loop to check bluetooth wifi change
      while (WiFi.status() != WL_CONNECTED) {
         delay(1000);
         if (!isBleClientConnected) {
            loopCount++;
            Serial.print("o");
         }
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
      BleInit(CLIENT_ID, true);
      while (true) {
         if (!isBleClientConnected) {
            countAttempt++;
            Serial.print(".");
         }
         delay(250);
         if (countAttempt % (4 * 10) == 0 && !isBleClientConnected) {
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
               // displaySetText("Bitte die WLAN Daten überprüfen!", false);
               // return false;
            }
         }
         if (WiFi.status() == WL_CONNECTED) {
            Serial.println("[NETWORK] Wifi got IP (softAP)");
            break;
         }
         // try to connect wifi if ble data is set
         if (wifiSettings.bleSSID.length() > 1 && wifiSettings.blePASS.length() > 1) {
            Serial.println("[NETWORK] got all BLE, try to connect with data");
            wifiSettings.wifiIsConnected = true;
            wifiConnectedCharacteristic->setValue(wifiSettings.wifiIsConnected);
            WiFi.begin(wifiSettings.bleSSID.c_str(), wifiSettings.blePASS.c_str());
            WiFi.waitForConnectResult(10000);
            if (WiFi.status() == WL_CONNECTED) {
               Serial.println("[NETWORK] Wifi got IP (softAP)");
               break;
            }
            WiFi.disconnect(true);
            wifiSettings.blePASS = "";
            wifiSettings.wifiIsConnected = false;
            wifiConnectedCharacteristic->setValue(wifiSettings.wifiIsConnected);
            // break;
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
   restoreSettingsToFlash(EEPROM_SETTINGS_ADR);  // restore settings that are in memory if no cloud data available
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
         displaySetOverlayOption(WIFI_OFFLINE_BIG, true);
         delay(10);
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
   WiFiClientSecure secureClient;
   secureClient.setInsecure();
   if (url.indexOf("https:") >= 0) {
      http.begin(secureClient, url);
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
      isBleClientConnected = true;
      if (wifiSettings.bleInitOk) {
         NimBLEDevice::startAdvertising();
      }
   }
   void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
      Serial.printf("\n[BLE] Client disconnected");
      isBleClientConnected = false;
      if (bleWriteBuffer != nullptr) {
         free(bleWriteBuffer);
         bleWriteBuffer = nullptr;
      }
      fwUpdateInProgress = false;
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
      std::string uuidStr = pCharacteristic->getUUID().toString();
      if (uuidStr == "10000004-0000-0000-0000-000000000001") {
         pCharacteristic->setValue((uint8_t*)&bleWriteBufferPos, 2);
         return;
      }
      Serial.printf("%s : onRead(), value: %s\n",
                    pCharacteristic->getUUID().toString().c_str(),
                    pCharacteristic->getValue().c_str());
   };

   void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
      std::string uuidStr = pCharacteristic->getUUID().toString();
      size_t dataLen = pCharacteristic->getValue().length();

      if (uuidStr == "10000004-0000-0000-0000-000000000001") {
         String cmd = pCharacteristic->getValue().c_str();
         Serial.printf("[BLE] Upload CMD: %s\n", cmd.c_str());
         if (cmd == "START_FW") {
            if (bleWriteBuffer == nullptr) {
               bleWriteBuffer = (uint8_t*)malloc(BLE_BUFFER_SIZE);
            }
            if (bleWriteBuffer == nullptr) {
               Serial.println("[BLE] ERROR: Failed to allocate memory for firmware buffer!");
               return;
            }
            fwUpdateInProgress = true;
            bleWriteBufferPos = 0;
            tickerFailsave.detach();
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
               Update.printError(Serial);
               if (bleWriteBuffer != nullptr) {
                  free(bleWriteBuffer);
                  bleWriteBuffer = nullptr;
               }
            } else {
               Serial.println("[BLE] Firmware Update STARTED.");
            }
         } else if (cmd == "FLUSH") {
            if (fwUpdateInProgress && bleWriteBufferPos > 0 && bleWriteBuffer != nullptr) {
               Update.write(bleWriteBuffer, bleWriteBufferPos);
               bleWriteBufferPos = 0;
            }
         } else if (cmd == "CLEAR") {
            bleWriteBufferPos = 0;
         } else if (cmd == "END_FW") {
            if (fwUpdateInProgress && bleWriteBufferPos > 0 && bleWriteBuffer != nullptr) {
               Update.write(bleWriteBuffer, bleWriteBufferPos);
               bleWriteBufferPos = 0;
            }
            fwUpdateInProgress = false;
            if (bleWriteBuffer != nullptr) {
               free(bleWriteBuffer);
               bleWriteBuffer = nullptr;
            }
            if (Update.end(true)) {
               Serial.println("[BLE] Firmware Update SUCCESS. Rebooting...");
               delay(500);
               ESP.restart();
            } else {
               Update.printError(Serial);
               Serial.println("[BLE] Firmware Update FAILED.");
            }
         }
      } else if (uuidStr == "10000003-0000-0000-0000-000000000001") {
         const uint8_t* pData = pCharacteristic->getValue().data();
         if (fwUpdateInProgress && pData && dataLen > 4 && bleWriteBuffer != nullptr) {
            uint32_t packetCrc = pData[0] | (pData[1] << 8) | (pData[2] << 16) | (pData[3] << 24);
            size_t actualDataLen = dataLen - 4;
            uint32_t calculatedCrc = calcCRC32(pData + 4, actualDataLen);

            if (packetCrc == calculatedCrc) {
               if (bleWriteBufferPos + actualDataLen <= BLE_BUFFER_SIZE) {
                  memcpy(bleWriteBuffer + bleWriteBufferPos, pData + 4, actualDataLen);
                  bleWriteBufferPos += actualDataLen;
               } else {
                  Serial.println("[BLE] ERROR: RAM buffer overflow!");
               }
            } else {
               Serial.printf("[BLE] CRC Error!\n");
            }
         }
      } else {
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

   // wifiDataService->start();
   // deviceDataService->start();

   wifiSsidCharacteristic->setValue("wifi-ssid");
   wifiPwCharacteristic->setValue("wifi-pw");
   wifiConnectedCharacteristic->setValue(wifiSettings.wifiIsConnected);
   wifiScanCharacteristic->setValue(wifiSsidScan);

   NimBLEService* epaperSettingsService = pServer->createService("10000000-0000-0000-0000-000000000001");
   NimBLECharacteristic* uploadDataCharacteristic = epaperSettingsService->createCharacteristic("10000003-0000-0000-0000-000000000001", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
   NimBLECharacteristic* uploadCmdCharacteristic = epaperSettingsService->createCharacteristic("10000004-0000-0000-0000-000000000001", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);

   uploadDataCharacteristic->setCallbacks(&chrCallbacks);
   uploadCmdCharacteristic->setCallbacks(&chrCallbacks);

   pAdvertising = NimBLEDevice::getAdvertising();
   pAdvertising->setName(deviceId.c_str());
   pAdvertising->addServiceUUID(wifiDataService->getUUID());
   pAdvertising->addServiceUUID(deviceDataService->getUUID());
   pAdvertising->addServiceUUID(epaperSettingsService->getUUID());
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
   WiFi.setSleep(false);
   HTTPClient http;
   http.setTimeout(10000);
   http.setReuse(true);

   WiFiClientSecure secureClient;
   secureClient.setInsecure();

   if (url.indexOf("https:") >= 0) {
      Serial.println("[DL] Download HTTPS");
      http.begin(secureClient, url);
   } else {
      Serial.println("[DL] Download HTTP");
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
            saveFile = SerialFlash.open(fileName.c_str());
            SerialFlash.remove(saveFile);
            saveFile.close();
         } else {
            saveFile.close();
         }
         Serial.printf("[FLASH] Create File Size: %d.\n", httpFileSize);
         if (SerialFlash.createErasable(fileName.c_str(), httpFileSize)) {
            saveFile = SerialFlash.open(fileName.c_str());
            if (!saveFile) {
               Serial.println("[FLASH] Error opening file");
               http.end();
               return -4;
            }
         } else {
            Serial.println("[FLASH] Error creating file");
            http.end();
            SerialFlash.eraseAll();
            while (SerialFlash.ready() == false) {
               delay(100);
            }
            Serial.println("[FLASH] Erase Done");
            return -3;
         }

         Serial.print("[DL] Download Size: ");
         Serial.println(len);
         int buff_size = 2048;
         unsigned char* buff = (unsigned char*)malloc(buff_size);

         WiFiClient* stream = http.getStreamPtr();
         size_t downloaded_data_size = 0;
         int bytesLeft = len;

         while (http.connected() && (bytesLeft > 0 || len == -1)) {
            size_t size = stream->available();
            if (size > 0) {
               int c = stream->readBytes(buff, ((size > buff_size) ? buff_size : size));
               saveFile.write(buff, c);
               if (bytesLeft > 0) {
                  bytesLeft -= c;
               }
               downloaded_data_size += c;
            } else {
               delay(1);
            }
            if (WiFi.status() != WL_CONNECTED) {
               http.end();
               saveFile.close();
               free(buff);
               return -4;
            }
         }
         systemFileSize = saveFile.size();
         int dif = systemFileSize - httpFileSize;
         int maxDif = (httpFileSize / 80) * -1;
         if (maxDif > -2000) {
            maxDif = -5000;
         }
         Serial.printf("[FLASH] Final Size: %d (Diff:%d/%d).\n", systemFileSize, dif, maxDif);
         free(buff);
         if (dif < maxDif) {
            // success = -8;
         }
         saveFile.close();
      }
   } else {
      Serial.println("[DL] Error on HTTP request");
      Serial.println(httpCode);
      success = -2;
   }
   http.end();
   return success;
}

bool downloadBMPToFlash(const char* url, const char* filename, bool forceDownload) {
   if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[BMP] Download failed: WiFi not connected");
      return false;
   }

   // Prepare flash
   if (SerialFlash.exists(filename)) {
      SerialFlashFile tempFile = SerialFlash.open(filename);
      if (forceDownload) {
         Serial.print("[BMP] File Exists, redownload start... ");
         SerialFlash.remove(tempFile);
      } else {
         Serial.print("[BMP] File Exists, skip download... ");
         return true;
      }
   }

   HTTPClient http;
   Serial.println(url);

   http.begin(url);
   int httpCode = http.GET();

   if (httpCode != HTTP_CODE_OK) {
      Serial.printf("[BMP] HTTP GET failed, code: %d\n", httpCode);
      http.end();
      return false;
   }

   int len = http.getSize();
   if (len <= 0) {
      Serial.println("[BMP] Invalid file size");
      http.end();
      return false;
   }

   if (!SerialFlash.createErasable(filename, len)) {
      SerialFlash.eraseAll();
      while (SerialFlash.ready() == false) {
         delay(100);
      }
      if (!SerialFlash.createErasable(filename, len)) {
         Serial.println("[BMP] Failed to create file in flash");
         http.end();
         return false;
      }
   }

   SerialFlashFile file = SerialFlash.open(filename);
   if (!file) {
      Serial.println("[BMP] Failed to open file in flash");
      http.end();
      return false;
   }

   WiFiClient* stream = http.getStreamPtr();
   uint8_t buffer[1024];
   int bytesWritten = 0;

   if (DEBUG_FLAG) Serial.println("[BMP] Streaming to SerialFlash...");
   int bytesLeft = len;
   while (http.connected() && bytesLeft > 0) {
      size_t size = stream->available();
      if (size > 0) {
         int c = stream->readBytes(buffer, ((size > sizeof(buffer)) ? sizeof(buffer) : size));
         file.write(buffer, c);
         bytesWritten += c;
         bytesLeft -= c;
      } else {
         delay(1);
      }
   }

   file.close();
   http.end();
   Serial.printf("[BMP] Download complete. Wrote %d bytes to %s\n", bytesWritten, filename);
   return true;
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
   if (DEBUG_FLAG) Serial.printf("[MEM] Wifi Connection lost state stored: %d\n", actualState);
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
         systemData.newSleepTimeSet = true;
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

int loadImageFromWeb(String url, String fileName) {
   if (url.length() < 1)
      return -2;
   Serial.print("[DL] Download Image file: ");
   Serial.println(url);
   // int setImage = 0;
   int downloadOk = 0;
   String newUrl = getRedirect(url);
   for (int i = 0; i <= 5; i++) {
      debugFS();
      downloadOk = downloadAndSaveFile(fileName, newUrl);
      if (downloadOk == 0) {
         return 0;
      } else {
         if (WiFi.status() != WL_CONNECTED) {
            WiFi.disconnect(true);
            WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.pss.c_str());
            delay(3000);
         }
      }
   }
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
      isNewVersion = false;
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
      net.stop();
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
      Serial.println("[AWS] Error: Certs not found");
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
      client.loop();
      delay(300);
      if (counter > 20) {
         return false;
      }
   }
   sprintf(TOPIC_RECEIVE, "$aws/things/%s/epaper/receive", CLIENT_ID);
   client.subscribe(TOPIC_RECEIVE);
   client.loop();
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
      if (client.publish(TOPIC_SEND, payload, false, 0))  // Publish the message(Temp and humidity)
      {
         Serial.printf("[AWS] Set State to: %s \n", state.c_str());
         client.loop();
         return true;
      } else {
         Serial.println("[AWS] Set State Error!");
      }
      counter++;
      delay(1000);
   }
   return true;
}

bool deactivateDevice() {
   if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[AWS] Deactivate failed: WiFi not connected");
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
      if (client.publish(TOPIC_ACTIVATE, payload, false, 0))  // Publish the message(Temp and humidity)
      {
         Serial.println("[AWS] Request Remove Device");
         client.loop();
      } else {
         Serial.println("[AWS] Request Remove Device FAILED");
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
      Serial.print("[MAIN] CERT FILE: ");
      Serial.println(file.name());

      file = root.openNextFile();
   }
   root.close();
   file.close();
   if (fileCount < 2) {
      Serial.println("[AWS] Error: Certs not found");
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
         if (client.publish(TOPIC_ACTIVATE, payload, false, 0)) {
            Serial.println("[AWS] Request Remove Device");
            client.loop();
         } else {
            Serial.println("[AWS] Request Remove Device FAILED");
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
   int sleepTime = storeSleepTimeMem();
   int predictedSleepTime = calculateSleepDuration(sleepTime, false, true);

   sprintf(payload, "{\"act\": 1,\"v\": \"%s\",\"file\": \"%d\",\"bat\": \"%d\",\"wake\": \"%d\",\"wifi\": \"%d, %d\",\"usb\": \"%d\",\"orient\": \"%d\",\"timeout\": \"%d\",\"timeoutPredict\": \"%d\"}", SOFTWARE_VERSION, oldVersion, systemData.vddValue, systemData.wakeupCause, wifiSettings.wifiRetries, wifiSettings.wifiQuality, systemData.usbConnected, systemData.deviceOrientation, sleepTime, predictedSleepTime);  // Create the payload for publishing
   int counter = 0;
   while (counter < 7 && deviceActivated == false && deviceActivationNotStarted == false) {
      awsConnect(true);
      if (client.publish(TOPIC_ACTIVATE, payload)) {
         Serial.println("[AWS] Request Activation");
         client.loop();
      } else {
         Serial.println("[AWS] Request Activation FAILED");
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
            Serial.println("[AWS] Request Image URL");
         } else {
            Serial.println("[AWS] Request Image URL FAILED");
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

void printDebugInfo() {
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
   int accTest = accInit();
   powerSupplyDisplay(true);
   delay(100);
   sdInit(true);

   // test flash
   int buffLen = 4;
   char testbuff[buffLen] = {'a', 'b', '1', '2'};
   char testbuffread[buffLen];
   SerialFlashFile saveFileTest;
   SerialFlashFile saveFileTestRead;
   if (SerialFlash.exists("testfile")) {
      Serial.println("[FLASH] Test File delete");
      SerialFlashFile file = SerialFlash.open("testfile");
      file.erase();
      saveFileTest.close();
   }
   SerialFlash.createErasable("testfile", 4);
   saveFileTest = SerialFlash.open("testfile");
   saveFileTest.write(testbuff, buffLen);
   saveFileTest.close();
   int testFileSize = saveFileTest.size();
   saveFileTestRead = SerialFlash.open("testfile");
   saveFileTestRead.seek(0);
   saveFileTestRead.read(testbuffread, 4);
   saveFileTestRead.close();

   info[1] = "";

   info[2] = "Device Status:";
   info[3] = "Activated: " + String(deviceActivated) + " NotStarted: " + String(deviceActivationNotStarted) + " Reset: " + String(deviceActivationReset);
   info[4] = "OTA URL: " + String(OTA_URL);
   info[5] = "DEV OTA URL: " + String(OTA_URL_DEV);
   info[6] = "";

   info[7] = "Flash:";
   info[8] = "Activated: " + String(getActivatedFromMem()) + " ActCount: " + String(readIntFromFlash(170)) + " FileVers: " + String(readIntFromFlash(150)) + " TestFile: " + String(testbuffread);
   info[9] = "SSID: " + ssidRead + " PW: " + pwRead;
   info[10] = "LINK: " + String(wifiSignal) + " BAT: " + String(systemData.vddValue) + "V USB: " + String(isUsb) + " ACC: " + String(accTest);
   Serial.println("[TEST] START");
   for (int i = 0; i < textLines; i++) {
      Serial.println(info[i]);
   }
   sdTest();
   Serial.println("[TEST] END");
}
// sleep x seconds
void gotToDeepSleep(int wakeuptimeout, bool showScreen, bool motionWake) {
   Serial.printf("[MAIN] Going to Sleep for %d seconds (MotionWake: %d)\n", wakeuptimeout, motionWake);
   initEpaperDisplay(SPI);
   checkOrientationInBackground(0, false);
   startupCounter(true);
   if (!settings.sleepDisabled) WiFi.disconnect(true);
   if (motionWake) {
      accIntSet(80);  // Set acc int wakeup /TODO: disable if no motion wakeup
   } else {
      accIntSet(0);
   }
   // struct timeval currentTime;
   // gettimeofday(&currentTime, NULL);
   // previousWakeup = currentTime;  // this is important for next wakeup calculation, do not delete it.
   waitDisplayComplete(false);
   if (wakeuptimeout <= 0 && showScreen) {
      displayTurnOn();  // Ich schlafe screen
      waitDisplayComplete(true);
   }
   ledBlink(0, false);
   deinitDisplay();
   delay(5);
   powerSupplyDisplay(false);
   pinMode(LED_PIN, INPUT);
   Serial.flush();
   delay(10);
   if (!settings.sleepDisabled) setCpuFrequencyMhz(40);
   delay(5);

   pinMode(DISP_POWER, INPUT);
   if (!settings.sleepDisabled) pinMode(LED_PIN, INPUT);
   pinMode(CS_FLASH_PIN, INPUT);
   pinMode(SCK_PIN, INPUT);
   pinMode(MOSI_PIN, INPUT);
   pinMode(MISO_PIN, INPUT);
   pinMode(I2C_SDA_PIN, INPUT);
   pinMode(I2C_SCL_PIN, INPUT);
   pinMode(BAT_VOLT_EN_PIN, INPUT);
   pinMode(CHG_EN_PIN, INPUT);
   pinMode(CS_SD_PIN, INPUT);
   if (settings.sleepDisabled) {
      int versionStored = newVersionSave;
      Serial.printf("[SLEEP] enter soft sleep\n");
      tickerFailsave.detach();
      awsConnect(true);
      ledBlink(2000, true);
      while (true) {
         delay(100);
         client.loop();
         if (newVersionSave != versionStored) {
            Serial.println("[SLEEP] new version detected during soft sleep, restarting to update");
            ESP.restart();
            break;
         }
         // TODO: add watchdog feed and also add fallback. disable timeout
      }
   }
   esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
   if (wakeuptimeout > 0) {
      esp_sleep_enable_timer_wakeup(wakeuptimeout * uS_TO_S_FACTOR);
   } else {
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
   }
   if (motionWake) {
      esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(GPIO_NUM_0), ESP_EXT1_WAKEUP_ANY_HIGH);
      rtc_gpio_pulldown_en(GPIO_NUM_0);
      rtc_gpio_pullup_dis(GPIO_NUM_0);
   }
   esp_deep_sleep_start();
   ESP.restart();  // will never reach in deep sleep
}

int calculateSleepDuration(int defaultTimeout, bool forceReset, bool getDataOnly) {
   struct timeval now;
   gettimeofday(&now, NULL);

   if (getDataOnly) {
      time_t diff = now.tv_sec - previousWakeup.tv_sec;
      int predictedSleep = defaultTimeout - diff;
      if (predictedSleep < SLEEP_RECALCULATION_PERIOD_SECONDS || predictedSleep <= 0) {
         predictedSleep = defaultTimeout;
      }
      Serial.printf("[SLEEP] Return predicted sleep data only: %d \n", predictedSleep);

      return predictedSleep;
   }

   if (systemData.wakeupCause == wakeup_reason_t::TIMER) {
      if (DEBUG_FLAG) Serial.printf("[SLEEP] no manual wakeup (timer) - using full wakeup period...\n");
      previousWakeup = now;
      return defaultTimeout;
   }
   if (systemData.wakeupCause == wakeup_reason_t::BUTTON) {
      // TODO: cloud time feedback is needed here to calculate new wakeup
      if (DEBUG_FLAG) Serial.printf("[SLEEP] button wakeup - not able to calculate new sleep internally...\n");
      previousWakeup = now;
      return defaultTimeout;
   }
   if (forceReset) {
      if (DEBUG_FLAG) Serial.printf("[SLEEP] force reset - using full wakeup period...\n");
      previousWakeup = now;
      return defaultTimeout;
   }

   if (previousWakeup.tv_sec > now.tv_sec) {
      if (DEBUG_FLAG) Serial.printf("[SLEEP] detected invalid previous wakeup, resetting...\n");
      previousWakeup = timeval{
          .tv_sec = 0,
          .tv_usec = 0};
   }

   time_t diff = now.tv_sec - previousWakeup.tv_sec;

   if (diff <= 0) {
      if (DEBUG_FLAG) Serial.printf("[SLEEP] detected non positive sleep difference, resetting...\n");
      previousWakeup = now;
      diff = 0;
   }

   Serial.printf("[SLEEP] current time: %lld, previous time: %lld, difference: %lld\n", now.tv_sec, previousWakeup.tv_sec, diff);
   int predictedSleep = defaultTimeout - diff;
   // if the user refreshes less than one minute before the actual refresh will occur, skip that next refresh and use the current one instead.
   if (predictedSleep < SLEEP_RECALCULATION_PERIOD_SECONDS || predictedSleep <= 0) {
      Serial.printf("[SLEEP] manual reset within %d seconds of wakeup window. Reset to full window next\n", SLEEP_RECALCULATION_PERIOD_SECONDS);
      previousWakeup = now;
      return defaultTimeout;
   }

   return predictedSleep;
}

wakeup_reason_t getWakeupReason() {
   int resetReason0 = rtc_get_reset_reason(0);
   int resetReason1 = rtc_get_reset_reason(1);
   int wakeupReason = esp_sleep_get_wakeup_cause();
#if DEBUG
   Serial.printf("[WAKE] Reset Reason 0: %d\n", resetReason0);
   Serial.printf("[WAKE] Reset Reason 1: %d\n", resetReason1);
   Serial.printf("[WAKE] Wake Reason: %d\n", wakeupReason);
#endif
   if (wakeupReason == esp_sleep_wakeup_cause_t::ESP_SLEEP_WAKEUP_UNDEFINED && resetReason0 == 1 && resetReason1 == 1) {
      Serial.printf("[WAKE] Got Button Wakeup or Power Loss Wakeup\n");
      return wakeup_reason_t::BUTTON;
   }
   if (wakeupReason == esp_sleep_wakeup_cause_t::ESP_SLEEP_WAKEUP_EXT1) {
      Serial.printf("[WAKE] Got Motion Wakeup\n");
      return wakeup_reason_t::MOTION;
   }
   if (wakeupReason == esp_sleep_wakeup_cause_t::ESP_SLEEP_WAKEUP_TIMER) {
      Serial.printf("[WAKE] Got Timer Wakeup \n");
      return wakeup_reason_t::TIMER;
   }
   Serial.printf("[WAKE] Got Button System Reset or Others\n");
   return wakeup_reason_t::SYSTEM_RESET;
}

void startupCounter(int reset) {
   preferences.begin("my-app", false);
   unsigned int counter = preferences.getUInt("counter", 0);
   counter++;
   if (reset || counter > 16) {
      tickerStatupCounter.detach();
      StartCounter = 0;
      counter = 0;
      Serial.println("[MAIN] Startup Counter RESET");
   }
   preferences.putUInt("counter", counter);
   preferences.end();
   StartCounter = counter;
   return;
}

bool resetAll(bool resetActivation, bool resetWifi) {
   checkOrientationInBackground(0, false);
   Serial.printf("[MAIN] Reset - ACT %d | WIFI %d \n", resetActivation, resetWifi);

   writeIntToFlash(0, 220);               // restore screen orient to default
   writeIntToFlash(0, 150);               // Reset picture version after ota to init update
   writeIntToFlash(0, 140);               // reset reconnect count
   setActivatedToMem(false);              // reset activation writeIntToFlash(0, 160);
   wifiConnectionLostStore(false, true);  // reset wifi lost flag
   storeSleepTimeMem(DEFAULT_SLEEP);      // restore sleep mem store to default
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
      powerSupplyDisplay(true);
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
      settings.clearscreen = true;
      settings.showBatteryWarning = true;
      settings.showWifiWarning = true;
      saveSettingsToFlash(EEPROM_SETTINGS_ADR);
      gotToDeepSleep(0, true, false);
   }
   return true;
}

void debugCheck() {
   Serial.println("[DEBUG] Deploy State");
   resetAll(false, true);
   setDisplayData(CLIENT_ID, systemData.vddValue);
   printDebugInfo();
   powerSupplyDisplay(true);
   delay(150);
   waitDisplayComplete(false);
   displaySetQuickRefresh(false);

   bool success = downloadBMPToFlash(ENV_SLEEP_SCREEN_URL_13, "cover.bmp", true);
   if (success) {
#ifdef EPD_TYPE_13INCH
      displaySetDownloadSleep_13();
#else
      displayDebugInfo();  // TODO: Implement sleep download screen for paper 7
#endif
   } else {
      displayDebugInfo();
   }

   delay(2000);
   gotToDeepSleep(DEFAULT_SLEEP, false, false);  // dont show the deep sleep screen stay with debug info
   return;
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
}
void sdTest(bool doLog) {
   // Test read/write
   FsFile testFile;
   if (testFile.open("test.txt", O_WRONLY | O_CREAT | O_TRUNC)) {
      if (doLog) Serial.println("Writing to test.txt...");
      testFile.println("SD Data OK");
      testFile.close();
      if (doLog) Serial.println("Write complete.");

      if (testFile.open("test.txt", O_RDONLY)) {
         if (doLog) Serial.println("Reading from test.txt:");
         while (testFile.available()) {
            Serial.write(testFile.read());
         }
         testFile.close();
         Serial.println("SD Test Done");
      } else {
         Serial.println("Failed to open test.txt for reading.");
      }
   } else {
      Serial.println("Failed to create test.txt.");
   }
}

bool sdInit(bool forceFormat) {
   if (systemData.sdIsInit && !forceFormat) return true;
   if (forceFormat || !sd.begin(SdSpiConfig(CS_SD_PIN, SHARED_SPI, DISPLAY_SPI_SPEED))) {
      Serial.println("[SD] mount failed, attempting to format the card...");
      FatFormatter fatFormatter;
      uint8_t buffer[512];
      SdCardFactory cardFactory;
      SdCard* m_card = cardFactory.newCard(SdSpiConfig(CS_SD_PIN, SHARED_SPI, DISPLAY_SPI_SPEED));

      if (!m_card || m_card->errorCode()) {
         Serial.println("[SD] Hardware error: could not detect SD card!");
         return false;
      } else {
         bool formatSuccess = fatFormatter.format(m_card, buffer, &Serial);
         if (formatSuccess) {
            Serial.println("Format complete! Retrying mount...");
            if (sd.begin(SdSpiConfig(CS_SD_PIN, SHARED_SPI, DISPLAY_SPI_SPEED))) {
               Serial.println("[SD] mounted successfully after format.");
               systemData.sdIsInit = true;
               return true;
            } else {
               Serial.println("[SD] mount failed even after format.");
               return false;
            }
         } else {
            Serial.println("[SD] Format failed!");
            return false;
         }
      }
   } else {
      Serial.println("[SD] initialization done.");
      systemData.sdIsInit = true;
      return true;
   }
   return false;
}

bool copyFileFromFlashToSD(String fileName, String fileNameSD) {
   if (!SerialFlash.exists(fileName.c_str())) {
      Serial.printf("[COPY] Fehler: Datei %s existiert nicht im Flash.\n", fileName.c_str());
      return false;
   }

   // Datei aus dem SerialFlash öffnen
   SerialFlashFile flashFile = SerialFlash.open(fileName.c_str());
   if (!flashFile) {
      Serial.println("[COPY] Fehler: Konnte Datei im Flash nicht öffnen.");
      return false;
   }

   FsFile sdFile;
   if (!sdFile.open(fileNameSD.c_str(), O_WRONLY | O_CREAT | O_TRUNC)) {
      Serial.println("[COPY] Fehler: Konnte/Wollte Zieldatei auf der SD-Karte nicht erstellen.");
      flashFile.close();
      return false;
   }

   const size_t buff_size = 4096;
   unsigned char* buff = (unsigned char*)malloc(buff_size);
   if (buff == NULL) {
      Serial.println("[COPY] Fehler: Nicht genug RAM für den Kopierpuffer!");
      sdFile.close();
      flashFile.close();
      return false;
   }

   size_t totalSize = flashFile.size();
   size_t copiedBytes = 0;

   Serial.printf("[COPY] Kopiere %s (%d Bytes) auf SD-Karte (File:%s)...\n", fileName.c_str(), totalSize, fileNameSD.c_str());

   while (copiedBytes < totalSize) {
      size_t bytesToRead = totalSize - copiedBytes;
      if (bytesToRead > buff_size) {
         bytesToRead = buff_size;
      }

      size_t bytesRead = flashFile.read(buff, bytesToRead);
      if (bytesRead == 0) {
         break;  // Sollte nicht passieren, wenn die Größe stimmt
      }

      size_t bytesWritten = sdFile.write(buff, bytesRead);
      if (bytesWritten != bytesRead) {
         Serial.println("[COPY] Fehler beim Schreiben auf die SD-Karte. Möglicherweise Speicher voll oder Karte entfernt.");
         free(buff);
         sdFile.close();
         flashFile.close();
         return false;
      }

      copiedBytes += bytesWritten;
   }

   free(buff);
   sdFile.close();
   flashFile.close();

   Serial.println("[COPY] Kopieren erfolgreich abgeschlossen!");
   return true;
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

int accInit(bool skipInit) {
   int orientation = 0;
   if (!skipInit) {
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
      } else {
         Serial.println("[ACC] Communication error, stopping.");
         return -1;
         // while (true);  // stop running sketch if failed
      }
#endif
   }
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

   if (DEBUG_FLAG && !skipInit) Serial.printf("[ACC] Values: X:%d Y:%d Z:%d Orient: %d \n", acc_x, acc_y, acc_z, orientation);

   // Put IMU back into standby
   myIMU.resetInterrupt();
   myIMU.standby(true);
   return orientation;
}

bool accIntSet(int sensity) {
   pinMode(INT_PIN, INPUT);
   if (sensity == 0) {
      myIMU.resetInterrupt();
      myIMU.standby(true);
      return true;
   }
   myIMU.resetInterrupt();
   delay(1);
   myIMU.intConf(sensity, 3, 10, HIGH, -1, true, false, true, false, true);
   delay(1);
   return true;
}

void accUpdateOrient() {
   systemData.deviceOrientation = accInit();
   if (systemData.deviceOrientation == 2 || systemData.deviceOrientation == 3) {
      displaySetRotation(1);
   } else {
      displaySetRotation(0);
   }
   return;
}

void recheckAccOrient(int setOrientValue) {
   if (stopAccRecheck) return;
   int accCheck = accInit(true);
   if (accCheck != readIntFromFlash(220)) {
      isOrientUpdate = true;
      systemData.deviceOrientation = accCheck;
      Serial.printf("[ACC] Update Orient to Mem: %d \n", systemData.deviceOrientation);
      writeIntToFlash(systemData.deviceOrientation, 220);
      // writeIntToFlash(0, 150);  // Reset picture version after ota to init update
      if (systemData.deviceOrientation == 2 || systemData.deviceOrientation == 3) {
         displaySetRotation(1);
      } else {
         displaySetRotation(0);
      }
      if (isEpaperActive()) {
         deinitDisplay();
         // ESP.restart();
      }
   } else {
      // if (DEBUG_FLAG) Serial.printf("[ACC] No Acc Update after recheck\n");
   }
}

void checkOrientationInBackground(int setOrientValue, bool isRunning) {
   if (isRunning) {
      stopAccRecheck = false;
      Serial.printf("[ACC] Background update start\n");

      periodicAccCheck.attach_ms(1000, recheckAccOrient, setOrientValue);
   } else {
      Serial.printf("[ACC] Background update stop\n");
      stopAccRecheck = true;
      periodicAccCheck.detach();
      return;
   }

   return;
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
      } else {
         Serial.println("[CHARGE] on - Charge Done");
         digitalWrite(CHG_EN_PIN, LOW);
      }
   } else {
      pinMode(CHG_EN_PIN, OUTPUT);
      digitalWrite(CHG_EN_PIN, HIGH);
      delay(2);
      chargeState = digitalRead(CHG_STAT_PIN);
      if (chargeState == LOW) {
         Serial.println("[CHARGE] off - Charging");
         isCharging = true;
      } else {
         Serial.println("[CHARGE] off - Charge Done");
         digitalWrite(CHG_EN_PIN, LOW);
      }
   }
   return isCharging;
}

// Power supply display
#ifdef EPD_TYPE_13INCH
bool powerSupplyDisplay(bool enable) {
   bool tempState = systemData.displayPowerOn;
   if (enable) {
      pinMode(DISP_POWER, OUTPUT);
      digitalWrite(DISP_POWER, HIGH);
      systemData.displayPowerOn = true;
   } else {
      pinMode(DISP_POWER, INPUT);
      digitalWrite(DISP_POWER, LOW);
      systemData.displayPowerOn = false;
   }
   if (tempState != systemData.displayPowerOn) {
      if (enable) {
         Serial.printf("[POWER] Supply ON\n");
         delay(100);
         sdInit();
      }
      return true;
   } else {
      return false;
   }
}
#else
bool powerSupplyDisplay(bool enable) {
   return false;
}
#endif

void checkDeviceBatch(int set = 0, int useMem = false) {
   int revNumber = 0;
   if (set > 0) {
      Serial.printf("[SETUP] write device revision to: %d\n", set);
      if (useMem) {
         writeIntToFlash(set, 195);
      } else {
         revNumber = set;
      }
   }
   if (useMem) {
      int revNumber = readIntFromFlash(195);
      if (revNumber > 10) {
         Serial.printf("[SETUP] Reset revision number from: %d to default\n", revNumber);
         if (useMem)
            writeIntToFlash(0, 195);
         return;
      }
   }

   switch (revNumber) {
      case 1:
         Serial.printf("[SETUP] set device to: %d (paper 13)\n", revNumber);
         break;
      default:
         Serial.printf("[SETUP] set device revision to default (paper 7)\n");
   }
}

void test() {
   // checkDeviceBatch(0);
   //  adcAttachPin(14);  // Any pin that is ADC capable
   ledBlink(0, false);
   char charBuffer[128];
   Serial.println("[DEBUG] Test Function");
   // gotToDeepSleep(86000, false, false);
   analogWrite(LED_PIN, 100);
   if (powerSupplyDisplay(true)) delay(100);

   // displaySetQrPartial();
   // displayPartialTest(false);
   bool quickref = true;
   int zufallszahl = random(2, 16);
   /*
      while (true) {
         accUpdateOrient();
         checkOrientationInBackground(systemData.deviceOrientation, true);
         int setSuccess = setImageFromFS("tmp.gz");
         if (isOrientUpdate) {
            checkOrientationInBackground(systemData.deviceOrientation, false);
            initEpaperDisplay(SPI);
            isOrientUpdate = false;
         }
      }


      while (true) {
         delay(5000);
      }*/

   wifiSmart();
   displaySetQuickRefresh(false);

   // downloadBMPToFlash("https://smarthome-agentur.de/wp-content/download/cover.bmp", "cover.bmp", true);
   displaySetDownloadSleep_13();
   while (true) {
      delay(5000);
   }
   getImageUrl(false);
   String fileName = "tmp.gz";
   int dlSuccess = loadImageFromWeb(DL_URL, fileName);
   setImageFromFS(fileName);

   while (true) {
      delay(5000);
   }
   /*
      displayOtaScreen();
      delay(2000);
   displaySetText("test", false, quickref);
   delay(2000);
   displayNoPicture();
   delay(2000);
   displayWifiActivate(false);
   delay(2000);
   displayTurnOn();
   delay(2000);
   displayWifiActivate(true);

   delay(2000);
   displaySetText("test black", true, quickref);
   delay(2000);
   displaySetBlankTest(zufallszahl, true);
   setImageFromFS("cover.bmp");

   while (true) {
      float temperature = temperatureRead();
      Serial.printf("Temp onBoard = %.2f °C\n", temperature);
      // bool testCharge = chargeMode(false);
      systemData.vddValue = readVDD(false);
      Serial.printf("VDD: %d mV\n", systemData.vddValue);
      delay(5000);
   };*/

   while (true) {
      float temperature = temperatureRead();
      Serial.printf("Temp onBoard = %.2f °C\n", temperature);
      // bool testCharge = chargeMode(false);
      systemData.vddValue = readVDD(false);
      Serial.printf("VDD: %d mV\n", systemData.vddValue);
      delay(5000);
   };

   displaySetQuickRefresh(true);
   displaySetRotation(0);

   setImageFromFS("tmp.gz");

   while (true) {
      delay(5000);
   }

   displayWifiActivate(false);
   delay(10000);
   displayOtaScreen();
   delay(10000);
   // WiFi.setSleep(true);
   // WiFi.disconnect(true);
   // setCpuFrequencyMhz(40);
   delay(5);
   // displaySetBlankTest(0, true);

   displaySetQuickRefresh(false);
   displayWifiActivate(false);
   ledBlink(500, true, LED_DIM_VALUE);
   // gotToDeepSleep(600, true, false);
   //  displayOtaScreen();

   gotToDeepSleep(60, false, true);

   // display.powerOff();
   //  BleInit(CLIENT_ID, true);
}

void setup() {
   chargeMode(false);  // enable charge mode
   powerSupplyDisplay(false);
   pinMode(BAT_VOLT_EN_PIN, OUTPUT);
#ifdef EPD_TYPE_13INCH
   digitalWrite(BAT_VOLT_EN_PIN, HIGH);
#else
   digitalWrite(BAT_VOLT_EN_PIN, LOW);
#endif
   pinMode(BAT_VOLT_SENSE_PIN, INPUT);
   Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
   pinMode(LED_PIN, OUTPUT);
   digitalWrite(LED_PIN, LOW);
   Serial.begin(115200);
   analogReadResolution(12);
   systemData.usbConnected = usbCheckConnect();
   delay(100);

   systemData.vddValue = readVDD(false);
   if (!systemData.usbConnected && systemData.vddValue < BAT_OFF_VALUE) {
      if (DEBUG_FLAG) Serial.printf("[MAIN] Bat low protection: %dmV\n", systemData.vddValue);
      analogWrite(LED_PIN, LED_DIM_VALUE);
      startupCounter(true);
      systemData.vddValue = 0;
      gotToDeepSleep(86000, false, false);
   }
   systemData.wakeupCause = getWakeupReason();
   if (systemData.wakeupCause == wakeup_reason_t::BUTTON) {
      buttonWake = true;
      startupCounter(false);
      if (StartCounter >= 5) {
      } else {
         int startupCounterDelay = 3000;
         if (DEBUG_FLAG) startupCounterDelay = 10000;
         tickerStatupCounter.once_ms(startupCounterDelay, startupCounter, 1);
      }
   } else {
      startupCounter(true);
   }

   EepromInit(EEPROM_SIZE);
   pinMode(INT_PIN, INPUT);
   pinMode(CS_SD_PIN, OUTPUT);
   pinMode(SCK_PIN, OUTPUT);
   pinMode(MOSI_PIN, OUTPUT);
   pinMode(MISO_PIN, INPUT_PULLUP);
   digitalWrite(CS_SD_PIN, HIGH);

   gpio_set_drive_capability((gpio_num_t)SCK_PIN, GPIO_DRIVE_CAP_1);
   gpio_set_drive_capability((gpio_num_t)MOSI_PIN, GPIO_DRIVE_CAP_1);

#if DEBUG
   displaySetOverlayOption(VERSION, true);
   displaySetOverlayOption(BATTERY_INFO, true);
   displaySetOverlayOption(WIFI_SIGNAL, true);
   myEsp32FOTA.setManifestURL(OTA_URL_DEV);
#else
   myEsp32FOTA.setManifestURL(OTA_URL);
#endif
   displaySetOverlayOption(DEVICE_INFO_STRING, true);
   char firmwareVersion[] = SOFTWARE_VERSION;

   ledBlink(500, true, LED_DIM_VALUE);
   Serial.begin(115200);
   sleep(1);

   Serial.printf("[MAIN] INIT Device V: %s\n", firmwareVersion);
   if (DEBUG_FLAG) Serial.printf("[MAIN] Current counter value: %u VDD: %d\n", StartCounter, systemData.vddValue);
   if (!SPIFFS.begin()) {
      Serial.println("[MEM] SPIFFS initialisation failed!");
   }
   debugFS();
   if (!systemData.usbConnected && systemData.vddValue < BAT_LOW_VALUE) displaySetOverlayOption(BATTERY_LOW_BIG, true);  // enable bat low display if needed
   WiFi.onEvent(WiFiEvent);
   SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);              // SCK(), MISO(),MOSI(), SS()
   SerialFlash.begin(CS_FLASH_PIN, DISPLAY_SPI_SPEED);  // proceed even if begin() fails
   powerSupplyDisplay(true);
   initEpaperDisplay(SPI);
   powerSupplyDisplay(false);
   chargeMode(false);

   setDeviceUid();
   setDisplayData(CLIENT_ID, systemData.vddValue);
   accUpdateOrient();

   float temperature = temperatureRead();
   if (DEBUG_FLAG) Serial.printf("[MAIN] Temp Main: %.2f °C\n", temperature);
   if (temperature < 21.0) {
      Serial.printf("[MAIN] Low Temp detected: %.2f °C - disable quick refresh\n", temperature);
      displaySetQuickRefresh(false);
   }

#if DEBUG
   // test();  //-----------------test---------please remove
#endif
   tickerFailsave.once_ms((FAILSAVE_TIMER * 1000) + (WIFI_INIT_TIME * 1000), timeoutFailsave, 0);
   testModeCheck();                   // check if needs to enter deploy state
   bool wifiConnected = wifiSmart();  // Replaces wifi begin

   ledBlink(500, true, LED_DIM_VALUE);
   tickerFailsave.detach();
   tickerFailsave.once_ms(FAILSAVE_TIMER * 1000, timeoutFailsave, 0);
   bool updateNeeded = false;
#if DEBUG
   updateNeeded = myEsp32FOTA.execHTTPcheck();
   Serial.printf("[OTA] V: %s OTA Needed: %d Set URL: %s \n", SOFTWARE_VERSION, updateNeeded, OTA_URL_DEV);
   if (StartCounter > 2) {
      startupCounter(true);
      powerSupplyDisplay(true);
      displaySetQuickRefresh(true);
      waitDisplayComplete(false);
      delay(200);
      displaySetText("DEV OTA Update...", true, true);
      ledBlink(2000, true, LED_DIM_VALUE);
      Serial.println("[OTA] Dev OTA Started.....");
      writeIntToFlash(0, 170);
      resetAll(false, false);
      myEsp32FOTA.execOTA();
      delay(2000);
   }
#else
   updateNeeded = myEsp32FOTA.execHTTPcheck();
   if (updateNeeded) {
      startupCounter(true);
      powerSupplyDisplay(true);
      displaySetQuickRefresh(true);
      waitDisplayComplete(false);
      delay(200);
      displayOtaScreen();
      ledBlink(2000, true, LED_DIM_VALUE);
      Serial.println("[OTA] OTA Started.....");
      writeIntToFlash(0, 170);  // Reset activation counter in case activation is in ota proccess
      resetAll(false, false);
      myEsp32FOTA.execOTA();
      delay(2000);
   } else {
      Serial.println("[OTA] no OTA needed");
   }
#endif

   bool activationDone = deployDevice();
   saveSettingsToFlash(EEPROM_SETTINGS_ADR);
   if (!getActivatedFromMem() && isTestMode) {
      if (systemData.wakeupCause == wakeup_reason_t::BUTTON) {
         debugCheck();
      } else {
         gotToDeepSleep(0, false, false);
      }
   }
   //  TODO reset if device is still found activated on first wifi connect and not pending

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
            if (powerSupplyDisplay(true)) delay(100);
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
   systemData.sleepPrediction = calculateSleepDuration(settings.timeout, systemData.newSleepTimeSet, false);

   // TODO: Force Update after wifi screen if just one connection loss and then back
   // Set EEPROM on wifi screen and read here. If true and picture version is not 0 then update
   bool isUpdate = getVersionUpdate();
   // if (DEBUG_FLAG) isUpdate = true;  // TODO: remove in production, only for testing
   if (!isUpdate) {
      if (newVersionSave <= 0) {
         if (powerSupplyDisplay(true)) delay(100);
         if (DEBUG_FLAG) setUpdateState("update_checked_nopicture");
         displayNoPicture();
      } else {
         if (DEBUG_FLAG) setUpdateState("update_checked_noupdate");
      }
      delay(10);
      // WiFi.disconnect(true);
      //  WiFi.setSleep(true);
      if (settings.timeout > 0) {
         gotToDeepSleep(systemData.sleepPrediction);  // Disable version Check here

      } else {
         gotToDeepSleep(DEFAULT_SLEEP);
      }
   }
   powerSupplyDisplay(true);

   if (SPIFFS.usedBytes() > 10000) {
      Serial.println("[MEM] SPIFFS seems to full ...");
   }

   bool success = getImageUrl(false);
   if (!success) {
      setUpdateState("update_failed");
      delay(500);
      gotToDeepSleep(systemData.sleepPrediction);
   }

   esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
   delay(10);
}

void loop() {
   if (downloadStart) {
      if (WiFi.status() == WL_CONNECTED) {
         int setSuccess = -1;
         downloadStart = false;
         delay(200);
         awsConnect(false);  // Disconnect AWS to free RAM for HTTPS download!
         String fileName = "tmp.gz";
         int dlSuccess = loadImageFromWeb(DL_URL, fileName);
         if (DEBUG_FLAG) setUpdateState("download_ok");  // also connects aws
         Serial.println("[DL] Done");
         WiFi.setSleep(true);
         esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
         waitDisplayComplete(false);
         if (dlSuccess == 0) {
            for (int i = 0; i < 5; i++) {
               accUpdateOrient();
               checkOrientationInBackground(systemData.deviceOrientation, true);
               setSuccess = setImageFromFS(fileName);
               checkOrientationInBackground(systemData.deviceOrientation, false);
               if (isOrientUpdate) {
                  setSuccess = -1;
                  initEpaperDisplay(SPI);
                  isOrientUpdate = false;
               } else {
                  break;
               }
            }
         }
         Serial.println("[EPD] Set Image Done");
         delay(500);
         debugFS();
         if (DEBUG_FLAG) Serial.printf("[MAIN] all picture updates done: %d %d\n", dlSuccess, setSuccess);

         if (isOrientUpdate) return;  // if orientation change during update stop process to avoid wrong update state
         if (dlSuccess == 0 && setSuccess == 0) {
            writeIntToFlash(newVersionSave, 150);
            setUpdateState("update_ok");
         } else {
            setUpdateState("update_failed");
            displaySetText("Error: Picture download failed, please try again", false);
         }
      }
      delay(500);
   }
   if (WiFi.status() == WL_CONNECTED) {
      ledBlink(500, true, LED_DIM_VALUE);

   } else {
      ledBlink(200, true, LED_DIM_VALUE);
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
         gotToDeepSleep(systemData.sleepPrediction);
      } else {
         gotToDeepSleep(DEFAULT_SLEEP);
      }
   }
}
#pragma once
#include <Arduino.h>

// ==========================================
// DISPLAY SELECTION
// ==========================================
#define EPD_TYPE_13INCH
// #define EPD_TYPE_7INCH
//    ==========================================

#define SOFTWARE_VERSION "0.0.0"
#define DEBUG 1

#define DISPLAY_SPI_SPEED 10000000
#define QR_VERSION 3
#define QR_QUIET_ZONE 4
// #define USE_QUICK_REFRESH

#ifdef EPD_TYPE_13INCH
#define SLEEP_SCREEN_URL ENV_SLEEP_SCREEN_URL_13
#else
#define SLEEP_SCREEN_URL ENV_SLEEP_SCREEN_URL_7
#endif

typedef enum {
   SYSTEM_RESET = 0,
   BUTTON = 1,
   MOTION = 2,
   TIMER = 3,
} wakeup_reason_t;

enum DisplayInfoKey {
   VERSION,
   BATTERY_INFO,
   BATTERY_LOW_BIG,
   WIFI_SIGNAL,
   DEVICE_INFO_STRING,
   WIFI_OFFLINE_BIG
};

struct WifiSettings {
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
};

struct DisplayInfo {
   bool version;
   bool batteryInfo;
   bool batteryLowBig;
   bool wifiSignal;
   bool deviceInfoString;
   bool wifiOfflineBig;
};

struct DisplaySettings {
   uint8_t rotationText;
   uint8_t rotationPicture;
   bool quickRefresh;
   bool globalQuickRefreshDisable;
   int displayQuickRefreshTime;
   int displayQuickRefreshWipeTime;
   uint16_t colorWhiteFast;
   uint16_t colorBlackFast;
   uint8_t colorWipeFast;
};

struct Settings {
   int timeout;       // Seconds to Sleep after update done
   String lut;        // Color Settings for EPD
   bool clearscreen;  // if clear screen before update
   bool showBatteryWarning;
   bool showWifiWarning;
   bool sleepDisabled;
};

struct SystemData {
   wakeup_reason_t wakeupCause;
   int vddValue;
   u_int8_t ledDimValue;
   int sleepPrediction;
   bool newSleepTimeSet;
   bool usbConnected;
   int deviceOrientation;
   bool displayPowerOn;
   bool sdIsInit;
};

struct DataLayout {
   int integer;
   char byte[4];
};
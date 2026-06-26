#pragma once
#include "types.h"
#include <Arduino.h>

#define COLOR_WHITE GxEPD_WHITE
#define COLOR_BLACK GxEPD_BLACK

#define CS_EPD_PIN 20
#define EPD_CS_S 19
#define BUSY_PIN 18
#define DC_PIN 19
#define RST_PIN 1

#define DISPLAY_SPI_SPEED 20000000
#ifdef EPD_TYPE_13INCH
#define FONT_MAIN u8g2_font_helvB24_tf  // Font for main text
#define FONT_BIG u8g2_font_helvB18_tf   // Font for big text
#define FONT_NORMAL u8g2_font_helvB14_tf
#define FONT_SMALL u8g2_font_helvR12_tf
#define FONT_INFO u8g2_font_7x14_tf
#define FONT_VERSION u8g2_font_tom_thumb_4x6_tf
#else
#define FONT_MAIN u8g2_font_helvB24_tf  // Font for main text
#define FONT_BIG u8g2_font_helvB14_tf   // Font for big text
#define FONT_NORMAL u8g2_font_helvB12_tf
#define FONT_SMALL u8g2_font_helvR08_tf
#define FONT_INFO u8g2_font_7x14_tf
#define FONT_VERSION u8g2_font_tom_thumb_4x6_tf
#endif

#include "Adafruit_GFX.h"
#include <GxEPD2_7C.h>
#include <SPI.h>  // Wichtig, falls nicht vorhanden
#include <U8g2_for_Adafruit_GFX.h>

#ifdef EPD_TYPE_13INCH
#define EPD_WIDTH 1600
#define EPD_HEIGHT 1200
#define EPD_QR_SIZE_SMALL 3
#define EPD_QR_SIZE_BIG 12
using DisplayType = GxEPD2_7C<GxEPD2_1330c_EL133UF3, GxEPD2_1330c_EL133UF3::HEIGHT / 8>;
#else
#define EPD_WIDTH 800
#define EPD_HEIGHT 480
#define EPD_QR_SIZE_SMALL 2
#define EPD_QR_SIZE_BIG 8
using DisplayType = GxEPD2_7C<GxEPD2_730c_GDEP073E01, GxEPD2_730c_GDEP073E01::HEIGHT / 4>;
#endif

void printQRBlock(uint16_t x, uint16_t y, uint8_t size, uint16_t col);

void displayTest();
void displayTypeDetect();
int accInit(bool skipInit = false);
void setDisplayData(const char* clientId, int vddValue);
void initEpaperDisplay(SPIClass& spiBus);
void deinitDisplay();
void displaySetOverlayOption(DisplayInfoKey key, bool value);

// Display Functions
void updateDisplayAsyncFunction(int functionNumber);
bool updateDisplayAsync(String functionName);
bool waitDisplayComplete(bool quick);
void displayOverlays(DisplayType& dispObj, DisplayInfo displayData, bool invertColors, bool fullcolor = false);
int setImageFromFS_7inch(String fileName);
int setImageFromFS_13inch(String fileName);
int setImageFromFS(String fileName);
void displayDebugInfo();
void displaySetText(String info, bool isBlackboard, bool quickRefresh = true, int position = 0);
void displayOtaScreen();
void displayNoPicture();
void displayTurnOn();
void displayWifiActivate(bool wifiProvisioningDone);
void displaySetDownloadSleep();
void displayWipe(bool quick, bool useAltInit = false, int color = 6);
void displaySetBlankTest(int offsetVar, bool doQuickRefresh, bool useAltInit = false);
void displayPartialTest(bool doQuickRefresh = false);
bool isEpaperActive();
void displaySetRotation(int orientation);
void displaySetQuickRefresh(bool enable, int refreshTime = 0, int wipeTime = 0);
uint16_t getColor(uint8_t color);

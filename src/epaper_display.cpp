#include "epaper_display.h"
#include "GxEPD2.h"
#include "SdFat.h"
#include "types.h"
#include <Arduino.h>
#include <SerialFlash.h>
#include <Ticker.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <WiFi.h>
#include <qrcode.h>

// Externs from main.cpp
Ticker onceDisplay;
extern SerialFlashFile saveFile;
QRCode QR;

// display extra infos on top of all screens
DisplayInfo displayInfos = {
    .version = false,
    .batteryInfo = false,
    .batteryLowBig = false,
    .wifiSignal = false,
    .deviceInfoString = false,
    .wifiOfflineBig = false};

DisplaySettings displaySettings = {
    .rotationText = 3,
    .rotationPicture = 2,
    .quickRefresh = true,
    .globalQuickRefreshDisable = false,
#ifdef EPD_TYPE_13INCH
    .displayQuickRefreshTime = 2500,
    .displayQuickRefreshWipeTime = 2000,
    .colorWhiteFast = GxEPD_BLACK,
    .colorBlackFast = GxEPD_BLUE
#else
    .displayQuickRefreshTime = 960,
    .displayQuickRefreshWipeTime = 960,
    .colorWhiteFast = GxEPD_WHITE_I,
    .colorBlackFast = GxEPD_BLACK_I
#endif
};

// Hardware externs
static char epd_client_id[20] = {0};
static int epd_vdd_value = 0;
static bool epaperIsUpdating = false;
static bool displayIsInit = false;
static SPIClass* epd_spi_bus = nullptr;

// Macros to substitute the original object calls

#ifdef EPD_TYPE_13INCH
DisplayType display(GxEPD2_1330c_EL133UF3(/*CS=*/CS_EPD_PIN, /*CS-S=*/EPD_CS_S, /*DC=*/-1, /*RST=*/RST_PIN, BUSY_PIN));
#else
DisplayType display(GxEPD2_730c_GDEP073E01(/*CS=*/CS_EPD_PIN, /*DC=*/DC_PIN, /*RST=*/RST_PIN, /*BUSY=*/BUSY_PIN));
#endif
U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;

void displayTest() {

   uint8_t buffer[100];

   display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
   display.init(115200);

   epd_spi_bus->endTransaction();
   Serial.printf("[EPD] ----Register Read START----\n");
   for (uint16_t reg = 0x00; reg <= 0xF9; reg++) {
      epd_spi_bus->beginTransaction(SPISettings(DISPLAY_SPI_SPEED, MSBFIRST, SPI_MODE0));
      digitalWrite(DC_PIN, LOW);
      digitalWrite(CS_EPD_PIN, LOW);
      epd_spi_bus->transfer((uint8_t)reg);
      digitalWrite(DC_PIN, HIGH);

      for (int i = 0; i < 100; i++) {
         buffer[i] = epd_spi_bus->transfer(0x00);
      }

      digitalWrite(CS_EPD_PIN, HIGH);
      epd_spi_bus->endTransaction();

      int printLength = 100;
      int consecutiveZeros = 0;
      for (int i = 0; i < 100; i++) {
         if (buffer[i] == 0x00) {
            consecutiveZeros++;
            if (consecutiveZeros == 5) {
               printLength = i - 4;
               break;
            }
         } else {
            consecutiveZeros = 0;
         }
      }

      if (printLength > 0) {
         Serial.printf("[EPD] Register 0x%02X Read (%d bytes): ", reg, printLength);
         for (int i = 0; i < printLength; i++) {
            if (i % 16 == 0 && i != 0) Serial.print("\n  ");
            Serial.printf("0x%02X ", buffer[i]);
         }
         Serial.println();
      }

      delay(10);  // Small delay to avoid flooding too fast
   }
   Serial.printf("[EPD] ----Register Read END----\n");
}

void setDisplayData(const char* clientId, int vddValue) {
   if (clientId) {
      strncpy(epd_client_id, clientId, sizeof(epd_client_id) - 1);
      epd_client_id[sizeof(epd_client_id) - 1] = '\0';
   }
   epd_vdd_value = vddValue;
}

void initEpaperDisplay(SPIClass& spiBus) {
   epd_spi_bus = &spiBus;
   if (displayIsInit) return;
   pinMode(RST_PIN, OUTPUT);
   pinMode(CS_EPD_PIN, OUTPUT);
   pinMode(DC_PIN, OUTPUT);
   pinMode(EPD_CS_S, OUTPUT);
   pinMode(BUSY_PIN, INPUT);
   pinMode(DC_PIN, OUTPUT);
   digitalWrite(EPD_CS_S, HIGH);
   digitalWrite(CS_EPD_PIN, HIGH);
   display.epd2.selectSPI(spiBus, SPISettings(DISPLAY_SPI_SPEED, MSBFIRST, SPI_MODE0));
   u8g2_for_adafruit_gfx.begin(display);
   displayIsInit = true;
   displayTypeDetect();
}

void deinitDisplay() {
   pinMode(RST_PIN, OUTPUT);
   digitalWrite(RST_PIN, 1);
   delay(50);  // needs a little longer
   digitalWrite(RST_PIN, 0);
   delay(20);
   displayIsInit = false;
   display.hibernate();
   pinMode(RST_PIN, INPUT);
   pinMode(CS_EPD_PIN, INPUT);
   pinMode(EPD_CS_S, INPUT);
   pinMode(DC_PIN, INPUT);
}

void displayTypeDetect() {
   uint8_t reg9A[2] = {0};
   uint8_t patternDKE[2] = {0x36, 0x42};
   uint8_t patternOKRA1[2] = {0x31, 0xC2};
   uint8_t patternOKRA2[2] = {0x33, 0x00};  // Das zweite Byte wird bei OKRA 2 manchmal nicht gesendet, wir prüfen primär das erste

   display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
   display.init(115200);

   epd_spi_bus->endTransaction();

   // Register 0x9A auslesen
   epd_spi_bus->beginTransaction(SPISettings(DISPLAY_SPI_SPEED, MSBFIRST, SPI_MODE0));
   digitalWrite(DC_PIN, LOW);
   digitalWrite(CS_EPD_PIN, LOW);
   epd_spi_bus->transfer(0x9A);
   digitalWrite(DC_PIN, HIGH);

   for (int i = 0; i < 2; i++) {
      reg9A[i] = epd_spi_bus->transfer(0x00);
   }
   digitalWrite(CS_EPD_PIN, HIGH);
   epd_spi_bus->endTransaction();

   // Pattern Matching
   if (memcmp(reg9A, patternDKE, 2) == 0) {
      Serial.println("[EPD] Match Found: DKE Display");
      displaySettings.displayQuickRefreshTime = 1500;
      displaySettings.displayQuickRefreshWipeTime = 500;
      displaySettings.colorWhiteFast = GxEPD_RED;
      displaySettings.colorBlackFast = GxEPD_BLUE;
   } else if (memcmp(reg9A, patternOKRA1, 2) == 0 || memcmp(reg9A, patternOKRA2, 2) == 0) {
      Serial.println("[EPD] Match Found: OKRA Display");
      displaySettings.displayQuickRefreshTime = 1500;  // 1400-1700
      displaySettings.displayQuickRefreshWipeTime = 3500;
      displaySettings.colorWhiteFast = GxEPD_YELLOW;
      displaySettings.colorBlackFast = GxEPD_BLUE;
   } else {
      Serial.println("[EPD] No matching pattern found. Unknown Display Type.");
      Serial.printf("[EPD] Register 0x9A Read (2 bytes): 0x%02X 0x%02X\n", reg9A[0], reg9A[1]);
      displaySettings.globalQuickRefreshDisable = true;
   }
}

void displaySetOverlayOption(DisplayInfoKey key, bool value) {
   switch (key) {
      case DisplayInfoKey::VERSION:
         displayInfos.version = value;
         break;
      case DisplayInfoKey::BATTERY_INFO:
         displayInfos.batteryInfo = value;
         break;
      case DisplayInfoKey::BATTERY_LOW_BIG:
         displayInfos.batteryLowBig = value;
         break;
      case DisplayInfoKey::WIFI_SIGNAL:
         displayInfos.wifiSignal = value;
         break;
      case DisplayInfoKey::DEVICE_INFO_STRING:
         displayInfos.deviceInfoString = value;
         break;
      case DisplayInfoKey::WIFI_OFFLINE_BIG:
         displayInfos.wifiOfflineBig = value;
         break;
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
      functionNumber = 3;  // not used right now
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

int setImageFromFS_7inch(String fileName) {
   saveFile = SerialFlash.open(fileName.c_str());
   if (!saveFile) {
      Serial.println("[BMP] File missing");
      return -1;
   }

   uint16_t width = EPD_WIDTH;
   uint16_t height = EPD_HEIGHT;
   int offsetData = 0;

   // Check if the file is a standard BMP (starts with 'BM')
   uint8_t magic[2];
   if (saveFile.read(magic, 2) == 2) {
      if (magic[0] == 'B' && magic[1] == 'M') {
         // It's a standard BMP file. Read the pixel data offset at byte 0x0A
         saveFile.seek(0x0A);
         uint32_t bmpOffset = 0;
         saveFile.read((uint8_t*)&bmpOffset, 4);
         offsetData = bmpOffset;
         Serial.printf("[BMP] Detected Windows BMP. Pixel Data starts at offset: %d\n", offsetData);
      } else {
         Serial.println("[BMP] Detected RAW payload (no BM magic). Reading from byte 0.");
      }
   }

   Serial.printf("[BMP] Loading Image H: %d W: %d\n", height, width);

   if (width > EPD_WIDTH || height > EPD_HEIGHT) {
      Serial.printf("[BMP] Image too wide or tall!");
      return -1;
   }

   display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
   display.init(115200);
   display.setRotation(displaySettings.rotationPicture);
   display.setFullWindow();
   Serial.print("[EPD] Update Display... \n");

   int bufferSize = width / 2;
   uint8_t* lineBuffer = (uint8_t*)malloc(bufferSize);

   display.firstPage();
   do {
      display.fillScreen(GxEPD_WHITE);
      saveFile.seek(offsetData);

      for (int y = 0; y < height; y++) {
         saveFile.read(lineBuffer, bufferSize);
         for (int x = 0; x < width; x++) {
            uint8_t colorData = lineBuffer[x / 2];
            uint8_t colorNibble = (x % 2 == 0) ? (colorData >> 4) : (colorData & 0x0F);
            if (colorNibble != 6) {  // Optimization: display is already filled with WHITE (6)
               display.drawPixel(x, y, getColor(colorNibble));
            }
         }
      }
   } while (display.nextPage());

   free(lineBuffer);
   Serial.println("[EPD] End Draw...");
   return 0;
}

int setImageFromFS_13inch(String fileName) {
   saveFile = SerialFlash.open(fileName.c_str());
   if (!saveFile) {
      Serial.println("[BMP] File missing");
      return -1;
   }

   uint16_t width = EPD_WIDTH;
   uint16_t height = EPD_HEIGHT;
   int offsetData = 0;

   // Check if the file is a standard BMP (starts with 'BM')
   uint8_t magic[2];
   saveFile.seek(0);
   if (saveFile.read(magic, 2) == 2) {
      if (magic[0] == 'B' && magic[1] == 'M') {
         // It's a standard BMP file. Read the pixel data offset at byte 0x0A
         saveFile.seek(0x0A);
         uint32_t bmpOffset = 0;
         saveFile.read((uint8_t*)&bmpOffset, 4);
         offsetData = bmpOffset;
         Serial.printf("[BMP] Detected Windows BMP. Pixel Data starts at offset: %d\n", offsetData);
      } else {
         Serial.println("[BMP] Detected RAW payload (no BM magic). Reading from byte 0.");
         offsetData = 0;
      }
   }

   Serial.printf("[BMP] Loading Image H: %d W: %d\n", height, width);

   if (width > EPD_WIDTH || height > EPD_HEIGHT) {
      Serial.printf("[BMP] Image too wide or tall!");
      return -1;
   }

   display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
   display.init(115200);
   display.clearScreen(0x01);  // Clear screen memory

   // EPD physical resolution is 1200x1600 (2 controllers of 600x1600 each)
   const int physWidth = EPD_HEIGHT;
   const int physHeight = EPD_WIDTH;

   int numChunks = 16;
   int linesPerChunk = physHeight / numChunks;
   int bytesPerHalfLine = physWidth / 4;

   uint8_t* chunkBuffer = (uint8_t*)malloc(linesPerChunk * bytesPerHalfLine);
   if (!chunkBuffer) {
      Serial.println("[BMP] Malloc for 16 chunks failed! Trying 32 chunks...");
      numChunks = 32;
      linesPerChunk = physHeight / numChunks;
      chunkBuffer = (uint8_t*)malloc(linesPerChunk * bytesPerHalfLine);

      if (!chunkBuffer) {
         Serial.println("[BMP] Malloc for 32 chunks failed! Trying 64 chunks...");
         numChunks = 64;
         linesPerChunk = physHeight / numChunks;
         chunkBuffer = (uint8_t*)malloc(linesPerChunk * bytesPerHalfLine);

         if (!chunkBuffer) {
            Serial.println("[BMP] Malloc for 64 chunks failed! Aborting.");
            return -1;
         }
      }
   }

   Serial.println("[EPD] Streaming Partial Image to Display... ");

   for (int half = 0; half < 2; half++) {
      int csPin = (half == 0) ? CS_EPD_PIN : EPD_CS_S;

      for (int chunk = 0; chunk < numChunks; chunk++) {
         int yImageStart = chunk * linesPerChunk;

         uint16_t xStartCtrl = 0;
         uint16_t xPixel = physWidth / 2;  // 600 Pixel
         uint16_t HRST = xStartCtrl * 2;
         uint16_t HRED = (xStartCtrl + xPixel) * 2 - 1;  // 1199
         uint16_t VRST = yImageStart / 2;
         uint16_t VRED = (yImageStart + linesPerChunk) / 2 - 1;

         for (int i = 0; i < linesPerChunk; i++) {
            int lineInImage = yImageStart + i;
            int byteOffsetInImage = (lineInImage * 600) + (half * 300);
            saveFile.seek(offsetData + byteOffsetInImage);
            saveFile.read(chunkBuffer + i * bytesPerHalfLine, bytesPerHalfLine);
         }
         epd_spi_bus->endTransaction();
         epd_spi_bus->beginTransaction(SPISettings(DISPLAY_SPI_SPEED, MSBFIRST, SPI_MODE0));

         // PTLW (Partial Window) Command Setting 0x83
         digitalWrite(csPin, LOW);
         epd_spi_bus->transfer(0x83);
         epd_spi_bus->transfer(HRST >> 8);
         epd_spi_bus->transfer(HRST & 0xFF);
         epd_spi_bus->transfer(HRED >> 8);
         epd_spi_bus->transfer(HRED & 0xFF);
         epd_spi_bus->transfer(VRST >> 8);
         epd_spi_bus->transfer(VRST & 0xFF);
         epd_spi_bus->transfer(VRED >> 8);
         epd_spi_bus->transfer(VRED & 0xFF);
         epd_spi_bus->transfer(0x01);  // PTLW_ENABLE
         digitalWrite(csPin, HIGH);
         // PTIN (Partial In) command 0x91
         digitalWrite(csPin, LOW);
         epd_spi_bus->transfer(0x91);
         digitalWrite(csPin, HIGH);
         // DTM command 0x10
         digitalWrite(csPin, LOW);
         epd_spi_bus->transfer(0x10);

         // Hardware LUT conversion
         for (int i = 0; i < linesPerChunk * bytesPerHalfLine; i++) {
            uint8_t low = getColor(chunkBuffer[i] & 0x0F);
            uint8_t high = getColor(chunkBuffer[i] >> 4);
            // uint8_t raw = chunkBuffer[i];
            chunkBuffer[i] = (high << 4) | low;
         }

         epd_spi_bus->writeBytes(chunkBuffer, linesPerChunk * bytesPerHalfLine);
         digitalWrite(csPin, HIGH);

         epd_spi_bus->endTransaction();
         delay(1);
      }
   }

   free(chunkBuffer);

   Serial.println("[EPD] End Partial Streaming... Refreshing now.");

   // PTLW (Partial Window) für beide Controller wieder deaktivieren,
   // damit der nachfolgende Refresh (DRF) den gesamten Bildschirm erfasst!
   digitalWrite(CS_EPD_PIN, LOW);
   epd_spi_bus->transfer(0x83);
   for (int i = 0; i < 9; i++) epd_spi_bus->transfer(0x00);
   digitalWrite(CS_EPD_PIN, HIGH);

   digitalWrite(EPD_CS_S, LOW);
   epd_spi_bus->transfer(0x83);
   for (int i = 0; i < 9; i++) epd_spi_bus->transfer(0x00);
   digitalWrite(EPD_CS_S, HIGH);

   display.refresh();

   saveFile.close();

   return 0;
}

int setImageFromFS(String fileName) {
   epaperIsUpdating = true;

#ifdef EPD_TYPE_13INCH
   return setImageFromFS_13inch(fileName);
#else
   return setImageFromFS_7inch(fileName);
#endif
   epaperIsUpdating = false;
}

void displayOverlays(DisplayType& dispObj, DisplayInfo displayData, bool invertColors, bool fullcolor) {
   int16_t tw = 0;
   int foreGround = displaySettings.colorBlackFast;
   int backGround = displaySettings.colorWhiteFast;
   if (fullcolor) {
      foreGround = COLOR_BLACK;
      backGround = COLOR_WHITE;
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

      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color

      sprintf(charBuffer, "ID: %s", epd_client_id);
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

   // invert for info at bottom
   int tempStore = foreGround;
   foreGround = backGround;
   backGround = tempStore;

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

      sprintf(charBuffer, "Bat: %dV", epd_vdd_value);

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

void displayDebugInfo() {
   String info1 = "Ich schlafe ...";
   String info2 = "Drücke die Taste auf der Rückseite";
   String info3 = "um mich zu wecken.";
   int testOffset = 200;

   char msg[128];
   sprintf(msg, "%s%s%s", "https://paperlesspaper.de/b?d=", epd_client_id, "&w=99");

   uint8_t QRData[qrcode_getBufferSize(QR_VERSION)];
   uint8_t blockSize;
   uint8_t page = 0;
   qrcode_initText(&QR, QRData, QR_VERSION, ECC_LOW, msg);
   blockSize = EPD_QR_SIZE_SMALL;
   uint16_t x0 = (EPD_HEIGHT - 30 * blockSize) / 2;

   int foreGround = COLOR_WHITE;
   int backGround = COLOR_BLACK;

   if (displaySettings.quickRefresh) {
      displayWipe(true);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
      display.init(115200);
      foreGround = displaySettings.colorWhiteFast;
      backGround = displaySettings.colorBlackFast;
   } else {
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
      display.init(115200);
   }

   display.setRotation(displaySettings.rotationText);
   display.setFullWindow();

   display.firstPage();
   do {
      int16_t tw = 0;
      display.fillScreen(backGround);
      display.drawRect(2, 2, EPD_HEIGHT - 4, EPD_WIDTH - 4, foreGround);
      display.drawRect(3, 3, EPD_HEIGHT - 6, EPD_WIDTH - 6, foreGround);
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFontDirection(0);             // left to right (this is default)
      u8g2_for_adafruit_gfx.setFontMode(1);                  // use u8g2 transparent mode (this is default)

      u8g2_for_adafruit_gfx.setFont(FONT_BIG);                 // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info2.c_str());  // text box width
      int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();      // positive
      int16_t td = u8g2_for_adafruit_gfx.getFontDescent();     // negative; in mathematicians view
      int16_t th = ta - td;
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, (EPD_WIDTH - th) / 2);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info2);                                            // UTF-8 string: "<" 550 448 664 ">"

      tw = u8g2_for_adafruit_gfx.getUTF8Width(info3.c_str());                                   // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + th + 3);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info3);

      u8g2_for_adafruit_gfx.setFont(FONT_MAIN);     // extended font
      ta = u8g2_for_adafruit_gfx.getFontAscent();   // positive
      td = u8g2_for_adafruit_gfx.getFontDescent();  // negative; in mathematicians view
      th = ta - td;
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info1.c_str());
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) - 100 - testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info1);

      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);
      ta = u8g2_for_adafruit_gfx.getFontAscent();                                                         // positive
      td = u8g2_for_adafruit_gfx.getFontDescent();                                                        // negative; in mathematicians view
      th = ta - td;                                                                                       // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("I am sleeping...");                                        // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + 100 + testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print("I am sleeping...");

      u8g2_for_adafruit_gfx.setFont(FONT_SMALL);
      ta = u8g2_for_adafruit_gfx.getFontAscent();                                                                  // positive
      td = u8g2_for_adafruit_gfx.getFontDescent();                                                                 // negative; in mathematicians view
      th = ta - td;                                                                                                // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Press the button on the back to wake me up.");                      // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + 100 + th + 3 + testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Press the button on the back to wake me up.");

      uint16_t y0 = ((EPD_WIDTH / 2) + 150 + testOffset + testOffset);
      display.fillRect(x0 + 2, y0 + 2, QR.size * blockSize + QR_QUIET_ZONE + blockSize - 2, QR.size * blockSize + QR_QUIET_ZONE + blockSize - 2, foreGround);

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

   } while (display.nextPage());
}

void displayOtaScreen() {
   Serial.print(F("[EPD] OTA Screen loading - \n"));
   int testOffset = 200;

   int foreGround = COLOR_WHITE;
   int backGround = COLOR_BLACK;
   bool fullColor = true;

   if (displaySettings.quickRefresh) {
      displayWipe(true);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
      display.init(115200);
      foreGround = displaySettings.colorWhiteFast;
      backGround = displaySettings.colorBlackFast;
      fullColor = false;
   } else {
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
      display.init(115200);
   }

   display.setRotation(displaySettings.rotationText);
   display.setFullWindow();

   display.firstPage();
   do {
      u8g2_for_adafruit_gfx.setFontDirection(0);             // left to right (this is default)
      display.fillScreen(backGround);                        // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFontMode(1);                  // use u8g2 transparent mode (this is default)
      int16_t tw;

      u8g2_for_adafruit_gfx.setFont(FONT_BIG);                                         // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Bitte schalte den Rahmen nicht aus.");  // text box width
      int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();                              // positive
      int16_t td = u8g2_for_adafruit_gfx.getFontDescent();                             // negative; in mathematicians view
      int16_t th = ta - td;                                                            // text box height
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, (EPD_WIDTH - th) / 2);    // start writing at this position
      u8g2_for_adafruit_gfx.print("Bitte schalte den Rahmen nicht aus.");

      tw = u8g2_for_adafruit_gfx.getUTF8Width("Das Update dauert etwa 45 Sekunden.");           // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + th + 3);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Das Update dauert etwa 45 Sekunden.");

      u8g2_for_adafruit_gfx.setFont(FONT_MAIN);     // extended font
      ta = u8g2_for_adafruit_gfx.getFontAscent();   // positive
      td = u8g2_for_adafruit_gfx.getFontDescent();  // negative; in mathematicians view
      th = ta - td;
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Software Update...");
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) - 100 - testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Software Update...");

      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);
      ta = u8g2_for_adafruit_gfx.getFontAscent();                                                         // positive
      td = u8g2_for_adafruit_gfx.getFontDescent();                                                        // negative; in mathematicians view
      th = ta - td;                                                                                       // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Updating Software");                                       // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + 100 + testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Updating Software");

      u8g2_for_adafruit_gfx.setFont(FONT_SMALL);                                                                           // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Please do not turn off the device. The Update takes approx. 45 seconds.");  // text box width
      ta = u8g2_for_adafruit_gfx.getFontAscent();                                                                          // positive
      td = u8g2_for_adafruit_gfx.getFontDescent();                                                                         // negative; in mathematicians view
      th = ta - td;
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + 100 + th + 3 + testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Please do not turn off the device. The Update takes approx. 45 seconds.");      // UTF-8 string: "<" 550 448 664 ">"

      displayOverlays(display, displayInfos, true, fullColor);

   } while (display.nextPage());
}

void displayNoPicture() {
   String info = "Bitte aktualisiere ein Bild in der App.";
   String info2 = "Aktivierung";
   String info3 = "abgeschlossen";
   int16_t tw = 0;
   int testOffset = 200;

   Serial.print(F("[EPD] No Picture Screen Loading - \n"));

   int foreGround = COLOR_BLACK;
   int backGround = COLOR_WHITE;
   bool fullColor = false;

   if (displaySettings.quickRefresh) {
      displayWipe(true);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
      display.init(115200);
      foreGround = displaySettings.colorBlackFast;
      backGround = displaySettings.colorWhiteFast;
   } else {
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
      display.init(115200);
      fullColor = true;
   }

   display.setRotation(displaySettings.rotationText);
   display.setFullWindow();
   display.firstPage();
   do {
      u8g2_for_adafruit_gfx.setFontDirection(0);  // left to right (this is default)
      display.fillScreen(backGround);
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFontMode(1);                  // use u8g2 transparent mode (this is default)

      u8g2_for_adafruit_gfx.setFont(FONT_BIG);                                       // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info.c_str());                         // text box width
      int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();                            // positive
      int16_t td = u8g2_for_adafruit_gfx.getFontDescent();                           // negative; in mathematicians view
      int16_t th = ta - td;                                                          // text box height
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, (EPD_WIDTH - th) / 2);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info);

      u8g2_for_adafruit_gfx.setFont(FONT_MAIN);     // extended font
      ta = u8g2_for_adafruit_gfx.getFontAscent();   // positive
      td = u8g2_for_adafruit_gfx.getFontDescent();  // negative; in mathematicians view
      th = ta - td;
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info2.c_str());
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) - 100 - testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info2);

      tw = u8g2_for_adafruit_gfx.getUTF8Width(info3.c_str());
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) - 100 + th + 3 - testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info3);

      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);
      ta = u8g2_for_adafruit_gfx.getFontAscent();                                                         // positive
      td = u8g2_for_adafruit_gfx.getFontDescent();                                                        // negative; in mathematicians view
      th = ta - td;                                                                                       // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Activation Successful");                                   // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + 100 + testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Activation Successful");

      u8g2_for_adafruit_gfx.setFont(FONT_SMALL);                                                                   // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Please update a picture in your App.");                             // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + 100 + th + 3 + testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Please update a picture in your App.");                                         // UTF-8 string: "<" 550 448 664 ">"

      displayOverlays(display, displayInfos, false, fullColor);

   } while (display.nextPage());
}

void displayTurnOn() {
   String info1 = "Ich schlafe ...";
   String info2 = "Drücke die Taste auf der Rückseite";
   String info3 = "um mich zu wecken.";
   Serial.print(F("\n[EPD] Press to turn on Screen Loading - "));
   int testOffset = 200;

   char msg[128];
   sprintf(msg, "%s%s%s", "https://paperlesspaper.de/b?d=", epd_client_id, "&w=99");

   uint8_t QRData[qrcode_getBufferSize(QR_VERSION)];
   uint8_t blockSize;
   uint8_t page = 0;
   qrcode_initText(&QR, QRData, QR_VERSION, ECC_LOW, msg);
   blockSize = EPD_QR_SIZE_SMALL;
   uint16_t x0 = (EPD_HEIGHT - 30 * blockSize) / 2;

   int foreGround = COLOR_WHITE;
   int backGround = COLOR_BLACK;
   bool fullColor = false;

   if (displaySettings.quickRefresh) {
      displayWipe(true);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
      display.init(115200);
      foreGround = displaySettings.colorWhiteFast;
      backGround = displaySettings.colorBlackFast;
   } else {
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
      display.init(115200);
      fullColor = true;
   }

   display.setRotation(displaySettings.rotationText);
   display.setFullWindow();
   display.firstPage();
   do {
      int16_t tw = 0;
      display.fillScreen(backGround);
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFontDirection(0);             // left to right (this is default)
      u8g2_for_adafruit_gfx.setFontMode(1);                  // use u8g2 transparent mode (this is default)

      u8g2_for_adafruit_gfx.setFont(FONT_BIG);                 // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info2.c_str());  // text box width
      int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();      // positive
      int16_t td = u8g2_for_adafruit_gfx.getFontDescent();     // negative; in mathematicians view
      int16_t th = ta - td;
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, (EPD_WIDTH - th) / 2);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info2);                                            // UTF-8 string: "<" 550 448 664 ">"

      tw = u8g2_for_adafruit_gfx.getUTF8Width(info3.c_str());                                   // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + th + 3);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info3);

      u8g2_for_adafruit_gfx.setFont(FONT_MAIN);     // extended font
      ta = u8g2_for_adafruit_gfx.getFontAscent();   // positive
      td = u8g2_for_adafruit_gfx.getFontDescent();  // negative; in mathematicians view
      th = ta - td;
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info1.c_str());
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) - 100 - testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print(info1);

      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);
      ta = u8g2_for_adafruit_gfx.getFontAscent();                                                         // positive
      td = u8g2_for_adafruit_gfx.getFontDescent();                                                        // negative; in mathematicians view
      th = ta - td;                                                                                       // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("I am sleeping...");                                        // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + 100 + testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print("I am sleeping...");

      u8g2_for_adafruit_gfx.setFont(FONT_SMALL);                                                                   // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Press the button on the back to wake me up.");                      // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + 100 + th + 3 + testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Press the button on the back to wake me up.");

      uint16_t y0 = ((EPD_WIDTH / 2) + 150 + testOffset + testOffset);
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

      displayOverlays(display, displayInfos, true, true);

   } while (display.nextPage());
}

void displayWifiActivate(bool wifiProvisioningDone) {
   // Paged display of the QR code with top left corner at (x0,y0)
   // on the display

   char msg[128];
   sprintf(msg, "%s%s%s%s", "https://paperlesspaper.de/b?d=", epd_client_id, "&w=", wifiProvisioningDone ? "1" : "0");
   int testOffset = 200;

   uint8_t QRData[qrcode_getBufferSize(QR_VERSION)];
   uint8_t blockSize;
   uint8_t page = 0;
   // QR Code 5*30 = 150px

   qrcode_initText(&QR, QRData, QR_VERSION, ECC_LOW, msg);
   // blockSize = (display.height() - (2 * QR_QUIET_ZONE)) / QR.size;
   blockSize = EPD_QR_SIZE_BIG;

   uint16_t x0 = (EPD_HEIGHT - 30 * blockSize) / 2;

   int foreGround = COLOR_BLACK;
   int backGround = COLOR_WHITE;
   bool fullColor = false;

#if DEBUG
   Serial.printf("[EPD] Wifi Activate Function: %d\n", wifiProvisioningDone);
#endif
   Serial.printf("[EPD] QR Size: %d | Locate: %d,%d | DispH: %d \n", QR.size, x0, y0, display.height());
   Serial.printf("[EPD] QR Block: %d | Data: %s \n", blockSize, msg);

   if (displaySettings.quickRefresh) {
      displayWipe(true);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
      display.init(115200);
      foreGround = displaySettings.colorBlackFast;
      backGround = displaySettings.colorWhiteFast;
   } else {
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
      display.init(115200);
      fullColor = true;
   }

   display.setRotation(displaySettings.rotationText);
   display.setFullWindow();
   display.firstPage();
   do {
      int16_t tw = 0;
      int16_t ta, td, th;
      display.fillScreen(backGround);
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFontMode(1);                  // use u8g2 transparent mode (this is default)

      if (wifiProvisioningDone) {
         u8g2_for_adafruit_gfx.setFont(FONT_MAIN);
         ta = u8g2_for_adafruit_gfx.getFontAscent();                                                         // positive
         td = u8g2_for_adafruit_gfx.getFontDescent();                                                        // negative; in mathematicians view
         th = ta - td;                                                                                       // extended font
         tw = u8g2_for_adafruit_gfx.getUTF8Width("Gerät aktivieren");                                        // text box width
         u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) - 100 - testOffset);  // start writing at this position
         u8g2_for_adafruit_gfx.print("Gerät aktivieren");
      } else {
         u8g2_for_adafruit_gfx.setFont(FONT_MAIN);                                                           // extended font
         tw = u8g2_for_adafruit_gfx.getUTF8Width("WLAN verbinden");                                          // text box width
         u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) - 100 - testOffset);  // start writing at this position
         u8g2_for_adafruit_gfx.print("WLAN verbinden");
      }
      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);
      ta = u8g2_for_adafruit_gfx.getFontAscent();                                                   // positive
      td = u8g2_for_adafruit_gfx.getFontDescent();                                                  // negative; in mathematicians view
      th = ta - td;                                                                                 // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("1. Scanne den QR Code um die App zu installieren");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, (EPD_WIDTH - th) / 2);                 // start writing at this position
      u8g2_for_adafruit_gfx.print("1. Scanne den QR Code um die App zu installieren");

      tw = u8g2_for_adafruit_gfx.getUTF8Width("2. Folge den Schritten in der App");             // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + th + 3);  // start writing at this position
      u8g2_for_adafruit_gfx.print("2. Folge den Schritten in der App");

      tw = u8g2_for_adafruit_gfx.getUTF8Width("Connecting WiFi");                                         // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + 100 + testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Connecting WiFi");

      u8g2_for_adafruit_gfx.setFont(FONT_SMALL);                                                                   // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Scan the QR code or open the App.");                                // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + 100 + th + 3 + testOffset);  // start writing at this position
      u8g2_for_adafruit_gfx.print("Scan the QR code or open the App.");

      uint16_t y0 = ((EPD_WIDTH / 2) + 50 + testOffset * 1.5);
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

      displayOverlays(display, displayInfos, false, fullColor);

   } while (display.nextPage());

   // Serial.print(F("\nEnd"));
}

void displaySetText(String info, bool isBlackboard, bool quickRefresh, int position) {
   int foreGround = COLOR_BLACK;
   int backGround = COLOR_WHITE;
   bool invert = false;

   if (quickRefresh) {
      displayWipe(true);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
      display.init(115200);
      foreGround = displaySettings.colorBlackFast;
      backGround = displaySettings.colorWhiteFast;
      if (isBlackboard) {
         invert = true;
         foreGround = displaySettings.colorWhiteFast;
         backGround = displaySettings.colorBlackFast;
      }
   } else {
      if (isBlackboard) {
         invert = true;
         foreGround = COLOR_WHITE;
         backGround = COLOR_BLACK;
      }
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
      display.init(115200);
   }

   display.setRotation(displaySettings.rotationText);
   display.setFullWindow();

   display.firstPage();
   do {
      display.fillScreen(backGround);
      u8g2_for_adafruit_gfx.setFontDirection(0);             // left to right (this is default)
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color

      u8g2_for_adafruit_gfx.setFont(FONT_BIG);  // extended font
      u8g2_for_adafruit_gfx.setFontMode(1);     // use u8g2 transparent mode (this is default)

      int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width(info.c_str());                                           // text box width
      int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();                                                      // positive
      int16_t td = u8g2_for_adafruit_gfx.getFontDescent();                                                     // negative; in mathematicians view
      int16_t th = ta - td;                                                                                    // text box height
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, ((EPD_WIDTH - th) / 2) + (position * (th + 2)));  // start writing at this position

      u8g2_for_adafruit_gfx.print(info);
      if (info.length() > 1) {
         displayInfos.deviceInfoString = true;
      }
   } while (display.nextPage());
}

#if DEBUG
void displaySetBlankTest(int offsetVar, bool doQuickRefresh, bool useAltInit) {
   int foreGround = COLOR_BLACK;
   int backGround = COLOR_WHITE;
   Serial.println("[EPD] Set Display to Test Pattern");
   Serial.println(displaySettings.displayQuickRefreshTime);

   if (doQuickRefresh) {
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
      if (useAltInit)
         display.initAlt(115200);
      else
         display.init(115200);
      foreGround = displaySettings.colorBlackFast;
      backGround = displaySettings.colorWhiteFast;
   } else {
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
      if (useAltInit)
         display.initAlt(115200);
      else
         display.init(115200);
   }

   // use u8g2 transparent mode (this is default)

   int offset = offsetVar * 40;
   int offset2 = offsetVar * 20;

   display.setRotation(displaySettings.rotationText);
   display.setFullWindow();
   display.firstPage();
   do {
      display.fillScreen(backGround);
      u8g2_for_adafruit_gfx.setFontDirection(0);             // left to right (this is default)
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround);  // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround);  // apply Adafruit GFX color

      // display.fillRect(0, 0, EPD_HEIGHT, EPD_WIDTH, GxEPD_BLACK);
      // display.fillRect(2, 2, EPD_HEIGHT - 4, EPD_WIDTH - 4, GxEPD_WHITE);
      // display.fillRect(5, 5, EPD_HEIGHT - 10, EPD_WIDTH - 10, GxEPD_BLACK);
      // display.fillRect(8, 8, EPD_HEIGHT - 16, EPD_WIDTH - 16, GxEPD_WHITE);
      //   Space all around
      //   display.fillRect(0, 0, EPD_WIDTH, EPD_HEIGHT, GxEPD_WHITE);
      u8g2_for_adafruit_gfx.setFont(FONT_BIG);                                             // extended font
      int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width("Das ist ein Test und wir Testen");  // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, offset);                      // start writing at this position
      u8g2_for_adafruit_gfx.print("Das ist ein Test und wir Testen");

      display.fillRect(40, offset, 40, 40, GxEPD_BLACK);
      display.fillRect(80, offset, 40, 40, GxEPD_BLUE);
      display.fillRect(120, offset, 40, 40, GxEPD_GREEN);
      display.fillRect(160, offset, 40, 40, GxEPD_RED);
      display.fillRect(200, offset, 40, 40, GxEPD_ORANGE);
      display.fillRect(240, offset, 40, 40, GxEPD_WHITE);
      display.fillRect(281, offset, 40, 40, GxEPD_YELLOW);

      display.fillRect(300, offset2, 20, 20, GxEPD_BLACK);
      display.fillRect(320, offset2, 20, 20, GxEPD_BLUE);
      display.fillRect(340, offset2, 20, 20, GxEPD_GREEN);
      display.fillRect(360, offset2, 20, 20, GxEPD_RED);
      display.fillRect(380, offset2, 20, 20, GxEPD_ORANGE);
      display.fillRect(400, offset2, 20, 20, GxEPD_WHITE);
      display.fillRect(421, offset2, 20, 20, GxEPD_YELLOW);

      // display.setCursor(220, 100 + offset2);
      // display.println(info);
   } while (display.nextPage());
}
#endif

void displayWipe(bool quick, bool useAltInit) {
   Serial.println("[EPD] Display Wipe");

   uint8_t wipeColor = 6;
   if (quick) {
      Serial.println(displaySettings.displayQuickRefreshWipeTime);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshWipeTime, true);
      if (useAltInit)
         display.initAlt(115200);
      else
         display.init(115200);
      // wipeColor = 0x06;
      wipeColor = 3;  // tested 3

   } else {
      display.enableQuickRefresh(displaySettings.displayQuickRefreshWipeTime, false);
      if (useAltInit)
         display.initAlt(115200);
      else
         display.init(115200);
   }
   int colorSet = getColor(wipeColor);
   display.clearScreen(colorSet);
   display.refresh();
}

void printQRBlock(uint16_t x, uint16_t y, uint8_t size, uint16_t col) {
   for (uint8_t i = 0; i < size; i++)
      for (uint8_t j = 0; j < size; j++)
         display.drawPixel(x + i, y + j, col);
}

bool isEpaperActive() {
   return epaperIsUpdating;
}

void displaySetRotation(int orientation) {
   switch (orientation) {
      case 0:
#ifdef EPD_TYPE_13INCH
         displaySettings.rotationText = 0;
         displaySettings.rotationPicture = 0;
#else
         displaySettings.rotationText = 3;
         displaySettings.rotationPicture = 2;
#endif
         break;
      case 1:
#ifdef EPD_TYPE_13INCH
         displaySettings.rotationText = 3;
         displaySettings.rotationPicture = 0;
#else
         displaySettings.rotationText = 1;
         displaySettings.rotationPicture = 0;
#endif
         break;
      default:
#ifdef EPD_TYPE_13INCH
         displaySettings.rotationText = 0;
         displaySettings.rotationPicture = 0;
#else
         displaySettings.rotationText = 3;
         displaySettings.rotationPicture = 2;
#endif
   }
}

void displaySetQuickRefresh(bool enable) {
   if (displaySettings.globalQuickRefreshDisable) {
      displaySettings.quickRefresh = false;
      return;
   }
   if (enable) {
      displaySettings.quickRefresh = true;

   } else {
      displaySettings.quickRefresh = false;
   }
}

uint16_t getColor(uint8_t color) {
   switch (color) {
      case 0:
#ifdef EPD_TYPE_13INCH
         return 0x00;
#else
         return GxEPD_BLACK;
#endif
      case 1:
#ifdef EPD_TYPE_13INCH
         return 0x05;
#else
         return GxEPD_BLUE;
#endif
      case 2:
#ifdef EPD_TYPE_13INCH
         return 0x06;
#else
         return GxEPD_GREEN;
#endif
      case 3:
#ifdef EPD_TYPE_13INCH
         return 0x03;
#else
         return GxEPD_RED;
#endif
      case 5:
#ifdef EPD_TYPE_13INCH
         return 0x02;
#else
         return GxEPD_YELLOW;
#endif
      case 6:
#ifdef EPD_TYPE_13INCH
         return 0x01;
#else
         return GxEPD_WHITE;
#endif
      default:
#ifdef EPD_TYPE_13INCH
         return 0x01;
#else
         return GxEPD_WHITE;
#endif
   }
}

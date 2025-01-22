/*
 * display.c
 *
 *  Created on: Jan 19, 2025
 *      Author: itsme
 */
#include "ssd1306_128x64_i2c.h"
#ifdef SSD1306
#include "fonts.h"
#include "ESP.h"
#include <string.h>

static char textBuffer[9];
static bool line2ndupdate;
bool SSD1306detected = false;

//static   uint8_t xpos;


void displayCreateStyle() {
  ssd1306_writeString(6, 0, "de Omgevingsmonitor", Font_5x8);
  ssd1306_writeString(49, 1, "KITT", Font_5x8);
  ssd1306_writeString(3, 2, "act:     -.- dBA", Font_5x8);
  ssd1306_writeString(3, 3, "peak:    -.- dBA", Font_5x8);
  ssd1306_writeString(3, 4, "PM2.5:     - µg/m³", Font_5x8);
  ssd1306_writeString(3, 5, "PM10:      - µg/m³", Font_5x8);
  ssd1306_writeString(3, 6, "VOC:  -      -,-°C", Font_5x8);
  ssd1306_writeString(3, 7, "NOx:  -      -,- %", Font_5x8);
  ssd1306_setScreenOn();
  line2ndupdate = false;
}

void display2ndmsg2ndline() {
  ssd1306_writeString(3, 1, "WOTS Gadget v", Font_5x8);
  sprintf(textBuffer, "%s", SRC_VERSION);
  ssd1306_writeString(81, 1, textBuffer, Font_5x8);
}

void displayStart() {
//  if (Check_USB_PowerOn() || userToggle) {
    if (ssd1306_Init()) {
      Info("Display SSD1306 not detected on I2C2");
      SSD1306detected = false;
    }
    else {
      Info("Display SSD1306 is detected on I2C2");
      SSD1306detected = true;
      displayCreateStyle();
    }
//  }
}

void displayActdBA(){
  sprintf(textBuffer, "% 4.1f", MeasVal.dBA);
  ssd1306_writeString((7 * (Font_5x8.FontWidth + 1)) + 3, 2, textBuffer, Font_5x8);
}

void displayPeakdBA(){
  sprintf(textBuffer, "% 4.1f", MeasVal.dBApeak);
  ssd1306_writeString((7 * (Font_5x8.FontWidth + 1)) + 3, 3, textBuffer, Font_5x8);
}

void displayPM2p5() {
    sprintf(textBuffer, "% 5.1f", MeasVal.PM2p5);
//  Debug("PM2.5 value in displayPMs %f", MeasVal.PM2p5);
    ssd1306_writeString((7 * (Font_5x8.FontWidth + 1)) + 3, 4, textBuffer, Font_5x8);
    if (!line2ndupdate) {
      display2ndmsg2ndline();
    }
}

void displayPM10() {
    sprintf(textBuffer, "% 5.1f", MeasVal.PM10p0);
//  Debug("PM10 value in displayPMs %f", MeasVal.PM10p0);
    ssd1306_writeString((7 * (Font_5x8.FontWidth + 1)) + 3, 5, textBuffer, Font_5x8);
}

void displayVOC() {
  sprintf(textBuffer, "% 4d", MeasVal.VOCIndex);
//  Debug("length of VOC output %d", strlen(textBuffer));
  ssd1306_writeString((4 * (Font_5x8.FontWidth + 1)) + 3, 6, textBuffer, Font_5x8);
}

void displayTemperature() {
  sprintf(textBuffer, "%2.2f", MeasVal.Temperature);
  ssd1306_writeString((11 * (Font_5x8.FontWidth + 1)) + 3, 6, textBuffer, Font_5x8);
}

void displayHumidity(){
  sprintf(textBuffer, "%2.2f", MeasVal.Humidity);
  ssd1306_writeString((11 * (Font_5x8.FontWidth + 1)) + 3, 7, textBuffer, Font_5x8);
}

void displayNOx() {
  sprintf(textBuffer, "% 4d", MeasVal.airNOx);
  ssd1306_writeString((4 * (Font_5x8.FontWidth + 1)) + 3, 7, textBuffer, Font_5x8);
}
#endif

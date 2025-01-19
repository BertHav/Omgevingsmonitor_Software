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

static char textBuffer[7];
//static   uint8_t xpos;

void displayCreateStyle() {
  ssd1306_writeString(0, 0, "de Omgevingsmonitor", Font_5x8);
  ssd1306_writeString(49, 1, "KITT", Font_5x8);
  ssd1306_writeString(0, 2, "act:     -.- dBA", Font_5x8);
  ssd1306_writeString(0, 3, "peak:    -.- dBA", Font_5x8);
  ssd1306_writeString(0, 4, "PM2.5:   - µg/m³", Font_5x8);
  ssd1306_writeString(0, 5, "PM10:    - µg/m³", Font_5x8);
  ssd1306_writeString(0, 6, "VOC:  -    -,-°C", Font_5x8);
  ssd1306_writeString(0, 7, "NOx:  -    -,- %", Font_5x8);
  ssd1306_setScreenOn();
}

void displayActdBA(){
  sprintf(textBuffer, "% 3.1f", MeasVal.dBA);
  ssd1306_writeString(7 * (Font_5x8.FontWidth + 1), 2, textBuffer, Font_5x8);
}

void displayPeakdBA(){
  sprintf(textBuffer, "% 3.1f", MeasVal.dBApeak);
  ssd1306_writeString(7 * (Font_5x8.FontWidth + 1), 3, textBuffer, Font_5x8);
}

void displayPMs() {
  sprintf(textBuffer, "% 3.1f", MeasVal.airPM2);
//  Debug("PM2.5 value in displayPMs %f", MeasVal.airPM2);
  ssd1306_writeString(6 * (Font_5x8.FontWidth + 1), 4, textBuffer, Font_5x8);
  sprintf(textBuffer, "%3.1f", MeasVal.airPM10);
//  Debug("PM10 value in displayPMs %f", MeasVal.airPM10);
  ssd1306_writeString(7 * (Font_5x8.FontWidth + 1), 5, textBuffer, Font_5x8);
}

void displayVOC(){
  sprintf(textBuffer, "% 4d", MeasVal.VOCIndex);
//  Debug("length of VOC output %d", strlen(textBuffer));
  ssd1306_writeString(4 * (Font_5x8.FontWidth + 1), 6, textBuffer, Font_5x8);
}

void displayTemperature() {
  sprintf(textBuffer, "%2.2f", MeasVal.Temperature);
  ssd1306_writeString(9 * (Font_5x8.FontWidth + 1), 6, textBuffer, Font_5x8);
}

void displayHumidity(){
  sprintf(textBuffer, "%2.2f", MeasVal.Humidity);
  ssd1306_writeString(9 * (Font_5x8.FontWidth + 1), 7, textBuffer, Font_5x8);
}

void displayNOx() {
  sprintf(textBuffer, "% 4d", MeasVal.airNOx);
  ssd1306_writeString(4 * (Font_5x8.FontWidth + 1), 7, textBuffer, Font_5x8);
}
#endif

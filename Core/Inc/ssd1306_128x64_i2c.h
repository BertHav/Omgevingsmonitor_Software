/*
** SSD1306 I2C 128x64 Monochrome OLED LCD Module display Library
** Copyright (C) 2009 - 2018 Radu Motisan, radu.motisan@gmail.com, www.pocketmagic.net
**
** This file is a part of "SSD1306 NO BUFFER" open source library.
**
** ILI9341 Library is free software; you can redistribute it and/or modify
** it under the terms of the GNU Lesser General Public License as published
** by the Free Software Foundation; either version 3 of the License,
** or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU Lesser General Public License for more details.
**
** You should have received a copy of the GNU Lesser General Public License
** along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdint.h>
#include "fonts.h"
// define build with display inclusion in project
//#define SSD1306

#define SSD1306_I2C_ADDR        0x78

// define size
#define SSD1306_WIDTH	128
#define SSD1306_HEIGHT	64

// define colors
#define BLACK 0
#define WHITE 1
#define INVERSE 2
#define TRANSPARENT 0xff

// define level 1 commands
#define SSD1306_CMD_SETCONTRAST 0x81
#define SSD1306_CMD_DISPLAYALLON_RESUME 0xA4
#define SSD1306_CMD_DISPLAYALLON 0xA5
#define SSD1306_CMD_NORMALDISPLAY 0xA6
#define SSD1306_CMD_INVERTDISPLAY 0xA7
#define SSD1306_CMD_DISPLAYOFF 0xAE
#define SSD1306_CMD_DISPLAYON 0xAF

#define SSD1306_CMD_SETDISPLAYOFFSET 0xD3
#define SSD1306_CMD_SETCOMPINS 0xDA

#define SSD1306_CMD_SETVCOMDETECT 0xDB

#define SSD1306_CMD_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_CMD_SETPRECHARGE 0xD9

#define SSD1306_CMD_SETMULTIPLEX 0xA8

#define SSD1306_CMD_SETLOWCOLUMN 0x00
#define SSD1306_CMD_SETHIGHCOLUMN 0x10

#define SSD1306_CMD_SETSTARTLINE 0x40

#define SSD1306_CMD_MEMORYMODE 0x20
#define SSD1306_CMD_COLUMNADDR 0x21
#define SSD1306_CMD_PAGEADDR   0x22

#define SSD1306_CMD_COMSCANINC 0xC0
#define SSD1306_CMD_COMSCANDEC 0xC8

#define SSD1306_CMD_SEGREMAP 0xA0

#define SSD1306_CMD_CHARGEPUMP 0x8D

#define SSD1306_CMD_EXTERNALVCC 0x1
#define SSD1306_CMD_SWITCHCAPVCC 0x2

#define SSD1306_CMD_ACTIVATE_SCROLL 0x2F
#define SSD1306_CMD_DEACTIVATE_SCROLL 0x2E


uint8_t  ssd1306_Init();
void ssd1306_powerOff();
void ssd1306_setPos(uint8_t x, uint8_t y);
void ssd1306_clearDisplay();
//void ssd1306_WriteCommand(uint8_t c);
void ssd1306_invertDisplay(uint8_t i);
void ssd1306_setScreenOn();
void ssd1306_setScreenOff();
void ssd1306_contrast(uint8_t contrast);
void ssd1306_drawPixel(int8_t x, int8_t y, uint8_t color);
void ssd1306_drawCircle(int8_t x, int8_t y, int8_t radius, int8_t color);
void ssd1306_drawChar(int8_t x, int8_t row, char c,FontDefSmall Font);				// one row is 8 pixels wide
void ssd1306_writeString(uint8_t x, uint8_t row, const char *string, FontDefSmall Font);// one row is 8 pixels wide


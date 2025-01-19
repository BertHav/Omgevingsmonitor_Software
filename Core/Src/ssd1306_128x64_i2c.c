#include "ssd1306_128x64_i2c.h"
#ifdef SSD1306
#include "i2c.h"
#include "utils.h"
#include "fonts.h"


//
//  Send a byte to the command register
//
static uint8_t ssd1306_WriteCommand(uint8_t command)
{
    return HAL_I2C_Mem_Write(&hi2c2, SSD1306_I2C_ADDR, 0x00, 1, &command, 1, 10);
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t ssd1306_WriteContent(const uint8_t* data, uint16_t count) {
  return (int8_t)HAL_I2C_Master_Transmit(&hi2c2, SSD1306_I2C_ADDR, (uint8_t*)data, count, 100);
}

uint8_t  ssd1306_Init() {
	// Init sequence
  // Wait for the screen to boot
  // HAL_Delay(100); // moved to power on routine to detect sen5x reliable

  int status = 0;
  status += ssd1306_WriteCommand(SSD1306_CMD_DISPLAYOFF);                    // 0xAE
  Debug("status %d", status);
  status += ssd1306_WriteCommand(SSD1306_CMD_SETDISPLAYCLOCKDIV);            // 0xD5
  status += ssd1306_WriteCommand(0x80);                                  // the suggested ratio 0x80
  HAL_Delay(1);
  status += ssd1306_WriteCommand(SSD1306_CMD_SETMULTIPLEX);                  // 0xA8
  status += ssd1306_WriteCommand(SSD1306_HEIGHT - 1);
  HAL_Delay(1);

  status += ssd1306_WriteCommand(SSD1306_CMD_SETDISPLAYOFFSET);              // 0xD3
  status += ssd1306_WriteCommand(0x0);                                   // no offset
  HAL_Delay(1);
  status += ssd1306_WriteCommand(SSD1306_CMD_SETSTARTLINE | 0x0);            // line #0
  HAL_Delay(1);
  status += ssd1306_WriteCommand(SSD1306_CMD_CHARGEPUMP);                    // 0x8D
  status += ssd1306_WriteCommand(0x14);
  HAL_Delay(1);
  status += ssd1306_WriteCommand(SSD1306_CMD_MEMORYMODE);                    // 0x20
  status += ssd1306_WriteCommand(0x00);                                  // 0x0 act like ks0108
  HAL_Delay(1);
  status += ssd1306_WriteCommand(SSD1306_CMD_SEGREMAP | 0x1);
  status += ssd1306_WriteCommand(SSD1306_CMD_COMSCANDEC);
  HAL_Delay(1);

	// 128x64
  status += ssd1306_WriteCommand(SSD1306_CMD_SETCOMPINS);                    // 0xDA
  status += ssd1306_WriteCommand(0x12);
  HAL_Delay(1);
  status += ssd1306_WriteCommand(SSD1306_CMD_SETCONTRAST);                   // 0x81
  status += ssd1306_WriteCommand(0xFF);
  HAL_Delay(1);
// was		ssd1306_WriteCommand(0xCF);

	
  status += ssd1306_WriteCommand(SSD1306_CMD_SETPRECHARGE);                  // 0xd9
  status += ssd1306_WriteCommand(0x22);
  HAL_Delay(1);
  status += ssd1306_WriteCommand(SSD1306_CMD_SETVCOMDETECT);                 // 0xDB
  status += ssd1306_WriteCommand(0x40);
  HAL_Delay(1);
  status += ssd1306_WriteCommand(SSD1306_CMD_DISPLAYALLON_RESUME);           // 0xA4
  HAL_Delay(1);
  status += ssd1306_WriteCommand(SSD1306_CMD_NORMALDISPLAY);                 // 0xA6
  HAL_Delay(1);

  status += ssd1306_WriteCommand(SSD1306_CMD_DEACTIVATE_SCROLL);

  ssd1306_clearDisplay();

//  status += ssd1306_WriteCommand(SSD1306_CMD_DISPLAYON);					//--turn on oled panel

  if (status != 0) {
    Error("Initialization of SSD1306 display failed.");
    return 1;
  }

	// clear 
//	ssd1306_clearDisplay();
	return 0;
}

void ssd1306_setScreenOn() {
  ssd1306_WriteCommand(SSD1306_CMD_DISPLAYON);          //--turn on oled panel
}

void ssd1306_setScreenOff() {
  ssd1306_WriteCommand(SSD1306_CMD_DISPLAYOFF);          //--turn off oled panel
}

void ssd1306_setPos(uint8_t x, uint8_t y) {
	// set column
  ssd1306_WriteCommand(SSD1306_CMD_COLUMNADDR);
  ssd1306_WriteCommand(x);   // Column start address (0 = reset)
  ssd1306_WriteCommand(SSD1306_WIDTH - 1); // Column end address (127 = reset)
	// set line
  ssd1306_WriteCommand(SSD1306_CMD_PAGEADDR);
  ssd1306_WriteCommand(y); // Page start address (0 = reset)
  ssd1306_WriteCommand(SSD1306_HEIGHT / 8 - 1); // Page end address
}


void ssd1306_clearDisplay() {
  ssd1306_setPos(0,0);
  uint8_t line = 0x00;
    for (int i=0; i<SSD1306_WIDTH * SSD1306_HEIGHT / 8; i++) {  
      HAL_I2C_Mem_Write(&hi2c2, SSD1306_I2C_ADDR, 0x40, 1, &line, 1, 100);
    }
}

/*
void ssd1306_writeCommand(uint8_t c) {
    ssd1306_WriteCommand(c);
}
*/
void ssd1306_invertDisplay(uint8_t i) {
	if (i) {
	  ssd1306_WriteCommand(SSD1306_CMD_INVERTDISPLAY);
	} else {
	  ssd1306_WriteCommand(SSD1306_CMD_NORMALDISPLAY);
	}
}

void ssd1306_contrast(uint8_t contrast) {
  ssd1306_WriteCommand(SSD1306_CMD_SETCONTRAST);
  ssd1306_WriteCommand(contrast);
}

void ssd1306_drawPixel(int8_t x, int8_t y, uint8_t color) {
  ssd1306_setPos(x, y / 8);
  uint8_t px;
    if (color) 
    	px = (1<<y%8);
    else
    	px = (~(1<<y%8));
    HAL_I2C_Mem_Write(&hi2c2, SSD1306_I2C_ADDR, 0x40, 1, &px, 1, 100);
}

void ssd1306_drawCircle(int8_t x, int8_t y, int8_t radius, int8_t color) {
    int16_t xp = 0, yp = radius;
    int16_t d = 3 - (2 * radius);
    while(xp <= yp) {
      ssd1306_drawPixel(x + xp, y + yp, color);
      ssd1306_drawPixel(x + yp, y + xp, color);
      ssd1306_drawPixel(x - xp, y + yp, color);
      ssd1306_drawPixel(x + yp, y - xp, color);
      ssd1306_drawPixel(x - xp, y - yp, color);
      ssd1306_drawPixel(x - yp, y - xp, color);
      ssd1306_drawPixel(x + xp, y - yp, color);
      ssd1306_drawPixel(x - yp, y + xp, color);
        if (d < 0)
        	d += (4 * xp) + 6;
        else {
            d += (4 * (xp - yp)) + 10;
            y -= 1;
        }
        xp++;
    }
}

void ssd1306_drawChar(int8_t x, int8_t row, char c, FontDefSmall Font) {
  if (row > SSD1306_HEIGHT / Font.FontHeight) {
    Error("Unexpected return from ssd1306_DirectdrawChar");
    return;
	}
//  Debug("position set to x=%d, row=%d", x, row);
	ssd1306_setPos(x, row);
	// draw. optimisation: last font line is set as 0, to lower font array size
	for (int8_t i=0; i <= Font.FontWidth; i++ ) {
    uint8_t line = (i == Font.FontWidth)? 0 : Font.data[(c * (Font.FontWidth)) + i];
    HAL_I2C_Mem_Write(&hi2c2, SSD1306_I2C_ADDR, 0x40, 1, &line, 1, 100);
//    Debug("display line content for char %c - nr: %d content 0x%02X, position in array: %d FontWidth: %d", c, c, line, ((c * (Font.FontWidth)) + i), Font.FontWidth);
	}
}

void ssd1306_writeString(uint8_t x, uint8_t row, const char *string, FontDefSmall Font) {
  unsigned char c;
  while ( (c = *string++) ) {
    if (c != 0xC2) {
      ssd1306_drawChar(x, row, c, Font);
      if (x <= SSD1306_WIDTH -2 * Font.FontWidth)
       	 x +=  Font.FontWidth+1;
      else {
       	x = 0;
       	row += Font.FontHeight;
      }
    }
  }
}


#endif

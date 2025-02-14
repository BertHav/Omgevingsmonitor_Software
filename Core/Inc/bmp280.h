/*
 * bmp280.h
 *
 *  Created on: Feb 10, 2025
 *      Author: itsme
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "math.h"
#include "stm32l0xx_hal.h"

#define BMP280_ADDRESS                            0x76 // the 7 bit address
#define BMP280_I2C_ADDR_SEC                       0x77 // if SDO is connected to Vddio
//#define BMP280_OK               0
//#define BMP280_E_DEV_NOT_FOUND -4
#define BMP280_CHIP_ID                            0x58

#define BMP280_REG_CHIP_ID                        0xD0
#define BMP280_REG_RESET                          0xE0
#define BMP280_REG_TEMP_PRESS_CALIB_DATA          0x88
#define BMP280_REG_HUMIDITY_CALIB_DATA            0xE1
#define BMP280_REG_CTRL_HUM                       0xF2
#define BMP280_REG_STATUS                         0xF3
#define BMP280_REG_PWR_CTRL                       0xF4
#define BMP280_REG_CTRL_MEAS                      0xF4
#define BMP280_REG_CONFIG                         0xF5
#define BMP280_REG_DATA                           0xF7
#define BMP280_REG_PRESS_LSB                      0xF8
#define BMP280_REG_PRESS_XLSB                     0xF9
#define BMP280_REG_TEMP_MSB                       0xFA
#define BMP280_REG_TEMP_LSB                       0xFB
#define BMP280_REG_TEMP_XLSB                      0xFC


#define BMP280_RESET_VALUE                        0xB6

//weatherstation configuration
#define BMP280_SLEEP_MODE                         0x00
#define BMP280_FORCED_MODE                        0x02
#define BMP280_NORMAL_MODE                        0x03
#define BMP280_OSRS_T                             0x20  // oversampling x1
#define BMP280_OSRS_T_2                           0x40  // oversampling x2

#define BMP280_OSRS_P                             0x04  // oversampling 1
#define BMP280_OSRS_P_4                           0x0C  // oversampling 1
#define BMP280_T_SB                               0x00
#define BMP280_T_SB_125                           0x40  // standby 125ms
#define BMP280_T_SB_500                           0x80  // standby 500ms

#define BMP280_FILTER                             0x00
#define BMP280_FILTER_4                           0x0C  // IIR filter coeff 4

#define BMP280_SPI_OFF                            0x00

#define BMP280_MEAS_RDY                           0x08  // relevant bit for measurement ready

#define BMP280_ERROR                              0xFF
typedef struct
{
  uint16_t T1;
  int16_t T2;
  int16_t T3;
  uint16_t P1;
  int16_t P2;
  int16_t P3;
  int16_t P4;
  int16_t P5;
  int16_t P6;
  int16_t P7;
  int16_t P8;
  int16_t P9;
} calibration_param_t;

typedef enum {
    BMP_STATE_INIT,
    BMP_SET_CONFIG,
    BMP_STATE_START_MEASUREMENTS,
    BMP_STATE_PROCESS_RESULTS,
    BMP_STATE_WAIT_FOR_READY,
    BMP_READ_MEASUREMENT_ARRAY,
    BMP_STATE_OFF,
    BMP_CHECK_CONFIG,
    BMP_STATE_WAIT,
    BMP_MODE_SELECT
} BMP280State;

typedef bool (*I2CReadMEM)(uint8_t address, uint16_t MemAddress, uint16_t MemSize, uint8_t* buffer, uint16_t nrBytes);
typedef bool (*I2CWriteMEM)(uint8_t address, uint16_t MemAddress, uint16_t MemSize, uint8_t* buffer, uint16_t nrBytes);

void BMP_Init(I2CReadMEM readMemFunction, I2CWriteMEM writeMemFunction);
void setBMP280TimeStamp(uint32_t ticks);
BMP280State BMP_Upkeep(void);
void BMP280_set_modus(uint8_t modus);
bool BMP280_DeviceConnected();


#endif /* INC_BMP280_H_ */

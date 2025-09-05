/*
 * bmp280.c
 *
 *  Created on: Feb 10, 2025
 *      Author: itsme
 */
#include <stdbool.h>

#include "bmp280.h"
#include "utils.h"
#include "measurement.h"
#include "ESP.h"
#include "statusCheck.h"
#include "RealTimeClock.h"
#include "i2c.h"
/*

*/

static I2CWriteMEM WriteMemFunction = NULL;
static I2CReadMEM ReadMemFunction = NULL;

static bool BMP280_set_mode();

static calibration_param_t dig;
static int32_t t_fine;        //used for pressure compensation, changes with temperature
static uint32_t BMP280TimeStamp;
static uint8_t bmp280samplecounter = 0;
static uint8_t bmp280I2Caddr = BMP280_ADDRESS;

BMP280State BMPState = BMP_MODE_SELECT;


static bool bmperror = false;
static uint8_t mode = BMP280_FORCED_MODE;
static int32_t raw_temp, raw_mpa;
static uint32_t BMP280TimeStamp;

static bool WriteMemRegister(uint16_t MemAddress, uint16_t MemSize, uint8_t* buffer, uint16_t nrBytes) {
  if (WriteMemFunction != NULL) {
    return WriteMemFunction(bmp280I2Caddr, MemAddress, MemSize, buffer, nrBytes);
  }
  return false;
}

static bool ReadMemRegister(uint16_t MemAddress, uint16_t MemSize, uint8_t* buffer, uint16_t nrBytes) {
  if (ReadMemFunction != NULL) {
    return ReadMemFunction(bmp280I2Caddr, MemAddress, MemSize, buffer, nrBytes);
  }
  return false;
}

void setBMP280TimeStamp(uint32_t ticks) {
  BMP280TimeStamp = HAL_GetTick() + ticks;
}

void BMP_Init(I2CReadMEM readMemFunction, I2CWriteMEM writeMemFunction) {
  ReadMemFunction = readMemFunction;
  WriteMemFunction = writeMemFunction;
}

static void BMP280_reset() {
  uint8_t data = BMP280_RESET_VALUE;
  bmperror = false;
  WriteMemRegister(BMP280_REG_RESET, 1, &data, 1);
  Info("BMP280 has been reset");
  HAL_Delay(100);
}

static void BMP280_get_calibration() {
  ReadMemRegister(BMP280_REG_TEMP_PRESS_CALIB_DATA, 1, (uint8_t *)&dig, sizeof(dig));
  Debug("BMP280 calibration data:");
  char stc = 'T';
  uint8_t idx = 1;
  for (uint8_t x = 0; x <= 11; x++) {
    if ((x == 0) || (x == 3)) {
      if (x==3) {
        idx = 1;
        stc = 'P';
      }
      Debug("dig.%c%d: %d", stc, idx, (uint16_t)dig.calarray[x]);
    }
    else {
    Debug("dig.%c%d: %d", stc, idx, dig.calarray[x]);
    }
    idx++;
  }
}

void BMP280_set_config() {
  uint8_t data = (BMP280_T_SB_500 | BMP280_FILTER_16 | BMP280_SPI_OFF);
  if (!WriteMemRegister(BMP280_REG_CONFIG, 1, &data, 1)) {
    Error("BMP280 writing to BMP280_REG_CONFIG");
  }
  ReadMemRegister(BMP280_REG_CONFIG, 1, &data, 1);
  Debug("BMP280_REG_CONFIG: 0x%02X", data);
}

 bool BMP280_DeviceConnected() {
   bool rslt = false;
    uint8_t bmpData;
    // Read the chip-id of bmp280 sensor
    bmp280I2Caddr = BMP280_ADDRESS;
    rslt = ReadMemRegister(BMP280_REG_CHIP_ID, 1, &bmpData, 1);
    if (bmpData != BMP280_CHIP_ID) {
      bmp280I2Caddr++;
      rslt = ReadMemRegister(BMP280_REG_CHIP_ID, 1, &bmpData, 1);
    }
    if (rslt && (bmpData == BMP280_CHIP_ID)) {
      Info("BMP280 with chip_id 0x%02X found at I2C address: 0x%02X", bmpData, bmp280I2Caddr);
      BMP280_reset(); // get an initial state
      HAL_Delay(3);
      BMP280_get_calibration();
      BMP280_set_config();
      BMP280_set_mode();
    }
    else {
      bmp280I2Caddr = 0;
      Error("BMP280 not found");
    }
    return bmp280I2Caddr;
}

static bool BMP280_get_measurement_values() {
  int8_t rslt = 1;
  uint8_t bmpData[6];
  HAL_Delay(9);
  BMP280TimeStamp = HAL_GetTick() + 1500;
  do {
    HAL_Delay(10);
    ReadMemRegister(BMP280_REG_STATUS, 1, &bmpData[0], 1);
    if(TimestampIsReached(BMP280TimeStamp)) {
      Error("BMP280 timeout while waiting for ready.");
      return false;
    }
    HAL_Delay(10);
  } while (((bmpData[0] & BMP280_NVM_RDY) == BMP280_NVM_RDY) || ((bmpData[0] & BMP280_MEAS_RDY) == BMP280_MEAS_RDY));
  rslt = ReadMemRegister(BMP280_REG_PRESS_MSB, 1, &bmpData[0], 6);
  raw_mpa = (int32_t)((((uint32_t)bmpData[0]) << 12) + (((uint32_t)bmpData[1]) << 4) + (((uint32_t)bmpData[2]) >> 4));
  if (bmpData[0] == 0x80) {
    Error("BMP280 Invalid read of barometric pressure.");
    Debug("Data[0]: 0x%02X, Data[1]: 0x%02X, Data[2]: 0x%02X, VALUE=0x%06X", bmpData[0], bmpData[1], bmpData[2], raw_mpa);
    SetAllBlueLED();
  }
  raw_temp = (int32_t)((((uint32_t)bmpData[3]) << 12) + (((uint32_t)bmpData[4]) << 4) + (((uint32_t)bmpData[5]) >> 4));
  if (bmpData[3] == 0x80) {
    Error("BMP280 Invalid read of temperature.");
    Debug("Data[3]: 0x%02X, Data[4]: 0x%02X, Data[5]: 0x%02X, VALUE=0x%06X", bmpData[3], bmpData[4], bmpData[5], raw_temp);
    SetAllBlueLED();
  }
//  Debug("raw_mpa: %ld, raw_temp: %ld ", raw_mpa, raw_temp);
  return rslt;
}

static uint8_t BMP280_get_mode() {
  int8_t rslt;
  uint8_t bmpData;
  rslt = ReadMemRegister(BMP280_REG_CTRL_MEAS, 1, &bmpData, 1);
  bmpData &= BMP280_NORMAL_MODE; //BMP280_NORMAL_MODE has all mode bits set 0x03;
  if (rslt == 0) {
    bmpData = 255;
  }
  return bmpData;
}

static bool BMP280_set_mode() {
  uint8_t data = (BMP280_OSRS_T_2 | BMP280_OSRS_P_16 | mode);
  WriteMemRegister(BMP280_REG_CTRL_MEAS, 1, &data, 1);
  HAL_Delay(10);
  data = BMP280_get_mode();
  data &= BMP280_NORMAL_MODE;
  Debug("BMP280 Operation mode: %s", data==0?"sleep":data==3?"normal":"forced");
  return true;
}


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// reads temperature value from internal bmp280 registers in centigrade
// copied from datasheet
static float BMP280_calc_temperature() {

  int32_t adc_T = raw_temp;
  int32_t var1, var2, T;
  float airTemp;

  var1 = ((((adc_T >> 3) - ((int32_t)dig.T1 << 1))) * ((int32_t)dig.T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig.T1)) * ((adc_T >> 4) - ((int32_t)dig.T1))) >> 12) * ((int32_t)dig.T3)) >> 14;

  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  airTemp = (float)T / 100.0;
  return airTemp;
}


// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// reads pressure value from internal bmp280 registers in pascal
// copied from datasheet
static float BMP280_calc_pressure() {

  int32_t adc_P = raw_mpa;
  int32_t var1, var2;
  uint32_t p;
  float airhPa;

  var1 = (((int32_t) t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((int32_t) dig.P6);
  var2 = var2 + ((var1 * ((int32_t) dig.P5))<<1);
  var2 = (var2>>2) + (((int32_t)dig.P4)<<16);
  var1 = (((dig.P3 * (((var1>>2) * (var1>>2)) >> 13))>>3) + ((((int32_t) dig.P2) * var1)>>1))>>18;
  var1 = ((((32768 + var1)) * ((int32_t) dig.P1))>>15);

  // Avoid exception caused by division with zero
  if (var1 == 0) {
    return 0;
  }
  p = ((uint32_t)(((int32_t)(1048576) - adc_P) - (var2>>12))) * 3125;

    // Check for overflows against UINT32_MAX/2; if pres is left-shifted by 1
  if (p < 0x80000000)
    {
      p = (p << 1) / ((uint32_t) var1);
    }
    else
    {
      p = (p / (uint32_t) var1) * 2;
    }
    var1 = (((int32_t) dig.P9) * ((int32_t) (((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((int32_t) (p>>2)) * ((int32_t) dig.P8))>>13;
    p = (uint32_t) ((int32_t)p + ((var1 + var2 + dig.P7)>>4));
    airhPa = (float)p / 100.0;
  return airhPa;
}

BMP280State BMP_Upkeep(void) {

  if(!TimestampIsReached(BMP280TimeStamp)){
    return BMPState;
  }
  switch(BMPState) {
  case BMP_MODE_SELECT:
    if ((getSensorLock() == FREE) || (getSensorLock() == BMP280)) {
      if (getSensorLock() == FREE) {
        setSensorLock(BMP280);
      }
      bmperror = false;
      bool changed = false;
      mode = BMP280_get_mode();
      mode &= BMP280_NORMAL_MODE;
      if (Check_USB_PowerOn()) {
        if (mode != BMP280_NORMAL_MODE) {
          mode =BMP280_NORMAL_MODE;
          changed = true;
          Info("BMP280 modus changed to normal");
        }
      }
      else {
        if ((mode == BMP280_NORMAL_MODE) || (mode == BMP280_SLEEP_MODE) || (mode == 255)) {
          changed = true;
          mode = BMP280_FORCED_MODE;
          if ((mode == BMP280_NORMAL_MODE)|| (mode == 255)) {
           Info("BMP280 modus changed to forced");
          }
        }
      }
      if (changed || (mode == BMP280_FORCED_MODE)) {
        BMP280_set_mode();
        changed = false;
      }
      if (BMP280_get_measurement_values()) {
        BMPState = BMP_STATE_PROCESS_RESULTS;
      }
      else {
        Error("BMP280 Error during reading measurement results array");
        bmperror = true;
        BMP280TimeStamp = HAL_GetTick() + 5000;
        BMPState = BMP_STATE_WAIT ;
      }
      setSensorLock(FREE);
    }
    else {
//      uint8_t locktype = getSensorLock();
//      Debug("Lock is not from BMP280, but from %s",
//        locktype==FREE?"FREE":locktype==HIDS?"HIDS":locktype==SGP40?"SGP40":locktype==AHT20?"AHT20":locktype==BMP280?"BMP280":"unknown");
      BMP280TimeStamp = HAL_GetTick() + 97;
    }
    break;

  case BMP_STATE_PROCESS_RESULTS:
    float airtemp, airhpa;
    airtemp = BMP280_calc_temperature();
    airhpa = BMP280_calc_pressure();
    Info("BMP280 barometric value: %.2fhPa, airtemperature: %2.2fC", airhpa, airtemp);

    if ((airhpa > 850.0) && (airhpa < 1100)) {
      setBMP280(airtemp, airhpa);
      if (Check_USB_PowerOn()) {
        BMP280TimeStamp = HAL_GetTick() + 28000;
        bmp280samplecounter = 1;
      }
      else {
        BMP280TimeStamp = HAL_GetTick() + 10000;
      }
    }
    else {
      Error("BMP280 value out of valid range, not stored/used");
      bmperror = true;
      BMP280TimeStamp = HAL_GetTick() + 10000;
    }
    BMPState = BMP_STATE_WAIT;
    break;

  case BMP_STATE_WAIT:
      BMPState = BMP_MODE_SELECT;
    break;


  default:
    // Handle unexpected state
    BMPState = BMP_MODE_SELECT;
    if (getSensorLock() == BMP280) {
      setSensorLock(FREE);
    }
    break;
  }
  return BMPState;
}

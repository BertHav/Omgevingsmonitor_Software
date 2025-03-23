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


static I2CWriteMEM WriteMemFunction = NULL;
static I2CReadMEM ReadMemFunction = NULL;
static calibration_param_t dig;

static uint8_t bmp280I2Caddr = BMP280_ADDRESS;
static uint8_t mode = BMP280_FORCED_MODE;
static int32_t t_fine;        /*used for pressure compensation, changes with temperature*/
static int32_t raw_temp, raw_mpa;
static uint32_t BMP280TimeStamp;

BMP280State BMPState = BMP_SET_CONFIG;

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

static void BMP280_reset() {
  uint8_t data = BMP280_RESET_VALUE;
  WriteMemRegister(BMP280_REG_RESET, 1, &data, 1);
  Info("BMP280 has been reset");
  HAL_Delay(100);
}

static void BMP280_get_calibration() {
  static uint8_t bmpData[32];
  ReadMemRegister(BMP280_REG_TEMP_PRESS_CALIB_DATA, 1, &bmpData[0], 26);
  dig.T1 = (uint16_t)((((uint16_t)bmpData[1]) << 8) + (uint16_t)bmpData[0]);
  dig.T2 = (int16_t)((((int16_t)bmpData[3]) << 8) + (int16_t)bmpData[2]);
  dig.T3 = (int16_t)((((int16_t)bmpData[5]) << 8) + (int16_t)bmpData[4]);
  dig.P1 = (uint16_t)((((uint16_t)bmpData[7]) << 8) + (uint16_t)bmpData[6]);
  dig.P2 = (int16_t)((((int16_t)bmpData[9]) << 8) + (int16_t)bmpData[8]);
  dig.P3 = (int16_t)((((int16_t)bmpData[11]) << 8) + (int16_t)bmpData[10]);
  dig.P4 = (int16_t)((((int16_t)bmpData[13]) << 8) + (int16_t)bmpData[12]);
  dig.P5 = (int16_t)((((int16_t)bmpData[15]) << 8) + (int16_t)bmpData[14]);
  dig.P6 = (int16_t)((((int16_t)bmpData[17]) << 8) + (int16_t)bmpData[16]);
  dig.P7 = (int16_t)((((int16_t)bmpData[19]) << 8) + (int16_t)bmpData[18]);
  dig.P8 = (int16_t)((((int16_t)bmpData[21]) << 8) + (int16_t)bmpData[20]);
  dig.P9 = (int16_t)((((int16_t)bmpData[23]) << 8) + (int16_t)bmpData[22]);
  Debug("BMP280 calibration data:");
  Debug("bmpData[1] 0x%02X, bmpData[0] 0x%02X, dig.T1 = %d", bmpData[1], bmpData[0], dig.T1);
  Debug("bmpData[3] 0x%02X, bmpData[2] 0x%02X, dig.T1 = %d", bmpData[3], bmpData[2], dig.T2);
  Debug("bmpData[5] 0x%02X, bmpData[4] 0x%02X, dig.T1 = %d", bmpData[5], bmpData[4], dig.T3);
  Debug("bmpData[7] 0x%02X, bmpData[6] 0x%02X, dig.T1 = %d", bmpData[7], bmpData[6], dig.P1);
  Debug("bmpData[9] 0x%02X, bmpData[8] 0x%02X, dig.T1 = %d", bmpData[9], bmpData[8], dig.P2);
  Debug("bmpData[11] 0x%02X, bmpData[10] 0x%02X, dig.T1 = %d", bmpData[11], bmpData[10], dig.P3);
  Debug("bmpData[13] 0x%02X, bmpData[12] 0x%02X, dig.T1 = %d", bmpData[13], bmpData[12], dig.P4);
  Debug("bmpData[15] 0x%02X, bmpData[14] 0x%02X, dig.T1 = %d", bmpData[15], bmpData[14], dig.P5);
  Debug("bmpData[17] 0x%02X, bmpData[16] 0x%02X, dig.T1 = %d", bmpData[17], bmpData[16], dig.P6);
  Debug("bmpData[19] 0x%02X, bmpData[18] 0x%02X, dig.T1 = %d", bmpData[19], bmpData[18], dig.P7);
  Debug("bmpData[21] 0x%02X, bmpData[20] 0x%02X, dig.T1 = %d", bmpData[21], bmpData[20], dig.P8);
  Debug("bmpData[23] 0x%02X, bmpData[22] 0x%02X, dig.T1 = %d", bmpData[23], bmpData[22], dig.P9);
}

static bool BMP280_probe() {
    int8_t rslt;
    uint8_t bmpData;
    /* Read the chip-id of bmp280 sensor */
    bmp280I2Caddr = BMP280_ADDRESS;
    rslt = ReadMemRegister(BMP280_REG_CHIP_ID, 1, &bmpData, 1);
    if (rslt && (bmpData != 0)) {
      Debug("BMP280 chip_id read on secondary address 0x%02X", bmpData);
    }
    if (bmpData != BMP280_CHIP_ID) {
      bmp280I2Caddr++;
      rslt = ReadMemRegister(BMP280_REG_CHIP_ID, 1, &bmpData, 1);
      if (rslt && (bmpData != 0)) {
        Debug("BMP280 chip_id read on secondary address 0x%02X", bmpData);
      }
    }
    if (bmpData == BMP280_CHIP_ID) {
      Info("BMP280 with chip_id 0x%02X found at I2C address: 0x%02X", bmpData, bmp280I2Caddr);
      BMP280_reset(); // get an initial state
      BMP280_get_calibration();
    }
    else {
      bmp280I2Caddr = 0;
      Error("BMP280 not found");
    }
    return bmp280I2Caddr;
}

bool BMP280_DeviceConnected() {
  return BMP280_probe();
}

void BMP_Init(I2CReadMEM readMemFunction, I2CWriteMEM writeMemFunction) {
  ReadMemFunction = readMemFunction;
  WriteMemFunction = writeMemFunction;
}


static bool BMP280_set_config() {
  int8_t rslt;
  uint8_t data = (BMP280_T_SB_500 | BMP280_FILTER_4 | BMP280_SPI_OFF);
  rslt = WriteMemRegister(BMP280_REG_CONFIG, 1, &data, 1);
  BMP280TimeStamp = HAL_GetTick() + 10;
  return rslt;
}

void BMP280_set_modus(uint8_t modus) {
  mode = modus;
}

static bool BMP280_set_mode() {
  int8_t rslt;
  uint8_t bmpData = BMP280_MEAS_RDY;
  uint8_t data = (BMP280_OSRS_T_2 | BMP280_OSRS_P_4 | mode);
//  Debug("Operation mode = %s", mode==0?"sleep":mode==3?"normal":"forced");
  rslt = WriteMemRegister(BMP280_REG_CTRL_MEAS, 1, &data, 1);
  BMP280TimeStamp = HAL_GetTick() + 1000;
  while (bmpData & BMP280_MEAS_RDY) {
    ReadMemRegister(BMP280_REG_STATUS, 1, &bmpData, 1);
    if(TimestampIsReached(BMP280TimeStamp)){
      Error("BMP280 measurement was blocking, cancelled.");
      break;
    }
    HAL_Delay(1);
  }
  BMP280TimeStamp = HAL_GetTick();
  return rslt;
}


static uint8_t BMP280_get_mode() {
  int8_t rslt;
  uint8_t bmpData;
  rslt = ReadMemRegister(BMP280_REG_CTRL_MEAS, 1, &bmpData, 1);
  bmpData &= BMP280_NORMAL_MODE; //BMP280_NORMAL_MODE has all mode bits set 0x03;
  BMP280TimeStamp = HAL_GetTick() + 10;
  if (!rslt) {
    bmpData = 255;
  }
  return bmpData;
}


static bool BMP280_get_measurement_values() {
  int8_t rslt;
  uint8_t bmpData[8];
  rslt = ReadMemRegister(BMP280_REG_DATA, 1, &bmpData[0], 6);
// check for valid value
  if (bmpData[0] != 0x80) {
    raw_mpa = (int32_t)((((uint32_t)bmpData[0]) << 12) + (((uint32_t)bmpData[1]) << 4) + (((uint32_t)bmpData[2]) >> 4));
  }
  else {
    Error("BMP280 Invalid read of barometric pressure.");
    Debug("bmpData[0] 0x%02X, bmpData[1] 0x%02X, bmpData[3] 0x%02X, VALUE=0x%06X", bmpData[0], bmpData[1], bmpData[2], raw_mpa);
    SetAllBlueLED();
    return false;
  }
  if (bmpData[3] != 0x80) {
    raw_temp = (int32_t)((((uint32_t)bmpData[3]) << 12) + (((uint32_t)bmpData[4]) << 4) + (((uint32_t)bmpData[5]) >> 4));
  }
  else {
    Error("BMP280 Invalid read of temperature.");
    Debug("bmpData[3] 0x%02X, bmpData[4] 0x%02X, bmpData[5] 0x%02X, VALUE=0x%06X", bmpData[3], bmpData[4], bmpData[5], raw_temp);
    SetAllBlueLED();
    return false;
  }
//  Debug("raw_mpa: %ld, raw_temp: %ld ", raw_mpa, raw_temp);
  return rslt;
}


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// reads temperature value from internal bmp280 registers in centigrade*/
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

  /* Avoid exception caused by division with zero */
  if (var1 == 0) {
    return 0;
  }
  p = ((uint32_t)(((int32_t)(1048576) - adc_P) - (var2>>12))) * 3125;

    /* Check for overflows against UINT32_MAX/2; if pres is left-shifted by 1 */
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
  case BMP_STATE_OFF:
    Debug("Measurements are turned off for barometric device BMP280.");
    BMP280TimeStamp = HAL_GetTick() + 3120000;  // once an hour
    break;

  case BMP_STATE_INIT:
    if (getSensorLock() != FREE) {
      break;
    }
    setSensorLock(BMP280);
    BMP280_reset();
    HAL_Delay(10); // wait for deferred DMA transfers
    setSensorLock(FREE);
    BMPState = BMP_SET_CONFIG;
    break;

  case BMP_SET_CONFIG:
    if (getSensorLock() != FREE) {
      break;
    }
    setSensorLock(BMP280);
    if (BMP280_set_config()) {
      BMPState = BMP_STATE_START_MEASUREMENTS;
    }
    else {
      Error("Error while configuring BMP280");
      BMP280TimeStamp = HAL_GetTick() + 10000;
      BMPState = BMP_STATE_WAIT ;
     }
    HAL_Delay(10); // wait for deferred DMA transfers
    setSensorLock(FREE);
  break;

  case BMP_STATE_START_MEASUREMENTS:
    if (getSensorLock() != FREE) {
      uint8_t locktype = getSensorLock();
      Debug("Lock is not from BMP280, but from %s",
          locktype==FREE?"FREE":locktype==HIDS?"HIDS":locktype==SGP40?"SGP40":locktype==AHT20?"AHT20":locktype==BMP280?"BMP280":"unknown");
      BMP280TimeStamp = HAL_GetTick() + 97;
      break;
    }
    if (getSensorLock() == FREE) {
      HAL_Delay(10);
      setSensorLock(BMP280);
    }
    if (BMP280_set_mode()) {
      BMPState = BMP_READ_MEASUREMENT_ARRAY;
    }
    else {
      Error("Error while setting BMP280 to forced mode");
      BMP280TimeStamp = HAL_GetTick() + 10000;
      BMPState = BMP_STATE_WAIT ;
    }
    HAL_Delay(10);
    setSensorLock(FREE);
    break;

  case BMP_READ_MEASUREMENT_ARRAY:
    if (getSensorLock() != FREE) {
      break;
    }
    HAL_Delay(10);
    setSensorLock(BMP280);
    if (BMP280_get_measurement_values()) {
      BMPState = BMP_STATE_PROCESS_RESULTS;
    }
    else {
      Error("BMP280 Error during reading measurement results array");
      BMP280TimeStamp = HAL_GetTick() + 10000;
      BMPState = BMP_STATE_WAIT ;
    }
    HAL_Delay(10);
    setSensorLock(FREE);
  break;

  case BMP_STATE_PROCESS_RESULTS:
    float airtemp, airhpa;
    airtemp = BMP280_calc_temperature();
    airhpa = BMP280_calc_pressure();
    if ((airhpa > 850.0) && (airhpa < 1100)) {
      sethPa(airhpa);
      Info("BMP280 airtemperature: %2.2fC barometric value: %.2fhPa", airtemp, airhpa);
      setBMP280(airtemp, airhpa);
      BMP280TimeStamp = HAL_GetTick() + 60000;
    }
    else {
      Error("BMP280 value out of valid range, not stored/used");
      BMP280TimeStamp = HAL_GetTick() + 10000;
    }
    BMPState = BMP_STATE_WAIT;
    break;

  case BMP_STATE_WAIT:
    BMPState = BMP_MODE_SELECT;
    break;

  case BMP_MODE_SELECT:
    if (getSensorLock() != FREE) {
      break;
    }
    setSensorLock(BMP280);
    if (BMP280_get_mode() == BMP280_NORMAL_MODE) {
      BMPState = BMP_READ_MEASUREMENT_ARRAY;
    }
    else {
      BMPState = BMP_STATE_START_MEASUREMENTS;
    }
    HAL_Delay(10);
    setSensorLock(FREE);
    BMP280TimeStamp = HAL_GetTick() + 23;
    break;

  default:
    // Handle unexpected state
    BMPState = BMP_STATE_INIT;
    if (getSensorLock() == BMP280) {
      setSensorLock(FREE);
    }
    break;
  }
  return BMPState;
}

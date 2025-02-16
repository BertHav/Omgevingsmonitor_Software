/*
 * bmp280.c
 *
 *  Created on: Feb 9, 2025
 *      Author: itsme
 */
#include <stdbool.h>
#include "aht20.h"
#include "utils.h"
#include "wsenHIDS.h"  // for CRC8
#include "measurement.h"

static uint8_t AHT20_start[3]      = {AHT20_INIT,0x08,0x00};
static uint8_t AHT20_soft_reset[1] = {AHT20_RESET};
static uint8_t AHT20_measure[3]    = {AHT20_MEASURE,0x33,0x00};
static uint8_t AHT20_calibrated[1] = {AHT20_STATUS};
static uint32_t AHT20TimeStamp = 0;
static bool calibrated = false;
static I2CReadCb ReadFunction = NULL;
static I2CWriteCB WriteFunction = NULL;
static uint8_t airtemphumraw[7];
AHT20State AHTState = AHT_STATE_START_MEASUREMENTS; // init is done by probing

static uint8_t CalculateCRC(uint8_t* data, uint8_t length);

static bool ReadRegister(uint8_t address, uint8_t* buffer, uint8_t nrBytes) {
  if (ReadFunction != NULL) {
    return ReadFunction(address, buffer, nrBytes);
  }
  return false;
}

static bool WriteRegister(uint8_t address, uint8_t* buffer, uint8_t nrBytes) {
  if (WriteFunction != NULL) {
    return WriteFunction(address, buffer, nrBytes);
  }
  return false;
}

uint8_t CalculateCRC(uint8_t* data, uint8_t length) {
  uint8_t crc = AHT20_CRC_INIT_VALUE;

  for (uint8_t i = 0; i < length; i++) {
    // XOR byte into least significant byte of crc
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++) {
      // If the leftmost (most significant) bit is set
      if (crc & AHT20_CRC_MSB_MASK) {
        // Shift left and XOR with polynomial
        crc = (crc << 1) ^ AHT_CRC_POLYNOMIAL;
      } else {
          crc <<= 1;
      }
    }
  }
//  Debug("CRC calculated value: 0x%X", crc);
  return crc;
}

void setAHT20TimeStamp(uint32_t ticks) {
  AHT20TimeStamp = HAL_GetTick() + ticks;
}

void AHT_Init(I2CReadCb readFunction, I2CWriteCB writeFunction) {
  ReadFunction = readFunction;
  WriteFunction = writeFunction;
}


bool AHT20_init(void) {
  AHT20TimeStamp = HAL_GetTick() + 50;
  return WriteRegister(AHT20_ADDRESS, AHT20_start, 3);
}

bool AHT20_calibration_start() {
  bool response = WriteRegister(AHT20_ADDRESS, AHT20_calibrated, 1);
  if (!response) {
    Error("AHT20 Write error during calibaration");
  }
  AHT20TimeStamp = HAL_GetTick() + 50;
  return response;
}

bool AHT20_calibration_complete(void) {
  airtemphumraw[0] = 0; // clear the buffer
  if (!ReadRegister(AHT20_ADDRESS, airtemphumraw, 1)) {
    Error("AHT20 Read error during calibaration");
    return false;
  }
  HAL_Delay(10);  // wait to be sure for completing the DMA transfer :(
  if (airtemphumraw[0] ==0xff) {
    airtemphumraw[0] = 0;
  }
//  Debug("status of AHT20 [0]= 0x%02x", airtemphumraw[0]);
  AHT20TimeStamp = HAL_GetTick() + 200;
  return (airtemphumraw[0] & 0x08);
}

 bool AHT20_DeviceConnected() {
   Debug("Init & probing AHT20");
   AHT20_init();
   HAL_Delay(50);
   AHT20_calibration_start();
   HAL_Delay(50);
   return AHT20_calibration_complete();
 }

bool AHT20_StartMeasurement(void) {
  bool response = WriteRegister(AHT20_ADDRESS, AHT20_measure, 3);
//  Debug("AHT20_StartMeasurement executed");
  if (!response) {
    Error("AHT20 Write error during start measurement");
  }
  AHT20TimeStamp = HAL_GetTick() + 100;
  return response;
}

bool AHT20_GetMeasurementValues() {
  bool response = ReadRegister(AHT20_ADDRESS, airtemphumraw, 7);
  AHT20TimeStamp = HAL_GetTick() + 100;
  return response;
}


bool AHT20_Calculate(float *airtemp, float *airhum) {
  uint32_t temperature;
  uint32_t humidity;
//  Debug("AHT20 entering AHT20_Calculate");
  if (airtemphumraw[6] != CalculateCRC(airtemphumraw, 6)) {
    Debug("response of AHT20 [0]= 0x%02x [1]=0x%02x  [2]=0x%02x  [3]=0x%02x  [4]=0x%02x  [5]=0x%02x, CRC-8[6]=0x%02x",
        airtemphumraw[0], airtemphumraw[1], airtemphumraw[2], airtemphumraw[3], airtemphumraw[4], airtemphumraw[5], airtemphumraw[6]);
    Error("CRC8 = 0x%02X, calculated CRC8 = 0x%02X", airtemphumraw[6], CalculateCRC(airtemphumraw, 6));
    *airtemp = 0.0;
    *airhum = 0.0;
    return false;
  }
  temperature = ((airtemphumraw[3] & 0x0f) << 16) + (airtemphumraw[4] << 8) + (airtemphumraw[5]);
  *airtemp = (((float)temperature / 1048576) * 200) - 50;
  humidity = (airtemphumraw[1] << 12) + (airtemphumraw[2] << 4) + (airtemphumraw[3]>>4);
  *airhum = ((float)humidity / 1048576) * 100;
  Debug("AHT20 air humidity = %f, temperature = %f", *airhum, *airtemp);
  return true;
}

bool AHT20_reset(void) {
  AHT20TimeStamp = HAL_GetTick() + 50;
  calibrated = false;
  return WriteRegister(AHT20_ADDRESS, AHT20_soft_reset, 1);
}

AHT20State AHT_Upkeep(void) {
//  static AHT20State AHTState = AHT20_STATE_INIT;
  if(!TimestampIsReached(AHT20TimeStamp)){
    return AHTState;
  }
  switch(AHTState) {
  case AHT_STATE_OFF:
    Debug("Measurements are turned off for AHT20.");
    break;

  case AHT_STATE_INIT:
    if (getSensorLock() != FREE) {
      break;
    }
    if (!AHT20_init()) {
      Debug("AHT20 Error during initialization");
      AHTState = AHT20_ERROR;
    }
    if(!calibrated) {
      AHTState = AHT_START_CALIBRATION;
    }
    else {
      AHTState = AHT_STATE_START_MEASUREMENTS;
    }
    break;

  case AHT_START_CALIBRATION:
  calibrated = false;
  if (getSensorLock() != FREE) {
    break;
  }
  setSensorLock(AHT20);
  if (AHT20_calibration_start()) {
    AHTState = AHT_CALIBRATED;
  }
  break;

  case AHT_CALIBRATED:
    if (AHT20_calibration_complete()) {
      calibrated = true;
      setSensorLock(FREE);
      AHTState = AHT_STATE_START_MEASUREMENTS;
      Info("AHT20 Calibration success");
    }
    else {
      AHTState = AHT_START_CALIBRATION;
      AHT20TimeStamp = HAL_GetTick() + 20;
    }
    setSensorLock(FREE);
    break;

  case AHT_STATE_START_MEASUREMENTS:
    if (getSensorLock() != FREE) {
      break;
    }
    setSensorLock(AHT20);
    if (!AHT20_StartMeasurement()) {
      AHTState = AHT20_ERROR;
    }
    else {
      AHTState = AHT_STATE_WAIT_FOR_COMPLETION;
    }
    break;

  case AHT_STATE_WAIT_FOR_COMPLETION:
    if(AHT20_GetMeasurementValues()) {
//      Debug("AHT20_GetMeasurementValues success");
      setSensorLock(FREE);
      AHTState = AHT_STATE_PROCESS_RESULTS;
    }
    else {
      AHT20TimeStamp = HAL_GetTick() + 40;
    }
    break;

  case AHT_STATE_PROCESS_RESULTS:
    float airtemp, airhum;
    if (!AHT20_Calculate(&airtemp, &airhum)) {
      AHTState = AHT20_ERROR;
      return AHTState;
    }
//    BMP280_setAirTemPHum(airtemp, airhum);
    AHTState = AHT_STATE_WAIT;
    AHT20TimeStamp = HAL_GetTick() + 60000;  // about every 1 minute
    break;

  case AHT_STATE_WAIT:
//    ResetMeasurementIndicator();
    AHTState = AHT_STATE_START_MEASUREMENTS;
  break;

  default:
    // Handle unexpected state
    AHT20_reset();
    AHTState = AHT_STATE_INIT;
    if (getSensorLock() == AHT20) {
      setSensorLock(FREE);
    }
    break;
  }
  return AHTState;
}

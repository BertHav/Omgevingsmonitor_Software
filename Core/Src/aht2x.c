/*
 * bmp280.c
 *
 *  Created on: Feb 9, 2025
 *      Author: itsme
 */
#include "aht2x.h"
#include "utils.h"
#include "ENS160.h"
#include "measurement.h"
#include "ESP.h"
#include "RealTimeClock.h"


static uint8_t AHT20_start[3]      = {AHT20_INIT,0x08,0x00};
static uint8_t AHT20_soft_reset[1] = {AHT20_RESET};
static uint8_t AHT20_measure[3]    = {AHT20_MEASURE,0x33,0x00};
static uint8_t AHT20_status[1] = {AHT20_STATUS};
static uint8_t AHT20_INIT_1[3] =  {0x1B, 0x00, 0x00};  // the magic from AOSONG

static uint32_t AHT20TimeStamp = 0;
static bool calibrated = false;
static I2CReadCb ReadFunction = NULL;
static I2CWriteCB WriteFunction = NULL;
static I2CReadDir ReadDirFunction = NULL;
static uint8_t airtemphumraw[7];
static uint8_t AHTerrors = 0;
static uint8_t offday;

AHT20State AHTState = AHT_STATE_START_MEASUREMENTS; // init is done by probing

static uint8_t CalculateCRC(uint8_t* data, uint8_t length);

static bool ReadDirRegister(uint8_t address, uint8_t* buffer, uint8_t nrBytes) {
  if (ReadDirFunction != NULL) {
    return ReadDirFunction(address, buffer, nrBytes);
  }
  return false;
}

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

void AHT_Init(I2CReadCb readFunction, I2CWriteCB writeFunction, I2CReadDir readDirFunction) {
  ReadDirFunction = readDirFunction;
  ReadFunction = readFunction;
  WriteFunction = writeFunction;
}

void AHT20_register_reset(uint8_t addr){
  AHT20_INIT_1[0] = addr;
  WriteRegister(AHT20_ADDRESS, AHT20_INIT_1, 3);
  HAL_Delay(15);
  ReadRegister(AHT20_ADDRESS, airtemphumraw, 3);
  HAL_Delay(15);
  airtemphumraw[0] = 0xB0;
  Debug("AHT20 Magic from AOSONG, Readed values from AHTxx device 2nd=0x%02X, 3rd=0x%02X", airtemphumraw[2], airtemphumraw[3]);
  WriteRegister(AHT20_ADDRESS, airtemphumraw, 3);
  HAL_Delay(15);
}

void AHT20_Start_Init(void)
{
  AHT20_register_reset(0x1b);
  AHT20_register_reset(0x1c);
  AHT20_register_reset(0x1e);
}

uint8_t AHT20_read_status() {
  WriteRegister(AHT20_ADDRESS, AHT20_status, 1);
  HAL_Delay(10);
  airtemphumraw[0] = 0;
  ReadRegister(AHT20_ADDRESS, airtemphumraw, 1);
  HAL_Delay(10);
  return airtemphumraw[0];
}

bool AHT20_init(void) {
  uint32_t ticks = HAL_GetTick();
  if (ticks < 120) {
    HAL_Delay(120-ticks); // wait for minimum startup time
  }
  AHT20_read_status();
  Debug("AHT20 Value of statusregister: 0x%02X", airtemphumraw[0]);
  if ((airtemphumraw[0] & 0x18) != 0x18) {
    WriteRegister(AHT20_ADDRESS, AHT20_start, 3);
    HAL_Delay(10);
    AHT20_Start_Init();
  }
  return true;
}


bool AHT20_calibrate() {
  bool response = WriteRegister(AHT20_ADDRESS, AHT20_status, 1);
  if (!response) {
    Error("AHT20 Write status request error during calibaration");
  }
  AHT20TimeStamp = HAL_GetTick() + 50;
  return response;
  airtemphumraw[0] = 0; // clear the buffer
  if (!ReadRegister(AHT20_ADDRESS, airtemphumraw, 1)) {
    Error("AHT20 Read status error during calibaration");
    return false;
  }
  HAL_Delay(10);  // wait to be sure for completing the DMA transfer :(
  if (airtemphumraw[0] ==0xff) {
    airtemphumraw[0] = 0;
  }
  Debug("status of AHT20 [0]= 0x%02x", airtemphumraw[0]);
  AHT20TimeStamp = HAL_GetTick() + 200;
  return (airtemphumraw[0]);
}

 bool AHT20_DeviceConnected() {
   Debug("Init & probing AHT20");
   AHT20_init();
   HAL_Delay(10);
   AHT20_calibrate();
   return (bool)AHT20_read_status();
 }

bool AHT20_StartMeasurement(void) {
  bool response = WriteRegister(AHT20_ADDRESS, AHT20_measure, 3);
//  Debug("AHT20_StartMeasurement executed");
  if (!response) {
    Error("AHT20 Write error during start measurement");
  }
  AHT20TimeStamp = HAL_GetTick() + 250;
  return response;
}

bool AHT20_GetMeasurementValues() {
//  Debug("AHT20_GetMeasurementValues executed");
  AHT20TimeStamp = HAL_GetTick() + 1000;
  while ((AHT20_read_status() & 0x80) == 0x80) {
    Info("AHT20 Device busy, waiting for results");
    if (TimestampIsReached(AHT20TimeStamp)) {
      Error("AHT2x timeout for getting values");
      AHT20TimeStamp = HAL_GetTick() + 2000;
      return false;
    }
    HAL_Delay(10);
  }
  HAL_Delay(10);
  for (uint8_t g = 0; g < 7; g++){
    airtemphumraw[g] = 0;
  }
  bool response = ReadDirRegister(AHT20_ADDRESS, airtemphumraw, 7);
  HAL_Delay(10);
//  Debug("response of AHT20 [0]= 0x%02x [1]=0x%02x  [2]=0x%02x  [3]=0x%02x  [4]=0x%02x  [5]=0x%02x, CRC-8[6]=0x%02x",
//      airtemphumraw[0], airtemphumraw[1], airtemphumraw[2], airtemphumraw[3], airtemphumraw[4], airtemphumraw[5], airtemphumraw[6]);
  AHT20TimeStamp = HAL_GetTick() + 100;
  return response;
}


bool AHT20_Calculate(float *airtemp, float *airhum) {
  uint32_t temperature;
  uint32_t humidity;
//  Debug("AHT20 entering AHT20_Calculate");
  if ((airtemphumraw[6] != CalculateCRC(airtemphumraw, 6)) && (airtemphumraw[6] != 0xFF)) {
    Debug("AHT20 Packet when calculated after CRC [0]= 0x%02x [1]=0x%02x  [2]=0x%02x  [3]=0x%02x  [4]=0x%02x  [5]=0x%02x, CRC-8[6]=0x%02x",
        airtemphumraw[0], airtemphumraw[1], airtemphumraw[2], airtemphumraw[3], airtemphumraw[4], airtemphumraw[5], airtemphumraw[6]);
    Error("CRC8 = 0x%02X, calculated CRC8 = 0x%02X", airtemphumraw[6], CalculateCRC(airtemphumraw, 6));
    *airtemp = 0.0;
    *airhum = 0.0;
    AHT20TimeStamp = HAL_GetTick() + 2000;
    return false;
  }
  AHTerrors = 0; // reset error counter

  temperature = ((airtemphumraw[3] & 0x0f) << 16) + (airtemphumraw[4] << 8) + (airtemphumraw[5]);
  *airtemp = (((float)temperature / 1048576) * 200) - 50;
  humidity = (airtemphumraw[1] << 12) + (airtemphumraw[2] << 4) + (airtemphumraw[3]>>4);
  *airhum = ((float)humidity / 1048576) * 100;
  Info("AHT20 air humidity = %2.2f%%, temperature = %2.2fC", *airhum, *airtemp);
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
    AHT20TimeStamp = HAL_GetTick() + 900000;  // about every 15 minute
    if (weekday != offday) {  // try to enable device again
      AHTState = AHT_STATE_WAIT;
    }
    break;

  case AHT_STATE_START_MEASUREMENTS:
    if (getSensorLock() != FREE) {
      break;
    }
    setSensorLock(AHT20);
    if (!AHT20_StartMeasurement()) {
      AHT20_reset();
      AHT20TimeStamp = HAL_GetTick() + 200;
      AHTState = AHT20_ERROR;
    }
    else {
      AHTState = AHT_STATE_WAIT_FOR_COMPLETION;
    }
    HAL_Delay(10); // wait for deferred DMA transfers
    break;

  case AHT_STATE_WAIT_FOR_COMPLETION:
    if(AHT20_GetMeasurementValues()) {
//      Debug("AHT20_GetMeasurementValues success");
      HAL_Delay(10); // wait for deferred DMA transfers
      setSensorLock(FREE);
      AHTState = AHT_STATE_PROCESS_RESULTS;
    }
//    else {
//      AHT20TimeStamp = HAL_GetTick() + 40;
//    }
    break;

  case AHT_STATE_PROCESS_RESULTS:
    float airtemp, airhum;
    if (!AHT20_Calculate(&airtemp, &airhum)) {
      AHTState = AHT20_ERROR;
      return AHTState;
    }
    setAHT2x(airtemp, airhum); //store to transmit
    ENS160_set_envdata(airtemp, airhum); // use the actual values to the gas sensor
    AHTState = AHT_STATE_WAIT;
    AHT20TimeStamp = HAL_GetTick() + 60000;  // about every 1 minute
    break;

  case AHT_STATE_WAIT:
//    ResetMeasurementIndicator();
    AHTState = AHT_STATE_START_MEASUREMENTS;
  break;

  default:
    // Handle unexpected state
    AHTState = AHT_STATE_START_MEASUREMENTS;
    if (getSensorLock() == AHT20) {
      setSensorLock(FREE);
    }
    AHTerrors++;
    if (AHTerrors > 25) {
      Error("AHT2x more than 25 consecutive errors detected. Device disabled.");
      AHTState = AHT_STATE_OFF;
      offday = weekday;
    }
    break;
  }
  return AHTState;
}

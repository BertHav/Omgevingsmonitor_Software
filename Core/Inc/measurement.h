/*
 * measurement.h
 *
 *  Created on: Jun 10, 2024
 *      Author: Joris Blankestijn
 *              redesigned Bert Havinga dec 2024
 */

#ifndef INC_MEASUREMENT_H_
#define INC_MEASUREMENT_H_
/*
#include "stm32l0xx_hal.h"
#include "ESP.h"
#include "utils.h"
*/
#include <stdbool.h>
#include "I2CSensors.h"

typedef enum {
    FREE,
    HIDS,
    AHT20,
    BMP280,
    ENS160,
    SGP40
} i2cLock;

typedef struct {
    bool HT_measurementEnabled;
    bool VOC_measurementEnabled;
    bool PM_measurementEnabled;
    bool MIC_measurementEnabled;
    bool AHT_measurementEnabled;
    bool BMP_measurementEnabled;
    bool ENS_measurementEnabled;
} EnabledMeasurements;

typedef struct {
  bool HT_Present;
  bool VOC_Present;
  bool AHT20_Present;
  bool ENS160_Present;
  bool BMP280_Present;
  bool PM_Present;
  bool MIC_Present;
  bool ESP_Present;
  bool SGP_Enabled;
}DevicePresent;

extern EnabledMeasurements Sensor;
extern DevicePresent SensorProbe;
extern i2cLock SensorLock;
//extern uint8_t SensorHasLock;
/*
extern uint8_t HIDSstate;
extern uint8_t SGPstate;
 */

void UpkeepI2Csensors();
void setSensorLock(uint8_t sensor);
uint8_t getSensorLock();
bool AllDevicesReady();
void Device_Test();
void showOMstatus();
void SetESPMeasurementDone();
bool IsSGPPresent();
bool IsAHT20SensorPresent();
bool IsBMP280SensorPresent();
bool IsENS160SensorPresent();
void SetVOCSensorDIS_ENA(bool setting);
bool IsPMSensorEnabled();
bool IsMICSensorEnabled();
bool GetPMSensorPresence();
void DisablePMSensor();
void SetPMSensorStatus(bool setting);
void SetVOCSensorStatus(bool setting);
void DisableConnectedDevices();
void EnabledConnectedDevices();
void Device_Init(I2C_HandleTypeDef* sensorI2C, I2S_HandleTypeDef* micI2s, ADC_HandleTypeDef* ADC_HANDLER, UART_HandleTypeDef* espUart);

#endif /* INC_MEASUREMENT_H_ */


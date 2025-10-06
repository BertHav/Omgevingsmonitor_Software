/*
 * measurement.c
 *
 *  Created on: Jun 10, 2024
 *      Author: Joris Blankestijn
 */
#include <aht2x.h>
#include "measurement.h"
#include "wsenHIDS.h"
#include "sgp40.h"
#include "bmp280.h"
#include "ENS160.h"
#include "microphone.h"
#include "RealTimeClock.h"
#include "sen5x.h"
#include "sound_measurement.h"
#include "statusCheck.h"
#include "display.h"

EnabledMeasurements Sensor;
DevicePresent SensorProbe;
i2cLock SensorLock;
static uint8_t SensorHasLock;
static uint8_t SGPstate;
static uint8_t HIDSstate;
static uint8_t AHTstate;
static uint8_t BMPstate;
static uint8_t ENSstate;
static bool sensorsdisablereq = false;

void testInit(){
  SensorProbe.HT_Present = false;
  SensorProbe.VOC_Present = false;
  SensorProbe.AHT20_Present = false;
  SensorProbe.ENS160_Present = false;
  SensorProbe.BMP280_Present = false;
  SensorProbe.PM_Present = false;
  SensorProbe.MIC_Present = false;
  SensorProbe.ESP_Present = false;
  SensorProbe.SGP_Enabled = false;
  Sensor.HT_measurementEnabled = true;
  Sensor.VOC_measurementEnabled = true;
  Sensor.PM_measurementEnabled = true;
  Sensor.MIC_measurementEnabled = true;
  Sensor.AHT_measurementEnabled = true;
  Sensor.BMP_measurementEnabled = true;
  Sensor.ENS_measurementEnabled = true;
}

bool GetPMSensorPresence(){
  return SensorProbe.PM_Present;
}

void DisablePMSensor() {
  SensorProbe.PM_Present = false;
}

bool IsHTSensorEnabled() {
  return Sensor.HT_measurementEnabled;
}

bool IsAHT20SensorPresent() {
  return SensorProbe.AHT20_Present;
}

bool IsBMP280SensorPresent() {
  return SensorProbe.BMP280_Present;
}

bool IsENS160SensorPresent() {
  return SensorProbe.ENS160_Present;
}

bool IsSGPPresent() {
  return SensorProbe.VOC_Present;
}

void SetVOCSensorDIS_ENA(bool setting) {

  SensorProbe.SGP_Enabled = setting;
  Sensor.VOC_measurementEnabled = setting;
  Debug("on-board SGP40 %s", setting?"enabled":"disabled");
}


bool IsVOCSensorEnabled() {
  return Sensor.VOC_measurementEnabled;
}

bool IsPMSensorEnabled() {
  return Sensor.PM_measurementEnabled;
}

bool IsMICSensorEnabled() {
  return Sensor.MIC_measurementEnabled;
}

void SetHTSensorStatus(bool setting) {
  Sensor.HT_measurementEnabled =  setting;
}

void SetAHT20SensorStatus(bool setting) {
  Sensor.AHT_measurementEnabled =  setting;
}

void SetBMP280SensorStatus(bool setting) {
  Sensor.BMP_measurementEnabled =  setting;
}

void SetENS160SensorStatus(bool setting) {
  Sensor.ENS_measurementEnabled =  setting;
}

void SetVOCSensorStatus(bool setting) {
  if (SensorProbe.SGP_Enabled) {
    Sensor.VOC_measurementEnabled = setting;
  }
//  Debug("SetVOCSensorStatus VOC_measurementEnabled = %d", setting);
}

void SetPMSensorStatus(bool setting) {
  Sensor.PM_measurementEnabled = setting;
}

void SetMICSensorStatus(bool setting) {
  Sensor.MIC_measurementEnabled = setting;
}

void SetESPMeasurementDone(){
  SensorProbe.ESP_Present = true;
}

void Device_Init(I2C_HandleTypeDef* sensorI2C, I2S_HandleTypeDef* micI2s, ADC_HandleTypeDef* ADC_HANDLER, UART_HandleTypeDef* espUart) {
  testInit();
  I2CSensors_Init(sensorI2C);
  if(!HIDS_DeviceConnected()) {
     Error("Humidity / Temperature sensor NOT connected!");
     SensorProbe.HT_Present = false;
     Sensor.HT_measurementEnabled = false;
     // HT Device NOT connected, turning LED on RED.
  }else {
    // HT Device is connected, turning led on GREEN.
    SensorProbe.HT_Present = true;
    Debug("Humidity / Temperature sensor initialised.");
  }
  if(!BMP280_DeviceConnected()) {
     Error("Air pressure / Temperature sensor NOT connected!");
     SensorProbe.BMP280_Present = false;
     Sensor.BMP_measurementEnabled = false;
  }else {
    SensorProbe.BMP280_Present = true;
    Debug("Air pressure / Temperature sensor initialised.");
  }
  if(!SGP_DeviceConnected()) {
    SensorProbe.VOC_Present = false;
     Error("SGP device not connected!");
     Sensor.VOC_measurementEnabled = false;
  }
  else{
    SensorProbe.SGP_Enabled = true;
    SensorProbe.VOC_Present = true;
    Debug("SGP sensor initialised.");
  }
  if(!ENS_DeviceConnected()) {
    SensorProbe.ENS160_Present = false;
     Error("ENS device not connected!");
     Sensor.ENS_measurementEnabled = false;
  }
  else{
    SensorProbe.ENS160_Present = true;
    Debug("ENS sensor initialised.");
  }
  if(!AHT20_DeviceConnected()) {
     Error("AHT20 Humidity / Temperature sensor NOT connected!");
     SensorProbe.AHT20_Present = false;
     Sensor.AHT_measurementEnabled = false;
  }else {
    SensorProbe.AHT20_Present = true;
    Debug("AHT20 Humidity / Temperature sensor initialised.");
  }
  if(SensorProbe.VOC_Present && SensorProbe.HT_Present){
    SetDBLED(false, true, false);
  }
  else{
    SetDBLED(true, false, false);
    HAL_GPIO_WritePin(MCU_LED_C_R_GPIO_Port, MCU_LED_C_R_Pin, 0);
    HAL_GPIO_WritePin(MCU_LED_C_G_GPIO_Port, MCU_LED_C_G_Pin, 1);
    HAL_GPIO_WritePin(MCU_LED_C_B_GPIO_Port, MCU_LED_C_B_Pin, 1);
  }
  if(Sensor.MIC_measurementEnabled) {
    Info("Device_Init calls enableMicrophone");
    if (!enableMicrophone(true)) {
      Error("Microphone device not connected! DMA Error.");
      SensorProbe.MIC_Present = false;
      Sensor.MIC_measurementEnabled = false;
    }
    else{
      SensorProbe.MIC_Present = true;
      Sensor.MIC_measurementEnabled = true;
      Debug("DMA and IRQ armed for Microphone sensor.");
    }
  }
  if (!probe_sen5x()) {
    Debug("PM sensor initialised.");
    SensorProbe.PM_Present = true; // not present
    Sensor.PM_measurementEnabled = true;
    if (((product_name[4] == '4') || (product_name[4] == '5'))) {
      Info("For power saving the SGP40 is disabled, VOCi measurement is done by sen54/sen55");
      Sensor.VOC_measurementEnabled = false;
      SensorProbe.SGP_Enabled = false;
      SGP_SoftReset();
    }
  }
  else {
    sen5x_Power_Off();      // switch off buck converter
    Debug("PM sensor not detected/connected.");
    SensorProbe.PM_Present = false;
    Sensor.PM_measurementEnabled = false;
  }
  Info("SensorProbe.HT_Present: %s", SensorProbe.HT_Present?"yes":"no");
  Info("SensorProbe.VOC_Present: %s", SensorProbe.VOC_Present?"yes":"no");
  Info("SensorProbe.AHT20_Present: %s", SensorProbe.AHT20_Present?"yes":"no");
  Info("SensorProbe.BMP280_Present: %s", SensorProbe.BMP280_Present?"yes":"no");
  Info("SensorProbe.ENS160_Present: %s", SensorProbe.ENS160_Present?"yes":"no");
  Info("SensorProbe.PM_Present: %s", SensorProbe.PM_Present?"yes":"no");
  Info("SensorProbe.MIC_Present: %s", SensorProbe.MIC_Present?"yes":"no");
  ESP_Init(espUart);
  Debug("Sensors initialized, probing ESP.");
}

void Device_Test(){
  if(!SensorProbe.MIC_Present){
    if(MIC_TestMeasurementDone()){
      //when this condition is met, the device is definite operational
      SensorProbe.MIC_Present = true;
      Sensor.MIC_measurementEnabled = true;
      SetStatusLED(LED_OFF, Calculate_LED_ON(), LED_OFF);
    }
    else{
      if (micSettlingComplete()) {
        // his has to be met first
        Sensor.MIC_measurementEnabled = true;
        SetStatusLED(Calculate_LED_ON(), LED_OFF, LED_OFF);
      }
    }
  }
  if(!SensorProbe.ESP_Present){
    ESP_WakeTest();  // calls in ESP.c  back to SetESPMeasurementDone()
  }
  if((SensorProbe.ESP_Present && SensorProbe.MIC_Present) || TimestampIsReached(deviceTimeOut)){
    Info("ESP function: %s", SensorProbe.ESP_Present?"passed": "failed");
    Info("MIC function: %s", SensorProbe.MIC_Present?"passed": "failed");
    Info("Test completed");
#ifdef  SSD1306
    if (Check_USB_PowerOn() || userToggle) {
      display2ndmsg2ndline();
    }
#endif
    SetTestDone();
  }
}

bool AllDevicesReady() {
  static bool prevstatus = true;
  static bool allinwait = false;
  static uint8_t iminute = 0;
  if (TimestampIsReached(deviceTimeOut)) {
    if (!sensorsdisablereq) {
      Debug("Requesting all devices ready");
      sensorsdisablereq = true;
    }
    if (HIDSstate == HIDS_STATE_WAIT) {
      Sensor.HT_measurementEnabled = false;
    }
    if ((AHTstate == AHT_STATE_WAIT) || !SensorProbe.AHT20_Present) {
      Sensor.AHT_measurementEnabled = false;
    }
    if ((BMPstate == BMP_STATE_WAIT) || !SensorProbe.BMP280_Present) {
      Sensor.BMP_measurementEnabled = false;
    }
    if ((ENSstate == ENS_STATE_WAIT) || !SensorProbe.ENS160_Present) {
      Sensor.ENS_measurementEnabled = false;
    }
    if ((SGPstate == SGP_STATE_WAIT) || !SensorProbe.SGP_Enabled) {
      Sensor.VOC_measurementEnabled = false;
    }
    if ((PMsamplesState == LIGHT_OUT) || (PMsamplesState == CHECK_SEN5X) || (PMsamplesState == S5X_DISABLED)) {
      Sensor.PM_measurementEnabled = false;
    }
    if (MICstate == MIC_STATE_WAIT){
      Sensor.MIC_measurementEnabled = false;
    }
    if ((ESPstate == ESP_STATE_RESET) || (ESPstate == ESP_STATE_INIT)) {
      bool status = !(Sensor.HT_measurementEnabled | Sensor.VOC_measurementEnabled | Sensor.AHT_measurementEnabled | Sensor.BMP_measurementEnabled |
          Sensor.ENS_measurementEnabled | Sensor.PM_measurementEnabled | Sensor.MIC_measurementEnabled);
      if (!status && ((prevstatus != status) || (iminute != lastminute))) {
        Debug("HIDS %d, AHT %d, BMP %d, ENS %d, SGP %d,PM %d, MIC %d, Lock is from sensor column : %d (0 is FREE)",Sensor.HT_measurementEnabled, Sensor.AHT_measurementEnabled,
          Sensor.BMP_measurementEnabled, Sensor.ENS_measurementEnabled, Sensor.VOC_measurementEnabled, Sensor.PM_measurementEnabled, Sensor.MIC_measurementEnabled, getSensorLock());
        prevstatus = status;
        allinwait = false;
        iminute = lastminute;
      }
      if (status && !allinwait) {
        Debug("All sensors in wait");
        prevstatus = status;
        allinwait = true;
        if (sendpwrmaildate == getDate()) {
          Info("Battery empty mail already send today");
        }

      }
      return status;
    }
  }
  return false;
}

void EnabledConnectedDevices() {
  if (SensorProbe.HT_Present) {
    Sensor.HT_measurementEnabled = true;
  }
  if ((SensorProbe.AHT20_Present) && (AHTState != AHT_STATE_OFF)) {
    Sensor.AHT_measurementEnabled = true;
  }
//  if ((SensorProbe.AHT20_Present) && (AHTState == AHT_STATE_OFF)) {
//    Info("AHT2x sensor is disabled");
//  }
  if (SensorProbe.BMP280_Present) {
    Sensor.BMP_measurementEnabled = true;
  }
  if (SensorProbe.ENS160_Present) {
    Sensor.ENS_measurementEnabled = true;
  }
  if ((SensorProbe.VOC_Present) && (SensorProbe.SGP_Enabled)) {
    Sensor.VOC_measurementEnabled = true;
  }
  if (SensorProbe.PM_Present) {
    Sensor.PM_measurementEnabled = true;
  }
  if (SensorProbe.MIC_Present) {
    Sensor.MIC_measurementEnabled = true;
  }
  deviceTimeOut = HAL_GetTick() + 1200000;
  sensorsdisablereq = false;
  Debug("Sensors enabled");
}

void DisableConnectedDevices() {
  Sensor.HT_measurementEnabled = false;
  Sensor.VOC_measurementEnabled = false;
  Sensor.AHT_measurementEnabled = false;
  Sensor.BMP_measurementEnabled = false;
  Sensor.ENS_measurementEnabled = false;
  Sensor.PM_measurementEnabled = false;
  Sensor.MIC_measurementEnabled = false;
  Debug("Sensors disabled");
}

void setSensorLock(uint8_t sensor) {
  SensorHasLock = sensor;
  HAL_Delay(10); // be sure the DMA of the previous has completed
}

uint8_t getSensorLock() {
  return SensorHasLock;
}

void UpkeepI2Csensors() {
//  Debug("Upkeep I2C Sensors");
  if (Sensor.HT_measurementEnabled) {
    HIDSstate = HIDS_Upkeep();
  }
  if (Sensor.VOC_measurementEnabled) {
    SGPstate = SGP_Upkeep();
  }
  if (Sensor.AHT_measurementEnabled) {
    AHTstate = AHT_Upkeep();
  }
  if (Sensor.BMP_measurementEnabled) {
    BMPstate = BMP_Upkeep();
  }
  if (Sensor.ENS_measurementEnabled) {
    ENSstate = ENS_Upkeep();
  }
}

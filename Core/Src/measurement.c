/*
 * measurement.c
 *
 *  Created on: Jun 10, 2024
 *      Author: Joris Blankestijn
 */
#include "stm32l0xx_hal.h"
#include "utils.h"

#include "measurement.h"

#include "I2CSensors.h"
#include "microphone.h"
#include "gasSensor.h"
#include "humidTemp.h"
#include "statusCheck.h"
#include "sound_measurement.h"
#include "print_functions.h"

typedef struct {
    float humidityPerc;
    float temperature;
    int32_t vocIndex;
    bool HT_measurementDone;
    bool VOC_measurementDone;
    bool PM_measurementDone;
//    bool MIC_measurementDone;
} MeasurementContext;

uint32_t StartTiming = 0;
typedef void (*StartMeasurementFunc)(void);
typedef bool (*IsMeasurementDoneFunc)(void);

typedef struct {
    StartMeasurementFunc startFunc;
    IsMeasurementDoneFunc doneFunc;
    bool* doneFlag;
    bool enabled;
} MeasurementParameters;

static MeasurementContext MeasurementCtx;
static MeasurementParameters Measurements[MEAS_MEASUREMENT_COUNT];
static EnabledMeasurements MeasEnabled;
static MeasurementTested MeasTest;
static uint8_t CurrentMeasurementIndex = 0;
static uint32_t MeasStamp;
static uint32_t MicStamp;

SoundData_t soundData = {0};


static void HT_StartMeasurementWrapper(void) {
  HT_StartMeasurement();
}

static bool HT_IsMeasurementDoneWrapper(void) {
  return HT_GetMeasurementValues(&MeasurementCtx.humidityPerc, &MeasurementCtx.temperature);
}

static void VOC_StartMeasurementWrapper(void) {
  Gas_StartMeasurement();
}

static bool VOC_IsMeasurementDoneWrapper(void) {
  return Gas_GetMeasurementValues(&MeasurementCtx.vocIndex);
}

static void PM_StartMeasurementWrapper(void) {
}

static bool PM_IsMeasurementDoneWrapper(void) {
  return true;
}

static bool MIC_IsMeasurementDoneWrapper(void) {
  return MIC_MeasurementDone();
}

bool MIC_IsTestMeasurementDoneWrapper(void) {
  return MIC_TestMeasurementDone();
}
void setMeasStamp(uint32_t nrTicks) {
  MeasStamp = HAL_GetTick() + nrTicks;
}

void testInit(){
  MeasTest.ESP_Tested = false;
  MeasTest.MIC_Tested = false;
  MeasTest.HT_Tested = false;
  MeasTest.VOC_Tested = false;
}
void Meas_Init(I2C_HandleTypeDef* sensorI2C, I2S_HandleTypeDef* micI2s, ADC_HandleTypeDef* ADC_HANDLER) {
  //MeasState = MEAS_STATE_INIT;
  testInit();
  if(MeasEnabled.HT_measurementEnabled || MeasEnabled.VOC_measurementEnabled) {
    I2CSensors_Init(sensorI2C);
    if(!HT_DeviceConnected()) {
       Error("Humidity / Temperature sensor NOT connected!");
       MeasTest.HT_Tested = false;
       MeasEnabled.HT_measurementEnabled = false;
       // HT Device NOT connected, turning LED on RED.
    }else {
      // HT Device is connected, turning led on GREEN.
      MeasTest.HT_Tested = true;
      Debug("Humidity / Temperature sensor initialised.");
    }
    if(!Gas_DeviceConnected()) {
      MeasTest.VOC_Tested = false;
       Error("SGP device not connected!");
       MeasEnabled.VOC_measurementEnabled = false;
    }else{
      MeasTest.VOC_Tested = true;
      Debug("SGP sensor initialised.");
    }
    if(MeasTest.VOC_Tested && MeasTest.HT_Tested){
      SetDBLED(false, true, false);
    }
    else{
      SetDBLED(true, false, false);
      HAL_GPIO_WritePin(MCU_LED_C_R_GPIO_Port, MCU_LED_C_R_Pin, 0);
      HAL_GPIO_WritePin(MCU_LED_C_G_GPIO_Port, MCU_LED_C_G_Pin, 1);
      HAL_GPIO_WritePin(MCU_LED_C_B_GPIO_Port, MCU_LED_C_B_Pin, 1);
    }
  }
  if(MeasEnabled.MIC_measurementEnabled) {
//    MIC_Init(micI2s);
//    Meas_TestStart();
    Info("Meas_Init calls enableMicrophone");
          enableMicrophone(true);

  }
  uint8_t offset = 0;
  Measurements[offset++] = (MeasurementParameters) {HT_StartMeasurementWrapper, HT_IsMeasurementDoneWrapper, &MeasurementCtx.HT_measurementDone, MeasEnabled.HT_measurementEnabled};
  Measurements[offset++] = (MeasurementParameters) {VOC_StartMeasurementWrapper, VOC_IsMeasurementDoneWrapper, &MeasurementCtx.VOC_measurementDone, MeasEnabled.VOC_measurementEnabled};
  Measurements[offset++] = (MeasurementParameters) {PM_StartMeasurementWrapper, PM_IsMeasurementDoneWrapper, &MeasurementCtx.PM_measurementDone, MeasEnabled.PM_measurementEnabled};
//  Measurements[offset++] = (MeasurementParameters) {MIC_StartMeasurementWrapper, MIC_IsMeasurementDoneWrapper, &MeasurementCtx.MIC_measurementDone, MeasEnabled.MIC_measurementEnabled};
}

void StartMeasurements(void) {
  for(CurrentMeasurementIndex = 0; CurrentMeasurementIndex < MEAS_MEASUREMENT_COUNT; CurrentMeasurementIndex++) {
    if(Measurements[CurrentMeasurementIndex].enabled) {
      Measurements[CurrentMeasurementIndex].startFunc();
    }
  }
}

void Meas_Test(){
  if(!MeasTest.ESP_Tested){
    ESP_WakeTest();
  }

  if(!MeasTest.MIC_Tested){
    if(MIC_IsTestMeasurementDoneWrapper()){
//      Info("In Meas_Test return of MIC_IsTestMeasurementDoneWrapper = 1");
      MeasTest.MIC_Tested = true;
      SetStatusLED(LED_OFF, LED_ON, LED_OFF);
    }
    else{
      if (micSettlingComplete()) {
        SetStatusLED(LED_ON, LED_OFF, LED_OFF);
      }
    }
  }
//  print("HT_Tested: %d, VOC_Tested: %d, ESP_Tested: %d, MIC_Tested: %d\r\n", MeasTest.HT_Tested, MeasTest.VOC_Tested, MeasTest.ESP_Tested, MeasTest.MIC_Tested);
  if(MeasTest.HT_Tested && MeasTest.VOC_Tested && MeasTest.ESP_Tested && MeasTest.MIC_Tested){
    Info("Test completed");
    SetTestDone();
  }
}
void ResetMeasurements(void) {
  MeasurementCtx.humidityPerc = 0;
  MeasurementCtx.temperature = 0;
  MeasurementCtx.vocIndex = 0;
  MeasurementCtx.HT_measurementDone = false;
  MeasurementCtx.VOC_measurementDone = false;
  MeasurementCtx.PM_measurementDone = false;
//  MeasurementCtx.MIC_measurementDone = false;
}

bool MeasurementsCompleted(void) {
  for(CurrentMeasurementIndex = 0; CurrentMeasurementIndex < MEAS_MEASUREMENT_COUNT; CurrentMeasurementIndex++) {
    if(Measurements[CurrentMeasurementIndex].enabled) {
      if(Measurements[CurrentMeasurementIndex].doneFunc()) {
        *Measurements[CurrentMeasurementIndex].doneFlag = true;
      }else {
        return false;
      }
    }
  }
  return true;
}


MicrophoneState Mic_Upkeep(){
  static MicrophoneState MicState = MIC_STATE_INIT;
  switch(MicState){

  case MIC_STATE_INIT:
    //reset if necesarry
    if (!enableMicrophone(true))
      {
        errorHandler(__func__, __LINE__, __FILE__);
      }
  MicState = MIC_STATE_START_MEASUREMENT;
    break;

  case MIC_STATE_START_MEASUREMENT:
    if (micSettlingComplete() || DataReady) {
      if (!startSPLcalculation())
      {
        errorHandler(__func__, __LINE__, __FILE__);
      }
      MicState = MIC_STATE_WAIT_FOR_COMPLETION;
    }
    break;

  case MIC_STATE_WAIT_FOR_COMPLETION:
    if (getSoundData(&soundData, true, true)) {
      clearMaximumAmplitude();
//      print("SPL_dBA: %u.%u peak_amp_mPa: %u.%02u   \r\n", soundData.SPL_dBA_int,
//             soundData.SPL_dBA_fr_1dp, soundData.peak_amp_mPa_int,
//             soundData.peak_amp_mPa_fr_2dp);
      char dBbuffer[8];

      sprintf(dBbuffer, "%u.%1u", soundData.SPL_dBA_int, soundData.SPL_dBA_fr_1dp);
      sprintf(mPabuffer, "%u.%02u", soundData.peak_amp_mPa_int, soundData.peak_amp_mPa_fr_2dp);
      dBValue = atof(dBbuffer);
      dBValue = ((int)(dBValue * 100 + .5) / 100.0);

      MIC_Print();
      if (!startSPLcalculation()) {
        errorHandler(__func__, __LINE__, __FILE__);
      }
      if (!enableMicrophone(false))
        {
          errorHandler(__func__, __LINE__, __FILE__);
        }

      MicStamp = HAL_GetTick() + 755;  // about every second
      MicState = MIC_STATE_WAIT;
      ResetMICIndicator();
    }
    break;

  case MIC_STATE_WAIT:
    if(TimestampIsReached(MicStamp)){
      if (!enableMicrophone(true))
        {
          errorHandler(__func__, __LINE__, __FILE__);
        }
      MicState = MIC_STATE_START_MEASUREMENT;
      SetMICIndicator();
    }
    break;

  default:
    Debug("Unexpected occurrence happened");
    MicState = MIC_STATE_INIT;
    break;
  }

  return MicState;
}

MeasurementState Meas_Upkeep(void) {
  static MeasurementState MeasState = MEAS_STATE_INIT;
  switch(MeasState) {
  case MEAS_STATE_OFF:

    Debug("Measurements are turned off.");
    break;

  case MEAS_STATE_INIT:
    ResetMeasurements();
    MeasState = MEAS_STATE_START_MEASUREMENTS;
    break;

  case MEAS_STATE_START_MEASUREMENTS:
    StartMeasurements();
    SetMeasurementIndicator();
    MeasState = MEAS_STATE_WAIT_FOR_COMPLETION;
   break;

  case MEAS_STATE_WAIT_FOR_COMPLETION:
    if(MeasurementsCompleted()) {
      MeasState = MEAS_STATE_PROCESS_RESULTS;
    }
    break;

  case MEAS_STATE_PROCESS_RESULTS:

    // TODO: Return values and let gadget handle with too high humidity and the sensor values
    // TODO: Check if all measurements are ready for the next measurement before switching states. Only check for the enabled measurements.
//    Debug("Processing results.");
    Debug("SGP40 index value: %d", MeasurementCtx.vocIndex);
    Debug("Humidity value: %3.2f%%, Temperature value: %3.2fC", MeasurementCtx.humidityPerc, MeasurementCtx.temperature);
    setMeasurement(MeasurementCtx.temperature, MeasurementCtx.humidityPerc, MeasurementCtx.vocIndex);
    ResetMeasurementIndicator();
    if (powerCheck() == USB_PLUGGED_IN) {
      MeasStamp = HAL_GetTick() + 10000;  // about every ten seconds when power is plugged
    }
    else {
      MeasStamp = HAL_GetTick() + 60000;  // once a minute
    }
    MeasState = MEAS_STATE_WAIT;
    break;

  case MEAS_STATE_WAIT:
    if(TimestampIsReached(MeasStamp)){
      MeasState = MEAS_STATE_INIT;
    }

    break;

  default:
    // Handle unexpected state
    MeasState = MEAS_STATE_INIT;
    break;
  }

  return MeasState;
}
/*
float getTemperature(){
  float Temperature = MeasurementCtx.temperature;
  return Temperature;
}

float getHumidity(){
  float Humidity = MeasurementCtx.humidityPerc;
  return Humidity;
}
*/
void Meas_SetEnabledSensors(EnabledMeasurements enabled) {
  uint8_t offset = 0;
  MeasEnabled = enabled;
  Measurements[offset++].enabled = enabled.HT_measurementEnabled;
  Measurements[offset++].enabled = enabled.VOC_measurementEnabled;
  Measurements[offset++].enabled = enabled.PM_measurementEnabled;
  Measurements[offset++].enabled = enabled.MIC_measurementEnabled;
}


static void Meas_TurnOff(void) {
  // Disabling all sensors
  uint8_t offset = 0;
  Measurements[offset++].enabled = false;
  Measurements[offset++].enabled = false;
  Measurements[offset++].enabled = false;
  Measurements[offset++].enabled = false;
}


void SetESPMeasurementDone(){
  MeasTest.ESP_Tested = true;
}

void Meas_DeInit(I2C_HandleTypeDef* sensorI2C, I2S_HandleTypeDef* micI2s) {
  // TODO: Add de-init like the i2c, i2s and all the pins.
  Meas_TurnOff();
  HAL_I2C_DeInit(sensorI2C);
  HAL_I2S_DeInit(micI2s);

}

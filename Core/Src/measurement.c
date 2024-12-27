/*
 * measurement.c
 *
 *  Created on: Jun 10, 2024
 *      Author: Joris Blankestijn
 */
/*
#include "stm32l0xx_hal.h"
#include "utils.h"
#include "measurement.h"
#include "wsenHIDS.h"
#include "I2CSensors.h"
#include "microphone.h"
//#include "gasSensor.h"
#include "humidTemp.h"
#include "statusCheck.h"
#include "sound_measurement.h"
#include "print_functions.h"
#include "sgp40.h"

typedef struct {
    float humidityPerc;
    float temperature;
//    int32_t vocIndex;
    bool HT_measurementDone;
    bool VOC_measurementDone;
    bool PM_measurementDone;
//    bool MIC_measurementDone;
} MeasurementContext;

//uint32_t StartTiming = 0;
typedef void (*StartMeasurementFunc)(void);
typedef bool (*IsMeasurementDoneFunc)(void);
*/
/*
typedef struct {
//    StartMeasurementFunc startFunc;
//    IsMeasurementDoneFunc doneFunc;
    bool* doneFlag;
    bool enabled;
} MeasurementParameters;
*/
//static EnabledMeasurements SensorSetTest = {
//    .HT_measurementEnabled = true,
//    .VOC_measurementEnabled = true,
//    .PM_measurementEnabled = true,
//    .MIC_measurementEnabled = true
//};

//static MeasurementContext MeasurementCtx;
//static MeasurementParameters Measurements[SENSOR_MEASUREMENT_COUNT];
//static EnabledMeasurements Sensor;
//static DevicePresent SensorProbe;
//static uint8_t CurrentMeasurementIndex = 0;
//static uint32_t MeasStamp;
// static uint32_t MICTimeStamp;

// SoundData_t soundData = {0};

/*
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
*/
/*
void testInit(){
//  SensorProbe.ESP_Present = false;
//  SensorProbe.MIC_Present = false;
//  SensorProbe.HT_Present = false;
//  SensorProbe.VOC_Present = false;
  SensorProbe.ESP_Present = false;
  SensorProbe.MIC_Present = false;
  SensorProbe.HT_Present = false;
  SensorProbe.VOC_Present = false;
  SensorProbe.PM_Present = false;
}
void Device_Init(I2C_HandleTypeDef* sensorI2C, I2S_HandleTypeDef* micI2s, ADC_HandleTypeDef* ADC_HANDLER, UART_HandleTypeDef* espUart) {
  //MeasState = MEAS_STATE_INIT;
  Meas_SetEnabledSensors(SensorSetTest);
  testInit();
  if(Sensor.HT_measurementEnabled || Sensor.VOC_measurementEnabled) {
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
    if(!SGP_DeviceConnected()) {
      SensorProbe.VOC_Present = false;
       Error("SGP device not connected!");
       Sensor.VOC_measurementEnabled = false;
    }else{
      SensorProbe.VOC_Present = true;
      Debug("SGP sensor initialised.");
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
  }
  if(Sensor.MIC_measurementEnabled) {
    Info("Device_Init calls enableMicrophone");
    if (!enableMicrophone(true)) {
      SensorProbe.MIC_Present = false;
       Error("Microphone device not connected!");
       Sensor.MIC_measurementEnabled = false;
    }
    else{
      SensorProbe.MIC_Present = true;
      Sensor.MIC_measurementEnabled = true;
      Debug("Microphone sensor initialised.");
    }
  }
//  uint8_t offset = 0;
//  Measurements[offset++] = (MeasurementParameters) {HT_StartMeasurementWrapper, HT_IsMeasurementDoneWrapper, &MeasurementCtx.HT_measurementDone, Sensor.HT_measurementEnabled};
//  Measurements[offset++] = (MeasurementParameters) {VOC_StartMeasurementWrapper, VOC_IsMeasurementDoneWrapper, &MeasurementCtx.VOC_measurementDone, Sensor.VOC_measurementEnabled};
//  Measurements[offset++] = (MeasurementParameters) {PM_StartMeasurementWrapper, PM_IsMeasurementDoneWrapper, &MeasurementCtx.PM_measurementDone, Sensor.PM_measurementEnabled};
//  Measurements[offset++] = (MeasurementParameters) {MIC_StartMeasurementWrapper, MIC_IsMeasurementDoneWrapper, &MeasurementCtx.MIC_measurementDone, Sensor.MIC_measurementEnabled};
  ESP_Init(espUart);
  Debug("Devices initialised.");
}
*/
//void StartMeasurements(void) {
//  for(CurrentMeasurementIndex = 0; CurrentMeasurementIndex < SENSOR_MEASUREMENT_COUNT; CurrentMeasurementIndex++) {
//    if(Measurements[CurrentMeasurementIndex].enabled) {
//      Measurements[CurrentMeasurementIndex].startFunc();
//    }
//  }
//}

/*
void Device_Test(){
  if(!SensorProbe.ESP_Present){
    ESP_WakeTest();  // calls in ESP.c  back to SetESPMeasurementDone()
  }

  if(!SensorProbe.MIC_Present){
    if(MIC_TestMeasurementDone()){
//      Info("In Meas_Test return of MIC_IsTestMeasurementDoneWrapper = 1");
      SensorProbe.MIC_Present = true;
      SetStatusLED(LED_OFF, LED_ON, LED_OFF);
    }
    else{
      if (micSettlingComplete()) {
        SetStatusLED(LED_ON, LED_OFF, LED_OFF);
      }
    }
  }
//  print("HT_Present: %d, VOC_Present: %d, ESP_Present: %d, MIC_Present: %d\r\n", SensorProbe.HT_Present, SensorProbe.VOC_Present, SensorProbe.ESP_Present, SensorProbe.MIC_Present);
  if(SensorProbe.HT_Present && SensorProbe.VOC_Present && SensorProbe.ESP_Present && SensorProbe.MIC_Present){
    Info("Test completed");
    SetTestDone();
  }
}
*/
/*
void ResetMeasurements(void) {
  MeasurementCtx.humidityPerc = 0;
  MeasurementCtx.temperature = 0;
  MeasurementCtx.vocIndex = 0;
  MeasurementCtx.HT_measurementDone = false;
  MeasurementCtx.VOC_measurementDone = false;
  MeasurementCtx.PM_measurementDone = false;
//  MeasurementCtx.MIC_measurementDone = false;
}
*/

/*
bool MeasurementsCompleted(void) {
  for(CurrentMeasurementIndex = 0; CurrentMeasurementIndex < SENSOR_MEASUREMENT_COUNT; CurrentMeasurementIndex++) {
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
*/
/*
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

      MICTimeStamp = HAL_GetTick() + 755;  // about every second
      MicState = MIC_STATE_WAIT;
      ResetMICIndicator();
    }
    break;

  case MIC_STATE_WAIT:
    if(TimestampIsReached(MICTimeStamp)){
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
*/
/*
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
*/

/*
void Meas_SetEnabledSensors(EnabledMeasurements enabled) {
  Sensor = enabled;
  Measurements[0].enabled = enabled.HT_measurementEnabled;
  Measurements[1].enabled = enabled.VOC_measurementEnabled;
  Measurements[2].enabled = enabled.PM_measurementEnabled;
  Measurements[3].enabled = enabled.MIC_measurementEnabled;
}


static void Meas_TurnOff(void) {
  // Disabling all sensors
  Measurements[0].enabled = false;
  Measurements[1].enabled = false;
  Measurements[2].enabled = false;
  Measurements[3].enabled = false;
}


void SetESPMeasurementDone(){
  SensorProbe.ESP_Present = true;
}
*/

/*
void Meas_DeInit(I2C_HandleTypeDef* sensorI2C, I2S_HandleTypeDef* micI2s) {
  // TODO: Add de-init like the i2c, i2s and all the pins.
  Meas_TurnOff();
  HAL_I2C_DeInit(sensorI2C);
  HAL_I2S_DeInit(micI2s);

}
*/

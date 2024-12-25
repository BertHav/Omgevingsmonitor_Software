/*
 * microphone.c
 *
 *  Created on: Jun 10, 2024
 *      Author: Joris Blankestijn
 *      modified dec 2024 Bert Havinga
 */

#include "microphone.h"
//#include "main.h"
#include "GPIO.h"
#include "sound_measurement.h"
#include "print_functions.h"

// TODO: include arm math libraries differently.
//#define CORTEX_M0
//#include "arm_math.h"
//#include "arm_const_structs.h"
//#include <stdlib.h>

//#define nrOfSamples 10

static volatile uint32_t StartTime = 0;
static volatile uint32_t StartupDoneTime = 0;
static volatile bool StartUpDone = false;

char mPabuffer[12] = {0};
static float dBA = 0.0;
float dBASamples[NUMBER_OF_SAMPLES] = {0.0};
float dBAAverage = 0.0;
float dBAsum;
float dBValue = 0.0;
float dBAmax = 0.0;
uint8_t counter = 0;
uint8_t denominator = 1;
bool samplebufferfilled = false;
static uint32_t MicStamp;
//bool averageReached = false;
//float sample[NUMBER_OF_SAMPLES];

SoundData_t soundData = {0};

void ResetDBACalculator(void) {
  counter = 0;
  denominator = 1;
  samplebufferfilled = false;
}

void MIC_Print(void) {
  dBA = dBValue; // dBValue is the actual sample
//  Info("IN MIC_Print dBA: %02.1f", dBA);
  dBASamples[counter] = dBA;
//  print("dBA: %f, counter: %d, dBASamples[counter]: %f\r\n", dBA, counter, dBASamples[counter]);
  counter++;

  if (counter < NUMBER_OF_SAMPLES && !samplebufferfilled) {
    denominator = counter;
  }
  else {
    denominator = NUMBER_OF_SAMPLES;
    samplebufferfilled = true;
  }
  dBAmax = 0.0;
  dBAsum = 0.0;
  for(uint8_t i=0; i < denominator; i++){
    dBAsum += dBASamples[i];
//    print("sum of dBAsum: %f after step i:%d dBASampels[i]: %f\r\n", dBAsum, i, dBASamples[i]);
    if (dBASamples[i] > dBAmax) {
      dBAmax = dBASamples[i];
  }
  dBAAverage = dBAsum/(float)denominator;
//    Debug("Average dBA value used for upload: %.1f", dBAAverage);
  setMic(dBAAverage);
  }
  print("SPL_dBA: %.1f, SPL_peak_mPa: %s, dBA peak: %.1f, dBA average: %.1f\r\n", dBA,mPabuffer, dBAmax, dBAAverage);

  if(counter > NUMBER_OF_SAMPLES){
    counter = 0;
  }

  if(dBA >= 90){//white
    SetDBLED(true, true, true);
  }
  if(dBA >= 80 && dBA < 90){ //red
    SetDBLED(true, false, false);
  }
  if(dBA >= 70 && dBA < 80){//yellow
    SetDBLED(true, true, false);
  }
  if(dBA >= 60 && dBA < 70){//Green
    SetDBLED(false, true, false);
  }
  if(dBA >= 50 && dBA < 60){//light blue
    SetDBLED(false, true, true);
  }
  if(dBA >= 40 && dBA < 50){//blue
    SetDBLED(false, false, true);
  }
  if(dBA >= 35 && dBA < 40){//purple
    SetDBLED(true, false, true);
  }
  if(dBA < 35){//off
    SetDBLED(false, false, false);
  }
}

bool MIC_MeasurementDone(void) {
  if(DataReady) {
    MIC_Print();
//    Debug("MIC measurement is done with %i samples.", Samples);
    ResetMICIndicator();
    return true;
  }
  return false;
}

bool MIC_TestMeasurementDone(void) {
  bool Check;
  Info("DataReady in MIC_TestMeasurementDone: %d", DataReady);
  if(DataReady) {
//    Check = MIC_Check();
    Check = micEnabled;
    Info("status micEnabled: %d",micEnabled );
    ResetMICIndicator();
    return Check;
  }
  return false;
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

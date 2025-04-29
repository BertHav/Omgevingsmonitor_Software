/*
  Based on ScioSense_ENS160.h - Library for the ENS160 sensor with I2C interface from ScioSense
  2023 Mar 23	v6	Christoph Friese	Bugfix measurement routine, prepare next release
  based on application note "ENS160 Software Integration.pdf" rev 0.01
*/

#include "math.h"
#include "ENS160.h"
#include "stm32l0xx_hal.h"
#include "utils.h"
#include "measurement.h"
#include "statusCheck.h"
#include "RealTimeClock.h"

bool debugENS160 = false;
uint32_t ENS160TimeStamp;
static uint8_t enscnt = 0;
static uint8_t offday;
static I2CReadMEM ReadMemFunction = NULL;
static I2CWriteMEM WriteMemFunction = NULL;

ENS160raw raw;
ENS160prediction pred;
ENS160hwsw hwsw;
ENS160State ENSState = ENS_STATE_INIT;

static bool WriteMemRegister(uint16_t MemAddress, uint16_t MemSize, uint8_t* buffer, uint16_t nrBytes) {
  if (WriteMemFunction != NULL) {
    return WriteMemFunction(hwsw._slaveaddr, MemAddress, MemSize, buffer, nrBytes);
  }
  return false;
}

static bool ReadMemRegister(uint16_t MemAddress, uint16_t MemSize, uint8_t* buffer, uint16_t nrBytes) {
  if (ReadMemFunction != NULL) {
    return ReadMemFunction(hwsw._slaveaddr, MemAddress, MemSize, buffer, nrBytes);
  }
  return false;
}

void ResetENS160samplecounter() {
  enscnt = 0;
}
void setENS160TimeStamp(uint32_t ticks) {
  ENS160TimeStamp = HAL_GetTick() + ticks;
}

void ENS160_set_addr(uint8_t slaveaddr) {
  hwsw._slaveaddr = slaveaddr; //ENS160_I2CADDR_1;
}

void ENS160_set_debug(bool debug) {
  debugENS160 = debug;
}

void ENS_Init(I2CReadMEM readFunction, I2CWriteMEM writeFunction) {
  ReadMemFunction = readFunction;
  WriteMemFunction = writeFunction;
}	



// Sends a reset to the ENS160. Returns false on I2C problems.
bool ENS160_reset(void)
{
  uint8_t data = ENS160_OPMODE_RESET;
	uint8_t result = WriteMemRegister(ENS160_REG_OPMODE, 1, &data, 1);
	HAL_Delay(ENS160_BOOTING+20);                   // Wait to boot after reset
	return result;
}

// Reads the part ID and confirms valid sensor
bool ENS160_checkPartID(void) {
	uint8_t i2cbuf[2] = {0};
	uint16_t part_id;
	bool result = false;
	
	ReadMemRegister(ENS160_REG_PART_ID, 1, &i2cbuf[0], 2);

	part_id = i2cbuf[0] | ((uint16_t)i2cbuf[1] << 8);
	if (debugENS160) {
		Debug("ENS160 checkPartID() result: %s", (part_id == ENS160_PARTID)?"ENS160 ok":(part_id == ENS161_PARTID)?"ENS161 ok":"nok");
	}	

	if (part_id == ENS160_PARTID) {
	  hwsw._revENS16x = 0;
	  result = true;
	}
	else if (part_id == ENS161_PARTID) {
	  hwsw._revENS16x = 1;
	  result = true;
	}
	return result;
}


// Initialize idle mode and confirms 
bool ENS160_clearCommand(void) {
	static uint8_t result;
	static uint8_t i2cbuf = ENS160_COMMAND_NOP;
	result = WriteMemRegister(ENS160_REG_COMMAND, 1, &i2cbuf, 1);
	HAL_Delay(10);
  i2cbuf = ENS160_COMMAND_CLRGPR;
  result &= WriteMemRegister(ENS160_REG_COMMAND, 1, &i2cbuf, 1);
	HAL_Delay(ENS160_BOOTING);                   // Wait to boot after reset
	
  result &= ReadMemRegister(ENS160_REG_DATA_STATUS, 1, &i2cbuf, 1);

	if (debugENS160) {
		Debug("clearCommand() status of ENS16X: 0x%02X, %s", i2cbuf, (i2cbuf == 00) ? "ok" : "nok");
	}
	HAL_Delay(ENS160_BOOTING);                   // Wait to boot after reset
		
	return result;
}

// Read firmware revisions
bool ENS160_getFirmware() {
  uint8_t i2cbuf[3];

	ENS160_clearCommand();
	
	HAL_Delay(ENS160_BOOTING);                   // Wait to boot after reset
	
	i2cbuf[0] = ENS160_COMMAND_GET_APPVER;
  WriteMemRegister(ENS160_REG_COMMAND, 1, &i2cbuf[0], 1);

	HAL_Delay(10);
	ReadMemRegister(ENS160_REG_GPR_READ_4, 1, &i2cbuf[0], 3);

	hwsw._fw_ver_major = i2cbuf[0];
	hwsw._fw_ver_minor = i2cbuf[1];
	hwsw._fw_ver_build = i2cbuf[2];
	
	if (hwsw._fw_ver_major > 6) {
	  hwsw._revENS16x = 1;
	}
	else {
	  hwsw._revENS16x = 0;
	}

	if (debugENS160) {
		Debug("Firmware version:  %d.%d", hwsw._fw_ver_major, hwsw._fw_ver_minor, hwsw._fw_ver_build);
	}
	HAL_Delay(ENS160_BOOTING);                   // Wait to boot after reset
	
	return (bool)i2cbuf[0];
}

// Set operation mode of sensor
bool ENS160_setMode(uint8_t mode) {
	 uint8_t result;
	
	//LP only valid for rev>0
	if ((mode == ENS160_OPMODE_LP) && (hwsw._revENS16x == 0)) {
	  result = 1;
	}
	else {
	  result = WriteMemRegister(ENS160_REG_OPMODE, 1, &mode, 1);
	}

	HAL_Delay(ENS160_BOOTING);                   // Wait to boot after reset
	
	return result;
}

// Initialize definition of custom mode with <n> steps
bool ENS160_initCustomMode(uint16_t stepNum) {
	static uint8_t result;
	static uint8_t data = ENS160_COMMAND_SETSEQ;
	if (stepNum > 0) {
	  hwsw._stepCount = stepNum;
		
		result = setMode(ENS160_OPMODE_IDLE);
		result &= ENS160_clearCommand();

		result &= WriteMemRegister(ENS160_REG_COMMAND, 1, &data, 1);
	} else {
		result = 1;
	}
	HAL_Delay(ENS160_BOOTING);                   // Wait to boot after reset
	
	return result;
}

// Init I2C communication, resets ENS160 and checks its PART_ID. Returns false on I2C problems or wrong PART_ID.
bool ENS_DeviceConnected() {
//  HAL_Delay(ENS160_BOOTING);                   // Wait to boot after reset
  bool _available = true;            // ENS160 available
  hwsw._slaveaddr = ENS160_I2CADDR_0;
  for (uint8_t tl= 0; tl < 2; tl++) {
    _available &= ENS160_reset();
    _available &= ENS160_checkPartID();
    if (!_available) {
      Info("ENS160 trying alternate address");
      hwsw._slaveaddr = ENS160_I2CADDR_1;
      _available = true;            // next try, ENS160 available?
    }
  }

  if (_available) {
    _available &= ENS160_setMode(ENS160_OPMODE_IDLE);
    _available &= ENS160_clearCommand();
    _available &= ENS160_getFirmware();
    if (debugENS160) {
      Debug("ENS160 in idle mode");
    }
  }
  return _available;
}


// Add a step to custom measurement profile with definition of duration, enabled data acquisition and temperature for each hotplate
bool ENS160_addCustomStep(uint16_t time, bool measureHP0, bool measureHP1, bool measureHP2, bool measureHP3, uint16_t tempHP0, uint16_t tempHP1, uint16_t tempHP2, uint16_t tempHP3) {
	uint8_t seq_ack;
	uint8_t temp;

	if (debugENS160) {
		Debug("setCustomMode() write step %d", hwsw._stepCount);
	}
	HAL_Delay(ENS160_BOOTING);                   // Wait to boot after reset

	temp = (uint8_t)(((time / 24)-1) << 6); 
	if (measureHP0) temp = temp | 0x20;
	if (measureHP1) temp = temp | 0x10;
	if (measureHP2) temp = temp | 0x8;
	if (measureHP3) temp = temp | 0x4;
	WriteMemRegister(ENS160_REG_GPR_WRITE_0, 1, &temp, 1);
  HAL_Delay(10);
	temp = (uint8_t)(((time / 24)-1) >> 2); 
  WriteMemRegister(ENS160_REG_GPR_WRITE_1, 1, &temp, 1);
  HAL_Delay(10);
  temp = (uint8_t)(tempHP0/2);
  WriteMemRegister(ENS160_REG_GPR_WRITE_2, 1, &temp, 1);
  HAL_Delay(10);
  temp = (uint8_t)(tempHP1/2);
  WriteMemRegister(ENS160_REG_GPR_WRITE_3, 1, &temp, 1);
  HAL_Delay(10);
  temp = (uint8_t)(tempHP2/2);
  WriteMemRegister(ENS160_REG_GPR_WRITE_4, 1, &temp, 1);
  HAL_Delay(10);
  temp = (uint8_t)(tempHP3/2);
  WriteMemRegister(ENS160_REG_GPR_WRITE_5, 1, &temp, 1);
  HAL_Delay(10);

  temp = (uint8_t)(hwsw._stepCount - 1);
  WriteMemRegister(ENS160_REG_GPR_WRITE_6, 1, &temp, 1);
  HAL_Delay(10);

  if (hwsw._stepCount == 1) {
    temp = 128;
    WriteMemRegister(ENS160_REG_GPR_WRITE_7, 1, &temp, 1);
    HAL_Delay(10);
  }
  else {
    temp = 0;
    WriteMemRegister(ENS160_REG_GPR_WRITE_7, 1, &temp, 1);
  }
  HAL_Delay(ENS160_BOOTING);
  ReadMemRegister(ENS160_REG_GPR_READ_7, 1, &seq_ack, 1);
	HAL_Delay(ENS160_BOOTING);                   // Wait to boot after reset
	
	if ((ENS160_SEQ_ACK_COMPLETE | hwsw._stepCount) != seq_ack) {
	  hwsw._stepCount = hwsw._stepCount - 1;
		return 0;
	}
	return 1;
	
}

uint8_t ENS160_readStatus(void) {
  uint8_t status;
  ReadMemRegister(ENS160_REG_DATA_STATUS, 1, &status, 1);

  if (debugENS160) {
    Debug("ENS160 Status: %d", status);
  }
  return status;
}


// Perform prediction measurement and stores result in internal variables
bool ENS160_measure(bool waitForNew) {
  bool newData = false;
	uint8_t i2cbuf[8];
	uint8_t status;

	// Set default status for early bail out
	if (debugENS160) {
	  Debug("ENS160 Start measurement");
	}
	
	if (waitForNew) {
		do {
			HAL_Delay(1);
			status = ENS160_readStatus();
		} while (!IS_NEWDAT(status));
	}
	else {
    HAL_Delay(10);
	}
	
	newData = true;
	ReadMemRegister(ENS160_REG_DATA_AQI, 1, &i2cbuf[0], 7);
	pred._data_aqi = i2cbuf[0];
	pred._data_tvoc = i2cbuf[1] | ((uint16_t)i2cbuf[2] << 8);
	pred._data_eco2 = i2cbuf[3] | ((uint16_t)i2cbuf[4] << 8);
	if (hwsw._revENS16x > 0) pred._data_aqi500 = ((uint16_t)i2cbuf[5]) | ((uint16_t)i2cbuf[6] << 8);
  	else pred._data_aqi500 = 0;
	
	return newData;
}

// Perform raw measurement
bool ENS160_measureRaw(bool waitForNew) {
	uint8_t i2cbuf[8];
  uint8_t status;
	bool newData = false;
	// Set default status for early bail out
	if (debugENS160) {
	  Debug("ENS160 Start measurement raw");
	}
	
	if (waitForNew) {
		do {
			HAL_Delay(1);
			ReadMemRegister(ENS160_REG_DATA_STATUS, 1, &status, 1);
		} while (!IS_NEWGPR(status));
	}
	
		// Read raw resistance values
    ReadMemRegister(ENS160_REG_GPR_READ_0, 1, &i2cbuf[0], 8);

    raw._hp0_rs = CONVERT_RS_RAW2OHMS_F((uint32_t)(i2cbuf[0] | ((uint16_t)i2cbuf[1] << 8)));
    raw._hp1_rs = CONVERT_RS_RAW2OHMS_F((uint32_t)(i2cbuf[2] | ((uint16_t)i2cbuf[3] << 8)));
    raw._hp2_rs = CONVERT_RS_RAW2OHMS_F((uint32_t)(i2cbuf[4] | ((uint16_t)i2cbuf[5] << 8)));
    raw._hp3_rs = CONVERT_RS_RAW2OHMS_F((uint32_t)(i2cbuf[6] | ((uint16_t)i2cbuf[7] << 8)));
	
		// Read baselines
    ReadMemRegister(ENS160_REG_DATA_BL, 1, &i2cbuf[0], 8);
    raw._hp0_bl = CONVERT_RS_RAW2OHMS_F((uint32_t)(i2cbuf[0] | ((uint16_t)i2cbuf[1] << 8)));
    raw._hp1_bl = CONVERT_RS_RAW2OHMS_F((uint32_t)(i2cbuf[2] | ((uint16_t)i2cbuf[3] << 8)));
    raw._hp2_bl = CONVERT_RS_RAW2OHMS_F((uint32_t)(i2cbuf[4] | ((uint16_t)i2cbuf[5] << 8)));
    raw._hp3_bl = CONVERT_RS_RAW2OHMS_F((uint32_t)(i2cbuf[6] | ((uint16_t)i2cbuf[7] << 8)));

    ReadMemRegister(ENS160_REG_DATA_MISR, 1, &i2cbuf[0], 1);
    raw._misr = i2cbuf[0];
	
	return newData;
}

bool ENS160_set_envdata210(uint16_t t, uint16_t h) {
  uint8_t trh_in[4];

  trh_in[0] = t & 0xff;
  trh_in[1] = (t >> 8) & 0xff;
  trh_in[2] = h & 0xff;
  trh_in[3] = (h >> 8) & 0xff;
  uint8_t result = WriteMemRegister(ENS160_REG_TEMP_IN, 1, &trh_in[0], 4);
  return result;
}


// Writes t (degC) and h (%rh) to ENV_DATA. Returns false on I2C problems.
bool ENS160_set_envdata(float t, float h) {
	uint16_t t_data = (uint16_t)((t + 273.15f) * 64.0f);
	uint16_t rh_data = (uint16_t)(h * 512.0f);
	return ENS160_set_envdata210(t_data, rh_data);
}

ENS160State ENS_Upkeep(void) {
  uint8_t status;
  if(!TimestampIsReached(ENS160TimeStamp)){
    return ENSState;
  }
  switch(ENSState) {
  case ENS_STATE_OFF:
    Debug("Measurements are turned off for gas device ENS160.");
    ENS160TimeStamp = HAL_GetTick() + 780000;  // 4 times an hour
    if (weekday != offday) {  // try to enable device again
      ENSState = ENS_STATE_WAIT;
    }
    break;

  case ENS_STATE_INIT:
    if (getSensorLock() != FREE) {
      break;
    }
    setSensorLock(ENS160);
    bool result = ENS160_setMode(ENS160_OPMODE_STD);
    Debug("ENS160 switched to standard mode %s", result?"done.":"failed.");
    HAL_Delay(10); // wait for deferred DMA transfers
    setSensorLock(FREE);
    ENSState = ENS_STATUS_CHECK;
    ENS160TimeStamp = HAL_GetTick() + 1000;
    break;

  case ENS_STATUS_CHECK:
    if (getSensorLock() != FREE) {
      break;
    }
    setSensorLock(ENS160);
    status = ENS160_readStatus();
    HAL_Delay(10); // wait for deferred DMA transfers
    setSensorLock(FREE);
    if ((status & 0x0C) != 0) {
      switch (status >> 2) {
      case 1:
        Debug("ENS160 Warm-Up phase");
        break;
      case 2:
        Debug("ENS160 Initial Start-UP phase");
        break;
      case 3:
        Debug("ENS160 Invalid output");
        break;
      }
      if ((status & 0x03) == 0) {
        ENS160TimeStamp = HAL_GetTick() + 1000;
//        ENSState = ENS_LOW_POWER;
//        ENSState = ENS_STATE_WAIT;
        break;
      }
    }
    ENSState = ENS_STATE_START_MEASUREMENTS;
    break;

  case ENS_STATE_START_MEASUREMENTS:
    if ((getSensorLock() != FREE) && (getSensorLock() != ENS160)) {
      uint8_t locktype = getSensorLock();
      Debug("Lock is not from ENS160, but from %s",
          locktype==FREE?"FREE":locktype==HIDS?"HIDS":locktype==SGP40?"SGP40":locktype==AHT20?"AHT20":locktype==ENS160?"ENS160":"unknown");
      ENS160TimeStamp = HAL_GetTick() + 333;
      break;
    }
    setSensorLock(ENS160);
    status = ENS160_readStatus();
    if ((status & 0x02) == 0) {
      ENS160TimeStamp = HAL_GetTick() + 500;
//      Debug("ENS160 status register is: %d", status);
      HAL_Delay(10); // wait for deferred DMA transfers
      setSensorLock(FREE);
      break;
    }
    ENS160_measure(false);
    HAL_Delay(10);
    ENS160_measureRaw(false);
    HAL_Delay(10);
    setSensorLock(FREE);
    ENSState = ENS_STATE_PROCESS_RESULTS;
    break;

  case ENS_STATE_PROCESS_RESULTS:
    if (enscnt == 1){
    Info("ENS160 AQI: %d, TVOC: %dppb, eCO2: %dppm", pred._data_aqi, pred._data_tvoc, pred._data_eco2);
    Info("R HP0: %d Ohm, Baseline: %d", raw._hp0_rs, raw._hp0_bl);
    Info("R HP1: %d Ohm, Baseline: %d", raw._hp1_rs, raw._hp1_bl);
    Info("R HP2: %d Ohm, Baseline: %d", raw._hp2_rs, raw._hp2_bl);
    Info("R HP3: %d Ohm, Baseline: %d", raw._hp3_rs, raw._hp3_bl);
    }
    (enscnt == 5)?enscnt=0:enscnt++;
    setENS160(pred._data_aqi, pred._data_tvoc, pred._data_eco2);
    ENSState = ENS_LOW_POWER;
    break;

  case ENS_LOW_POWER:
    ENS160TimeStamp = HAL_GetTick() + 1000;
    if (!usbPluggedIn && !userToggle && (enscnt >= 2)) {
      if (getSensorLock() != FREE) {
        break;
      }
      setSensorLock(ENS160);
      bool result = ENS160_setMode(ENS160_OPMODE_DEP_SLEEP);
      Debug("ENS160 switched to deep sleep %s, sample counter is: %d", result?"done.":"failed.", enscnt);
      HAL_Delay(10); // wait for deferred DMA transfers
      setSensorLock(FREE);
      ENS160TimeStamp = HAL_GetTick() + 45000;
    }
    ENSState = ENS_STATE_WAIT;
    break;

  case ENS_STATE_WAIT:
      if (getSensorLock() != FREE) {
        break;
      }
      setSensorLock(ENS160);
      uint8_t data;
      ReadMemRegister(ENS160_REG_OPMODE, 1, &data, 1);
      if (data == 0) {
        bool result = ENS160_setMode(ENS160_OPMODE_STD);
        Debug("ENS160 switched to standard operating mode %s", result?"done.":"failed.");
      }
      HAL_Delay(10); // wait for deferred DMA transfers
      setSensorLock(FREE);
      ENSState = ENS_STATUS_CHECK;
    break;


  default:
    // Handle unexpected state
    ENSState = ENS_STATE_INIT;
    if (getSensorLock() != ENS160) {
      setSensorLock(FREE);
    }
    break;
  }
  return ENSState;
}

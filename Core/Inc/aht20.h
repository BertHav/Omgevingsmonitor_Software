/*
 * bmp280.h
 *
 *  Created on: Feb 9, 2025
 *      Author: itsme
 */

#ifndef INC_AHT20_H_
#define INC_AHT20_H_

#include "math.h"
#include "stm32l0xx_hal.h"

#define AHT20_ADDRESS 0x38
#define AHT20_INIT    0xBE
#define AHT20_RESET   0xBA
#define AHT20_MEASURE 0xAC
#define AHT20_STATUS  0x71
#define AHT20_CRC_INIT_VALUE 0xFF
#define AHT20_CRC_MSB_MASK 0x80
#define AHT_CRC_POLYNOMIAL 0x31
#define AHT20_ERROR 255


typedef enum {
    AHT_STATE_INIT,
    AHT_START_CALIBRATION,
    AHT_CALIBRATED,
    AHT_STATE_START_MEASUREMENTS,
    AHT_STATE_WAIT_FOR_COMPLETION,
    AHT_STATE_PROCESS_RESULTS,
    AHT_STATE_WAIT_FOR_READY,
    AHT_STATE_OFF,
    AHT_WAIT_STATE_MODE,
    AHT_STATE_WAIT
} AHT20State;

typedef bool (*I2CReadCb)(uint8_t address, uint8_t* buffer, uint8_t nrBytes);
typedef bool (*I2CWriteCB)(uint8_t address, uint8_t* buffer, uint8_t nrBytes);

void AHT_Init(I2CReadCb readFunction, I2CWriteCB writeFunction);
AHT20State AHT_Upkeep(void);
bool AHT20_DeviceConnected();

#endif /* INC_AHT20_H_ */

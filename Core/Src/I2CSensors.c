/*
 * I2CSensors.c
 *
 *  Created on: Jun 10, 2024
 *      Author: Joris Blankestijn
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "utils.h"
#include <sgp40.h>
#include "I2CSensors.h"
#include "wsenHIDS.h"
#include "bmp280.h"
#include "aht2x.h"
#include "ENS160.h"

static I2C_HandleTypeDef* SensorI2C = NULL;

static bool ReadI2CDirect(uint8_t address, uint8_t* buffer, uint8_t nrBytes);
static bool ReadI2C(uint8_t address, uint8_t* buffer, uint8_t nrBytes);
static bool WriteI2C(uint8_t address, uint8_t* buffer, uint8_t nrBytes);
static bool ReadI2CMem(uint8_t address, uint16_t MemAddress, uint16_t MemSize, uint8_t* buffer, uint16_t nrBytes);
static bool WriteI2CMem(uint8_t address, uint16_t MemAddress, uint16_t MemSize, uint8_t* buffer, uint16_t nrBytes);

void I2CSensors_Init(I2C_HandleTypeDef* sensorI2C) {
    SensorI2C = sensorI2C;
    HIDS_Init(ReadI2C, WriteI2C);
    SGP_Init(ReadI2C, WriteI2C);
    AHT_Init(ReadI2C, WriteI2C, ReadI2CDirect);
    BMP_Init(ReadI2CMem, WriteI2CMem);
//    BMP_Init(ReadI2CDirect, WriteI2CDirect);
    ENS_Init(ReadI2CMem, WriteI2CMem);
}

static bool ReadI2CDirect(uint8_t address, uint8_t* buffer, uint8_t nrBytes) {
//  Debug("ReadI2CDirect address 0x%02X", address);
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(SensorI2C, ((address << 1)+1), buffer, nrBytes, 1000);
    if (status != HAL_OK) {
        return false;
    }
    return true;
}

static bool ReadI2C(uint8_t address, uint8_t* buffer, uint8_t nrBytes) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_DMA(SensorI2C, ((address << 1)+1), buffer, nrBytes);
    if (status != HAL_OK) {
        return false;
    }
    return true;
}

static bool WriteI2C(uint8_t address, uint8_t* buffer, uint8_t nrBytes) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(SensorI2C, (address << 1), buffer, nrBytes);
    if (status != HAL_OK) {
        return false;
    }
    return true;
}

static bool ReadI2CMem(uint8_t address, uint16_t MemAddress, uint16_t MemSize, uint8_t* buffer, uint16_t nrBytes) {
//  HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(SensorI2C, (address << 1), MemAddress, MemSize, buffer, nrBytes);
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(SensorI2C, ((address << 1)+1), MemAddress, MemSize, buffer, nrBytes,1000);
   if (status != HAL_OK) {
     return false;
   }
   return true;
}

static bool WriteI2CMem(uint8_t address, uint16_t MemAddress, uint16_t MemSize, uint8_t* buffer, uint16_t nrBytes) {
   HAL_StatusTypeDef status = HAL_I2C_Mem_Write(SensorI2C, (address << 1), MemAddress, MemSize, buffer, nrBytes,1000);  // de dma verwijderd 16-8-2025
   if (status != HAL_OK) {
     return false;
   }
   return true;
}

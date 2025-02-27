/*
 * ESP.h
 *
 *  Created on: Jun 28, 2024
 *      Author: Joris Blankestijn
 */

#ifndef INC_ESP_H_
#define INC_ESP_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32l0xx_hal.h"
#include "utils.h"
#include "gpio.h"
#include "measurement.h"
#include "PC_Config.h"
#include "statusCheck.h"
#include "ssd1306_128x64_i2c.h"

#define LONGMESSAGES true  // show long messages f.i. the datagram on debug UART
#define LONGDATAGRAM  // use beurs or opensensemap, if not defined use nodeRed
//#define OPENSENSEMAP // use optimum diagram for opensensemap

#define ESP_MAX_UART_RETRIES 2
#define ESP_MAX_BUFFER_SIZE 256
#define ESP_TX_BUFFER_SIZE 512
#define ESP_START_UP_TIME 700
#define ESP_RESPONSE_TIME 10
#define ESP_RESPONSE_LONG 50
#define ESP_WIFI_INIT_TIME 500
#define ESP_DMA_TIMEOUT 100
//#define ESP_AT_COMMANDS_COUNT 4
#define ESP_WIFI_RETRY_TIME 500
#define ESP_WIFI_WAIT_RESPONSE_TIME_FACTOR 20
#define ESP_1ST_DATAGRAM_AFTER_NTP_INIT 32000; // ticks afer NTP init request before the first measurement datagram is send
#define ESP_UNTIL_NEXT_SEND 240000  // about every 5 minutes
#define ESP_UNTIL_NEXT_RETRANSMIT_SEND 50000  //when failed try it next minute
#define ESP_NTP_INIT_DELAY 0;  // getting time is part of the ESP startup procedure, executed direct after ESP init
#define ESP_UNTIL_NEXT_NTP 75398  //about every 24 hours
#define ESP_MAX_RETRANSMITIONS 3
#define ESP_SEND_TEMP "\"temp\""
#define ESP_SEND_HUMID "\"humid\""
#define ESP_SEND_SOUND "\"sound\""
#define ESP_SEND_VOC "\"voc\""
#define ESP_SEND_BAT "\"Bat\""
#define ESP_SEND_SOL "\"sol\""

#define AT_RESPONSE_OK "OK"
#define AT_RESPONSE_ERROR "ERROR"
#define AT_RESPONSE_FAIL "FAIL"
#define AT_RESPONSE_READY "ready"
#define AT_RESPONSE_START ">"
#define AT_RESPONSE_WIFI "WIFI CONNECTED"
#define AT_RESPONSE_TIME_UPDATED "+TIME_UPDATED"

#define AT_COMMANDS_SIZE 21

typedef struct
{
  float Temperature;
  float Humidity;
  float PM1p0;
  float PM1p0max;
  float PM2p5;
  float PM2p5max;
  float PM4p0;
  float PM4p0max;
  float PM10p0;
  float PM10p0max;
  float ambient_humidity;
  float ambient_temperature;
  float AHT2x_humidity;
  float AHT2x_humiditymax;
  float AHT2x_temperature;
  float AHT2x_temperaturemax;
  float BMP280_temperature;
  float BMP280_temperaturemax;
  float BMP280_airpressure;
  float BMP280_airpressuremax;
  uint16_t VOCIndex;
  uint16_t TVOCIndex;
  uint16_t airNOx;
  uint16_t airNOxmax;
  uint16_t eCO2Index;
  uint16_t eCO2Indexmax;
  uint8_t AQIndex;
  uint8_t AQIndexmax;
  float dBA;
  float dBApeak;
  float dBAaverage;
  float hPaValue;
  float hPaValuemax;
} MeasurementValues;

extern MeasurementValues MeasVal;

typedef enum {
  ESP_TEST_INIT,
  ESP_TEST_SEND,
  ESP_TEST_RECEIVE,
  ESP_TEST_VALIDATE,
  ESP_TEST_DEINIT,
  ESP_TEST_BOOT
}ESP_Test;

typedef enum {
  AT_MODE_INIT,
  AT_MODE_CONFIG,
  AT_MODE_SEND,
  AT_MODE_RECONFIG,
  AT_MODE_TEST,
  AT_MODE_GETTIME
}AT_Mode;

typedef enum {
  RECEIVE_STATUS_OK,
  RECEIVE_STATUS_ERROR,
  RECEIVE_STATUS_READY,
  RECEIVE_STATUS_INCOMPLETE,
  RECEIVE_STATUS_RETRY,
  RECEIVE_STATUS_START,
  RECEIVE_STATUS_TIMEOUT,
  RECEIVE_STATUS_UNPROGGED,
  RECEIVE_STATUS_HOME,
  RECEIVE_STATUS_SSID,
  RECEIVE_STATUS_LOOP,
  RECEIVE_STATUS_TIME
}Receive_Status;

typedef enum {
  RECEIVE_EXPECTATION_OK,
  RECEIVE_EXPECTATION_READY,
  RECEIVE_EXPECTATION_START,
  RECEIVE_EXPECTATION_WIFI,
  RECEIVE_EXPECTATION_SSID,
  RECEIVE_EXPECTATION_TIME
} AT_Expectation;

typedef enum {
  ESP_STATE_OFF,
  ESP_STATE_IDLE,
  ESP_STATE_INIT,
  ESP_STATE_WAIT_FOR_REPLY,
  ESP_STATE_SEND,
  ESP_STATE_NEXT_AT,
  ESP_STATE_PROCESS_AT,
  ESP_STATE_ERROR,
  ESP_STATE_WAIT_TO_SEND,
  ESP_STATE_RESET,
  ESP_STATE_RECEIVE_DATA,
  ESP_STATE_MODE_SELECT,
  ESP_STATE_FAULT,
  ESP_STATE_DEINIT,
  ESP_STATE_CONFIG,
  ESP_STATE_WAIT_AWAKE
} ESP_States;

typedef enum {
  AT_WAKEUP,
  AT_SET_RFPOWER,
  AT_CHECK_RFPOWER,
  AT_RESTORE,
  AT_CWINIT,
  AT_CWMODE1,
  AT_CWMODE2,
  AT_CWAUTOCONN,
  AT_CWJAP,
  AT_CWSTATE,
  AT_CWMODE3,
  AT_CWSAP,
  AT_CIPMUX,
  AT_WEBSERVER,
  AT_HTTPCPOST,
  AT_SENDDATA,
  AT_SLEEP,
  AT_CIPSNTPCFG,
  AT_CIPSNTPTIME,
  AT_CIPSNTPINTV,
  AT_END
} AT_Commands;

typedef struct BeursConfig {
  char SSID[50];
  char Password[50];
}WifiConfig;

typedef struct {
  char User[30];
  char ChipId[32];
  char BoxAddress[30];
  char TempAddress[30];
  char HumidAddress[30];
  char SoundAddress[30];
  char VOCAddress[30];
  char BatteryChargeAddress[30];
}APIConfig;

extern bool ESPTransmitDone;
extern bool EspTurnedOn;
extern bool ReconfigSet;
void uint8ArrayToString(char *destination, uint8_t data[]);
bool GetReconfigMode();
void ESP_Init(UART_HandleTypeDef* espUart);
ESP_States ESP_Upkeep(void);
void ESP_Reset(void);
void ESP_Sleep(void);
void ESP_DeInit(void);
void ESP_WakeTest();
void setVOC(uint16_t voc);
void setAHT2x(float airhum, float airtemp);
void setBMP280(float airtemp, float airhpa);
void setENS160(uint8_t aqi, uint16_t tvoc, uint16_t eco2);
void sethPa(float hPa);
void setHIDS(float temp, float humid);
void setMic(float dB, float dBmax, float dBAavg);
void setPM1p0(uint16_t PM1);
void setPM2p5(uint16_t PM2);
void setPM4p0(uint16_t PM4);
void setPM10(uint16_t PM10);
void setNOx(uint16_t nox);
void SetConfigMode();
void ESP_GetHT(float temp, float humid);
void forceNTPupdate();
void setESPTimeStamp(uint32_t delayms);

#endif /* INC_ESP_H_ */


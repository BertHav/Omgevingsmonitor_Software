/*
 * ESP.c
 *
 *  Created on: Jun 28, 2024
 *      Author: Joris Blankestijn
 *              Bert Havinga nov 2024 - july 2025
 */

#include <eeprom.h>
#include "ESP.h"
#include <string.h>
#include <stdio.h>
#include "Config.h"
#include "microphone.h"
#include "PowerUtils.h"
#include "RealTimeClock.h"
#include "sen5x.h"
#include "statusCheck.h"
#include "main.h"
#include <stdint.h>
#include "ssd1306_128x64_i2c.h"
#ifdef SSD1306
#include "display.h"
#endif

static UART_HandleTypeDef* EspUart = NULL;
extern DMA_HandleTypeDef hdma_usart4_rx;

static volatile bool RxComplete = false;

static uint8_t RxBuffer[ESP_MAX_BUFFER_SIZE] = {0};
//static const char user[] = "Test";
static bool testRound = true;
bool EspTurnedOn = false;
static bool InitIsDone = false;
static bool WifiReset = false;
bool ReconfigSet = false;
static bool ConnectionMade = false;
static bool APtested = false;
static bool setTime = true;
static bool msgdone = false;
bool ESPTransmitDone = false;
#ifndef OPENSENSEMAP
static uint32_t uid[3];
#endif
static uint32_t start;
static uint32_t stop;
static uint16_t txLength = 0;
static uint8_t oldEspState = 255;
float solarCharge = 0.0;
static char message[192];
#define HEADER1 "\"content-type: application/json\""
static AT_Commands ATCommandArray[10];
static AT_Commands AT_INIT[] = {AT_WAKEUP, AT_SET_RFPOWER, AT_CHECK_RFPOWER, AT_CWINIT, AT_CWAUTOCONN, AT_CWMODE1, AT_CWJAP, AT_CIPMUX};
static AT_Commands AT_SEND[] = {AT_WAKEUP,  AT_HTTPCPOST, AT_SENDDATA};
static AT_Commands AT_TEST[] = {AT_WAKEUP, AT_CWSTATE};
static AT_Commands AT_WIFI_CONFIG[] = {AT_WAKEUP, AT_CWINIT, AT_CWMODE3, AT_CWAUTOCONN, AT_CWJAP, AT_CIPMUX};
static AT_Commands AT_WIFI_RECONFIG[] = {AT_WAKEUP, AT_CWMODE3, AT_CWSAP, AT_CIPMUX, AT_WEBSERVER};
static AT_Commands AT_SNTP[] = {AT_WAKEUP, AT_CIPSNTPCFG, AT_CIPSNTPTIME, AT_CIPSNTPINTV};
#ifdef USE_MAIL
static AT_Commands AT_MAIL[] = {AT_WAKEUP, AT_HTTPCPOST_MAILAPI, AT_SENDMAIL};
static const char APIMail[] = "\"https://api.smtp2go.com/v3/email/send\"";
#endif
uint8_t ATState;
uint8_t ATCounter = 0;
static uint8_t errorcntr = 0;
static uint8_t timeoutcntr = 0;
static uint32_t ESPTimeStamp = 0;
uint32_t ESPNTPTimeStamp = ESP_NTP_INIT_DELAY;  // ticks before the NTP request starts
static uint32_t savedESPTimeStamp = ESP_1ST_DATAGRAM_AFTER_NTP_INIT ; // ticks afer NTP init request before the first measurement datagram is send
static uint8_t retry = 0;

WifiConfig Credentials;

typedef struct {
    char* ATCommand;
    bool* doneFlag;
} ATCommandsParameters;

static AT_Expectation ATExpectation = RECEIVE_EXPECTATION_OK;
static AT_Commands ATCommand = AT_WAKEUP;
static ESP_States EspState = ESP_STATE_INIT;
static AT_Mode Mode;
static ESP_Test TestState = ESP_TEST_INIT;

MeasurementValues MeasVal;

void showESPcontrols() {
  Debug("EspState: %d ATcmd: %d Mode: %d ATExp: %d", oldEspState, ATCommand, Mode, ATExpectation);
}

void forceNTPupdate() {
  ESPNTPTimeStamp = 0;
}

void setESPTimeStamp(uint32_t delayms) {
  ESPTimeStamp = HAL_GetTick() + delayms;
}

void setCharges(){
  solarCharge = ReadSolarVoltage() / 1000.0;
}

void getWifiCred(void){
  ReadUint8ArrayEEprom(SSIDConfigAddr, (uint8_t*)Credentials.SSID, SSIDMaxLength);
  ReadUint8ArrayEEprom(pwdConfigAddr, (uint8_t*)Credentials.Password, pwdMaxLength);
  if ((Credentials.SSID[0] == 0) || (Credentials.Password[0] == 0)) {
    Error("Wifi credentials not found, reprogram or connect to PC and type Helpme");
  }
  Info("The SSID is: %s", Credentials.SSID);
  Info("The Password is: %s", Credentials.Password);
}

bool checkEEprom(){
  static uint8_t tempConfig[IdSize];
  static uint32_t configSum = 0;
  static bool test;
  ReadUint8ArrayEEprom(TempConfigAddr, tempConfig, IdSize);
  for(uint8_t i = 0; i < IdSize; i++){
    configSum += tempConfig[i];
  }
  test = (configSum == 0);
  return test;
}

bool checkName(){
  static uint8_t nameConfig[CustomNameMaxLength];
  static uint32_t configSum = 0;
  static bool test;
  ReadUint8ArrayEEprom(CustomNameConfigAddr, nameConfig, CustomNameMaxLength);
  for(uint8_t i = 0; i < IdSize; i++){
    configSum += nameConfig[i];
  }
  test = (configSum != 0);
  return test;
}

#ifdef USE_MAIL
void setModePowerMail() {
  uint8_t MailAPIKeyConfig[MailAPIKeyMaxLength];
  ReadUint8ArrayEEprom(MailAPIKeyConfigAddr, MailAPIKeyConfig, MailAPIKeyMaxLength);
  if ( strlen((char*)MailAPIKeyConfig) == 0) {
    Error("No mail API key defined");
    return;
  }
//  Debug("Powermail is forced.");
  sendpwremail = DO_PWR_MAIL;
  Mode = AT_MODE_MAIL;
  EspState = ESP_STATE_INIT;
  savedESPTimeStamp = ESPTimeStamp;
  ESPTimeStamp = 0;
  sendpwrmaildate = getDate();
}

void pwrmailTodaySend() {
  if (sendpwrmaildate != getDate()) {
    sendpwremail = CLEAR;
  }
}
#endif

void setHIDS(float temp, float humid){
  MeasVal.Temperature = temp;
  MeasVal.Humidity = humid;
#ifdef SSD1306
  if (SSD1306detected && (Check_USB_PowerOn() || userToggle)) {
    displayTemperature();
    displayHumidity();
  }
#endif
}

void setVOC(uint16_t voc) {
  MeasVal.VOCIndex = voc;
#ifdef SSD1306
  if (SSD1306detected &&(Check_USB_PowerOn() || userToggle)) {
    displayVOC();
  }
#endif
}

void setAHT2x(float airtemp, float airhum) {
  MeasVal.AHT2x_humidity = airhum;
  MeasVal.AHT2x_temperature = airtemp;
}

void setBMP280(float airtemp, float airhpa) {
  MeasVal.BMP280_temperature = airtemp;
  MeasVal.BMP280_airpressure = airhpa;
}

void setENS160(uint8_t aqi, uint16_t tvoc, uint16_t eco2) {
  MeasVal.AQIndex = aqi;
  if (aqi > MeasVal.AQIndexmax) {
    MeasVal.AQIndexmax = aqi;
  }
  MeasVal.TVOCIndex = tvoc;
  MeasVal.eCO2Index = eco2;
  if (eco2 > MeasVal.eCO2Indexmax) {
    MeasVal.eCO2Indexmax = eco2;
  }
}

void setMic(float dB, float dBmax, float dBAavg){
  MeasVal.dBA = dB;
  MeasVal.dBApeak = dBmax;
  MeasVal.dBAaverage = dBAavg;
#ifdef SSD1306
  if (SSD1306detected &&(Check_USB_PowerOn() || userToggle)) {
    displayActdBA();
    displayPeakdBA();
  }
#endif
}

void setPM1p0(uint16_t PM1) {
  MeasVal.PM1p0 = PM1 / 10.0f;
  if (MeasVal.PM1p0 > MeasVal.PM1p0max) {
    MeasVal.PM1p0max = MeasVal.PM1p0;
  }
}

void setPM2p5(uint16_t PM2) {
  MeasVal.PM2p5 = PM2 / 10.0f;
  if (MeasVal.PM2p5 > MeasVal.PM2p5max) {
    MeasVal.PM2p5max = MeasVal.PM2p5;
  }
#ifdef SSD1306
  if (SSD1306detected && (Check_USB_PowerOn() || userToggle)) {
      displayPM2p5();
  }
#endif
}

void setPM4p0(uint16_t PM4) {
  MeasVal.PM4p0 = PM4 / 10.0f;
  if (MeasVal.PM4p0 > MeasVal.PM4p0max) {
    MeasVal.PM4p0max = MeasVal.PM4p0;
  }
}

void setPM10(uint16_t PM10) {
  MeasVal.PM10p0 = PM10 / 10.0f;
  if (MeasVal.PM10p0 > MeasVal.PM10p0max) {
  MeasVal.PM10p0max = MeasVal.PM10p0;
  }
  #ifdef SSD1306
  if (SSD1306detected && (Check_USB_PowerOn() || userToggle)) {
      displayPM10();
  }
#endif
}

void setNOx(uint16_t nox) {
  MeasVal.airNOx = nox;
  if (nox > MeasVal.airNOxmax) {
    MeasVal.airNOxmax = nox;
  }
#ifdef SSD1306
  if (SSD1306detected && (Check_USB_PowerOn() || userToggle)) {
    displayNOx();
  }
#endif
}

void SetSEN545temphum(float airtemp, float airhum) {
  MeasVal.sen55_temperature = airtemp / 200.0f;
  MeasVal.sen55_humidity = airhum / 100.0f;
}

void resetMaxMeasurementValues() {
  if (sen5x_Get_sen5x_enable_state()) {
    MeasVal.PM1p0max = 0.0f;
    MeasVal.PM2p5max = 0.0f;
    MeasVal.PM4p0max = 0.0f;
    MeasVal.PM10p0max = 0.0f;
    MeasVal.airNOxmax = 0;
  }
  MeasVal.eCO2Indexmax = 0;
  MeasVal.AQIndexmax = 0;
}

void SetConfigMode(){
  if (!ReconfigSet) {
    Debug("ReconfigSet in SetConfigMode");
  }
  ReconfigSet = true;
  usblog = false;
}

bool GetReconfigMode() {
  return ReconfigSet;
}

#ifndef OPENSENSEMAP
void ESP_GetUID(){
  uid[0] = HAL_GetUIDw0();
  uid[1] = HAL_GetUIDw1();
  uid[2] = HAL_GetUIDw2();
}
#endif

void ESP_Init(UART_HandleTypeDef* espUart) {
  EspUart = espUart;
  EspState = ESP_STATE_INIT;
#ifndef OPENSENSEMAP
  ESP_GetUID();
#endif
}

static bool ESP_Send(uint8_t* command, uint16_t length) {
  HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(EspUart, command, length);
  if (status != HAL_OK) {
    Error("Error in HAL_UART_Transmit_DMA");
    return false;
  }
  if ((length > 90) && usblog && Check_USB_PowerOn()) {
    char splitchar;
    splitchar = command[SPLIT_POS];
    command[SPLIT_POS] = '\0';
    printf_USB((char*)command);
    command[SPLIT_POS] = splitchar;
    printf_USB((char*)command+SPLIT_POS);
    printf("ESP_Send: %s", command);
  }
  else
    Debug("ESP_Send: %s", command);
  return true;
}

static bool ESP_Receive(uint8_t* reply, uint16_t length) {
  RxComplete = false;
#ifndef IGNORE_PARITY_ERRORS
  bool reset = false;
#endif
  HAL_StatusTypeDef status = HAL_UART_Receive_DMA(EspUart, reply, length);
  if (status != HAL_OK) {
    Error("Error in HAL_UART_Receive_DMA. errorcode: %d", EspUart->ErrorCode);
#ifndef SMALLBUILD
    char uartespmod[] =" error in UART to ESP module";
    if (status & HAL_UART_ERROR_PE) {
      Error("Parity%s", uartespmod);
#ifndef IGNORE_PARITY_ERRORS
      reset = true;
#endif
    }
    if (status & HAL_UART_ERROR_NE) {
      Error("Noise%s", uartespmod);
#ifndef IGNORE_PARITY_ERRORS
      reset = true;
#endif
    }
    if (status & HAL_UART_ERROR_FE) {
      Error("Frame%s", uartespmod);
    }
    if (status & HAL_UART_ERROR_ORE) {
      Error("Overrun%s", uartespmod);
    }
    if (status & HAL_UART_ERROR_DMA) {
      Error("DMA transfer%s", uartespmod);
    }
    if (status & HAL_UART_ERROR_RTO) {
      Error("Receiver Timeout%s", uartespmod);
    }
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    if (status & HAL_UART_ERROR_INVALID_CALLBACK) {
      Error("Invalid Callback%s", uartespmod);
    }
#endif
#ifndef IGNORE_PARITY_ERRORS
    if (reset) {
      //switch off the ESP and reset the system
      HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_RESET);
      HAL_Delay(10);
      HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_RESET);
      HAL_Delay(10);
      HAL_GPIO_WritePin(ESP32_BOOT_GPIO_Port, ESP32_BOOT_Pin, 0);
      // line below from: https://stackoverflow.com/questions/71287996/stm32-uart-in-dma-mode-stops-receiving-after-receiving-from-a-host-with-wrong-ba
      UART_Start_Receive_DMA(EspUart, EspUart->pRxBuffPtr, EspUart->RxXferSize);
      for (uint8_t resl = 0; resl < 10; resl++) { //Wait some time to reset
        SetAllREDLED();
        HAL_Delay(100);
      }
      HAL_NVIC_SystemReset();
    }
    RxComplete = true;
    return false;
#endif
#endif
  }
  return true;
}

// Callback for reception complete
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//  if (huart == EspUart) {
//    RxComplete = true;
//    Debug("RxComplete");
//  }
//}

// Callback for UART error
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  char espconnuart[] = "ESP connection UART ";
  char comcalb[] = " Complete";
  char cid[] = " Callback ID";
  if (huart == EspUart) {
    if (huart->ErrorCode == 4) {
      return;
    }
    Error("A callback error has occurred, errorcode: 0x%X", huart->ErrorCode);
    switch (huart->ErrorCode) {
      case HAL_UART_TX_HALFCOMPLETE_CB_ID:
        Error("%sTx Half%s%s", espconnuart, comcalb, cid);
        break;
      case HAL_UART_TX_COMPLETE_CB_ID:
        Error("%sTx%s%s", espconnuart, comcalb, cid);
        break;
      case HAL_UART_RX_HALFCOMPLETE_CB_ID:
        Error("%sRx Half%s%s", espconnuart, comcalb, cid);
        break;
      case HAL_UART_RX_COMPLETE_CB_ID:
        Error("%sRx%s%s", espconnuart, comcalb, cid);
        break;
      case HAL_UART_ERROR_CB_ID:
        Error("%sError%s", espconnuart, cid);
        break;
      case HAL_UART_ABORT_COMPLETE_CB_ID:
        Error("%sAbort%s%s", espconnuart, comcalb, cid);
        break;
      case HAL_UART_ABORT_TRANSMIT_COMPLETE_CB_ID:
        Error("%sAbort Transmit%s%s", espconnuart, comcalb, cid);
        break;
      case HAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID:
        Error("%sAbort Receive%s%s", espconnuart, comcalb, cid);
        break;
      case HAL_UART_WAKEUP_CB_ID:
        Error("%sWakeup%s", espconnuart, cid);
        break;
      case HAL_UART_MSPINIT_CB_ID:
        Error("%sMspInit%s", espconnuart, cid);
        break;
      case HAL_UART_MSPDEINIT_CB_ID:
        Error("%sMspDeInit%s", espconnuart, cid);
        break;
      default:
        Error("%sUnknown error");
    }
    HAL_UART_Abort(huart);
  }
}

bool isKeyValid(uint8_t data[], char *sensormodel, char *sensortype) {
  if ((data[0] > 66) && (data[0] != 0xFF)) {
    return true;
  }
  else {
    uint8ArrayToString(message, data);
    data[12] = '\0';
    Error("Error key for %s has no stored key for %s: %s\r\n", sensormodel, sensortype, message);
 /*
    for (int i = 0; i < 12; i++) {
      if (usblog && Check_USB_PowerOn()) {
        printf_USB("%02x", data[i]);
      }
      printf("%02x", data[i]);
    }
    if (usblog && Check_USB_PowerOn()) {
      printf_USB("\r\n");
    }
    printf("\r\n");
*/
    return false;
  }
}

void uint8ArrayToString(char *destination, uint8_t data[]) {
  for (int i = 0; i < 12; i++) {
    sprintf(&destination[i * 2], "%02x", data[i]);
  }
}

#ifdef USE_MAIL
uint16_t CreateMailMessage(bool *txstat, bool send) {
  static bool status = false;
  static bool retstat = true;
  static uint8_t nameConfig[CustomNameMaxLength];
  static uint8_t SendFromnameConfig[SendFromNameMaxLength];
  static uint8_t SendTonameConfig[SendToNameMaxLength];
  uint16_t lngth = 0;
  ReadUint8ArrayEEprom(CustomNameConfigAddr, nameConfig, CustomNameMaxLength);
  ReadUint8ArrayEEprom(SendFromNameConfigAddr, SendFromnameConfig, SendFromNameMaxLength);
  ReadUint8ArrayEEprom(SendToNameConfigAddr, SendTonameConfig, SendToNameMaxLength);
  sprintf(message, "{\r\n\"sender\": \"%s\",\r\n\"to\": [\r\n\"%s\"\r\n],\r\n\"subject\": \"Battery status\",\r\n", (char*)SendFromnameConfig, (char*)SendTonameConfig);
  lngth = strlen(message);
  if (send) {
    status = ESP_Send((uint8_t*)message, strlen(message));
    retstat &= status;
  }
  sprintf(message, "\"text_body\": \"Battery of device %s is nearly empty. Actual voltage is %.2fV\"\r\n}\r\n", (char*)nameConfig, ReadBatteryVoltage());
  lngth += strlen(message);
  if (send) {
    status = ESP_Send((uint8_t*)message, strlen(message));
    retstat &= status;
  }
  *txstat = retstat;
  return lngth;
}
#endif

uint16_t CreateMessage(bool *txstat, bool send) {
  static bool status = false;
  static bool retstat = true;
  static uint8_t nameConfig[CustomNameMaxLength];
  static char uptimeBuf[14];
#ifdef LONGDATAGRAM
  static uint8_t keybuffer[IdSize];
  static char Buffer[(IdSize*2)+1];
#endif

  if(checkName()){
    ReadUint8ArrayEEprom(CustomNameConfigAddr, nameConfig, CustomNameMaxLength);
  }
  else{
    strncpy((char*)nameConfig, "Test", 5);
  }
  setCharges();
  uint16_t index = 0;
#ifdef LONGDATAGRAM
  ReadUint8ArrayEEprom(TempConfigAddr, keybuffer, IdSize);
  uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
  sprintf(message, "[{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.Temperature);
#else
  sprintf(message, "[{\"name\":\"temp\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.2f, \"unit\":\"C\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.Temperature);
#endif
index = strlen(message);
  if (send) {
    status = ESP_Send((uint8_t*)message, strlen(message));
    retstat &= status;
  }
  ReadUint8ArrayEEprom(HumidConfigAddr, keybuffer, IdSize);
  uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
  sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.Humidity);
#else
  sprintf(message, ",{\"name\":\"humid\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"%%\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.Humidity);
#endif
  index += strlen(message);
  if (send) {
    status = ESP_Send((uint8_t*)message, strlen(message));
    retstat &= status;
  }

  ReadUint8ArrayEEprom(VocIndexConfigAddr, keybuffer, IdSize);
  uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
  sprintf(message, ",{\"sensor\": \"%s\", \"value\":%d}", Buffer, MeasVal.VOCIndex);
#else
  sprintf(message, ",{\"name\":\"voc\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%d, \"unit\":\"VOCi\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.VOCIndex);
#endif
  index += strlen(message);
  if (send) {
    status = ESP_Send((uint8_t*)message, strlen(message));
    retstat &= status;
  }

  ReadUint8ArrayEEprom(ChargerStatConfigAddr, keybuffer, IdSize);
  if (isKeyValid(keybuffer, "ChargeStat", "true/false")) {
    uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
    sprintf(message, ",{\"sensor\": \"%s\", \"value\":\"%d\"}", Buffer, batteryChargeMode);
#else
    sprintf(message, ",{\"name\":\"charging\", \"sensor\": \"%s\", \"value\":\"%d\"}", Buffer, batteryChargeMode);
#endif
    index += strlen(message);
    if (send) {
      status = ESP_Send((uint8_t*)message, strlen(message));
      retstat &= status;
    }
  }

  ReadUint8ArrayEEprom(UptimeConfigAddr, keybuffer, IdSize);
  if (isKeyValid(keybuffer, "Uptime", "dhhmm")) {
    uint8ArrayToString(Buffer, keybuffer);
    getUptime(uptimeBuf);
#ifdef OPENSENSEMAP
    sprintf(message, ",{\"sensor\": \"%s\", \"value\":\"%s\"}", Buffer, uptimeBuf);
#else
    sprintf(message, ",{\"name\":\"uptime\", \"sensor\": \"%s\", \"value\":\"%s\"}", Buffer, uptimeBuf);
#endif
    index += strlen(message);
    if (send) {
      status = ESP_Send((uint8_t*)message, strlen(message));
      retstat &= status;
    }
  }

  if (IsBMP280SensorPresent()) {
    ReadUint8ArrayEEprom(hPaConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "BMP280", "hPa") && MeasVal.BMP280_airpressure) {
      uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.BMP280_airpressure);
#else
      sprintf(message, ",{\"name\":\"BMP280 hPa\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"hPa\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.BMP280_airpressure);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }

    ReadUint8ArrayEEprom(BMPTempConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "BMP280", "Temperature")) {
      uint8ArrayToString(Buffer, keybuffer);
  #ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.BMP280_temperature);
  #else
      sprintf(message, ",{\"name\":\"BMP280 Temp\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"C\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.BMP280_temperature);
  #endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }
  }

  ReadUint8ArrayEEprom(dBAConfigAddr, keybuffer, IdSize);
  if (isKeyValid(keybuffer, "MIC", "dBA")) {
    uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
    sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.dBApeak);
#else
    sprintf(message, ",{\"name\":\"Sound\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.2f, \"unit\":\"dB(A)\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.dBApeak);
#endif
    index += strlen(message);
    if (send) {
      status = ESP_Send((uint8_t*)message, strlen(message));
      retstat &= status;
    }
  }

  ReadUint8ArrayEEprom(SolVoltConfigAddr, keybuffer, IdSize);
  if (isKeyValid(keybuffer, "Solar", "Volt")) {
    uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
    sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, solarCharge);
#else
    sprintf(message, ",{\"name\":\"Solar voltage\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.2f, \"unit\":\"V\"}", uid[2], (char*)nameConfig, Buffer, solarCharge);
#endif
    index += strlen(message);
    if (send) {
      status = ESP_Send((uint8_t*)message, strlen(message));
      retstat &= status;
    }
  }

  if ((product_name[4] == '5') && Check_USB_PowerOn()) {  // the NOx has only sense in case of continuous operation
    ReadUint8ArrayEEprom(NOxIndexConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "NOx", "NOxr")) {
      uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%d}", Buffer, MeasVal.airNOxmax);
#else
      sprintf(message, ",{\"name\":\"NOx\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%d, \"unit\":\"NOxr\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.airNOxmax);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }
  }

  ReadUint8ArrayEEprom(SEN55TempConfigAddr, keybuffer, IdSize);
  if (((product_name[4] == '4') || (product_name[4] == '5')) && isKeyValid(keybuffer, "SEN54/5", "temperature")) {
    uint8ArrayToString(Buffer, keybuffer);
    if (isKeyValid(keybuffer, "Sen5x", "temp")) {
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.1f}", Buffer, MeasVal.sen55_temperature);
#else
      sprintf(message, ",{\"name\":\"SEN54/5 temp\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"C\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.sen55_temperature);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }
  }

  ReadUint8ArrayEEprom(SEN55HumidConfigAddr, keybuffer, IdSize);
  if (((product_name[4] == '4') || (product_name[4] == '5')) && isKeyValid(keybuffer, "SEN54/5", "humidity")) {
    uint8ArrayToString(Buffer, keybuffer);
    if (isKeyValid(keybuffer, "Sen5x", "hum")) {
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.1f}", Buffer, MeasVal.sen55_humidity);
#else
      sprintf(message, ",{\"name\":\"SEN54/5 humid\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"%%\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.sen55_humidity);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }
  }

  if (sen5x_Get_sen5x_enable_state() && (batteryStatus > BATTERY_LOW)) {
    ReadUint8ArrayEEprom(PM1ConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "PM1", "particle")) {
      uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.PM1p0max);
#else
      sprintf(message, ",{\"name\":\"PM1\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"µg/m3\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.PM1p0max);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }
    ReadUint8ArrayEEprom(PM2ConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "PM2p5", "particle")) {
      uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.PM2p5max);
#else
      sprintf(message, ",{\"name\":\"PM2.5\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"µg/m3\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.PM2p5max);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }
    ReadUint8ArrayEEprom(PM4ConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "PM4", "particle")) {
      uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.PM4p0max);
#else
      sprintf(message, ",{\"name\":\"PM4\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"µg/m3\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.PM4p0max);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }

    ReadUint8ArrayEEprom(PM10ConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "PM10", "particle")) {
      uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.PM10p0max);
#else
      sprintf(message, ",{\"name\":\"PM10\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"µg/m3\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.PM10p0max);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }
  }

  if (IsAHT20SensorPresent()) {
    ReadUint8ArrayEEprom(AHTTempConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "AHT2x", "temperature")) {
      uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.AHT2x_temperature);
#else
      sprintf(message, ",{\"name\":\"AHT2x Temp\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"C\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.AHT2x_temperature);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }

    ReadUint8ArrayEEprom(AHTHumidConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "AHT2x", "humidity")) {
      uint8ArrayToString(Buffer, keybuffer);
  #ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.AHT2x_humidity);
  #else
      sprintf(message, ",{\"name\":\"AHT2x humid\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"%%\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.AHT2x_humidity);
  #endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }
  }

  if (IsENS160SensorPresent()) {
    ReadUint8ArrayEEprom(ENSAQIConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "ENS160", "air quality index")) {
      uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%d}", Buffer, MeasVal.AQIndexmax);
#else
      sprintf(message, ",{\"name\":\"ENS160 AQI\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%d, \"unit\":\"i\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.AQIndexmax);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }

    ReadUint8ArrayEEprom(ENSTVOCConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "ENS160", "TVOC")) {
      uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%d}", Buffer, MeasVal.TVOCIndex);
#else
      sprintf(message, ",{\"name\":\"ENS160 TVOC\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%d, \"unit\":\"ppb\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.TVOCIndex);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }

    ReadUint8ArrayEEprom(ENSeCO2ConfigAddr, keybuffer, IdSize);
    if (isKeyValid(keybuffer, "ENS160", "eCO2")) {
      uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
      sprintf(message, ",{\"sensor\": \"%s\", \"value\":%d}", Buffer, MeasVal.eCO2Indexmax);
#else
      sprintf(message, ",{\"name\":\"ENS160 eCO2\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%d, \"unit\":\"ppm\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.eCO2Indexmax);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }
  }

  ReadUint8ArrayEEprom(BatVoltConfigAddr, keybuffer, IdSize);
  uint8ArrayToString(Buffer, keybuffer);
#ifdef OPENSENSEMAP
  sprintf(message, ",{\"sensor\": \"%s\", \"value\":%.2f}]\r\n", Buffer, batteryCharge);
#else
  sprintf(message, ",{\"name\":\"battery\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.2f, \"unit\":\"V\"}]\r\n", uid[2], (char*)nameConfig, Buffer, batteryCharge);
#endif
  index += strlen(message);

#else
    sprintf(message, "[{\"Temperature\":%.2f},{\"Humidity\":%.1f},{\"Sound\":%.2f},{\"VOC\":%d},", MeasVal.Temperature, MeasVal.Humidity, MeasVal.dBApeak, MeasVal.VOCIndex);
    index = strlen(message);

    if (send) {
      status = ESP_Send((uint8_t*)message, strlen(message));
      retstat &= status;
    }

    sprintf(message, "{\"BatteryVoltage\":%.2f},{\"SolarVoltage\":%.2f},{\"PM1\":%.1f},{\"PM2p5\":%.1f},", batteryCharge, solarCharge, MeasVal.PM1p0max, MeasVal.PM2p5max);
    index += strlen(message);

    if (send) {
      status = ESP_Send((uint8_t*)message, strlen(message));
      retstat &= status;
    }

    getUptime(uptimeBuf);
    sprintf(message, "{\"PM4\":%.1f},{\"PM10\":%.1f},{\"NOX\":%d},{\"Uptime\":\"%s\"}]", MeasVal.PM4p0max , MeasVal.PM10p0max, MeasVal.airNOxmax, uptimeBuf);  // 22
    index += strlen(message);

#endif
  if (send) {
    status = ESP_Send((uint8_t*)message, strlen(message));
    retstat &= status;
  }
  *txstat = retstat;
  return index;
}

void StartProg(){
  HAL_Delay(100);
  HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(ESP32_BOOT_GPIO_Port, ESP32_BOOT_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(ESP32_BOOT_GPIO_Port, ESP32_BOOT_Pin, GPIO_PIN_SET);
  HAL_Delay(40);
}
 uint8_t ParseBuffer(uint8_t* buffer, uint16_t len, uint8_t expectation) {
  char tempBuf[256];
  memset(tempBuf, '\0', 256);
  char status = RECEIVE_STATUS_INCOMPLETE;
  for(uint16_t i=0; i<len; i++){
    tempBuf[i] = (char)buffer[i];
  }
  tempBuf[len] = '\0';
  if (GetVerboseLevel() == VERBOSE_ALL) {
#ifdef LONGMESSAGES
  if (usblog && Check_USB_PowerOn()) {
    printf_USB("%s\r\n", tempBuf);
  }
  printf("Receive ParseBuffer: %s\r\n", tempBuf );
#else
  Debug("Receive ParseBuffer: %s", tempBuf );
#endif
  }
  char * ParsePoint = 0;
  const char OK[] = AT_RESPONSE_OK;
  const char ERROR[] = AT_RESPONSE_ERROR;
  const char FAIL[] = AT_RESPONSE_FAIL;
  const char ready[] = AT_RESPONSE_READY;
  const char start[] = AT_RESPONSE_START;
  const char WIFI[] = AT_RESPONSE_WIFI;
  const char TIME[] = AT_RESPONSE_TIME_UPDATED;
  if(expectation == RECEIVE_EXPECTATION_OK){
    ParsePoint = strstr(tempBuf, OK);
  }
  if(expectation == RECEIVE_EXPECTATION_READY){
    ParsePoint = strstr(tempBuf, ready);
  }
  if(expectation == RECEIVE_EXPECTATION_START){
    ParsePoint = strstr(tempBuf, start);
  }
  if(expectation == RECEIVE_EXPECTATION_TIME){
    ParsePoint = strstr(tempBuf, TIME);
  }
  char *ParsePoint2 = strstr(tempBuf, ERROR);
  char *ParsePoint3 = strstr(tempBuf, WIFI);
  char *ParsePoint4 = strstr(tempBuf, Credentials.SSID);
  char *ParsePoint5 = strstr(tempBuf, FAIL);
  if(len > 1 ){
    if(ParsePoint != 0 && *ParsePoint == 'O'){
// call function to update time in realtimeclock.c
      status = RECEIVE_STATUS_OK;
      if ( ATCommand == AT_CIPSNTPTIME ) {
        if ((len == 43) && (tempBuf[33] == '2' )) {  // validity check
          ParseTime(tempBuf);
        }
        else {
          Error("Error getting time");
          return RECEIVE_STATUS_TIMEOUT;
        }
      }
    }
    if(ParsePoint != 0 && *ParsePoint == 'r') {
      status = RECEIVE_STATUS_READY;
    }
    if(ParsePoint != 0 && *ParsePoint == '>') {
      status = RECEIVE_STATUS_START;
    }
    if(ParsePoint != 0 && *ParsePoint == '+') {
      status = RECEIVE_STATUS_TIME;
    }
    if((ParsePoint2 != 0 && *ParsePoint2 == 'E') || (ParsePoint5 != 0 && *ParsePoint5 == 'F')) {
      status = RECEIVE_STATUS_ERROR;
    }
    if(ParsePoint3 != 0 && *ParsePoint3 == 'W'){
      ConnectionMade = true;
    }
    if(ParsePoint4 != 0 && *ParsePoint4 == '2'){
    }
  }
  return(status);

}

 //PollAwake, RFPOWER and CheckRFPower necesarry when comming out of sleep mode.
bool PollAwake(){
  char* atCommand = "ATE0\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool RFPower(){
  char* atCommand = "AT+RFPOWER=70\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool CheckRFPower(){
  char* atCommand = "AT+RFPOWER?\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

//Only necesarry on first init
bool ATRestore(){
  char* atCommand = "AT+RESTORE\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool CWINIT(){
  char* atCommand = "AT+CWINIT=1\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool CWMODE1(){
  char* atCommand = "AT+CWMODE=1\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool CWMODE2(){
  char* atCommand = "AT+CWMODE=2\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool CWAUTOCONN(){
  char* atCommand = "AT+CWAUTOCONN=1\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool CWJAP()
{
  APtested = true;
  getWifiCred();
  static char atCommandBuff[112];
  memset(atCommandBuff, '\0', 112);
  sprintf(atCommandBuff, "AT+CWJAP=\"%s\",\"%s\"\r\n", Credentials.SSID, Credentials.Password);
  uint8_t len = strlen(atCommandBuff);
  char atCommand[len + 1];
  memset(atCommand, '\0', len + 1);
  strncpy(atCommand, atCommandBuff, len);
  return ESP_Send((uint8_t*)atCommand, len);
}


bool CWMODE3(){
  char* atCommand = "AT+CWMODE=3\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool CWSTATE(){
  char* atCommand = "AT+CWSTATE?\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool CWSAP(){
  char* atCommand = "AT+CWSAP=\"WOTS_Config\",\"\",11,0,1\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool CIPMUX(){
  char* atCommand = "AT+CIPMUX=0\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

//This command sets the webserver, only necessary for first initialization.
bool WEBSERVER(){
  char* atCommand = "AT+WEBSERVER=1,80,60\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

//These are the commands necessary for sending data.
bool HTTPCPOST(){
  bool txresult = false;
  uint16_t length = CreateMessage(&txresult, false);
  static uint8_t boxConfig[IdSize];
  static char Buffer[1+(2*IdSize)];
  static uint8_t URLToUpload[URLToUploadMaxLength];
  ReadUint8ArrayEEprom(BoxConfigAddr, boxConfig, IdSize);
  uint8ArrayToString(Buffer, boxConfig);
//  sprintf(message, "AT+HTTPCPOST=%s/%s/data\",%d,1,%s\r\n", API, Buffer, length, header1);
  ReadUint8ArrayEEprom(URLToUploadConfigAddr, URLToUpload, URLToUploadMaxLength);
  if (strlen((char*)URLToUpload) == 0) {
    strcpy ((char*)URLToUpload,API);
  }
  sprintf(message, "AT+HTTPCPOST=\"%s/%s/data\",%d,1,%s\r\n", (char*)URLToUpload, Buffer, length, HEADER1);
  uint16_t len = strlen(message);
//  Debug("ESP_send result of header: %d, transmitted data %d chars", txresult, len);
  if(ESP_Send((uint8_t*)message, len)){
    return true;
  }
  else{
    return false;
  }
}

#ifdef USE_MAIL
bool SENDMAIL() {
  bool result = false;
  txLength = CreateMailMessage(&result, true);
//  Debug("SENDMAIL ESP_Send result = %d, transmitted data %d chars", result, txLength);
  return result;
}

bool HTTPCPOST_MAILAPI() {
  bool txresult = false;
  uint16_t maillength = CreateMailMessage(&txresult, false);
  uint8_t MailAPIKeyConfig[MailAPIKeyMaxLength];
  ReadUint8ArrayEEprom(MailAPIKeyConfigAddr, MailAPIKeyConfig, MailAPIKeyMaxLength);
  sprintf(message, "AT+HTTPCPOST=%s,%d,3,%s,\"accept: application/json\",\"X-Smtp2go-Api-Key: %s\"\r\n", APIMail, maillength, HEADER1, (char*)MailAPIKeyConfig);
  uint16_t len = strlen(message);
  if(ESP_Send((uint8_t*)message, len)){
    return true;
  }
  return false;
}
#endif

bool SENDDATA(){
  bool result = false;
  txLength = CreateMessage(&result, true);
//  Debug("SENDDATA ESP_Send result = %d, transmitted data %d chars", result, txLength);
  return result;
}

bool SLEEP(){
  char* atCommand = "AT+GSLP=30000\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool CIPSNTPCFG(){
  char* atCommand = "AT+CIPSNTPCFG=1,100,\"nl.pool.ntp.org\",\"time.google.com\",\"time.windows.com\"\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    HAL_Delay(1000);
    return true;
  }
  else{
    return false;
  }
}

bool CIPSNTPTIME(){
  char* atCommand = "AT+CIPSNTPTIME?\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

bool CIPSNTPINTV(){
  char* atCommand = "AT+CIPSNTPINTV=14400\r\n";
  if(ESP_Send((uint8_t*)atCommand, strlen(atCommand))) {
    return true;
  }
  else{
    return false;
  }
}

Receive_Status DMA_ProcessBuffer(uint8_t expectation) {
    uint16_t pos = ESP_MAX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart4_rx);
    static volatile uint8_t OldPos = 0;
    static volatile uint8_t TempPos = 0;
    Receive_Status status = RECEIVE_STATUS_INCOMPLETE;
    if(pos > ESP_MAX_BUFFER_SIZE) {
      pos = ESP_MAX_BUFFER_SIZE;
    }
    if(pos == OldPos){
      if(retry > ESP_WIFI_WAIT_RESPONSE_TIME_FACTOR){
        retry = 0;
        //EspState = ESP_STATE_SEND;
        if(ATCommand == AT_WAKEUP && testRound == true){
          status = RECEIVE_STATUS_UNPROGGED;
        }
        if(ATCommand == AT_CWJAP){
          EspState = ESP_STATE_MODE_SELECT;
        }
        else{
          status = RECEIVE_STATUS_TIMEOUT;
        }
      }
     else{
       retry ++;
       ESPTimeStamp = HAL_GetTick() + ESP_WIFI_RETRY_TIME;
       status = RECEIVE_STATUS_RETRY;
      }
    }
    if (pos != OldPos) {
      retry = 0;
      if(TempPos == OldPos){
        TempPos = pos;
        status = RECEIVE_STATUS_LOOP;
      }
      else{
        if(TempPos != pos){
          TempPos = pos;
          status = RECEIVE_STATUS_LOOP;
        }
        else{
          if (pos > OldPos) {
              status = ParseBuffer(&RxBuffer[OldPos], (pos - OldPos), expectation);
          }
          else {
              // Buffer wrap-around
              status = ParseBuffer(&RxBuffer[OldPos], ESP_MAX_BUFFER_SIZE - OldPos, expectation);
              if (pos > 0) {
                  status = ParseBuffer(&RxBuffer[0], pos, expectation);
              }
          }
          OldPos = pos;
        }
      }
    }
    return status;
}

void clearDMABuffer(){
  memset(RxBuffer, '\0', ESP_MAX_BUFFER_SIZE);
}

//Compares the received status to the expected status (OK, ready, >).
bool ATCompare(uint8_t AT_Command_Received, uint8_t AT_Command_Expected){
  bool value = false;
  if(AT_Command_Expected == RECEIVE_EXPECTATION_OK){
    value = (AT_Command_Received == RECEIVE_STATUS_OK);
  }
  if(AT_Command_Expected == RECEIVE_EXPECTATION_READY){
    value = (AT_Command_Received == RECEIVE_STATUS_READY);
  }
  if(AT_Command_Expected == RECEIVE_EXPECTATION_START){
    value = (AT_Command_Received == RECEIVE_STATUS_START);
  }
  if(AT_Command_Expected == RECEIVE_EXPECTATION_TIME){
    value = (AT_Command_Received == RECEIVE_STATUS_TIME);
  }
  return(value);
}

bool AT_Send(AT_Commands state){
  bool ATCommandSend = false;
  switch (state){

  case AT_WAKEUP:
  if(TimestampIsReached(ESPTimeStamp)){
    Debug("AT_WAKEUP");
    ATCommandSend = PollAwake();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_LONG;
  }
  break;

  case AT_SET_RFPOWER:
//    Debug("Setting RF Power");
    ATCommandSend = RFPower();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CHECK_RFPOWER:
//    Debug("Checking RF Power");
    ATCommandSend = CheckRFPower();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_RESTORE:
//    Debug("Restoring ESP");
    ATCommandSend = ATRestore();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_LONG;
    break;

  case AT_CWINIT:
//    Debug("Initializing Wi-Fi");
    ATCommandSend = CWINIT();
    ESPTimeStamp = HAL_GetTick() + ESP_WIFI_INIT_TIME;
    break;

  case AT_CWSTATE:
//    Debug("Checking current SSID");
    ATCommandSend = CWSTATE();
    ESPTimeStamp = HAL_GetTick() + ESP_WIFI_INIT_TIME;
    break;

  case AT_CWMODE1:
//    Debug("Setting to station mode");
    ATCommandSend = CWMODE1();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CWMODE2:
//    Debug("Setting to station mode");
    ATCommandSend = CWMODE2();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CWAUTOCONN:
//    Debug("Setting auto connect");
    ATCommandSend = CWAUTOCONN();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CWJAP:
    Debug("Connect to Wi-Fi");
    ATCommandSend = CWJAP();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_LONG;
    break;

  case AT_CWMODE3:
//    Debug("SET in station/soft-ap mode");
    ATCommandSend = CWMODE3();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CWSAP:
//    Debug("SET soft AP mode parameters");
    ATCommandSend = CWSAP();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CIPMUX:
//    Debug("ATCommandSend = CIPMUX()");
    ATCommandSend = CIPMUX();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_WEBSERVER:
//    Debug("ATCommandSend = WEBSERVER()");
    ATCommandSend = WEBSERVER();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_HTTPCPOST:
    ATCommandSend = HTTPCPOST();
    ESPTimeStamp = HAL_GetTick() + ESP_WIFI_INIT_TIME;
    break;

  case AT_SENDDATA:
//    Debug("Send the data");
    ATCommandSend = SENDDATA();
    ESPTimeStamp = HAL_GetTick() + ESP_WIFI_INIT_TIME; // + 7000;
    break;

  case AT_SLEEP:
    Debug("Setting ESP in sleep mode for 5 min");
    ATCommandSend = SLEEP();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CIPSNTPCFG:
//    Debug("Config SNTP client");
    ATCommandSend = CIPSNTPCFG();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CIPSNTPTIME:
//    Debug("Get time from internet");
    ATCommandSend = CIPSNTPTIME();
    ESPTimeStamp = HAL_GetTick() + ESP_WIFI_INIT_TIME;
    break;

  case AT_CIPSNTPINTV:
//    Debug("Set the interval to timesync");
    ATCommandSend = CIPSNTPINTV();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;
#ifdef USE_MAIL
  case AT_HTTPCPOST_MAILAPI:
    Debug("Start EMAIL via API");
    ATCommandSend = HTTPCPOST_MAILAPI();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_LONG;
    break;

  case AT_SENDMAIL:
//    Debug("Send Email content");
    ATCommandSend = SENDMAIL();
    ESPTimeStamp = HAL_GetTick() + ESP_WIFI_INIT_TIME; // + 7000;
    break;
#endif

  case AT_END:
    break;
  }

  return(ATCommandSend);
}

bool is_OM_configured(void) {
//bert
  uint8_t boxConfig[IdSize];
  char Buffer[1+(2*IdSize)];
  ReadUint8ArrayEEprom(BoxConfigAddr, boxConfig, IdSize);
  uint8ArrayToString(Buffer, boxConfig);
  Buffer[12] = '\0';
  if (!isKeyValid(boxConfig, "box", "upload")) {
    ESPTimeStamp = HAL_GetTick() + ESP_UNTIL_NEXT_SEND;
    return false;
  }
  return true;
}

void ESP_WakeTest(void) {
  bool ATSend = false;
  static Receive_Status ATReceived = RECEIVE_STATUS_INCOMPLETE;
  switch (TestState){

    case ESP_TEST_INIT:
      if(!EspTurnedOn){
        EspTurnedOn = true;
        HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(10);
        // Reset ESP, so we're sure that we're in the right state.
        HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(ESP32_BOOT_GPIO_Port, ESP32_BOOT_Pin, 1);
        HAL_Delay(50); // wait for 5RC
        HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
        batteryCharge = ReadBatteryVoltage();  // read voltage in loaded condition
        ESPTimeStamp = HAL_GetTick() + ESP_START_UP_TIME;
        EspTurnedOn = true;
      }
      if(ESP_Receive(RxBuffer, ESP_MAX_BUFFER_SIZE)) {
        TestState = ESP_TEST_SEND;
      }
      break;

    case ESP_TEST_SEND:
      if(TimestampIsReached(ESPTimeStamp)){
        ATSend = AT_Send(ATCommand);
        if(ATSend){
          TestState = ESP_TEST_RECEIVE;
        }
      }
      break;

    case ESP_TEST_RECEIVE:
      if(TimestampIsReached(ESPTimeStamp)){
        ATReceived = DMA_ProcessBuffer(ATExpectation);
        bool proceed = ATCompare(ATReceived, ATExpectation);
        if(ATReceived == RECEIVE_STATUS_ERROR){
          TestState = ESP_TEST_SEND;
        }
        if(ATReceived == RECEIVE_STATUS_RETRY){
          //TestState = ESP_TEST_SEND;
          //ESPTimeStamp = HAL_GetTick() + 2*ESP_START_UP_TIME;
        }
        if(ATReceived == RECEIVE_STATUS_UNPROGGED){
          StartProg();
          TestState = ESP_TEST_BOOT;
        }
        if(ATReceived == RECEIVE_STATUS_INCOMPLETE){
          ESPTimeStamp = HAL_GetTick() + 50;
          TestState = ESP_TEST_SEND;
        }
        if(proceed){
          TestState = ESP_TEST_VALIDATE;
        }
      }
      break;

    case ESP_TEST_VALIDATE:
      //Set measurement completed
      TIM3 -> CCR1 = LED_OFF;
      TIM3 -> CCR2 = Calculate_LED_ON();
      TIM3 -> CCR3 = LED_OFF;
      TestState = ESP_TEST_DEINIT;

      break;

    case ESP_TEST_DEINIT:
      testRound = false;
      HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ESP32_BOOT_GPIO_Port, ESP32_BOOT_Pin, 0);
      SetESPMeasurementDone();
      EspTurnedOn = false;
      break;

    default:
      TestState = ESP_TEST_INIT;
      break;

    case ESP_TEST_BOOT:
      TIM3 -> CCR1 = LED_OFF;
      TIM3 -> CCR2 = LED_OFF;
      TIM3 -> CCR3 = Calculate_LED_ON();
      break;
  }
}

ESP_States ESP_Upkeep(void) {
  bool ATSend = false;
  static uint32_t timeoutTimer = 0;
  static Receive_Status ATReceived = RECEIVE_STATUS_INCOMPLETE;
  static uint8_t espminute;
// Het lijkt er op dat ESP32  niet meer start indien de batterijspanning onder de 3,77 Volt daalt.
// Om uart fouten te voorkomen mogelijk ESP niet meer afhandelen.
  if ((EspState != oldEspState) && (GetVerboseLevel() == VERBOSE_ALL)) {
    oldEspState = EspState;
    espminute = lastminute;
#ifdef USE_MAIL
    if ( !((oldEspState == 3) && ((ATCommand == AT_HTTPCPOST) || (ATCommand == AT_HTTPCPOST_MAILAPI))) ) {
#else
      if ( !((oldEspState == 3) && (ATCommand == AT_HTTPCPOST)) ) {
#endif
        showESPcontrols();
    }
  }
  switch (EspState) {
    case ESP_STATE_OFF:
      // Turning off the ESP
      // Disable UART

//      EspTurnedOn = false;
      EspState = ESP_STATE_IDLE;
      break;

    case ESP_STATE_IDLE:
      // Waiting for wake up call.
      break;

    case ESP_STATE_INIT:
//      Debug("entry in ESP_STATE_INIT");
      deviceTimeOut = 0;
      if (!AllDevicesReady()) {
        if (espminute != lastminute) {
          Debug("ESP_STATE_INIT Waiting for all devices ready");
        }
        break;
      }
      SetESPIndicator();
      if(!EspTurnedOn){
        HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        // Reset ESP, so we're sure that we're in the right state.
        HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(ESP32_BOOT_GPIO_Port, ESP32_BOOT_Pin, 1);
        HAL_Delay(50);
        HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        ESPTimeStamp = HAL_GetTick() + ESP_START_UP_TIME;
        EspTurnedOn = true;
        Debug("ESP powered on.");
        SetBatteryReadTimer(ESP_START_UP_TIME/2);  // read battery voltage during boot of ESP32
      }
      // Wait for ESP to be ready
      // Start reading DMA buffer for AT commands
      if(ESP_Receive(RxBuffer, ESP_MAX_BUFFER_SIZE)) {
        EspState = ESP_STATE_WAIT_AWAKE;
        timeoutTimer = HAL_GetTick() + 2000;
      }
      break;

    case ESP_STATE_WAIT_AWAKE:
//        Debug("entry in ESP_STATE_WAIT_AWAKE");
        ATReceived = DMA_ProcessBuffer(RECEIVE_EXPECTATION_READY);
        bool proceed = ATCompare(ATReceived, RECEIVE_EXPECTATION_READY);
        if(proceed || TimestampIsReached(timeoutTimer)){
          EspState = ESP_STATE_MODE_SELECT;
        }
        break;

    case ESP_STATE_MODE_SELECT:
//      Debug("entry in ESP_STATE_MODE_SELECT");
      memset(ATCommandArray, AT_END, 9);
      if(!InitIsDone || WifiReset){
        memcpy(ATCommandArray, AT_INIT, sizeof(AT_INIT));
        EspState = ESP_STATE_SEND;
        ATCounter = 0;
        Mode = AT_MODE_INIT;
        ATCommand = ATCommandArray[ATCounter];
        ATExpectation = RECEIVE_EXPECTATION_OK;
      }
      if(InitIsDone && !ConnectionMade){
        memcpy(ATCommandArray, AT_WIFI_CONFIG, 6);
        EspState = ESP_STATE_SEND;
        ATCounter = 0;
        Mode = AT_MODE_CONFIG;
        ATCommand = ATCommandArray[ATCounter];
        ATExpectation = RECEIVE_EXPECTATION_OK;
      }
      if(InitIsDone && ConnectionMade && !APtested){
        memcpy(ATCommandArray, AT_TEST, 2);
        EspState = ESP_STATE_SEND;
        ATCounter = 0;
        Mode = AT_MODE_TEST;
        ATCommand = ATCommandArray[ATCounter];
        ATExpectation = RECEIVE_EXPECTATION_OK;
      }
#ifdef USE_MAIL
      if(InitIsDone && ConnectionMade && APtested && (sendpwremail == DO_PWR_MAIL)){
        memcpy(ATCommandArray, AT_MAIL, 3);
        EspState = ESP_STATE_SEND;
        ATCounter = 0;
        Mode = AT_MODE_MAIL;
        ATCommand = ATCommandArray[ATCounter];
        ATExpectation = RECEIVE_EXPECTATION_OK;
      }
#endif
      if(InitIsDone && ConnectionMade && APtested && !setTime && (sendpwremail != DO_PWR_MAIL)){
        memcpy(ATCommandArray, AT_SEND, 3);
        EspState = ESP_STATE_SEND;
        ATCounter = 0;
        Mode = AT_MODE_SEND;
        start = HAL_GetTick();
        ATCommand = ATCommandArray[ATCounter];
        ATExpectation = RECEIVE_EXPECTATION_OK;
      }
      if(InitIsDone && ConnectionMade && APtested && setTime && (sendpwremail != DO_PWR_MAIL)){
        memcpy(ATCommandArray, AT_SNTP, 4);
        EspState = ESP_STATE_SEND;
        ATCounter = 0;
        Mode = AT_MODE_GETTIME;
        start = HAL_GetTick();
        ATCommand = ATCommandArray[ATCounter];
        ATExpectation = RECEIVE_EXPECTATION_OK;
      }
      if(ReconfigSet){
        memcpy(ATCommandArray, AT_WIFI_RECONFIG, 5);
        Debug("Reconfig mode for local wifi config selected");
        DisableConnectedDevices();
        usblog = false;
        EspState = ESP_STATE_SEND;
        ATCounter = 0;
        Mode = AT_MODE_RECONFIG;
        ATCommand = ATCommandArray[ATCounter];
        ATExpectation = RECEIVE_EXPECTATION_OK;
      }
    break;

    case ESP_STATE_SEND:
//      Debug("entry in ESP_STATE_SEND");
        ATSend = AT_Send(ATCommand);
        if(ATSend){
          EspState = ESP_STATE_WAIT_FOR_REPLY;
        }
    break;

    case ESP_STATE_WAIT_FOR_REPLY:
      if ((ReconfigSet) && (Mode != AT_MODE_RECONFIG)) {
        EspState = ESP_STATE_MODE_SELECT;
        break;
      }
      if(TimestampIsReached(ESPTimeStamp)){
        ATReceived = DMA_ProcessBuffer(ATExpectation);
        bool proceed = ATCompare(ATReceived, ATExpectation);
        if(ATReceived == RECEIVE_STATUS_ERROR){
          if(ATCommand == AT_SENDDATA){
            ATCommand = AT_HTTPCPOST;
            ATExpectation = RECEIVE_EXPECTATION_START;
            ATCounter = 1;
          }
          if(ATCommand == AT_SENDMAIL){
            ATCommand = AT_HTTPCPOST_MAILAPI;
            ATExpectation = RECEIVE_EXPECTATION_START;
            ATCounter = 1;
          }
          EspState = ESP_STATE_SEND;
          errorcntr++;
          if (errorcntr >= ESP_MAX_RETRANSMITIONS) {
            ESPTimeStamp = HAL_GetTick() + ESP_UNTIL_NEXT_SEND;
            ESPTransmitDone = true;
            clearDMABuffer();
            stop = HAL_GetTick();
            Error("ESP to many retransmits, terminated after %lu ms", (stop-start));
            EspState = ESP_STATE_DEINIT;
            break;
          }
        }
        if(ATReceived == RECEIVE_STATUS_INCOMPLETE){
          ESPTimeStamp = HAL_GetTick() + 10;
        }
        if(ATReceived == RECEIVE_STATUS_LOOP){
          ESPTimeStamp = HAL_GetTick() + 10;
        }
        if(ATReceived == RECEIVE_STATUS_TIMEOUT){
          timeoutcntr++;
          Error("In ESP_STATE_WAIT_FOR_REPLY: RECEIVE_STATUS_TIMEOUT reached");
          if (timeoutcntr >= ESP_MAX_RETRANSMITIONS) {
            ESPTimeStamp = HAL_GetTick() + ESP_UNTIL_NEXT_RETRANSMIT_SEND;
            ESPTransmitDone = true;
            clearDMABuffer();
            stop = HAL_GetTick();
            Error("ESP to many timeouts, terminated after %lu ms", (stop-start));
            EspState = ESP_STATE_DEINIT;
            ATCommand = AT_END;
            ATExpectation = RECEIVE_EXPECTATION_OK;
            break;
          }
          if(ATCommand != AT_SENDDATA){
            EspState = ESP_STATE_SEND;
          }
          else{
            ATCommand = AT_HTTPCPOST;
            ATCounter -= 1;
            ATExpectation = RECEIVE_EXPECTATION_START;
            EspState = ESP_STATE_SEND;
          }
        }
        if(proceed){
          EspState = ESP_STATE_NEXT_AT;
        }
      }
      break;

    case ESP_STATE_NEXT_AT:
//      Debug("entry in ESP_STATE_NEXT_AT");
      ATCounter += 1;
      ATCommand = ATCommandArray[ATCounter];
      if(ATCommand == AT_RESTORE){
         ATExpectation = RECEIVE_EXPECTATION_READY;
      }
      if(ATCommand == AT_HTTPCPOST){
        ATExpectation = RECEIVE_EXPECTATION_START;
      }
      if(ATCommand != AT_HTTPCPOST && ATCommand != AT_RESTORE){
        ATExpectation = RECEIVE_EXPECTATION_OK;
      }
      if(ATCommand == AT_CIPSNTPCFG){
         ATExpectation = RECEIVE_EXPECTATION_TIME;
      }
#ifdef USE_MAIL
      if(ATCommand == AT_SENDMAIL){
        ATExpectation = RECEIVE_EXPECTATION_OK;
      }
#endif
      EspState = ESP_STATE_SEND;
      if(ATCommand == AT_END){
        if(Mode == AT_MODE_SEND){
          ESPTimeStamp = HAL_GetTick() + ESP_UNTIL_NEXT_SEND;
          clearDMABuffer();
          stop = HAL_GetTick();
          Info("Message send in %lu ms", (stop-start));
          ResetdBAmax();
          resetMaxMeasurementValues();
          showTime();
          ESPTransmitDone = true;
          EspState = ESP_STATE_DEINIT;
        }
        else if (Mode == AT_MODE_GETTIME) {
            setTime = false;
            ESPNTPTimeStamp = calculateNextNTPTime();
            ESPNTPTimeStamp += ESP_UNTIL_NEXT_NTP;
            Info("Time synchronized by NTP, next NTP should be called in %lu seconds", ESP_UNTIL_NEXT_NTP);
            ESPTimeStamp = savedESPTimeStamp;
            clearDMABuffer();
            stop = HAL_GetTick();
            Info("Message time update in %lu ms", (stop-start));
            if (HAL_GetTick() < DEVICE_INIT_TIMEOUT) { // during startup the sensors are active after getting time
              deviceTimeOut = DEVICE_INIT_TIMEOUT;
              EnabledConnectedDevices();
            }
            EspState = ESP_STATE_DEINIT;
            Mode = AT_MODE_SEND;
          }
#ifdef USE_MAIL
        else if (Mode == AT_MODE_MAIL) {
            clearDMABuffer();
            ESPTimeStamp = savedESPTimeStamp;
            sendpwremail = DONE;
            EspState = ESP_STATE_DEINIT;
            Mode = AT_MODE_SEND;
            EnabledConnectedDevices();
          }
#endif
        else{
          EspState = ESP_STATE_RESET;
        }
      }
    break;

    case ESP_STATE_DEINIT:
//      Debug("entry in ESP_STATE_DEINIT");
      EspTurnedOn = false;
      HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_RESET);
      HAL_Delay(1);
      HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_RESET);
      HAL_Delay(1);
      HAL_GPIO_WritePin(ESP32_BOOT_GPIO_Port, ESP32_BOOT_Pin, 0);
      EspState = ESP_STATE_RESET;
      HAL_Delay(1);
      ResetESPIndicator();
      Debug("ESP powered off.");
      if (Check_USB_PowerOn() || userToggle) {
        EnabledConnectedDevices();
      }
      errorcntr = 0;
      timeoutcntr = 0;
      break;

    case ESP_STATE_RESET:
      if(TimestampIsReached(ESPTimeStamp) || ReconfigSet){
        ESPTransmitDone = false;
        if((Mode == AT_MODE_INIT) && is_OM_configured()){
          InitIsDone = true;
          EspState = ESP_STATE_MODE_SELECT;
        }
        if(Mode == AT_MODE_CONFIG){
          ConnectionMade = true;
          EspState = ESP_STATE_MODE_SELECT;
        }
        if((Mode == AT_MODE_SEND) && is_OM_configured()) {
          EspState = ESP_STATE_INIT;
        }
#ifdef USE_MAIL
        if((Mode == AT_MODE_MAIL) && is_OM_configured()){
          EspState = ESP_STATE_CONFIG;
        }
#endif
        if(Mode == AT_MODE_RECONFIG){
          EspState = ESP_STATE_CONFIG;
          Info("Do nothing until reset");
        }
        if(Mode == AT_MODE_TEST){
          EspState = ESP_STATE_MODE_SELECT;
          APtested = true;  // deze lijkt mij dubieus wordt in CWJAP gezet.
        }
        if ((ReconfigSet) && (Mode != AT_MODE_RECONFIG)) {
          EspState = ESP_STATE_INIT;
        }
      }
      else if (calculateNextNTPTime() > ESPNTPTimeStamp) {
        if(Mode == AT_MODE_SEND ) {
           Mode = AT_MODE_GETTIME;
           EspState = ESP_STATE_INIT;
           savedESPTimeStamp = ESPTimeStamp;
           setTime = true;
        }
      }
      break;

    case ESP_STATE_CONFIG:
      if (!msgdone) {
        Info("Do nothing until reset");
        msgdone = true;
      }
      Process_PC_Config(GetUsbRxPointer());
      break;

    case ESP_STATE_ERROR:
      // Handle error state
      Error("ESP Error occurred");
      EspState = ESP_STATE_INIT;
      break;

    default:
      // Handle unexpected state
      Error("Something unknown went wrong with the ESP_STATE");
      EspState = ESP_STATE_ERROR;
      break;
  }
  return EspState;
}

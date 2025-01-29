/*
 * ESP.c
 *
 *  Created on: Jun 28, 2024
 *      Author: Joris Blankestijn
 *              Bert Havinga nov-dec
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
#ifdef PUBLIC
#include "cred_pub.h"
#else
#include "cred.h"
#endif
#include "ssd1306_128x64_i2c.h"
#ifdef SSD1306
#include "display.h"
#endif

static UART_HandleTypeDef* EspUart = NULL;
extern DMA_HandleTypeDef hdma_usart4_rx;

static volatile bool RxComplete = false;

static uint8_t RxBuffer[ESP_MAX_BUFFER_SIZE] = {0};
static const char user[] = "Test";
static bool testRound = true;
bool EspTurnedOn = false;
static bool InitIsDone = false;
static bool WifiReset = false;
static bool ReconfigSet = false;
static bool ConnectionMade = false;
static bool beursTest = false;
static bool beurs = false;
static bool setTime = true;
static bool msgdone = false;
bool ESPTransmitDone = false;
static uint32_t uid[3];
static uint32_t start;
static uint32_t stop;
static uint16_t txLength = 0;
static uint8_t oldEspState = 255;
float batteryCharge = 0.0;
float solarCharge = 0.0;
static char message[144];
static const char APIBeurs[] = "\"https://deomgevingsmonitor.nl//api/set_device_data.php\"";
static const char API[] = "\"https://api.opensensemap.org/boxes/";
static AT_Commands ATCommandArray[10];
static AT_Commands AT_INIT[] = {AT_WAKEUP, AT_SET_RFPOWER, AT_CHECK_RFPOWER, AT_CWINIT, AT_CWAUTOCONN, AT_CWMODE1, AT_CIPMUX};
static AT_Commands AT_SEND[] = {AT_WAKEUP,  AT_HTTPCPOST, AT_SENDDATA};
static AT_Commands AT_BEURSTEST[] = {AT_WAKEUP, AT_CWSTATE};
static AT_Commands AT_WIFI_CONFIG[] = {AT_WAKEUP, AT_CWINIT, AT_CWMODE3, AT_CWAUTOCONN, AT_CWJAP, AT_CIPMUX};
static AT_Commands AT_WIFI_RECONFIG[] = {AT_WAKEUP, AT_CWMODE3, AT_CWSAP, AT_CIPMUX, AT_WEBSERVER};
static AT_Commands AT_SNTP[] = {AT_WAKEUP, AT_CIPSNTPCFG, AT_CIPSNTPTIME, AT_CIPSNTPINTV};
uint8_t ATState;
uint8_t ATCounter = 0;
static uint8_t errorcntr = 0;
static uint8_t timeoutcntr = 0;
static uint32_t ESPTimeStamp = 0;
static uint32_t ESPNTPTimeStamp = 20000;
static uint32_t savedESPTimeStamp = 28000;
static uint8_t retry = 0;


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

void forceNTPupdate() {
  ESPNTPTimeStamp = 0;
}

void setESPTimeStamp(uint32_t delayms) {
  ESPTimeStamp = HAL_GetTick() + delayms;
}
void setCharges(){
  batteryCharge = ReadBatteryVoltage();
  solarCharge = ReadSolarVoltage() / 1000.0;
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
  if (voc > MeasVal.VOCIndexmax) {
    MeasVal.VOCIndexmax = voc;
  }
#ifdef SSD1306
  if (SSD1306detected &&(Check_USB_PowerOn() || userToggle)) {
    displayVOC();
  }
#endif
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
//  Debug("SetNOx entered");
  MeasVal.airNOx = nox;
  if (nox > MeasVal.airNOxmax) {
    MeasVal.airNOxmax = nox;
  }
#ifdef SSD1306
  if (SSD1306detected && (Check_USB_PowerOn() || userToggle)) {
//    Debug("calling display NOx update");
    displayNOx();
  }
#endif
}

void resetMaxMeasurementValues() {
  if (sen5x_Get_sen5x_enable_state()) {
    MeasVal.PM2p5max = 0.0f;
    MeasVal.PM10p0max = 0.0f;
    MeasVal.airNOxmax = 0;
  }
  MeasVal.VOCIndexmax = 0;
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

void ESP_GetUID(){
  uid[0] = HAL_GetUIDw0();
  uid[1] = HAL_GetUIDw1();
  uid[2] = HAL_GetUIDw2();
}
void ESP_Init(UART_HandleTypeDef* espUart) {
  EspUart = espUart;
  EspState = ESP_STATE_INIT;
  ESP_GetUID();
  beurs = checkEEprom();
}

static bool ESP_Send(uint8_t* command, uint16_t length) {
  HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(EspUart, command, length);
  if (status != HAL_OK) {
    Error("Error in HAL_UART_Transmit_DMA");
    return false;
  }
#ifdef LONGMESSAGES
  printf("ESP_Send: %s\r\n", command);
#else
  Debug("ESP_Send: %s", command);
#endif
  return true;
}
static bool ESP_Receive(uint8_t* reply, uint16_t length) {
  RxComplete = false;
  bool reset = false;
  HAL_StatusTypeDef status = HAL_UART_Receive_DMA(EspUart, reply, length);
  if (status != HAL_OK) {
    Error("Error in HAL_UART_Receive_DMA. errorcode: %d", EspUart->ErrorCode);
    if (status & HAL_UART_ERROR_PE) {
      Error("Parity error in UART to ESP module");
      reset = true;
    }
    if (status & HAL_UART_ERROR_NE) {
      Error("Noise error in UART to ESP module");
    }
    if (status & HAL_UART_ERROR_FE) {
      Error("Frame error in UART to ESP module");
    }
    if (status & HAL_UART_ERROR_ORE) {
      Error("Overrun error in UART to ESP module");
    }
    if (status & HAL_UART_ERROR_DMA) {
      Error("DMA transfer error in UART to ESP module");
    }
    if (status & HAL_UART_ERROR_RTO) {
      Error("Receiver Timeout error in UART to ESP module");
    }
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    if (status & HAL_UART_ERROR_INVALID_CALLBACK) {
      Error("Invalid Callback error in UART to ESP module");
    }
#endif
    if (reset) {
      SetAllREDLED();
      HAL_NVIC_SystemReset();
    }
    RxComplete = true;
    return false;
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
  if (huart == EspUart) {
    if (huart->ErrorCode == 4) {
      return;
    }
    Debug("A callback error has occurred, errorcode %0X", huart->ErrorCode);
    switch (huart->ErrorCode) {
      case HAL_UART_TX_HALFCOMPLETE_CB_ID:
        Error("ESP connection UART Tx Half Complete Callback ID");
        break;
      case HAL_UART_TX_COMPLETE_CB_ID:
        Error("ESP connection UART Tx Complete Callback ID");
        break;
      case HAL_UART_RX_HALFCOMPLETE_CB_ID:
        Error("ESP connection UART Rx Half Complete Callback ID");
        break;
      case HAL_UART_RX_COMPLETE_CB_ID:
        Error("ESP connection UART Rx Complete Callback ID");
        break;
      case HAL_UART_ERROR_CB_ID:
        Error("ESP connection UART Error Callback ID");
        break;
      case HAL_UART_ABORT_COMPLETE_CB_ID:
        Error("ESP connection UART Abort Complete Callback ID");
        break;
      case HAL_UART_ABORT_TRANSMIT_COMPLETE_CB_ID:
        Error("ESP connection UART Abort Transmit Complete Callback ID");
        break;
      case HAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID:
        Error("ESP connection UART Abort Receive Complete Callback ID");
        break;
      case HAL_UART_WAKEUP_CB_ID:
        Error("ESP connection UART Wakeup Callback ID");
        break;
      case HAL_UART_MSPINIT_CB_ID:
        Error("ESP connection UART MspInit callback ID");
        break;
      case HAL_UART_MSPDEINIT_CB_ID:
        Error("ESP connection UART MspDeInit callback ID");
        break;
      default:
        Error("ESP connection UART Unknown error");
    }
  }
}

void uint8ArrayToString(char *destination, uint8_t data[]) {
  for (int i = 0; i < 12; i++) {
    sprintf(&destination[i * 2], "%02x", data[i]);
  }
}

uint16_t CreateMessage(bool onBeurs, bool *txstat, bool send) {
  static bool status = false;
  static bool retstat = true;
  static uint8_t tempConfig[IdSize];
  static uint8_t humidConfig[IdSize];
  static uint8_t soundConfig[IdSize];
  static uint8_t vocConfig[IdSize];
  static uint8_t batteryConfig[IdSize];
  static uint8_t solarConfig[IdSize];
  static uint8_t noxConfig[IdSize];
  static uint8_t PM2Config[IdSize];
  static uint8_t PM10Config[IdSize];
  static uint8_t nameConfig[CustomNameMaxLength];
#ifdef LONGDATAGRAM
  static char Buffer[25];
#endif
  ReadUint8ArrayEEprom(TempConfigAddr, tempConfig, IdSize);
  ReadUint8ArrayEEprom(HumidConfigAddr, humidConfig, IdSize);
  ReadUint8ArrayEEprom(dBAConfigAddr, soundConfig, IdSize);
  ReadUint8ArrayEEprom(VocIndexConfigAddr, vocConfig, IdSize);
  ReadUint8ArrayEEprom(BatVoltConfigAddr, batteryConfig, IdSize);
  ReadUint8ArrayEEprom(SolVoltConfigAddr, solarConfig, IdSize);
  ReadUint8ArrayEEprom(NOxIndexConfigAddr, noxConfig, IdSize);
  ReadUint8ArrayEEprom(PM2ConfigAddr, PM2Config, IdSize);
  ReadUint8ArrayEEprom(PM10ConfigAddr, PM10Config, IdSize);
  if(checkName()){
    ReadUint8ArrayEEprom(CustomNameConfigAddr, nameConfig, CustomNameMaxLength);
  }
  else{
    strncpy((char*)nameConfig, user, 5);
  }
  setCharges();
  uint16_t index = 0;
  sprintf(&message[index], "[");
#ifdef LONGDATAGRAM
//  memset(message, '\0', 144); \\ unnecessary sprintf terminates with \0

  uint8ArrayToString(Buffer, tempConfig);
#ifdef OPENSENSEMAP
  sprintf(&message[1], "{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.Temperature);
#else
  sprintf(&message[1], "{\"name\":\"temp\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.2f, \"unit\":\"C\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.Temperature);
#endif
index = strlen(message);
  if (send) {
    status = ESP_Send((uint8_t*)message, strlen(message));
    retstat &= status;
  }

  uint8ArrayToString(Buffer, humidConfig);
#ifdef OPENSENSEMAP
  sprintf(&message[0], ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.Humidity);
#else
  sprintf(&message[0], ",{\"name\":\"humid\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"%%\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.Humidity);
#endif
  index += strlen(message);
  if (send) {
    status = ESP_Send((uint8_t*)message, strlen(message));
    retstat &= status;
  }

  uint8ArrayToString(Buffer, vocConfig);
#ifdef OPENSENSEMAP
  sprintf(&message[0], ",{\"sensor\": \"%s\", \"value\":%d}", Buffer, MeasVal.VOCIndexmax);
#else
  sprintf(&message[0], ",{\"name\":\"voc\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%d, \"unit\":\"VOCi\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.VOCIndexmax);
#endif
  index += strlen(message);
  if (send) {
    status = ESP_Send((uint8_t*)message, strlen(message));
    retstat &= status;
  }

  uint8ArrayToString(Buffer, soundConfig);
#ifdef OPENSENSEMAP
  sprintf(&message[0], ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.dBApeak);
#else
  sprintf(&message[0], ",{\"name\":\"Sound\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.2f, \"unit\":\"dB(A)\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.dBApeak);
#endif
  index += strlen(message);
  if (send) {
    status = ESP_Send((uint8_t*)message, strlen(message));
    retstat &= status;
  }

  if(!onBeurs){
    uint8ArrayToString(Buffer, solarConfig);
#ifdef OPENSENSEMAP
    sprintf(&message[0], ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, solarCharge);
#else
    sprintf(&message[0], ",{\"name\":\"Solar voltage\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.2f, \"unit\":\"V\"}", uid[2], (char*)nameConfig, Buffer, solarCharge);
#endif
    index += strlen(message);
    if (send) {
      status = ESP_Send((uint8_t*)message, strlen(message));
      retstat &= status;
    }

    if ((product_name[4] == '5') && Check_USB_PowerOn()) {  // the NOx has only sense in case of continuous operation
      uint8ArrayToString(Buffer, noxConfig);
#ifdef OPENSENSEMAP
      sprintf(&message[0], ",{\"sensor\": \"%s\", \"value\":%d}", Buffer, MeasVal.airNOxmax);
#else
      sprintf(&message[0], ",{\"name\":\"NOx\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%d, \"unit\":\"NOxr\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.airNOxmax);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }
    if (sen5x_Get_sen5x_enable_state()) {
      uint8ArrayToString(Buffer, PM2Config);
#ifdef OPENSENSEMAP
      sprintf(&message[0], ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.PM2p5max);
#else
      sprintf(&message[0], ",{\"name\":\"PM2.5\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"µg/m3\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.PM2p5max);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }

      uint8ArrayToString(Buffer, PM10Config);
#ifdef OPENSENSEMAP
      sprintf(&message[0], ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, MeasVal.PM10p0max);
#else
      sprintf(&message[0], ",{\"name\":\"PM10\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.1f, \"unit\":\"µg/m3\"}", uid[2], (char*)nameConfig, Buffer, MeasVal.PM10p0max);
#endif
      index += strlen(message);
      if (send) {
        status = ESP_Send((uint8_t*)message, strlen(message));
        retstat &= status;
      }
    }
  }
  uint8ArrayToString(Buffer, batteryConfig);
#ifdef OPENSENSEMAP
  sprintf(&message[0], ",{\"sensor\": \"%s\", \"value\":%.2f}", Buffer, batteryCharge);
#else
  sprintf(&message[0], ",{\"name\":\"battery\", \"id\": %ld, \"user\": \"%s\", \"sensor\": \"%s\", \"value\":%.2f, \"unit\":\"V\"}", uid[2], (char*)nameConfig, Buffer, batteryCharge);
#endif

#else
//  memset(message, '\0', 255);
    uint8_t arridx = 0;
//    sprintf(&message[index], "[");
    index = strlen(message);


    sprintf(&message[index], "{\"Temperature\":%.2f},", MeasVal.Temperature);
    index = strlen(message);


    sprintf(&message[index], "{\"Humidity\":%.1f},", MeasVal.Humidity);
    index += strlen(message);


    sprintf(&message[index], "{\"Sound\":%.2f},", MeasVal.dBApeak);
    index += strlen(message);


    sprintf(&message[index], "{\"VOC\":%d},", MeasVal.VOCIndexmax);
    index += strlen(message);


    sprintf(&message[index], "{\"BatteryVoltage\":%.2f},", batteryCharge);
    index += strlen(message);
    if (send) {
      status = ESP_Send((uint8_t*)message, strlen(message));
      retstat &= status;
    }

    sprintf(&message[arridx], "{\"SolarVoltage\":%.2f}", solarCharge);
    arridx += strlen(message);

    sprintf(&message[arridx], "{\"PM2p5\":%.2f}", MeasVal.PM2p5max);
    arridx += strlen(message);

    sprintf(&message[arridx], "{\"PM10\":%.2f}", MeasVal.PM10p0max);
    arridx += strlen(message);
    index +=arridx;

    sprintf(&message[arridx], "{\"NOX\":%d}", MeasVal.airNOxmax);
#endif
  sprintf(&message[strlen(message)], "]");
  index += strlen(message);
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
  printf("Receive ParseBuffer: %s", tempBuf );
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
  char *ParsePoint4 = strstr(tempBuf, SSIDBeurs);
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
      beurs = true;
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

bool CWJAP(){
  beursTest = true;
  char atCommandBuff[100];
  memset(atCommandBuff, '\0', 100);
  sprintf(atCommandBuff, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSIDBeurs, PasswordBeurs);
  uint8_t len = strlen(atCommandBuff);
  char atCommand[len+1];
  memset(atCommand, '\0', len+1);
  strncpy(atCommand, atCommandBuff, len);
  if(ESP_Send((uint8_t*)atCommand, len)) {
    return true;
  }
  else{
    return false;
  }
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

//These are the commands necesarry for sending data.
bool HTTPCPOST(){
//  char atCommandBuff[256];
  bool txresult = false;

//  memset(atCommandBuff, '\0', 256);
  uint16_t length = CreateMessage(beurs, &txresult, false);
  if(beurs){
//    sprintf(atCommandBuff, "AT+HTTPCPOST=%s,%d,1,\"content-type: application/json\"\r\n", APIBeurs, length);
    sprintf(message, "AT+HTTPCPOST=%s,%d,1,\"content-type: application/json\"\r\n", APIBeurs, length);
  }
  else{
    static uint8_t boxConfig[IdSize];
    static char Buffer[25];
    ReadUint8ArrayEEprom(BoxConfigAddr, boxConfig, IdSize);
    uint8ArrayToString(Buffer, boxConfig);
//    sprintf(atCommandBuff, "AT+HTTPCPOST=%s%s/data\",%d,1,\"content-type: application/json\"\r\n", API, Buffer, length);
    sprintf(message, "AT+HTTPCPOST=%s%s/data\",%d,1,\"content-type: application/json\"\r\n", API, Buffer, length);
  }
//  uint16_t len = strlen(atCommandBuff);
  uint16_t len = strlen(message);
  Debug("length of message (former atCommandBuff) during header tx: %d bool value of tx result %d", len, txresult);
//  if(ESP_Send((uint8_t*)atCommandBuff, len)){
  if(ESP_Send((uint8_t*)message, len)){
    return true;
  }
  else{
    return false;
  }
}

bool SENDDATA(){
  bool result = false;
/*
  uint16_t len = strlen(message);
  if(ESP_Send((uint8_t*)message, len)) {
    return true;
  }
  else{
    return false;
  }
*/
  txLength = CreateMessage(beurs, &result, true);
  Debug("SENDDATA ESP_Send result = %d, transmitted data %d chars", result, txLength);
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
    Debug("Setting RF Power");
    ATCommandSend = RFPower();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CHECK_RFPOWER:
    Debug("Checking RF Power");
    ATCommandSend = CheckRFPower();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_RESTORE:
    Debug("Restoring ESP");
    ATCommandSend = ATRestore();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_LONG;
    break;

  case AT_CWINIT:
    Debug("Initializing Wi-Fi");
    ATCommandSend = CWINIT();
    ESPTimeStamp = HAL_GetTick() + ESP_WIFI_INIT_TIME;
    break;

  case AT_CWSTATE:
    Debug("Checking current SSID");
    ATCommandSend = CWSTATE();
    ESPTimeStamp = HAL_GetTick() + ESP_WIFI_INIT_TIME;
    break;

  case AT_CWMODE1:
    Debug("Setting to station mode");
    ATCommandSend = CWMODE1();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CWMODE2:
    Debug("Setting to station mode");
    ATCommandSend = CWMODE2();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CWAUTOCONN:
    Debug("Setting auto connect");
    ATCommandSend = CWAUTOCONN();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CWJAP:
    Debug("Connect to Wi-Fi");
    ATCommandSend = CWJAP();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_LONG;
    break;

  case AT_CWMODE3:
    Debug("SET in station/soft-ap mode");
    ATCommandSend = CWMODE3();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CWSAP:
    Debug("SET soft AP mode parameters");
    ATCommandSend = CWSAP();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CIPMUX:
    Debug("ATCommandSend = CIPMUX()");
    ATCommandSend = CIPMUX();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_WEBSERVER:
    Debug("ATCommandSend = WEBSERVER()");
    ATCommandSend = WEBSERVER();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_HTTPCPOST:
    ATCommandSend = HTTPCPOST();
    ESPTimeStamp = HAL_GetTick() + ESP_WIFI_INIT_TIME;
    break;

  case AT_SENDDATA:
    Debug("Send the data");
    ATCommandSend = SENDDATA();
    ESPTimeStamp = HAL_GetTick() + ESP_WIFI_INIT_TIME; // + 7000;
    break;

  case AT_SLEEP:
    Debug("Setting ESP in sleep mode for 5 min");
    ATCommandSend = SLEEP();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CIPSNTPCFG:
    Debug("Config SNTP client");
    ATCommandSend = CIPSNTPCFG();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_CIPSNTPTIME:
    Debug("Get time from internet");
    ATCommandSend = CIPSNTPTIME();
    ESPTimeStamp = HAL_GetTick() + ESP_WIFI_INIT_TIME;
    break;

  case AT_CIPSNTPINTV:
    Debug("Set the interval to timesync");
    ATCommandSend = CIPSNTPINTV();
    ESPTimeStamp = HAL_GetTick() + ESP_RESPONSE_TIME;
    break;

  case AT_END:
    break;
  }

  return(ATCommandSend);
}

void ESP_WakeTest(void) {
  bool ATSend = false;
  static Receive_Status ATReceived = RECEIVE_STATUS_INCOMPLETE;
  switch (TestState){

    case ESP_TEST_INIT:
      if(!EspTurnedOn){
        HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(10);
        // Reset ESP, so we're sure that we're in the right state.
        HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(ESP32_BOOT_GPIO_Port, ESP32_BOOT_Pin, 1);
        HAL_Delay(10);
        HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_SET);
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
      EspTurnedOn = false;
      HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ESP32_BOOT_GPIO_Port, ESP32_BOOT_Pin, 0);
      SetESPMeasurementDone();
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

  if ((EspState != oldEspState) && (GetVerboseLevel() == VERBOSE_ALL)) {
    oldEspState = EspState;
    if (!((oldEspState == 3) && (ATCommand == AT_HTTPCPOST)) ) {
      Debug("EspState: %d ATcmd: %d Mode: %d ATExp: %d", oldEspState, ATCommand, Mode, ATExpectation);
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
      DisableConnectedDevices();
      SetESPIndicator();
      if(!EspTurnedOn){
        HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(Wireless_PSU_EN_GPIO_Port, Wireless_PSU_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
        // Reset ESP, so we're sure that we're in the right state.
        HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(ESP32_BOOT_GPIO_Port, ESP32_BOOT_Pin, 1);
        HAL_Delay(1);
        HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_SET);
        ESPTimeStamp = HAL_GetTick() + ESP_START_UP_TIME;
        EspTurnedOn = true;
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
        memcpy(ATCommandArray, AT_INIT, 7);
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
      if(InitIsDone && ConnectionMade && !beursTest){
        memcpy(ATCommandArray, AT_BEURSTEST, 2);
        EspState = ESP_STATE_SEND;
        ATCounter = 0;
        Mode = AT_MODE_TEST;
        ATCommand = ATCommandArray[ATCounter];
        ATExpectation = RECEIVE_EXPECTATION_OK;
      }
      if(InitIsDone && ConnectionMade && beursTest && !setTime){
        memcpy(ATCommandArray, AT_SEND, 3);
        EspState = ESP_STATE_SEND;
        ATCounter = 0;
        Mode = AT_MODE_SEND;
        start = HAL_GetTick();
        ATCommand = ATCommandArray[ATCounter];
        ATExpectation = RECEIVE_EXPECTATION_OK;
      }
      if(InitIsDone && ConnectionMade && beursTest && setTime){
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
        Debug("Reconfig mode voor local wifi config selected");
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
            ESPNTPTimeStamp = HAL_GetTick()+ESP_UNTIL_NEXT_NTP;
            Info("Time synchronized by NTP, next NTP should be called at tick: %lu", ESPNTPTimeStamp);
            ESPTimeStamp = savedESPTimeStamp;
            clearDMABuffer();
            stop = HAL_GetTick();
            Info("Message time update in %lu ms", (stop-start));
            EspState = ESP_STATE_DEINIT;
            Mode = AT_MODE_SEND;
          }
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
      EnabledConnectedDevices();
      HAL_Delay(1);
      ResetESPIndicator();
      errorcntr = 0;
      timeoutcntr = 0;
      break;

    case ESP_STATE_RESET:
      if(TimestampIsReached(ESPTimeStamp) || ReconfigSet){
        ESPTransmitDone = false;
        if(Mode == AT_MODE_INIT){
          InitIsDone = true;
          EspState = ESP_STATE_MODE_SELECT;
        }
        if(Mode == AT_MODE_CONFIG){
          ConnectionMade = true;
          beurs = true;
          EspState = ESP_STATE_MODE_SELECT;
        }
        if(Mode == AT_MODE_SEND){
          EspState = ESP_STATE_INIT;
        }
        if(Mode == AT_MODE_RECONFIG){
          EspState = ESP_STATE_CONFIG;
          Info("Do nothing until reset");
        }
        if(Mode == AT_MODE_TEST){
          EspState = ESP_STATE_MODE_SELECT;
          beursTest = true;
        }
        if ((ReconfigSet) && (Mode != AT_MODE_RECONFIG)) {
          EspState = ESP_STATE_INIT;
        }
      }
      else if (TimestampIsReached(ESPNTPTimeStamp)) {
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

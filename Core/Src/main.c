/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "usart.h"
#include "rtc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gadget.h"
#include "microphone.h"
#include "I2CSensors.h"
#include "utils.h"
#include "measurement.h"
#include "globals.h"
#include "ESP.h"
#include "PowerUtils.h"
#include "usbd_cdc_if.h"
#include "statusCheck.h"
#include "RealTimeClock.h"
#include "sound_measurement.h"
#include "print_functions.h"
#include "sen5x.h"
#include "sgp40.h"
#include "wsenHIDS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    bool HT_measurementEnabled;
    bool VOC_measurementEnabled;
    bool PM_measurementEnabled;
    bool MIC_measurementEnabled;
} EnabledMeasurements;

typedef struct {
  bool HT_Present;
  bool VOC_Present;
  bool PM_Present;
  bool MIC_Present;
  bool ESP_Present;
}DevicePresent;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static EnabledMeasurements Sensor = {
    .HT_measurementEnabled = true,
    .VOC_measurementEnabled = true,
    .PM_measurementEnabled = true,
    .MIC_measurementEnabled = true
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  bool testDone = false;
  bool ESP_Programming = false;
  bool batteryEmpty = false;
  static uint8_t SGPstate;
  static uint8_t HIDSstate;
  bool waitforSamples = false;
  uint8_t hidscount = 0;
  uint8_t u1_rx_buff[16];  // rxbuffer for serial logger
  uint8_t RxData[UART_CDC_DMABUFFERSIZE] = {0};  //rx buffer for USB
  uint16_t IndexRxData = 0;
  uint32_t deviceTimeOut = 0;
  uint32_t LastRxTime = 0;
  uint32_t batteryReadTimer = 0;
  uint32_t timeReadTimer = 0;
  uint32_t sleepTime = 0;
  uint16_t size = 0;
  static EnabledMeasurements Sensor;
  static DevicePresent SensorProbe;

  Battery_Status charge;
  ESP_States ESP_Status;
  MicrophoneState mic_Status;
  extern DMA_HandleTypeDef hdma_spi2_rx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void SetTestDone(){
  testDone = true;
//  Info("testDone true in SetTestDone\r\n");
  HAL_Delay(1000);
  SetDBLED(false, false, true);
  SetStatusLED(4000, 4000, 3000);
  SetVocLED(4000, 4000, 3000);
  HAL_Delay(1000);
  SetDBLED(false, false, false);
  SetStatusLED(4000, 4000, 4000);
  SetVocLED(4000, 4000, 4000);
  InitDone();
}

void FlashLEDs(){
  for (uint8_t i=0; i<5 ; i++){
    SetDBLED(true, true, true);
    SetStatusLED(4000, 4000, 3000);
    SetVocLED(4000, 4000, 3000);
    HAL_Delay(250);
    SetLEDsOff();
    HAL_Delay(250);
  }
}

void ESP_Programming_Read_Remaining_DMA()
{
  //ESP programmer section

  if (LastRxTime != 0 && ESP_Programming)
  {
    if ((LastRxTime + 100) < HAL_GetTick()) //120
    {
      HAL_UART_DMAPause(&hlpuart1);
      size = __HAL_DMA_GET_COUNTER(hlpuart1.hdmarx);
      if (size > (UART_CDC_DMABUFFERSIZE / 2))
      {
        size = UART_CDC_DMABUFFERSIZE - size;
      }
      else
      {
        size = (UART_CDC_DMABUFFERSIZE / 2) - size;
      }
      if (size > 0)
      {
        CDC_Transmit_FS(&RxData[IndexRxData], size);
        LastRxTime = 0;
        IndexRxData += size;
      }
      HAL_UART_DMAResume(&hlpuart1);
    }
  }
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  // printf_USB("\r\nDMA counter Full: %d\r\nDMA size Full: %d\r\n", __HAL_DMA_GET_COUNTER(hlpuart1.hdmarx), size);
//  //printf_USB("DMA size Full: %d\r\n", size);
//
//  if (huart->Instance == LPUART1)
//  {
//    CDC_Transmit_FS(&RxData[IndexRxData], (UART_CDC_DMABUFFERSIZE / 2) - size);
//
//    size = 0;
//    IndexRxData = 0;
//    LastRxTime = HAL_GetTick();
//  }
//}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void testInit(){
  SensorProbe.HT_Present = false;
  SensorProbe.VOC_Present = false;
  SensorProbe.PM_Present = false;
  SensorProbe.MIC_Present = false;
  SensorProbe.ESP_Present = false;
}

bool GetPMSensorPresence(){
  return SensorProbe.PM_Present;
}

bool IsHTSensorEnabled() {
  return Sensor.HT_measurementEnabled;
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

void SetVOCSensorStatus(bool setting) {
  Sensor.VOC_measurementEnabled = setting;
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
  if(!SGP_DeviceConnected()) {
    SensorProbe.VOC_Present = false;
     Error("SGP device not connected!");
     Sensor.VOC_measurementEnabled = false;
  }
  else{
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
  }
  else {
    sen5x_Power_Off();      // switch off buck converter
    Debug("PM sensor not detected/connected.");
    SensorProbe.MIC_Present = false;
    Sensor.PM_measurementEnabled = false;
  }
  Info("SensorProbe.HT_Present: %s", SensorProbe.HT_Present?"yes":"no");
  Info("SensorProbe.VOC_Present: %s", SensorProbe.VOC_Present?"yes":"no");
  Info("SensorProbe.PM_Present: %s", SensorProbe.PM_Present?"yes":"no");
  Info("SensorProbe.MIC_Present: %s", SensorProbe.MIC_Present?"yes":"no");
  ESP_Init(espUart);
  Debug("Sensors initialized, probing ESP.");
}

void Device_Test(){
  if(!SensorProbe.MIC_Present){
    if(MIC_TestMeasurementDone()){
      //when this condition is met, the device is definite operational
//      Debug("MIC_TestMeasurementDone() is true");
      SensorProbe.MIC_Present = true;
      Sensor.MIC_measurementEnabled = true;
      SetStatusLED(LED_OFF, LED_ON, LED_OFF);
    }
    else{
      if (micSettlingComplete()) {
        // his has to be met first
//        Debug("micSettlingComplete() is true");
        Sensor.MIC_measurementEnabled = true;
        SetStatusLED(LED_ON, LED_OFF, LED_OFF);
      }
    }
  }
  if(!SensorProbe.ESP_Present){
    ESP_WakeTest();  // calls in ESP.c  back to SetESPMeasurementDone()
  }
  if((SensorProbe.ESP_Present && SensorProbe.MIC_Present) || TimestampIsReached(deviceTimeOut)){
    Info("Test completed");
    Info("ESP function: %s", SensorProbe.ESP_Present?"passed": "failed");
    Info("MIC function:%s", SensorProbe.MIC_Present?"passed": "failed");
    SetTestDone();
  }
}

bool AllDevicesReady() {
  if (TimestampIsReached(deviceTimeOut)) {
    if (HIDSstate == HIDS_STATE_WAIT) {
      Sensor.HT_measurementEnabled = false;
    }
    if (SGPstate == SGP_STATE_WAIT) {
      Sensor.VOC_measurementEnabled = false;
    }
    if (PMsamplesState == LIGHT_OUT) {
      Sensor.PM_measurementEnabled = false;
    }
    if (mic_Status == MIC_STATE_WAIT){
      Sensor.MIC_measurementEnabled = false;
    }
    if (ESP_Status == ESP_STATE_RESET) {
      return !(Sensor.HT_measurementEnabled || Sensor.VOC_measurementEnabled ||
          Sensor.PM_measurementEnabled || Sensor.MIC_measurementEnabled);
    }
  }
  return false;
}

void EnabledConnectedDevices() {
  if (SensorProbe.HT_Present) {
    Sensor.HT_measurementEnabled = true;
  }
  if (SensorProbe.VOC_Present) {
    Sensor.VOC_measurementEnabled = true;
  }
  if (SensorProbe.PM_Present) {
    Sensor.PM_measurementEnabled = true;
  }
  if (SensorProbe.MIC_Present) {
    Sensor.MIC_measurementEnabled = true;
  }
}

void DisableConnectedDevices() {
  if (SensorProbe.HT_Present) {
    Sensor.HT_measurementEnabled = false;
  }
  if (SensorProbe.VOC_Present) {
    Sensor.VOC_measurementEnabled = false;
  }
  if (SensorProbe.PM_Present) {
    Sensor.PM_measurementEnabled = false;
  }
  if (SensorProbe.MIC_Present) {
    Sensor.MIC_measurementEnabled = false;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART4_UART_Init();
  MX_ADC_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  MX_LPUART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // General TODO 's
	/*
	 * : Put SSID in EEPROM
	 * : Turn on heater if humidity is too high
	 * : LEDs indicator for air quality
	 * : Default network: Sensor community
	 * : Different modes for outside and inside (check solar or check LED on/off mode?)
	 * : Add CLI via usb-c
	 * : Network not found? Sleep
	 */
  GPIO_InitPWMLEDs(&htim2, &htim3);
  if(UserButton_Pressed()){
    EnableESPProg();
    ESP_Programming = true;
  }
  //uint32_t LedBlinkTimestamp = HAL_GetTick() + LED_BLINK_INTERVAL;
  SetVerboseLevel(VERBOSE_ALL);
  BinaryReleaseInfo();
  HAL_UART_Receive_IT(&huart1, u1_rx_buff, 1);
  InitClock(&hrtc);
  Debug("Clock init done");

  if (!soundInit(&hdma_spi2_rx, &hi2s2, &htim6, DMA1_Channel4_5_6_7_IRQn)) {
    errorHandler(__func__, __LINE__, __FILE__);
  }

//  Gadget_Init(&hi2c1, &hi2s2, &huart4, &hadc);
  Device_Init(&hi2c1, &hi2s2, &hadc, &huart4);
  deviceTimeOut = HAL_GetTick() + 5000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if(TimestampIsReached(batteryReadTimer)){
      charge = Battery_Upkeep();
      batteryReadTimer  = HAL_GetTick() + 50000;
        showTime();
    }

    //==== disable for power measurements in test condition
        if(charge == BATTERY_LOW || charge == BATTERY_CRITICAL){
          FlashLEDs();
        }
        if(charge == BATTERY_CRITICAL && ESP_Status == ESP_STATE_RESET){
          batteryEmpty = true;
          Enter_Standby_Mode(); // we are going in deep sleep, nearly off and no wakeup from RTC
        }
        else{
          batteryEmpty = false;
        }
    //====

    if (!usbPluggedIn) {
      if (!userToggle && AllDevicesReady() && ESPTransmitDone) {     // check if all sensors are ready
//        Debug("SensorProbe.ESP_Present?WAIT_WITH_PM:WAIT_WITHOUT_PM => %d", SensorProbe.PM_Present?WAIT_WITH_PM:WAIT_WITHOUT_PM);
        EnabledConnectedDevices();
        Enter_Stop_Mode(SensorProbe.PM_Present?WAIT_WITH_PM:WAIT_WITHOUT_PM);
        deviceTimeOut = HAL_GetTick() + 3000;
      }
    }
    if(testDone && !ESP_Programming && !batteryEmpty){
      if (SGPstate != SGP_STATE_START_MEASUREMENTS && SGPstate != SGP_STATE_WAIT_FOR_COMPLETION && Sensor.HT_measurementEnabled) {
        HIDSstate = HIDS_Upkeep();
      }
      if (HIDSstate != HIDS_STATE_START_MEASUREMENTS && HIDSstate != HIDS_STATE_WAIT_FOR_COMPLETION && Sensor.VOC_measurementEnabled) {
        SGPstate = SGP_Upkeep();
      }
      if (Sensor.MIC_measurementEnabled) {
        mic_Status = Mic_Upkeep();
      }
      if(((charge > BATTERY_LOW) || (charge == USB_PLUGGED_IN)) && Sensor.PM_measurementEnabled) {
        if  (charge > BATTERY_LOW) {
          sen5x_statemachine(0);
        }
        else {
          if (charge == USB_PLUGGED_IN) {
            sen5x_statemachine(USB_PLUGGED_IN);
          }
          else  {
            Info("Battery level insufficient for sen5x operation");
          }
        }
      }
      ESP_Status = ESP_Upkeep();
    }
    if(!testDone && !ESP_Programming && !batteryEmpty){
      Device_Test();  // for device with startup time
    }
    configCheck();


    //    if(TimestampIsReached(LedBlinkTimestamp)) {
    // Red LED
//
//      LedBlinkTimestamp = HAL_GetTick() + LED_BLINK_INTERVAL;
//    }

    // Optional colours:
    // Red, Yellow, Magenta, White, Cyan, Blue, Green.  black ;)

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Provide a print interface for print_functions.
void printString(const char * str, uint16_t length)
{
    HAL_UART_Transmit(&huart1, (uint8_t*) str, length, 0xFFFF);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart1, u1_rx_buff, 1);
  switch (u1_rx_buff[0]){
    case (uint8_t)'a':
      printf("VerboseLevel set to all\r\n");
      SetVerboseLevel(VERBOSE_ALL);
      break;
    case (uint8_t)'f':
      forceNTPupdate();  // sync the time now
    break;
    case (uint8_t)'i':
      printf("VerboseLevel set to info\r\n");
      SetVerboseLevel(VERBOSE_INFO);
      break;
    case (uint8_t)'m':
      printf("VerboseLevel set to minimal\r\n");
      SetVerboseLevel(VERBOSE_MINIMAL);
      break;
    case (uint8_t)'n':
      printf("VerboseLevel set to none\r\n");
      SetVerboseLevel(VERBOSE_NONE);
      break;
    case (uint8_t)'s':
      sen5xReadTimer = HAL_GetTick();  // on request fire up the sen5x
      break;
    case (uint8_t)'t':
      showTime(); // show me the current time
      break;
    default:
      Error("Error unknown request from Serial UART1 (TTY)\r\n");
      printf("Possible commands:\r\n\r\n");
      printf("a - VerboseLevel set to all\r\n");
      printf("f - Force NTP time synchronization\r\n");
      printf("i - VerboseLevel set to info\r\n");
      printf("m - VerboseLevel set to minimal\r\n");
      printf("n - VerboseLevel set to none\r\n");
      printf("s - Start particle measurement\r\n");
      printf("t - Show actual systemtime\r\n");
  break;
  }
  HAL_UART_Receive_IT(&huart1, u1_rx_buff, 1); //Re-arm the interrupt
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  if (GPIO_Pin == BOOT0_Pin) {
    setuserToggle();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  Error("Trapped in Error_Handler, wait for reset");
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

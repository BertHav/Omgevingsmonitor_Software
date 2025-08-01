/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef enum {
CLEAR,
DO_PWR_MAIL,
DONE
}PWR_MAIL_Mode;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint32_t deviceTimeOut;
extern uint8_t MICstate;
extern uint8_t ESPstate;
extern uint8_t sendpwremail;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ESP32_EN_Pin GPIO_PIN_0
#define ESP32_EN_GPIO_Port GPIOH
#define ESP32_BOOT_Pin GPIO_PIN_1
#define ESP32_BOOT_GPIO_Port GPIOH
#define MCU_LED_B_R_Pin GPIO_PIN_0
#define MCU_LED_B_R_GPIO_Port GPIOA
#define MCU_LED_B_G_Pin GPIO_PIN_2
#define MCU_LED_B_G_GPIO_Port GPIOA
#define MCU_LED_B_B_Pin GPIO_PIN_3
#define MCU_LED_B_B_GPIO_Port GPIOA
#define VusbDetect_Pin GPIO_PIN_7
#define VusbDetect_GPIO_Port GPIOA
#define BatMeasure_Pin GPIO_PIN_4
#define BatMeasure_GPIO_Port GPIOC
#define SolarMeasure_Pin GPIO_PIN_5
#define SolarMeasure_GPIO_Port GPIOC
#define MCU_LED_C_R_Pin GPIO_PIN_0
#define MCU_LED_C_R_GPIO_Port GPIOB
#define MCU_LED_A_R_Pin GPIO_PIN_6
#define MCU_LED_A_R_GPIO_Port GPIOC
#define MCU_LED_A_G_Pin GPIO_PIN_7
#define MCU_LED_A_G_GPIO_Port GPIOC
#define MCU_LED_A_B_Pin GPIO_PIN_8
#define MCU_LED_A_B_GPIO_Port GPIOC
#define Charger_status_Pin GPIO_PIN_9
#define Charger_status_GPIO_Port GPIOC
#define Wireless_PSU_PG_Pin GPIO_PIN_8
#define Wireless_PSU_PG_GPIO_Port GPIOA
#define BOOT0_Pin GPIO_PIN_15
#define BOOT0_GPIO_Port GPIOA
#define BOOT0_EXTI_IRQn EXTI4_15_IRQn
#define Wireless_PSU_EN_Pin GPIO_PIN_11
#define Wireless_PSU_EN_GPIO_Port GPIOC
#define Boost_Enable_Pin GPIO_PIN_12
#define Boost_Enable_GPIO_Port GPIOC
#define User_Button_Pin GPIO_PIN_2
#define User_Button_GPIO_Port GPIOD
#define MCU_LED_C_G_Pin GPIO_PIN_8
#define MCU_LED_C_G_GPIO_Port GPIOB
#define MCU_LED_C_B_Pin GPIO_PIN_9
#define MCU_LED_C_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SRC_VERSION "5.00 BH"
#define CURRENT_WEEK "2025w31-2"  // -x is day number of week i.e. Monday = 1

// Mail variables. Uncomment the next line and a mail will be sent at
// a nearly dead battery event to the mailapi of smtp2go
#define USE_MAIL

//#define STLINK_V3PWR true

#define WAIT_WITH_PM 860
#define WAIT_WITHOUT_PM 880
#define WAIT_WITH_PM_SUN 260 // with sun shining and battery full loaded with sen5x
#define WAIT_WITHOUT_PM_SUN 280  // with sun shining and battery full loaded
#define BATTERY_READ_CYCLE 50000  // About once a minute
#define DEVICE_INIT_TIMEOUT 25000 // This is the initial time out for sensors during startup
#define DEVICE_TIMEOUT 4500 // during battery power the device become about 4,5 second to take 3 samples after they are disabled


#define UART_CDC_DMABUFFERSIZE 32
#define LED_BLINK_INTERVAL 1000
void SetTestDone();
void SystemClock_Config(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

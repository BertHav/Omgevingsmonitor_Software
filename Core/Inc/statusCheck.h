/*
 * statusCheck.h
 *
 *  Created on: Sep 17, 2024
 *      Author: Danny
 */

#ifndef INC_STATUSCHECK_H_
#define INC_STATUSCHECK_H_

#include "PowerUtils.h"
#include "ESP.h"
#include "gpio.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_gpio.h"


#define LED_OFF 4000
#define LED_ON 3000
#define LED_RED 1
#define LED_GREEN 2
typedef enum {
  BATTERY_CRITICAL,
  BATTERY_LOW,
  BATTERY_GOOD,
  BATTERY_FULL,
  USB_PLUGGED_IN
  }Battery_Status;

extern bool usbPluggedIn;
extern bool userToggle;

void setuserToggle(void);
void configCheck();
void SetAllREDLED();
void WalkAllRedLED();
void SetAllBlueLED();
Battery_Status Battery_Upkeep();
Battery_Status powerCheck();
Battery_Status batteryChargeCheck();
void powerDisplay(Battery_Status status);
void SetStatusLED(uint16_t red, uint16_t green, uint16_t blue);   // Sets Status LED to (RGB) color
uint16_t Calculate_LED_ON();
void SetDBLED(bool red, bool green, bool blue);       // Sets dB LED to (RGB) color
void SetVocLED(uint16_t red, uint16_t green, uint16_t blue);      // Sets VOC LED to (RGB) color
void SetLEDsOff(void);
void SetVOCindicator(uint16_t VOCi);
void InitDone();
void SetMeasurementIndicator();
void ResetMeasurementIndicator();
void SetMICIndicator();
void ResetMICIndicator();
void SetESPIndicator();
void ResetESPIndicator();
void SetPMIndicator();
void ResetPMIndicator();
#endif /* INC_STATUSCHECK_H_ */

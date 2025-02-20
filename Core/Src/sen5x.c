/*
 * sen5x.c
 *
 *  Created on: Dec 1, 2024
 *      Author: Bert Havinga, parts from KITT and Sensirion grouped together
 */
#include "stm32l0xx_hal.h"
#include "sen5x.h"
#include "main.h"
#include "sen5x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "statusCheck.h"
#include "utils.h"
#include "i2c.h"
#include "RealTimeClock.h"
#include "ESP.h"
#include "measurement.h"
#include "ssd1306_128x64_i2c.h"
#include "display.h"

bool fanCleaningDone = false;
bool sen5x_On = false;
bool VOCNOx = false;
static bool sen5x_Enable = false;
unsigned char product_name[8];
uint8_t sen5xSamples = 0;
uint8_t sen5xErrors = 0;
/*
static uint16_t pm2p5;
static uint16_t pm10p0;
static uint16_t s5xvoci;
static uint16_t s5xnoxi;
*/
uint32_t sen5xReadTimer = 0;
sen5x_states PMsamplesState = LIGHT_OUT;
SEN5X_DateTypeDef sen5x_data;

bool sen5x_Get_sen5x_enable_state() {
  return sen5x_Enable;
}

void sen5x_Set_sen5x_state(bool status) {
  sen5x_Enable = status;
}

void setsen5xReadTimer(uint32_t delayms) {
  sen5xReadTimer = HAL_GetTick() + delayms;
}

void setsen5xSamplecounter(uint8_t samples) {
  sen5xSamples = 0;
}

bool sen5x_enable(uint32_t sleepTime) {
  if (IsPMSensorEnabled()) {
    if (batteryChargeCheck() == BATTERY_FULL) {
      sen5x_Enable = true;
    }
    else {
    sen5x_Enable = !sen5x_Enable;
    }
    if (sen5x_Enable) {
      setsen5xReadTimer(0);
    }
    else {
      //The ticker starts after 880*100, effective, this cycle the sen5x device will not start
      setsen5xReadTimer(HAL_GetTick() + (sleepTime*100));
    }
    Info("This cycle the sen5x is: %s", sen5x_Enable?"enabled":"disabled");
  }
  else {
    Info("sen5x measurement is disabled");
  }
  PMsamplesState = LIGHT_OUT; // just to be sure if USB_power is disconnected during measurement cycle
  return sen5x_Enable;
}

void sen5x_Power_On(void) {
  HAL_GPIO_WritePin(Boost_Enable_GPIO_Port, Boost_Enable_Pin, GPIO_PIN_SET);
  Debug("executing sen5x_Power_On");
  HAL_Delay(150);
  if (sen5x_device_reset()) {
    Error("sen5x device reset error after power on");
  }
  else {
    sen5x_On = true;
  }
#ifdef SSD1306
  if (userToggle || Check_USB_PowerOn()) {
    displayStart();
  }
#endif
}

void sen5x_Power_Off(void) {
  if (VOCNOx) {
    Debug("VOC and NOx measurement enabled, no power off");
  }
  else {
#ifdef SSD1306
    if (SSD1306detected && (usbPluggedIn || userToggle)) {
      Info("Display detected and USB power or userToggle enabled");
    }
    else {
      ssd1306_powerOff();
      Debug("Display switched off");
#endif
      Debug("executing sen5x_Power_Off");
      HAL_GPIO_WritePin(Boost_Enable_GPIO_Port, Boost_Enable_Pin, GPIO_PIN_RESET);
      sen5x_On = false;
#ifdef SSD1306
    }
#endif
  }
}

void reset_fanCleaningDone(void) {
  fanCleaningDone = false;
}

int16_t probe_sen5x(void) {
  int16_t error = 0;
  unsigned char serial_number[32];
  uint8_t serial_number_size = 32;
  uint8_t product_name_size = 32;
  sen5x_Power_On();  // switch buck converter
  error = sen5x_device_reset();
  if (error) {
      Error("Error executing sen5x_device_reset(): %i", error);
      return error;
  }
  error = sen5x_get_serial_number(serial_number, serial_number_size);
  if (error) {
      printf("Error executing sen5x_get_serial_number(): %i\r\n", error);
      return error;
  } else {
      printf("Serial number: %s\r\n", serial_number);
  }
  error = sen5x_get_product_name(product_name, product_name_size);
  if (error) {
      printf("Error executing sen5x_get_product_name(): %i\r\n", error);
      return error;
  } else {
      printf("Product name: %s\r\n", product_name);
  }

  uint8_t firmware_major;
  uint8_t firmware_minor;
  bool firmware_debug;
  uint8_t hardware_major;
  uint8_t hardware_minor;
  uint8_t protocol_major;
  uint8_t protocol_minor;
  error = sen5x_get_version(&firmware_major, &firmware_minor, &firmware_debug,
                            &hardware_major, &hardware_minor, &protocol_major,
                            &protocol_minor);

  if (error) {
      printf("Error executing sen5x_get_version(): %i\r\n", error);
      return error;
  } else {
      printf("Firmware: %u.%u, Hardware: %u.%u\r\n", firmware_major,
             firmware_minor, hardware_major, hardware_minor);
  }

// set a temperature offset - supported by SEN54 and SEN55 sensors
//
// By default, the temperature and humidity outputs from the sensor
// are compensated for the modules self-heating. If the module is
// designed into a device, the temperature compensation might need
// to be adapted to incorporate the change in thermal coupling and
// self-heating of other device components.
//
// A guide to achieve optimal performance, including references
// to mechanical design-in examples can be found in the app note
// “SEN5x – Temperature Compensation Instruction�? at www.sensirion.com.
// Please refer to those application notes for further information
// on the advanced compensation settings used in
// `sen5x_set_temperature_offset_parameters`,
// `sen5x_set_warm_start_parameter` and
// `sen5x_set_rht_acceleration_mode`.
//
// Adjust temp_offset in degrees celsius to account for additional
// temperature offsets exceeding the SEN module's self heating.
  float temp_offset = 0.0f;
  int16_t default_slope = 0;
  uint16_t default_time_constant = 0;
  error = sen5x_set_temperature_offset_parameters(
      (int16_t)(200 * temp_offset), default_slope, default_time_constant);
  if (error) {
      Error("Error executing sen5x_set_temperature_offset_parameters(): %i", error);
      return error;
  } else {
      Info("Temperature Offset set to %.2f °C (SEN54/SEN55 only)", temp_offset);
  }
  if (!Check_USB_PowerOn()) {
    sen5x_Power_Off();
    sen5xReadTimer  = HAL_GetTick() + 2000; // after 25 second first measurement
  }
  else {
    sen5xReadTimer  = HAL_GetTick() + 28000; // after 25 second first measurement
  }
  return error;
}

int16_t sen5x_lightup_measurement(void) {
// Start Measurement
  int16_t error = 0;
//  Debug("entering sen5x_lightup_measurement");
  if (VOCNOx) {
    Info("Continous measurement without PM is active");
    error =sen5x_start_measurement_without_pm();
  }
  else {
    Info("Measurement with PM is active");
    error = sen5x_start_measurement(); // start full measurement mode
  }
  if (error) {
    Error("Error executing sen5x_lightup_measurement(): %i", error);
  }
  else {
    showTime();
    Info("sen5x_start_measurement executed");
  }
  return error;
}

int16_t sen5x_extinguish_measurement(void) {
  int16_t error = 0;
  if (VOCNOx) {
    Info("Continuous VOC & NOx is active, sensor not powered off");
    Info("PM measurement is disabled");
    error =sen5x_start_measurement_without_pm();
    if (error) {
      Error("Error executing switching to measurement without PM code: %i", error);
    }
  }
  else {
    error = sen5x_stop_measurement();
    showTime();
    Info("sen5x_stop_measurement executed");
    if (error) {
      Error("Error executing sen5x_stop_measurement(): %i", error);
    }
  }
  return error;
}

int16_t sen5x_read_measurement(SEN5X_DateTypeDef* sen5x_data) {
  int16_t error = 0;
  uint16_t mass_concentration_pm1p0;
  uint16_t mass_concentration_pm2p5;
  uint16_t mass_concentration_pm4p0;
  uint16_t mass_concentration_pm10p0;
  int16_t ambient_humidity;
  int16_t ambient_temperature;
  int16_t voc_index;
  int16_t nox_index;

  error = sen5x_read_measured_values(
          &mass_concentration_pm1p0, &mass_concentration_pm2p5,
          &mass_concentration_pm4p0, &mass_concentration_pm10p0,
          &ambient_humidity, &ambient_temperature, &voc_index, &nox_index);
  if (error) {
    return error;
  }
  // it is a pitty that sensirion does not structures in there API
  sen5x_data->mass_concentration_pm1p0 = mass_concentration_pm1p0;
  sen5x_data->mass_concentration_pm2p5 = mass_concentration_pm2p5;
  sen5x_data->mass_concentration_pm4p0 = mass_concentration_pm4p0;
  sen5x_data->mass_concentration_pm10p0 = mass_concentration_pm10p0;
  sen5x_data->ambient_humidity = ambient_humidity;
  sen5x_data->ambient_temperature = ambient_temperature;
  sen5x_data->voc_index = voc_index;
  sen5x_data->nox_index = nox_index;
  return 0;
}

void sen5x_printvalues(void) {
  if (sen5x_data.mass_concentration_pm1p0 != 0xFFFF) {
      printf("Mass concentration pm1p0: %.1f µg/m³\r\n", sen5x_data.mass_concentration_pm1p0 / 10.0f);
  }
  if (sen5x_data.mass_concentration_pm2p5 != 0xFFFF) {
        printf("Mass concentration pm2p5: %.1f µg/m³\r\n", sen5x_data.mass_concentration_pm2p5 / 10.0f);
  }
  if (sen5x_data.mass_concentration_pm4p0 != 0xFFFF) {
        printf("Mass concentration pm4p0: %.1f µg/m³\r\n", sen5x_data.mass_concentration_pm4p0 / 10.0f);
  }
  if (sen5x_data.mass_concentration_pm10p0 != 0xFFFF) {
        printf("Mass concentration pm10p0: %.1f µg/m³\r\n", sen5x_data.mass_concentration_pm10p0 / 10.0f);
  }
  if (sen5x_data.ambient_humidity != 0x7fff) {
        printf("sen5x Ambient humidity: %.1f %%RH\r\n", sen5x_data.ambient_humidity / 100.0f);
  }
  if (sen5x_data.ambient_temperature != 0x7fff) {
        printf("sen5x Ambient temperature: %.1f °C\r\n", sen5x_data.ambient_temperature / 200.0f);
  }
  if (sen5x_data.voc_index != 0x7fff) {
        printf("sen55 VOC index: %d\r\n", sen5x_data.voc_index / 10);
  }
  if (sen5x_data.nox_index != 0x7fff) {
        printf("sen55 NOx index: %d\r\n", sen5x_data.nox_index / 10);
  }
}

void sen5xStore() {
  if (sen5x_data.mass_concentration_pm2p5 != 0xFFFF) {
    setPM2p5(sen5x_data.mass_concentration_pm2p5);
  }
  if (sen5x_data.mass_concentration_pm10p0 != 0xFFFF) {
    setPM10(sen5x_data.mass_concentration_pm10p0);
  }
  if (((product_name[4] == '4') || (product_name[4] == '5'))) {
    if (!VOCNOx || usbPluggedIn) {
      if (sen5x_data.voc_index != 0x7fff) {
        SetVOCindicator(sen5x_data.voc_index / 10);
        setVOC(sen5x_data.voc_index / 10);
      }
      if (sen5x_data.nox_index != 0x7fff) {
        setNOx(sen5x_data.nox_index / 10);
      }
    }
  }
    //    Debug("pm2p5 = %d, pm10p0 = %d, s5xvoci = %d, s5xnoxi = %d", pm2p5, pm10p0, s5xvoci, s5xnoxi);
}
/**
 * Release all resources initialized by sensirion_i2c_hal_init().
*/

//void sensirion_i2c_hal_free(void) {
//}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint16_t count) {
  return (int8_t)HAL_I2C_Master_Receive(&hi2c2, (uint16_t)(address << 1), data, count, 100);
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint16_t count) {
  return (int8_t)HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)(address << 1), (uint8_t*)data, count, 100);
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * @param useconds the sleep time in microseconds
 */

void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
  uint32_t msec = useconds / 1000;
  if (useconds % 1000 > 0) {
    msec++;
  }
  HAL_Delay(msec);
}

bool sen5x_check_for_errors(void){
  uint32_t device_status = 0;
  if (sen5x_read_device_status(&device_status)) {
    Error("Error reading sen5x device status register");
    return true;
  }
  if (device_status == 0) {
    return false;
  }
  if (device_status & SEN5X_FAN_SPEED_ERROR) {
    Debug("sen5x Fan speed out of range");
  }
  if (device_status & SEN5X_FAN_CLEANING) {
    Debug("sen5x Fan cleaning active");
  }
  if (device_status & SEN5X_GAS_SENSOR_ERROR) {
    Debug("sen5x Gas sensor error (VOC & NOx)");
  }
  if (device_status & SEN5X_RHT_ERROR) {
    Debug("sen5x RHT communication error");
  }
  if (device_status & SEN5X_LASER_ERROR) {
    Debug("sen5x Laser failure");
  }
  if (device_status & SEN5X_FAN_BLOCKED_ERROR) {
    Debug("sen5x Fan failure, fan is mechanically blocked or broken.");
  }
  return 1;
}

void set_light_on_state(void) {
  if (!sen5x_On) {
    sen5x_Power_On();
    Debug("sen5x powered on, warming up for 30 sec.");
  }
  else {
    sen5xReadTimer = HAL_GetTick();
    Debug("sen5x already powered");
  }
  if (sen5x_lightup_measurement()) {  // start selected measurement mode
    Error("Error executing sen5x_lightup_measurement()");
  }
  PMsamplesState = CHECK_SEN5X;
}

void sen5x_statemachine() {
  bool data_ready = false;
  if (TimestampIsReached(sen5xReadTimer)) {
    switch (PMsamplesState) {
    case S5X_DISABLED:
      Error("sen5x device is disabled due to too many errors");
      SetPMSensorStatus(false);
      DisablePMSensor();
      sen5xReadTimer = HAL_GetTick() + SEN5X_DISPLAY_DISABLED_MSG; //some more less then an hour a message when continue operated.
      break;
    case LIGHT_OUT:
      sen5xReadTimer = HAL_GetTick() + SEN5X_STARTUP_DELAY; // wait about 30s when started up
      set_light_on_state();
      break;
    case CHECK_SEN5X:
      PMsamplesState = LIGHT_ON;
      if (sen5xErrors > 5) {
        PMsamplesState = S5X_DISABLED;
        sen5x_Power_Off();
        }
      else {
        if (sen5x_check_for_errors()) {
          if (sen5x_device_reset()) {
            Error("Error resetting sen5x");
            sen5xErrors++;
            PMsamplesState = CHECK_SEN5X;
          }
          else {
            Info("sen5x reset executed");
          }
          sen5xReadTimer = HAL_GetTick();
        }
        else {
          if (sen5xErrors != 0) {
            sen5xErrors = 0;
            Debug("sen5xErrors reset");
          }
        }
      }
      break;
    case LIGHT_ON:
      sen5x_read_data_ready(&data_ready);  // is new data ready in the sensor module?
      if (data_ready) {
        SetPMIndicator();
        if (sen5x_read_measurement(&sen5x_data)) {
          Error("Error executing sen5x_read_measured_values()");
          sen5xErrors++;
        }
        sen5xSamples++;
        if (sen5xSamples == 31) { // about two times a minute
          sen5xSamples = 0;  // enable display on serial
        }
        if (sen5xSamples > 1) {
          sen5xStore();
        }
        if (sen5xSamples == 2) { // take 2 samples, show 1 sample before we continue in the state machine
#ifndef STLINK_V3PWR
          sen5x_printvalues(); // print the values
#else
          Info("!!==Values are bogus, voltage for sen5x is out of range when powered by the STLINK_V3PWR==!!");
          sen5x_printvalues(); // print the values
          Info("!!==Values are bogus, voltage for sen5x is out of range when powered by the STLINK_V3PWR==!!");
#endif
//          HAL_Delay(1000);
        }
      }
      if (usbPluggedIn || (sen5xSamples > 1)) {
        PMsamplesState = CLEAN_FAN;
      }
      break;
    case CLEAN_FAN:
      // start the cleaning procedure once a week
      if ((RTC_GetWeekday() == MONDAY ) && !fanCleaningDone) {
        sen5x_start_fan_cleaning();
        Info("executing fan cleaning");
        sen5xReadTimer = HAL_GetTick() + SEN5X_FAN_CLEANING_PERIOD;  // fan cleaning takes 10 seconds
        fanCleaningDone = true;
        sen5x_lightup_measurement();
      }
      PMsamplesState = SAMPLES_TAKEN;
      break;
    case SAMPLES_TAKEN:
      if (!usbPluggedIn && !userToggle) {
        if (sen5x_extinguish_measurement()) {
          Error("Error executing sen5x_extinguish_measurement()");
        }
        sen5xSamples = 0;
        sen5x_Power_Off();
//        if (!userToggle) {
          SetPMSensorStatus(false);
//        }
        PMsamplesState = LIGHT_OUT;
      }
      else {
        PMsamplesState = CHECK_SEN5X;
      }
      ResetPMIndicator();
      sen5xReadTimer = HAL_GetTick() + SEN5X_SAMPLE_INTERVAL;
    }
  }
}

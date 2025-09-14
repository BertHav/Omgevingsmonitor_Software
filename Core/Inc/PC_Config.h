#ifndef PC_CONFIG_H
#define PC_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_uart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define MAX_PAYLOAD_SIZE 256
#define HEADER_SIZE 3
#define CRC_SIZE 2
#define TOTAL_BUFFER_SIZE (MAX_PAYLOAD_SIZE + HEADER_SIZE + CRC_SIZE)

#define ERROR_CMD 255

#define PREAMBLE "#"
#define PREAMBLE_F "$"
#define PREAMBLE_S "S"
#define PREAMBLE_L "L"
#define PREAMBLE_B "B"

#define BoxConfigCmd 0
#define TempConfigCmd 1
#define HumidConfigCmd 2
#define NOxIndexConfigCmd 3
#define VocIndexConfigCmd 4
#define dBAConfigCmd 5  // was 6
#define dBcConfigCmd 6
#define PM2ConfigCmd 7
#define PM10ConfigCmd 8
#define BatVoltConfigCmd 9
#define SolVoltConfigCmd 10
#define ChargerStatConfigCmd 11
#define CustomNameConfigCmd 12
#define SSIDConfigCmd 13
#define PasswordConfigCmd 14
#define AHTTempConfigCmd 15
#define AHTHumidConfigCmd 16
#define BMPTempConfigCmd 17
#define ENSAQIConfigCmd 18
#define ENSTVOCConfigCmd 19
#define ENSeCO2ConfigCmd 20
#define PM1ConfigCmd 21  // was 13
#define PM4ConfigCmd 22  // was 14
#define SEN55TempConfigCmd 23
#define SEN55HumidConfigCmd 24
#define SendFromNameConfigCmd 25
#define SendToNameConfigCmd 26
#define MailAPIKeyConfigCmd 27
#define hPaConfigCmd 28 // was 5
#define UptimeConfigCmd 29  // was 28
#define URLToUploadConfigCmd 30
#define clearDefsCmd 31
#define maxCmd 32
#define ClearConfigCmd 253
#define ClearEepromCmd 254
#define ErrorCmd 255

typedef struct
{
    uint8_t     Command;
    uint8_t     PayloadLength;
    uint8_t     Payload[MAX_PAYLOAD_SIZE];
    uint16_t    Crc;
} Receive_MSG;

void Process_PC_Config(uint8_t* data);//, uint16_t length);
void PC_show_Keys();
bool Process_USB_input(uint8_t* data);
void Create_Message(uint8_t command, uint8_t payload[], uint8_t payloadLength);
void printf_USB(const char* message, ...);

#endif

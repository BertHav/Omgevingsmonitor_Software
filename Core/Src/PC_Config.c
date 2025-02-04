#include <eeprom.h>
#include <ctype.h>
#include "../Inc/PC_Config.h"
#include "../Inc/Config.h"
#include "utils.h"
#include "ESP.h"
#include "statusCheck.h"

Receive_MSG received;
uint8_t result = 0;
uint32_t formerlength = 0;

static uint16_t CRC16_ARC(uint8_t data[], uint16_t size);
void ProcessCmd(Receive_MSG msg);

//*************************************Preview Uart Config message**********************************
// #, Command, Payload lengte, Payload, MSB CRC16_ARC, LSB CRC16_ARC
// Command value:   0 = ...
//                  1 = ...
//**************************************************************************************************

void Process_PC_Config(uint8_t* data) //, uint16_t length)
{
    uint32_t length = GetUsbRxDataSize();
    if (length > 5)
    {
        uint8_t* message = (unsigned char*)strstr((const char*)data, PREAMBLE);
        if(message != NULL)// && strlen((const char*)message) > 5)
        {
            received.Command = message[1];
            received.PayloadLength = message[2];
            if (length >= (uint32_t)(received.PayloadLength + HEADER_SIZE + CRC_SIZE))
            {
                memcpy(received.Payload, &message[3], received.PayloadLength);
                received.Crc = message[3 + received.PayloadLength] << 8 | message[3 + received.PayloadLength + 1];

                if (received.Crc != CRC16_ARC(message, received.PayloadLength + 3))
                {
                    Create_Message(ERROR, received.Payload, received.PayloadLength);
                    //Handel het foutief ontvangen bericht af
                }
                else
                {
                    //Handel het correct ontvangen bericht af
                    ProcessCmd(received);
                    Create_Message(received.Command, received.Payload, received.PayloadLength);
                }
                ResetUsbRxDataSize();
                return;
            }
            GetUsbRxNextChunk(length);
        }
        else
        {
            ResetUsbRxDataSize();
        }
    }
}

void ProcessCmd(Receive_MSG msg)
{
    switch (msg.Command)
    {
        case BoxConfigCmd:  // 0
            WriteUint8ArrayEepromSafe(BoxConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case TempConfigCmd:  // 1
            WriteUint8ArrayEepromSafe(TempConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case HumidConfigCmd:  // 2
            WriteUint8ArrayEepromSafe(HumidConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case NOxIndexConfigCmd:  // 3
            WriteUint8ArrayEepromSafe(NOxIndexConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case VocIndexConfigCmd:  // 4
            WriteUint8ArrayEepromSafe(VocIndexConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case dBaConfigCmd:  // 5 wille be airpressure ??
            WriteUint8ArrayEepromSafe(dBaConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case dBcConfigCmd: // 5 will be dBAConfigCMD
            WriteUint8ArrayEepromSafe(dBAConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case CustomNameConfigCmd:  // 7
            WriteUint8ArrayEepromSafe(CustomNameConfigAddr, msg.Payload, msg.PayloadLength, CustomNameMaxLength);
        break;
        case SolVoltConfigCmd:  // 8
            WriteUint8ArrayEepromSafe(SolVoltConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case BatVoltConfigCmd:  // 9
            WriteUint8ArrayEepromSafe(BatVoltConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case PM10ConfigCmd:  // :
            WriteUint8ArrayEepromSafe(PM10ConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case PM2ConfigCmd:  // ;
            WriteUint8ArrayEepromSafe(PM2ConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case ClearConfigCmd:
            ClearEEprom(EEPromStartAddr, ConfigSize);
        break;
        case ClearEepromCmd:
        {
            uint16_t size = ((uint16_t)msg.Payload[0] << 8 | msg.Payload[1]);
            if (size < EEPROM_SIZE)
            {
                ClearEEprom(EEPromStartAddr, size);
            }
            else
            {
                ClearEEprom(EEPROM_START, EEPROM_SIZE);
            }
            break;
        }
    }

}

void Create_Message(uint8_t command, uint8_t *payload, uint8_t payloadLength)
{
    static uint8_t message[TOTAL_BUFFER_SIZE];
    message[0] = (uint8_t)PREAMBLE[0];
    message[1] = command;
    message[2] = payloadLength;
    memcpy(&message[3], payload, payloadLength);
    uint16_t crcIndex = (HEADER_SIZE + payloadLength);
    uint16_t calculatedCRC = CRC16_ARC(message, crcIndex);
    message[crcIndex] = calculatedCRC >> 8;
    message[crcIndex + 1] = calculatedCRC & 0xFF;
    CDC_Transmit_FS(message, (crcIndex + 2));
}

static uint16_t CRC16_ARC(uint8_t data[], uint16_t size)
{
    uint16_t crc = 0;

    for (int i = 0; i < size; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc; 
}

void printf_USB(const char* message, ...)
{
  char string[150];
  va_list args;
  va_start(args, message);
  vsprintf(string, message, args);
  va_end(args);
  CDC_Transmit_FS((uint8_t*)string, strlen(string));
}

void PC_selectout(char *msg, bool usb_out) {
  if (usb_out){
    printf_USB(msg);
  }
  else {
    printf(msg);
  }
  HAL_Delay(10); //wait for the host poll of the USB buffer
}

void PC_show_Keys() {
  static bool usb_out= false;
  static uint8_t boxConfig[IdSize];
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

  static char Buffer[25];
  static char msg[64];
  if(Check_USB_PowerOn()){
    usb_out = true;
  }
  else {
    usb_out = false;
  }
  sprintf(msg, "Overview of stored keys:\r\n");
  PC_selectout(&msg[0], usb_out);
  ReadUint8ArrayEEprom(BoxConfigAddr, boxConfig, IdSize);
  uint8ArrayToString(Buffer, boxConfig);
  sprintf(msg, "Box id ----------------------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(TempConfigAddr, tempConfig, IdSize);
  uint8ArrayToString(Buffer, tempConfig);
  sprintf(msg, "1 - Temperature sensor id: --: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(HumidConfigAddr, humidConfig, IdSize);
  uint8ArrayToString(Buffer, humidConfig);
  sprintf(msg, "2 - Humidity sensor id ------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(NOxIndexConfigAddr, noxConfig, IdSize);
  uint8ArrayToString(Buffer, noxConfig);
  sprintf(msg, "3 - NOx sensor id -----------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(VocIndexConfigAddr, vocConfig, IdSize);
  uint8ArrayToString(Buffer, vocConfig);
  sprintf(msg, "4 - VOC sensor id -----------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(dBaConfigAddr, soundConfig, IdSize);
  uint8ArrayToString(Buffer, soundConfig);
  sprintf(msg, "5 is former dBa unused\r\n");
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "5 - Air pressure sensor id --: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(dBAConfigAddr, soundConfig, IdSize);
  uint8ArrayToString(Buffer, soundConfig);
  sprintf(msg, "6 is former dBc\r\n");
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "6 - Sound dBA sensor id -----: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(CustomNameConfigAddr, nameConfig, CustomNameMaxLength);
  sprintf(msg, "7 - Stored name -------------: ");
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "%s\r\n", (char*)nameConfig);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(SolVoltConfigAddr, solarConfig, IdSize);
  uint8ArrayToString(Buffer, solarConfig);
  sprintf(msg, "8 - Solar voltage sensor id -: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(BatVoltConfigAddr, batteryConfig, IdSize);
  uint8ArrayToString(Buffer, batteryConfig);
  sprintf(msg, "9 - Battery voltage sensor id: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(PM10ConfigAddr, PM10Config, IdSize);
  uint8ArrayToString(Buffer, PM10Config);
  sprintf(msg, ": - PM10 sensor id ----------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(PM2ConfigAddr, PM2Config, IdSize);
  uint8ArrayToString(Buffer, PM2Config);
  sprintf(msg, "; - PM2p5 sensor id ---------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  printf_USB("\r\nOnly the last two nibbles are necessary.\r\n");

  printf_USB("Command example for air pressure => #5,6a\r\n");

  if (!usb_out) {
    printf("A key can only be changed via USB input.\r\n");
  }
}

bool Process_USB_input(uint8_t* data) {
  static uint8_t boxConfig[IdSize];
  uint32_t length = GetUsbRxDataSize();
  static char Buffer[25];
  if (length > 5) {  //#2,34
    printf_USB("USB input: %s\r\n", (const char*)data);
    uint8_t* message = (unsigned char*)strstr((const char*)data, PREAMBLE);  // zoek op #
    if(message != NULL) {
      received.Command = (message[1] & 0x0F);
      if (message[2] == ',') {
        for (uint8_t i=3; i < 5; i++) {
          if (isxdigit(message[i])) {
            result = (result << 4) | (isdigit(message[i]) ? message[i] - '0' : toupper(message[i]) - 'A' + 10);
          }
          else {
            printf_USB("Invalid hexadecimal character: '%c'\r\n", message[i]);
            return false; // Of een andere foutwaarde
          }
        }
        ReadUint8ArrayEEprom(BoxConfigAddr, boxConfig, IdSize);
        boxConfig[11] = result; //overwrite the last byte
        memcpy(received.Payload, boxConfig, IdSize);
        received.PayloadLength = IdSize;
        uint8ArrayToString(Buffer, received.Payload);
        ProcessCmd(received);
        ResetUsbRxDataSize();
        PC_show_Keys();
        for (uint8_t i=0; i < 8; i++) {
          data[i] = '\0';
        }
        return true;
      }
      else {
        printf_USB("Invalid input; Command comma not found\r\n");
        ResetUsbRxDataSize();
      }
    }
    else {
      PC_show_Keys();
      ResetUsbRxDataSize();
    }
    for (uint8_t i=0; i < 8; i++) {
      data[i] = '\0';
    }

  }
  if (formerlength != length) {
    printf_USB("USB input: %s\r", (const char*)data);
    formerlength = length;
  }
  GetUsbRxNextChunk(length);

  return false;
}


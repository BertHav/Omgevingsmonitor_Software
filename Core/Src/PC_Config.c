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
        case hPaConfigCmd:  // 5 will be airpressure => hPaconfigCMD hPaConfigAddr ??
            WriteUint8ArrayEepromSafe(hPaConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case dBAConfigCmd: // 6 will be dBAConfigCMD
            WriteUint8ArrayEepromSafe(dBAConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case PM2ConfigCmd:  // 7
            WriteUint8ArrayEepromSafe(PM2ConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case PM10ConfigCmd:  // 8
            WriteUint8ArrayEepromSafe(PM10ConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case BatVoltConfigCmd:  // 9
            WriteUint8ArrayEepromSafe(BatVoltConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case SolVoltConfigCmd:  // 10
            WriteUint8ArrayEepromSafe(SolVoltConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case CustomNameConfigCmd:  // 12
            WriteUint8ArrayEepromSafe(CustomNameConfigAddr, msg.Payload, msg.PayloadLength, CustomNameMaxLength);
        break;
        case PM1ConfigCmd:  // 13
            WriteUint8ArrayEepromSafe(PM1ConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case PM4ConfigCmd:  // 14
            WriteUint8ArrayEepromSafe(PM4ConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case AHTTempConfigCmd: // 15
          WriteUint8ArrayEepromSafe(AHTTempConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case AHTHumidConfigCmd: // 16
          WriteUint8ArrayEepromSafe(AHTHumidConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case BMPTempConfigCmd: // 17
          WriteUint8ArrayEepromSafe(BMPTempConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case ENSAQIConfigCmd: // 18
          WriteUint8ArrayEepromSafe(ENSAQIConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case ENSTVOCConfigCmd: // 19
          WriteUint8ArrayEepromSafe(ENSTVOCConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case ENSeCO2ConfigCmd: // 20
          WriteUint8ArrayEepromSafe(ENSeCO2ConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case SSIDConfigCmd: // 21
          WriteUint8ArrayEepromSafe(SSIDConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case pwdConfigCmd: // 22
          WriteUint8ArrayEepromSafe(pwdConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;

        case ClearConfigCmd: // 253
            ClearEEprom(EEPromStartAddr, ConfigSize);
        break;
        case ClearEepromCmd: //254
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
//  else {
    printf(msg);
//  }
  HAL_Delay(10); //wait for the host poll of the USB buffer
}

void PC_show_Keys() {
  static bool usb_out= false;
  static uint8_t boxConfig[IdSize];          // 0
  static uint8_t tempConfig[IdSize];         // 1
  static uint8_t humidConfig[IdSize];        // 2
  static uint8_t noxConfig[IdSize];          // 3
  static uint8_t vocConfig[IdSize];          // 4
  static uint8_t hPaConfig[IdSize];          // 5
  static uint8_t soundConfig[IdSize];        // 6
  static uint8_t PM2Config[IdSize];          // 7
  static uint8_t PM10Config[IdSize];         // 8
  static uint8_t batteryConfig[IdSize];      // 9
  static uint8_t solarConfig[IdSize];        // 10
  static uint8_t ChargerStatConfig[IdSize];  // 11 is not used, see config.h
  static uint8_t PM1Config[IdSize];          // 13
  static uint8_t PM4Config[IdSize];          // 14
  static uint8_t AHTTempConfig[IdSize];      // 15
  static uint8_t AHTHumidConfig[IdSize];     // 16
  static uint8_t BMPTempConfig[IdSize];      // 17
  static uint8_t ENSAQIConfig[IdSize];       // 18
  static uint8_t ENSTVOCConfig[IdSize];      // 19
  static uint8_t ENSeCO2Config[IdSize];      // 20

  static uint8_t nameConfig[CustomNameMaxLength]; // 12
  static uint8_t SSIDConfig[SSIDMaxLength];       // 21
  static uint8_t pwdConfig[pwdMaxLength];         // 22

  static char Buffer[25];
  static char msg[70];
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
  sprintf(msg, "Box id --------------------------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(TempConfigAddr, tempConfig, IdSize);
  uint8ArrayToString(Buffer, tempConfig);
  sprintf(msg, "01 - Temperature sensor id: -----: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(HumidConfigAddr, humidConfig, IdSize);
  uint8ArrayToString(Buffer, humidConfig);
  sprintf(msg, "02 - Humidity sensor id ---------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(NOxIndexConfigAddr, noxConfig, IdSize);
  uint8ArrayToString(Buffer, noxConfig);
  sprintf(msg, "03 - NOx sensor id --------------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(VocIndexConfigAddr, vocConfig, IdSize);
  uint8ArrayToString(Buffer, vocConfig);
  sprintf(msg, "04 - VOC sensor id --------------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(hPaConfigAddr, hPaConfig, IdSize);
  uint8ArrayToString(Buffer, hPaConfig);
  sprintf(msg, "05 is former dBa unused\r\n");
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "05 - Air pressure sensor id -----: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(dBAConfigAddr, soundConfig, IdSize);
  uint8ArrayToString(Buffer, soundConfig);
  sprintf(msg, "06 is former dBc\r\n");
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "06 - Sound dBA sensor id --------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(PM2ConfigAddr, PM2Config, IdSize);
  uint8ArrayToString(Buffer, PM2Config);
  sprintf(msg, "07 - PM2p5 sensor id ------------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(PM10ConfigAddr, PM10Config, IdSize);
  uint8ArrayToString(Buffer, PM10Config);
  sprintf(msg, "08 - PM10 sensor id -------------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(BatVoltConfigAddr, batteryConfig, IdSize);
  uint8ArrayToString(Buffer, batteryConfig);
  sprintf(msg, "09 - Battery voltage sensor id --: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(SolVoltConfigAddr, solarConfig, IdSize);
  uint8ArrayToString(Buffer, solarConfig);
  sprintf(msg, "10 - Solar voltage sensor id ----: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(ChargerStatConfigAddr, ChargerStatConfig, IdSize);
  uint8ArrayToString(Buffer, ChargerStatConfig);
  sprintf(msg, "11 - Charger status (not used) --: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(CustomNameConfigAddr, nameConfig, CustomNameMaxLength);
  sprintf(msg, "12 - Stored name ----------------: ");
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "%s\r\n", (char*)nameConfig);  // probably too long to held in same buffer
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(PM1ConfigAddr, PM1Config, IdSize);
  uint8ArrayToString(Buffer, PM1Config);
  sprintf(msg, "13 - PM1p0 sensor id ------------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(PM4ConfigAddr, PM4Config, IdSize);
  uint8ArrayToString(Buffer, PM4Config);
  sprintf(msg, "14 - PM4p0 sensor id ------------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(AHTTempConfigAddr, AHTTempConfig, IdSize);
  uint8ArrayToString(Buffer, AHTTempConfig);
  sprintf(msg, "15 - AHT2x Temperature sensor id : %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(AHTHumidConfigAddr, AHTHumidConfig, IdSize);
  uint8ArrayToString(Buffer, AHTHumidConfig);
  sprintf(msg, "16 - AHT2x Humidity sensor id ---: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(BMPTempConfigAddr, BMPTempConfig, IdSize);
  uint8ArrayToString(Buffer, BMPTempConfig);
  sprintf(msg, "17 - BMP280 Temperature sensor id: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(ENSAQIConfigAddr, ENSAQIConfig, IdSize);
  uint8ArrayToString(Buffer, ENSAQIConfig);
  sprintf(msg, "18 - ENS160 AQI sensor id -------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(ENSTVOCConfigAddr, ENSTVOCConfig, IdSize);
  uint8ArrayToString(Buffer, ENSTVOCConfig);
  sprintf(msg, "19 - ENS160 TVOC sensor id ------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(ENSeCO2ConfigAddr, ENSeCO2Config, IdSize);
  uint8ArrayToString(Buffer, ENSeCO2Config);
  sprintf(msg, "20 - ENS160 eCO2 sensor id ------: %s\r\n", Buffer);
  PC_selectout(&msg[0], usb_out);

  printf_USB("\r\nOnly the last two nibbles are necessary.\r\n");
  HAL_Delay(10);
  printf_USB("\r\n!!NO LINE EDITING!!\r\n");
  HAL_Delay(10);
  printf_USB("Command example for air pressure => #05,6a\r\n");
  HAL_Delay(10);
  printf_USB("For the full key variant copy and paste the key sequence");
  HAL_Delay(10);
  printf_USB("from opensensemap.org in your account to this input.\r\n");
  HAL_Delay(10);
  printf_USB("Command example for a full key for air pressure =>");
  HAL_Delay(10);
  printf_USB(" $05,67af09374cdef30007b35055\r\n");
  HAL_Delay(10);
  if (!usb_out) {
    printf("A key can only be changed by USB input or the configuration programm.\r\n");
  }
}

uint8_t ascii_to_uint8(uint8_t *inchar) {
  if (!isdigit(inchar[0]) || !isdigit(inchar[1])) {
    printf_USB("Error: two decimal numbers expected\r\n");
    return 100;
  }
  uint8_t value = (inchar[0] - '0') * 10 + (inchar[1] - '0');
  if (value > 20) {
    printf_USB("Error: value out of range\r\n");
    return 100;
  }
  return (uint8_t)value;
}

bool Process_USB_input(uint8_t* data) {
  static uint8_t boxConfig[IdSize];
  static uint32_t len = 6;
  uint32_t length = GetUsbRxDataSize();
  static uint8_t r = 0;
//  uint8_t* message;
  static char Buffer[24];
  uint8_t* message = (unsigned char*)strstr((const char*)data, PREAMBLE_F);  // zoek op $
  if ((length == 1) && (message != NULL) && (len != 28)){
      Debug("Switching to input length of 28 for full opensensemap keylength");
      len = 28;
  }
  if (length > len) {
//    printf_USB("minimum required USB input reached: %s\r\n", (const char*)data);
    printf_USB("USB input: %s\r\n", (const char*)data);
    message = (unsigned char*)data;
    if (message[0] == '$') {
      len = 28;
    }
    if((message[0] == '#') || (message[0] == '$')) {
      received.Command = ascii_to_uint8(&message[1]);
      if (received.Command == 100) {
        return false; // value out of range
      }
      if (message[3] == ',') {
        for (uint8_t i=4; i < len; i++) {
//          printf_USB("handling character %c as nr: %d for pos: %d\r\n", message[i], i, r);
          HAL_Delay(10);
          if (isxdigit(message[i])) {
            result = (result << 4) | (isdigit(message[i]) ? message[i] - '0' : toupper(message[i]) - 'A' + 10);
//            printf_USB("Result is 0x%2X\r\n", result);
            HAL_Delay(10);
            if (len == 28) {
              if ((i % 2) == 1) {
                message[r] = result;
//                Debug("message[%d] = 0x%02X",r, message[r]);
                r++;
              }
            }
          }
          else {
            printf_USB("Invalid hexadecimal character: '%c at position %d'\r\n", message[i], i);
            ResetUsbRxDataSize();
            return false; // Of een andere foutwaarde
          }
        }
        if (len == 6) {
          ReadUint8ArrayEEprom(BoxConfigAddr, boxConfig, IdSize);
          boxConfig[11] = result; //overwrite the last byte
          memcpy(received.Payload, boxConfig, IdSize);
        }
        else {
          memcpy(received.Payload, message, IdSize);
        }
        received.PayloadLength = IdSize;
        uint8ArrayToString(Buffer, received.Payload);
        ProcessCmd(received);
        ResetUsbRxDataSize();
        PC_show_Keys();
        for (uint8_t i=0; i < 32; i++) {
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
      len = 6;
      PC_show_Keys();
      ResetUsbRxDataSize();
    }
    for (uint8_t i=0; i < 32; i++) {
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


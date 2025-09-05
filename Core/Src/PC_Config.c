#include <eeprom.h>
#include <ctype.h>
#include "../Inc/PC_Config.h"
#include "../Inc/Config.h"
#include "utils.h"
#include "ESP.h"
#include "statusCheck.h"
#include "RealTimeClock.h"

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
        case dBAConfigCmd: // 5 was 6 will be dBAConfigCMD
            WriteUint8ArrayEepromSafe(dBAConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case dBcConfigCmd:  // 6
            WriteUint8ArrayEepromSafe(dBcConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
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
        case SSIDConfigCmd: // 13 was 21
          ClearEEprom(SSIDConfigAddr, SSIDMaxLength);
          WriteUint8ArrayEepromSafe(SSIDConfigAddr, msg.Payload, msg.PayloadLength, SSIDMaxLength);
        break;
        case PasswordConfigCmd: // 14 was 22
          ClearEEprom(pwdConfigAddr, pwdMaxLength);
          printf_USB("(command: %d, payload to write: %s, length: %d\r\n",msg.Command, msg.Payload, msg.PayloadLength);
          WriteUint8ArrayEepromSafe(pwdConfigAddr, msg.Payload, msg.PayloadLength, pwdMaxLength);
        break;
        case PM1ConfigCmd:  // 21 was 13
            WriteUint8ArrayEepromSafe(PM1ConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case PM4ConfigCmd:  // 22 was 14
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
        case SEN55TempConfigCmd: // 23
          WriteUint8ArrayEepromSafe(SEN55TempConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case SEN55HumidConfigCmd: // 24
          WriteUint8ArrayEepromSafe(SEN55HumidConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case SendFromNameConfigCmd:  // 25
          ClearEEprom(SendFromNameConfigAddr, SendFromNameMaxLength);
          WriteUint8ArrayEepromSafe(SendFromNameConfigAddr, msg.Payload, msg.PayloadLength, SendFromNameMaxLength);
        break;
        case SendToNameConfigCmd:  // 26
          ClearEEprom(SendToNameConfigAddr, SendToNameMaxLength);
          printf_USB("(command: %d, payload to write: %s, length: %d\r\n",msg.Command, msg.Payload, msg.PayloadLength);
          WriteUint8ArrayEepromSafe(SendToNameConfigAddr, msg.Payload, msg.PayloadLength, SendToNameMaxLength);
        break;
        case MailAPIKeyConfigCmd:  // 27
          ClearEEprom(MailAPIKeyConfigAddr, MailAPIKeyMaxLength);
          WriteUint8ArrayEepromSafe(MailAPIKeyConfigAddr, msg.Payload, msg.PayloadLength, MailAPIKeyMaxLength);
        break;
        case hPaConfigCmd:  // 28 will be airpressure => hPaconfigCMD hPaConfigAddr ??
          WriteUint8ArrayEepromSafe(hPaConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case UptimeConfigCmd: // 29
          WriteUint8ArrayEepromSafe(UptimeConfigAddr, msg.Payload, msg.PayloadLength, IdSize);
        break;
        case URLToUploadConfigCmd: // 30
          printf_USB("(command: %d, payload to write: %s, length: %d\r\n",msg.Command, msg.Payload, msg.PayloadLength);
          WriteUint8ArrayEepromSafe(URLToUploadConfigAddr, msg.Payload, msg.PayloadLength, URLToUploadMaxLength);
        break;

        case ClearConfigCmd: // 253
            ClearEEprom(EEPromStartAddr, ConfigSize);
            ClearEEprom(SSIDStartAddr, IPrelatedConfigSize);
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
  vsnprintf(string, 150, message, args);
  va_end(args);
  CDC_Transmit_FS((uint8_t*)string, strlen(string));
}

void PC_selectout(char *msg, bool usb_out) {
  if (usb_out){
    printf_USB(msg);
    HAL_Delay(10); //wait for the host poll of the USB buffer
  }
//  else {
    printf(msg);
//  }
}

void PC_show_Keys() {
  static bool usb_out= false;
  static uint8_t boxConfig[IdSize];          // 0
  static uint8_t tempConfig[IdSize];         // 1
  static uint8_t humidConfig[IdSize];        // 2
  static uint8_t noxConfig[IdSize];          // 3
  static uint8_t vocConfig[IdSize];          // 4
  static uint8_t dBcConfig[IdSize];          // 5
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
  static uint8_t SEN55TempConfig[IdSize];         // 23
  static uint8_t SEN55HumidConfig[IdSize];        // 24
#ifdef USE_MAIL
  static uint8_t SendFromnameConfig[SendFromNameMaxLength]; //25
  static uint8_t SendTonameConfig[SendToNameMaxLength]; //26
  static uint8_t MailAPIKeyConfig[MailAPIKeyMaxLength]; //27
#endif
  static uint8_t hPaConfig[IdSize];               // 28 was 5
  static uint8_t UptimeConfig[IdSize];            // 29
  static uint8_t URLToUploadConfig[URLToUploadMaxLength]; // 30
  static char Buffer[25];
  static char msg[70];
  if(Check_USB_PowerOn()){
    usb_out = true;
  }
  else {
    usb_out = false;
  }
  sprintf(msg, "\r\nOverview of stored keys:\r\n");
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(BoxConfigAddr, boxConfig, IdSize);
  uint8ArrayToString(Buffer, boxConfig);
  sprintf(msg, "%02d - Box id ------------------------: %s\r\n", BoxConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(TempConfigAddr, tempConfig, IdSize);
  uint8ArrayToString(Buffer, tempConfig);
  sprintf(msg, "%02d - Temperature sensor id: --------: %s\r\n", TempConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(HumidConfigAddr, humidConfig, IdSize);
  uint8ArrayToString(Buffer, humidConfig);
  sprintf(msg, "%02d - Humidity sensor id ------------: %s\r\n", HumidConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(NOxIndexConfigAddr, noxConfig, IdSize);
  uint8ArrayToString(Buffer, noxConfig);
  sprintf(msg, "%02d - NOx sensor id -----------------: %s\r\n", NOxIndexConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(VocIndexConfigAddr, vocConfig, IdSize);
  uint8ArrayToString(Buffer, vocConfig);
  sprintf(msg, "%02d - VOC sensor id -----------------: %s\r\n", VocIndexConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(dBAConfigAddr, soundConfig, IdSize);
  uint8ArrayToString(Buffer, soundConfig);
  sprintf(msg, "%02d - dBA sensor id -----------------: %s\r\n", dBAConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(dBcConfigAddr, dBcConfig, IdSize);
  uint8ArrayToString(Buffer, dBcConfig);
  sprintf(msg, "%02d - Sound dBc sensor id -----------: %s\r\n", dBcConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(PM2ConfigAddr, PM2Config, IdSize);
  uint8ArrayToString(Buffer, PM2Config);
  sprintf(msg, "%02d - PM2p5 sensor id ---------------: %s\r\n", PM2ConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(PM10ConfigAddr, PM10Config, IdSize);
  uint8ArrayToString(Buffer, PM10Config);
  sprintf(msg, "%02d - PM10 sensor id ----------------: %s\r\n", PM10ConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(BatVoltConfigAddr, batteryConfig, IdSize);
  uint8ArrayToString(Buffer, batteryConfig);
  sprintf(msg, "%02d - Battery voltage sensor id -----: %s\r\n", BatVoltConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(SolVoltConfigAddr, solarConfig, IdSize);
  uint8ArrayToString(Buffer, solarConfig);
  sprintf(msg, "%02d - Solar voltage sensor id -------: %s\r\n", SolVoltConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(ChargerStatConfigAddr, ChargerStatConfig, IdSize);
  uint8ArrayToString(Buffer, ChargerStatConfig);
  sprintf(msg, "%02d - Charger status (not used) -----: %s\r\n", ChargerStatConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(CustomNameConfigAddr, nameConfig, CustomNameMaxLength);
  sprintf(msg, "%02d - Box name ---max 12 char--------: ", CustomNameConfigCmd);
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "%s\r\n", (char*)nameConfig);  // probably too long to held in same buffer
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(SSIDConfigAddr, SSIDConfig, SSIDMaxLength);
//  uint8ArrayToString(Buffer, SSIDConfig);
  sprintf(msg, "%02d - SSID name ---------------------: ", SSIDConfigCmd);
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "%s\r\n", (char*)SSIDConfig);  // probably too long to held in same buffer
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(pwdConfigAddr, pwdConfig, pwdMaxLength);
//  uint8ArrayToString(Buffer, pwdConfig);
  sprintf(msg, "%02d - WiFi password -----------------: ", PasswordConfigCmd);
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "%s\r\n", (char*)pwdConfig);  // probably too long to held in same buffer
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(AHTTempConfigAddr, AHTTempConfig, IdSize);
  uint8ArrayToString(Buffer, AHTTempConfig);
  sprintf(msg, "%02d - AHT2x Temperature sensor id ---: %s\r\n", AHTTempConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(AHTHumidConfigAddr, AHTHumidConfig, IdSize);
  uint8ArrayToString(Buffer, AHTHumidConfig);
  sprintf(msg, "%02d - AHT2x Humidity sensor id ------: %s\r\n", AHTHumidConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(BMPTempConfigAddr, BMPTempConfig, IdSize);
  uint8ArrayToString(Buffer, BMPTempConfig);
  sprintf(msg, "%02d - BMP280 Temperature sensor id --: %s\r\n", BMPTempConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(ENSAQIConfigAddr, ENSAQIConfig, IdSize);
  uint8ArrayToString(Buffer, ENSAQIConfig);
  sprintf(msg, "%02d - ENS160 AQI sensor id ----------: %s\r\n", ENSAQIConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(ENSTVOCConfigAddr, ENSTVOCConfig, IdSize);
  uint8ArrayToString(Buffer, ENSTVOCConfig);
  sprintf(msg, "%02d - ENS160 TVOC sensor id ---------: %s\r\n", ENSTVOCConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(ENSeCO2ConfigAddr, ENSeCO2Config, IdSize);
  uint8ArrayToString(Buffer, ENSeCO2Config);
  sprintf(msg, "%02d - ENS160 eCO2 sensor id ---------: %s\r\n", ENSeCO2ConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(PM1ConfigAddr, PM1Config, IdSize);
  uint8ArrayToString(Buffer, PM1Config);
  sprintf(msg, "%02d - PM1p0 sensor id ---------------: %s\r\n", PM1ConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(PM4ConfigAddr, PM4Config, IdSize);
  uint8ArrayToString(Buffer, PM4Config);
  sprintf(msg, "%02d - PM4p0 sensor id ---------------: %s\r\n", PM4ConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(SEN55TempConfigAddr, SEN55TempConfig, IdSize);
  uint8ArrayToString(Buffer, SEN55TempConfig);
  sprintf(msg, "%02d - SEN54/55 Temperature sensor id : %s\r\n", SEN55TempConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(SEN55HumidConfigAddr, SEN55HumidConfig, IdSize);
  uint8ArrayToString(Buffer, SEN55HumidConfig);
  sprintf(msg, "%02d - SEN54/55 Humidity sensor id ---: %s\r\n", SEN55HumidConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

#ifdef USE_MAIL
  ReadUint8ArrayEEprom(SendFromNameConfigAddr, SendFromnameConfig, SendFromNameMaxLength);
  sprintf(msg, "%02d - Stored Send from name ---------: ", SendFromNameConfigCmd);
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "%s\r\n", (char*)SendFromnameConfig);  // probably too long to held in same buffer
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(SendToNameConfigAddr, SendTonameConfig, SendToNameMaxLength);
  sprintf(msg, "%02d - Stored Send to name -----------: ", SendToNameConfigCmd);
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "%s\r\n", (char*)SendTonameConfig);  // probably too long to held in same buffer
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(MailAPIKeyConfigAddr, MailAPIKeyConfig, MailAPIKeyMaxLength);
  sprintf(msg, "%02d - Stored SMTP2go API key --------: ", MailAPIKeyConfigCmd);
  PC_selectout(&msg[0], usb_out);
  sprintf(msg, "%s\r\n", (char*)MailAPIKeyConfig);  // probably too long to held in same buffer
  PC_selectout(&msg[0], usb_out);
#endif

  ReadUint8ArrayEEprom(hPaConfigAddr, hPaConfig, IdSize);
  uint8ArrayToString(Buffer, hPaConfig);
  sprintf(msg, "%02d - BMP280 Air pressure sensor id -: %s\r\n", hPaConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(UptimeConfigAddr, UptimeConfig, IdSize);
  uint8ArrayToString(Buffer, UptimeConfig);
  sprintf(msg, "%02d - Uptime sensor id --------------: %s\r\n", UptimeConfigCmd, Buffer);
  PC_selectout(&msg[0], usb_out);

  ReadUint8ArrayEEprom(URLToUploadConfigAddr, URLToUploadConfig, URLToUploadMaxLength);
  sprintf(msg, "%02d - URL to upload -----------------: ", URLToUploadConfigCmd);
  PC_selectout(&msg[0], usb_out);
  if (strlen((char*)URLToUploadConfig) == 0) {
    sprintf(msg, "Undefined, defaulted to ");
    PC_selectout(&msg[0], usb_out);
    PC_selectout((char*)API, usb_out);
    PC_selectout("\r\n\0", usb_out);
  }
  else {
    sprintf(msg, "%s\r\n", (char*)URLToUploadConfig);  // probably too long to held in same buffer
    PC_selectout(&msg[0], usb_out);
  }

  sprintf(msg, "%02d - Clear all definitions in EEPROM\r\n", clearDefsCmd);
  PC_selectout(&msg[0], usb_out);

  printf_USB("\r\n!!NO LINE EDITING!!\r\n");
  HAL_Delay(10);
  printf_USB("If the key differs only the last two bytes,");
  HAL_Delay(10);
  printf_USB(" command example for air pressure => #28,6a\r\n");
  HAL_Delay(10);
  printf_USB("For the full key variant copy and paste the key sequence");
  HAL_Delay(10);
  printf_USB(" from opensensemap.org in your account to this input.\r\n");
  HAL_Delay(10);
  printf_USB("Command example for a full key for PM10 sensor =>");
  HAL_Delay(10);
  printf_USB(" $08,67af09374cdef30007b35055\r\n");
  HAL_Delay(10);
  printf_USB("For changing string entries use prefix S, example: ");
  HAL_Delay(10);
  printf_USB("S12,Testsysteem (max 12 chars)\r\n");
  HAL_Delay(10);
  printf_USB("To clear a string: $30,000000000000000000000000\r\n");
  HAL_Delay(10);
  printf_USB("L - toggle logging on/off, current: %s\r\n", usblog?"on":"off");
  HAL_Delay(10);
  printf_USB("B - show build information\r\n");
  HAL_Delay(10);
  if (!usb_out) {
    printf("A sensor key can only be changed by USB input or the by configuration programm.\r\n");
  }
}

uint8_t ascii_to_uint8(uint8_t *inchar) {
  if (!isdigit(inchar[0]) || !isdigit(inchar[1])) {
    printf_USB("Error: two decimal numbers expected\r\n");
    return 100;
  }
  uint8_t value = (inchar[0] - '0') * 10 + (inchar[1] - '0');

  if (value >= maxCmd) {
    printf_USB("Error: value out of range\r\n");
    return 100;
  }
  if (value == clearDefsCmd) {
    value = ClearConfigCmd;
  }
  return (uint8_t)value;
}

bool Process_USB_input(uint8_t* data) {
  uint8_t boxConfig[IdSize];
  static uint32_t len = 6;
  uint32_t length = GetUsbRxDataSize();
  uint8_t r = 0;
  uint8_t i = 0;
  char Buffer[pwdMaxLength];
  uint8_t* message = (unsigned char*)strstr((const char*)data, PREAMBLE_F);  // zoek op $
  if ((length == 1) && (message != NULL) && (len != 28)){
      len = 28;
  }
  message = (unsigned char*)strstr((const char*)data, PREAMBLE_S);  // zoek op S
  if ((length == 1) && (message != NULL) && (len != pwdMaxLength)){
      len = pwdMaxLength;
  }
  message = (unsigned char*)strstr((const char*)data, PREAMBLE_L);  // Search for 'L'to toggle USB logging
  if ((length == 1) && (message != NULL)){
    usblog = !usblog; // log info to usb too
    HAL_FLASHEx_DATAEEPROM_Unlock();
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, USBlogstatusConfigAddr, usblog);
    HAL_FLASHEx_DATAEEPROM_Lock();
    printf_USB("\r\nSwitching USB logging to %s\r\n", usblog?"on":"off");
    length = 0;
    data[0] = '\0';
    ResetUsbRxDataSize();
    return true;
  }
  message = (unsigned char*)strstr((const char*)data, PREAMBLE_B);  // Search for 'B'to show the build
  if ((length == 1) && (message != NULL)){
    BinaryReleaseInfo();
    showUpTime();
    length = 0;
    data[0] = '\0';
    ResetUsbRxDataSize();
    return true;
  }
  if ((length >= len) || (data[length-1] == 13)) {
    // 'S' is for entering a ASCII string
    if (data[length-1] == 13) {
      printf_USB("Inputstring detected, string terminated\r\n");
      data[length-1] = 0;
    }
    if((data[0] == '#') || (data[0] == '$') || (data[0] == 'S') || (data[0] == 'E')) {
      received.Command = ascii_to_uint8(&data[1]);  // calculate the command number
      printf_USB("Command nr determined: %d", received.Command);
      if (received.Command == 100) {
        printf_USB("\r\nCommandvalue out of range.\r\n");
        ResetUsbRxDataSize();
        PC_show_Keys();
        for (uint8_t i=0; i < 32; i++) {
          data[i] = '\0';
        }
        return false; // value out of range
      }
      if (data[3] == ',') {
        if ((data[0] == 'S') || (data[0] == 'E')) {
          if ((data[0] == 'E') && (received.Command == clearDefsCmd)) {
//            printf_USB("\r\nClear EEPROM request\r\n");
            received.Command = ClearConfigCmd;
          }
        }
        if ((data[0] == '$') || (data[0] == '#')) {
          for (i=4; i < len; i++) {
            HAL_Delay(10);
            if (isxdigit(data[i])) {
              result = (result << 4) | (isdigit(data[i]) ? data[i] - '0' : toupper(data[i]) - 'A' + 10);
//              printf_USB("Result is 0x%02X\r\n", result);
              HAL_Delay(10);
              if (len == 28) {
                if ((i % 2) == 1) {
                  data[r] = result;
//                  printf_USB("data[%d] = 0x%02X",r, data[r]);
                  r++;
                }
              }
            }
            else {
              printf_USB("\r\nInvalid hexadecimal character: '%c at position %d\r\n", data[i], i);
              ResetUsbRxDataSize();
              PC_show_Keys();
              for (uint8_t i=0; i < length; i++) {
                data[i] = '\0';
                return false; // Of een andere foutwaarde
              }
            }
          }  // end for
//          printf_USB("\r\n");
        }
        if (len < pwdMaxLength) {
          if (len == 6) {
            ReadUint8ArrayEEprom(BoxConfigAddr, boxConfig, IdSize);
            boxConfig[11] = result; //overwrite the last byte of the key
            memcpy(received.Payload, boxConfig, IdSize);
          }
          else {
            memcpy(received.Payload, data, IdSize);
          }
          received.Payload[12] = '\0';
          received.PayloadLength = IdSize;
        }
        else {
          received.PayloadLength = length-4;  // The string terminator counts
        }
        if ((len < pwdMaxLength) && (len != 6))  {
          uint8ArrayToString(Buffer, received.Payload);
        }
        else if (len == pwdMaxLength) {
          memcpy(received.Payload, &data[4], received.PayloadLength);
        }
        ProcessCmd(received);
        ResetUsbRxDataSize();
        PC_show_Keys();
        for (uint8_t i=0; i < length; i++) {
          data[i] = '\0';
        }
        return true;
      }
      else {
        printf_USB("\r\nInvalid input; comma not found\r\n");
//        ResetUsbRxDataSize();
//        for (uint8_t i=0; i < length; i++) {
//          data[i] = '\0';
//        }
      }
    }
    PC_show_Keys();
    ResetUsbRxDataSize();
    for (uint8_t i=0; i < length; i++) {
      data[i] = '\0';
    }
    len = 6;
  }
  if (formerlength != length) {
    printf_USB("USB input: %s\r", (const char*)data);
    formerlength = length;
  }
  GetUsbRxNextChunk(length);
  return false;
}


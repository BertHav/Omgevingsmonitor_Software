#ifndef CONFIG_H
#define CONFIG_H

#define EEPromStartAddr 0x08080000

//#define IdCount 18

#define uint8_tSize 1
#define IdSize 12
#define SendFromNameMaxLength 48
#define SendToNameMaxLength 48
#define MailAPIKeyMaxLength 48
#define CustomNameMaxLength 32
#define SSIDMaxLength 32
#define pwdMaxLength 64
#define BoxConfigAddr EEPromStartAddr + (IdSize * 0)
#define TempConfigAddr EEPromStartAddr + (IdSize * 1)
#define HumidConfigAddr EEPromStartAddr + (IdSize * 2)
#define NOxIndexConfigAddr EEPromStartAddr + (IdSize * 3)
#define VocIndexConfigAddr EEPromStartAddr + (IdSize * 4)
#define dBcConfigAddr EEPromStartAddr + (IdSize * 5)
#define dBAConfigAddr EEPromStartAddr + (IdSize * 6)
#define PM2ConfigAddr EEPromStartAddr + (IdSize * 7)
#define PM10ConfigAddr EEPromStartAddr + (IdSize * 8)
#define BatVoltConfigAddr EEPromStartAddr + (IdSize * 9)
#define SolVoltConfigAddr EEPromStartAddr + (IdSize * 10)
#define ChargerStatConfigAddr EEPromStartAddr + (IdSize * 11) // not used
#define CustomNameConfigAddr EEPromStartAddr + (IdSize * 12)
#define PM1ConfigAddr CustomNameConfigAddr + CustomNameMaxLength
#define PM4ConfigAddr PM1ConfigAddr + IdSize
#define AHTTempConfigAddr PM4ConfigAddr + IdSize
#define AHTHumidConfigAddr AHTTempConfigAddr + IdSize
#define BMPTempConfigAddr AHTHumidConfigAddr + IdSize
#define ENSAQIConfigAddr BMPTempConfigAddr + IdSize
#define ENSTVOCConfigAddr ENSAQIConfigAddr + IdSize
#define ENSeCO2ConfigAddr ENSTVOCConfigAddr + IdSize


//#define SSIDConfigAddr ENSeCO2ConfigAddr + IdSize
//#define pwdConfigAddr SSIDConfigAddr + SSIDMaxLength

#define SEN55TempConfigAddr ENSeCO2ConfigAddr  + IdSize + pwdMaxLength  + SSIDMaxLength
#define SEN55HumidConfigAddr SEN55TempConfigAddr + IdSize
#define hPaConfigAddr SEN55HumidConfigAddr + IdSize
#define USBlogstatusConfigAddr hPaConfigAddr + IdSize  // uint8_tSize has size of uint8_tSize
#define UptimeConfigAddr USBlogstatusConfigAddr + IdSize

#define ConfigSize USBlogstatusConfigAddr + uint8_tSize - EEPromStartAddr

#define SSIDStartAddr 0x08080200

#define SSIDConfigAddr SSIDStartAddr
#define pwdConfigAddr SSIDConfigAddr + SSIDMaxLength
#define SendFromNameConfigAddr pwdConfigAddr + pwdMaxLength
#define SendToNameConfigAddr SendFromNameConfigAddr + SendFromNameMaxLength
#define MailAPIKeyConfigAddr SendToNameConfigAddr + SendToNameMaxLength

#define IPrelatedConfigSize MailAPIKeyConfigAddr + MailAPIKeyMaxLength - SSIDStartAddr

#endif

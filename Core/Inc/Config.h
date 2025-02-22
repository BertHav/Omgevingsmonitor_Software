#ifndef CONFIG_H
#define CONFIG_H

#define EEPromStartAddr 0x08080000

#define IdCount 18

#define IdSize 12

#define ConfigSize (IdCount*IdSize) + CustomNameMaxLength
//#define ConfigSize (IdCount*IdSize) + SSIDMaxLength

#define BoxConfigAddr EEPromStartAddr + (IdSize * 0)
#define TempConfigAddr EEPromStartAddr + (IdSize * 1)
#define HumidConfigAddr EEPromStartAddr + (IdSize * 2)
#define NOxIndexConfigAddr EEPromStartAddr + (IdSize * 3)
#define VocIndexConfigAddr EEPromStartAddr + (IdSize * 4)
#define hPaConfigAddr EEPromStartAddr + (IdSize * 5)
#define dBAConfigAddr EEPromStartAddr + (IdSize * 6)
#define PM2ConfigAddr EEPromStartAddr + (IdSize * 7)
#define PM10ConfigAddr EEPromStartAddr + (IdSize * 8)
#define BatVoltConfigAddr EEPromStartAddr + (IdSize * 9)
#define SolVoltConfigAddr EEPromStartAddr + (IdSize * 10)
#define ChargerStatConfigAddr EEPromStartAddr + (IdSize * 11)
#define CustomNameConfigAddr EEPromStartAddr + (IdSize * 12)
#define CustomNameMaxLength 32

/*
#define SSIDConfigAddr EEPromStartAddr + (IdSize * 15)
#define SSIDMaxLength 32

#define pwdConfigAddr EEPromStartAddr + (IdSize * 18)
#define pwdMaxLength 63
*/
#endif

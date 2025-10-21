/*
 * utils.c
 *
 *  Created on: 25 Aug 2023
 *      Author: frank
 */
#include "main.h"
#include "utils.h"
#include "usbd_cdc_if.h"
#include "PC_Config.h"
#include "PowerUtils.h"
//#include "../Inc/PC_Config.h"

// Default verbose level
VerboseLevel CurrentVerboseLevel = VERBOSE_ALL;
#define TEXTBUFFER_LEN 192

extern UART_HandleTypeDef huart1;

bool TimestampIsReached(uint32_t timestamp) {
  uint32_t now = HAL_GetTick();
  const uint32_t HALF_TIME = 0x7FFFFFFF; // ~25 days max timestamp delay

  // Ensures that the difference between now and timestamp is less than half of
  // the maximum value of the counter. This confirms that the timestamp is not
  // too far in the past.
  bool noOverflow = (now >= timestamp && now - timestamp < HALF_TIME);
  // Ensures that the timestamp is so far ahead that it would have wrapped
  // around past the maximum value of the counter indicating an overflow
  // scenario.
  bool overflowWithTimestampAhead =
      (now < timestamp && timestamp - now > HALF_TIME);

  if (noOverflow || overflowWithTimestampAhead) {
    // Timestamp has been reached
    return true;
  }

  // Timestamp not yet reached
  return false;
}

static void AddTimestamp(char *stamp) {
  sprintf(stamp, "[%08lu] ", (uint32_t)HAL_GetTick());
}


void CreateLine(VerboseLevel verboseLevel, char *tag, char *format, ...) {

  if (verboseLevel > CurrentVerboseLevel) {
    return;
  }

  char textBuffer[TEXTBUFFER_LEN];

  // Format the initial part of the message with the tag and timestamp
  sprintf(textBuffer, "%s ", tag);

  if (CurrentVerboseLevel > VERBOSE_MINIMAL) {
    AddTimestamp(&textBuffer[strlen(textBuffer)]);
  }
  int lineOffset = strlen(textBuffer);

  // Format the rest of the message with the variable arguments
  va_list args;
  va_start(args, format);
  vsnprintf(&textBuffer[lineOffset], TEXTBUFFER_LEN - lineOffset - 2, format, args);
  va_end(args);

  if (usblog && Check_USB_PowerOn()){
    printf_USB("%s\r\n", textBuffer);
/*
    if (strlen(textBuffer) >= (TEXTBUFFER_LEN / 2)) {
      char endchar = textBuffer[(TEXTBUFFER_LEN / 2)];
      textBuffer[(TEXTBUFFER_LEN / 2)] = '\0';
      printf_USB(textBuffer);
      textBuffer[(TEXTBUFFER_LEN / 2)] = endchar;
      printf_USB("%s\r\n", &textBuffer[(TEXTBUFFER_LEN / 2)]);
    }
    else {
      printf_USB("%s\r\n", textBuffer);
    }
*/
  }
  // Print the final formatted message to serial
  printf("%s\r\n", textBuffer);
}

int _write(int fd, void *buf, size_t count) {
  HAL_UART_Transmit(&huart1, buf, count, 100);
  return count;
}

void SetVerboseLevel(VerboseLevel level) {
  CurrentVerboseLevel = level;
}

uint8_t GetVerboseLevel() {
  return CurrentVerboseLevel; }


void BinaryReleaseInfo() {
#ifndef LARGEBATTERY
  char msgout[42];
#else
  char msgout[56];
#endif
#ifdef IGNORE_PARITY_ERRORS
  sprintf(msgout,"Build ignores parity errors from ESP32");
  if (!usblog) {
    printf_USB("%s\r\n", msgout);  // alway forced shown even if usb logging is off
  }
#endif
#ifdef DEBUG
  #ifndef LARGEBATTERY
  sprintf(msgout,"Software version: %s, Debug build", SRC_VERSION);
  #else
  sprintf(msgout,"Software version: %s Large Battery, Debug build", SRC_VERSION);
  #endif
#else
  #ifndef LARGEBATTERY
  sprintf(msgout,"Software version: %s, Release build", SRC_VERSION);
  #else
  sprintf(msgout,"Software version: %s Large Battery, Release build", SRC_VERSION);
  #endif
#endif
  if (!usblog) {
    printf_USB("%s\r\n", msgout);  // alway forced shown even if usb logging is off
  }
  Info(msgout);
  sprintf(msgout,"Build on: %s at %s", __DATE__, __TIME__);
  if (!usblog) {
    printf_USB("%s\r\n", msgout);  // always forced shown even if usb logging is off
  }
  Info(msgout);
  // Format: YY'w'WWv
  Info("Git: %s", CURRENT_WEEK);
}

//
// Call this as: errorHandler(__func__, __LINE__, __FILE__);
//
void errorHandler(const char * func, const uint32_t line, const char * file)
{
    Error("Error in %s at line %lu in file: %s\r\n", func, line, file);
//    while (true)
//    {
//    }
}



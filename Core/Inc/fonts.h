#ifndef _FONTS_H
#define _FONTS_H

#include <stdint.h>

//
//  Structure used to define fonts
//
typedef struct {
    const uint8_t FontWidth;    /* Font width in pixels */
    uint8_t FontHeight;         /* Font height in pixels */
    const uint16_t *data;       /* Pointer to data font data array */
} FontDef;

typedef struct {
    const uint8_t FontWidth;    /* Font width in pixels */
    uint8_t FontHeight;         /* Font height in pixels */
    const uint8_t *data;       /* Pointer to data font data array */
} FontDefSmall;

//
//  Export the available font
//
extern FontDefSmall Font_5x8;
extern FontDefSmall Font_6x8;
extern FontDef Font_7x10;

#endif  // _FONTS_H

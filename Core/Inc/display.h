/*
 * display.h
 *
 *  Created on: Jan 19, 2025
 *      Author: itsme
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

extern bool SSD1306detected;

void displayCreateStyle();
void display2ndmsg2ndline();
void displayStart();
void displayActdBA();
void displayPeakdBA();
void displayPM2p5();
void displayPM10();
void displayVOC();
void displayNOx();
void displayTemperature();
void displayHumidity();

#endif /* INC_DISPLAY_H_ */

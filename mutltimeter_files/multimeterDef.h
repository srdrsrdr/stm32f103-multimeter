/*
 * multimeterDef.h
 *
 *  Created on: 28 Nis 2021
 *      Author: Sr_Dr
 */

#ifndef MULTIMETERDEF_H_
#define MULTIMETERDEF_H_

#include "util.h"


#define scopeBufLen				480
#define scopeBackColor			BLACK
#define scopeCenterLineColor 	GRAY5
#define scopeLineColor 			GRAY6
#define scopeSignalColor 		GREEN

#define menuColor 				0x0a6a
#define menuActiveColor 		LBBLUE

#define screenTop 				14
#define screenBottom			134

#define encoderButPeriod		10

typedef struct {
	uint16_t	regPSC;
	uint16_t	regARR;
} timeDiv_t;

typedef struct {
	uint8_t		timeDivIndex;
	uint8_t		scrollIndex;
	uint8_t		voltDivIndex;
	uint8_t		measurementIndex;
} scopeIndexes_t;

typedef struct {
	const String *pointer;
	uint8_t maxIndex;
	uint8_t index;
	uint8_t x;
	uint8_t w;
	uint8_t level;
} menu_t;

#define	timDivMenuStart		0
#define	timDivMenuWidth		50
#define	menuTimeDivLen		6
const String menuTimeDiv[] = {
		"20ms",
		"10ms",
		"1ms",
		"500us",
		"100us",
		"20us",
};

#define	voltDivMenuStart	70
#define	voltDivMenuWidth	50
#define	menuVoltDivLen		6
const String menuVoltDiv[] = {
		"30V",
		"5V",
		"1V",
		"500mV",
		"100mV",
		"1mV",
};

#define	measurementMenuStart	120
#define	measurementMenuWidth	50
#define	measurementMenuLen		5
const String menuMeasurement[] = {
		"Vp-p",
		"Vmean",
		"Veff",
		"Freq",
		"Duty"
};

#define scopeMenuLen	4
enum{
	timeDiv_,
	scrollVH_,
	voltDiv_,
	measurement_
} scopeMenu;

menu_t ScopeMenu[3] = {
		{ &menuTimeDiv, 	menuTimeDivLen - 1, 0,	timDivMenuStart,		timDivMenuWidth, 		2},
		{ &menuVoltDiv, 	menuVoltDivLen - 1, 0,	voltDivMenuStart,		voltDivMenuWidth, 		2},
		{ &menuMeasurement, menuTimeDivLen - 1, 0,	measurementMenuStart,	measurementMenuWidth, 	2},
};


#define	scrollMenuStart			50
#define	scrollWidth				20



#endif /* MULTIMETERDEF_H_ */

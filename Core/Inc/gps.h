#ifndef __GPS_H
#define __GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

enum GRESULT {G_OK, G_PARSE, G_NOLOC};

typedef struct {
	RTC_TimeTypeDef ti;
	RTC_DateTypeDef da;
	float la;
	float lo;

} GPSData;

int parseNMEA(char* nmea, GPSData* gps);


#ifdef __cplusplus
}
#endif

#endif

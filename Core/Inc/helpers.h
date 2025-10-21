#ifndef __HELPERS_H
#define __HELPERS_H

#include "fatfs.h"

extern ADC_HandleTypeDef hadc1;
extern FIL LogFile;
extern FIL DataFile;
extern char log_path[256];
extern char data_path[256];

void initSDCard();

FRESULT createLogFile();

FRESULT createDataFile();


extern float ADC_off;
extern float MOS_drop;

extern uint32_t start_ms;

float pollBatteryVoltage();

void setupAccSleep();

void setupAccWake();

void waitForInterruptAndWakeup();

void ErrorHandler();

#endif

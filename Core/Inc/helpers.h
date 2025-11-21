#ifndef __HELPERS_H
#define __HELPERS_H

#include "fatfs.h"

extern ADC_HandleTypeDef hadc1;
extern FIL LogFile;
extern FIL DataFile;
extern char log_path[256];
extern char data_path[256];
extern uint8_t SD_buffer[32768];

enum PREFIX {
	M_LOG, M_ADC, M_GPS, M_TP, M_ACC, M_IMU
};

void initSDCard();

FRESULT createLogFile();

FRESULT createDataFile();


extern float ADC_off;
extern float MOS_drop;

extern uint32_t start_ms;

float pollBatteryVoltage();

void pollADC(uint8_t** buf);

void write_buf(uint8_t** buf,void* val, uint32_t sz);

void flush_buf(uint8_t** buf);

void setupTPSens();

void setupAccSleep();

void setupAccWake();

void waitForInterruptAndWakeup();

void ErrorHandler();

#endif

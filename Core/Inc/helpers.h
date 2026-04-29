#ifndef __HELPERS_H
#define __HELPERS_H

#include "fatfs.h"
#include "bmi270.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;
extern FIL LogFile;
extern FIL DataFile;
extern char log_path[256];
extern char data_path[256];
extern uint8_t SD_buffer[32768];
extern uint8_t bmi270addr;
extern struct bmi2_dev b270dev;
extern uint8_t button_wake;
extern uint8_t* SD_writebuf;

int8_t bmi2_i2c_write(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, uint8_t* interfacePtr);
int8_t bmi2_i2c_read(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, uint8_t* interfacePtr);
void bmi2_delay_us(uint32_t period, void *intf_ptr);

enum PREFIX {
	M_LOG, M_ADC, M_GPS, M_TP, M_ACC, M_IMU, M_TIME
};

void initSDCard();

void setupIMU();

FRESULT createLogFile();

FRESULT createDataFile();


extern float ADC_off;
extern float MOS_drop;

extern uint32_t start_ms;

float pollBatteryVoltage();

void pollADC(uint8_t** buf);

void write_buf(uint8_t** buf,uint8_t* val, uint32_t sz);

void flush_buf(uint8_t** buf);

void setupTPSens();

void setupAccSleep();

void setupAccWake();

void waitForInterruptAndWakeup();

void ErrorHandler();

#endif

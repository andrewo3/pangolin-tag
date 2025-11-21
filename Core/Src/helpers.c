#include "helpers.h"
#include "fatfs.h"
#include "main.h"

FIL LogFile;
FIL DataFile;
char log_path[256];
char data_path[256];
uint8_t SD_buffer[32768];

float ADC_off = 0.15;
float MOS_drop = 0.12;

float pollBatteryVoltage() {
	HAL_ADC_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t adc = HAL_ADC_GetValue(&hadc1);
	float V = ((3.3 * (float)adc/4095 * 3) + ADC_off + MOS_drop) * 2;
	return V;
}

void pollADC(uint8_t** buffer) {
	HAL_ADC_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t adc = HAL_ADC_GetValue(&hadc1);
	enum PREFIX type = M_ADC;
	write_buf(buffer,&type,sizeof(type));
	write_buf(buffer,&adc,sizeof(adc));
	return;
}

void write_buf(uint8_t** buf,void* val, uint32_t sz) {
	uint32_t current_sz = (*buf)-SD_buffer;
	if (current_sz + sz >= 32768) { //buffer overflow
		// write to SD
		 FRESULT res = f_open(&DataFile, data_path, FA_OPEN_APPEND|FA_WRITE);
		 if (res != FR_OK) {
			log_printf("ERR: Failed to reopen %s - error code: %i\r\n",data_path,res);
			Error_Handler();
		 }
		 res = f_write(&DataFile,SD_buffer, current_sz, NULL);
		 f_close(&DataFile);
		 *buf = SD_buffer;
	}
	memcpy(*buf,val,sz);
	*buf+=sz;
	printf("Write - new buf size: %i\r\n",(*buf)-SD_buffer);
}

void flush_buf(uint8_t** buf) {
	uint32_t current_sz = (*buf)-SD_buffer;
	FRESULT res = f_open(&DataFile, data_path, FA_OPEN_APPEND|FA_WRITE);
	 if (res != FR_OK) {
		log_printf("ERR: Failed to reopen %s - error code: %i\r\n",data_path,res);
		Error_Handler();
	 }
	 res = f_write(&DataFile,SD_buffer, current_sz, NULL);
	 char* end_char = "e";
	 f_write(&DataFile,end_char,1,NULL);
	 f_close(&DataFile);
	 *buf = SD_buffer;
}

void setupTPSens() {
	uint8_t addr = 0x60;
	uint8_t out_bytes[5];
	uint8_t CTRL_REG1 = 0x26;
	uint8_t PT_DATA_CFG = 0x13;
	uint8_t STATUS = 0x00;

	//enable data flags
	Write_I2C_Reg(addr,PT_DATA_CFG,0x7,1);
	//0x3A to CTRL_REG1 to activate one reading
	Write_I2C_Reg(addr,CTRL_REG1,0x39,1);
}

void setupAccSleep() {
	//setup accelerometer for generating interrupts on movement
	log_printf("LOG: Configuring accelerometer for sleep\r\n");
	uint8_t ACC_ADDR = 0b0011001;
	uint8_t x_reg = 0x28;
	uint8_t y_reg = 0x2A;
	uint8_t z_reg = 0x2C;

	uint8_t CTRL_REG1 = 0x20;
	uint8_t CTRL_REG2 = 0x21;
	uint8_t CTRL_REG3 = 0x22;
	uint8_t CTRL_REG4 = 0x23;
	uint8_t CTRL_REG5 = 0x24;
	uint8_t INT1_THS = 0x32;
	uint8_t INT1_DURATION = 0x33;
	uint8_t INT1_CFG = 0x30;
	uint8_t REFERENCE = 0x26;

	//set 100 Hz low-power mode
	Write_I2C_Reg(ACC_ADDR, CTRL_REG1, 0b01011111,2);
	// use high pass filter
	Write_I2C_Reg(ACC_ADDR, CTRL_REG2, 0b00001001,2);
	//enable interrupt 1
	Write_I2C_Reg(ACC_ADDR, CTRL_REG3, 0b01000000,2);
	// set +-2g range
	Write_I2C_Reg(ACC_ADDR, CTRL_REG4, 0b00000000,2);
	// latch interrupt 1
	Write_I2C_Reg(ACC_ADDR, CTRL_REG5, 0b00001000,2);
	// set 384 mg threshold on interrupt
	Write_I2C_Reg(ACC_ADDR, INT1_THS, 0b00011000,2);
	//set 20 ms duration before interrupt is recognized
	Write_I2C_Reg(ACC_ADDR, INT1_DURATION, 0b00000010,2);

	uint8_t ref = Read_I2C_Reg(ACC_ADDR, REFERENCE,2);

	//generate interrupt on high events of x, y, or z
	Write_I2C_Reg(ACC_ADDR, INT1_CFG, 0b00101010,2);
	log_printf("LOG: successfully configured accelerometer.\r\n");


}

void setupAccWake() {
	//setup accelerometer for generating interrupts on movement
	uint8_t ACC_ADDR = 0b0011001;
	uint8_t x_reg = 0x28;
	uint8_t y_reg = 0x2A;
	uint8_t z_reg = 0x2C;

	uint8_t CTRL_REG1 = 0x20;
	uint8_t CTRL_REG2 = 0x21;
	uint8_t CTRL_REG3 = 0x22;
	uint8_t CTRL_REG4 = 0x23;
	uint8_t CTRL_REG5 = 0x24;

	uint8_t INT1_CFG = 0x30;
	uint8_t INT1_SRC = 0x31;

	//read interrupt res to clear it
	uint8_t int_event = Read_I2C_Reg(ACC_ADDR, INT1_SRC);

	//low-power 1 Hz
	Write_I2C_Reg(ACC_ADDR, CTRL_REG1, 0b00011111,2);
	//no filters
	Write_I2C_Reg(ACC_ADDR, CTRL_REG2, 0b00000000,2);
	//disable interrupts
	Write_I2C_Reg(ACC_ADDR, CTRL_REG3, 0b00000000,2);
	//+-4g range
	Write_I2C_Reg(ACC_ADDR, CTRL_REG4, 0b00001000,2);
	//no interrupt functions
	Write_I2C_Reg(ACC_ADDR, CTRL_REG5, 0b00000000,2);

	//disable interrupts on high events of x, y, or z
	Write_I2C_Reg(ACC_ADDR, INT1_CFG, 0b00000000,2);


}

void initSDCard() {
	if (retSD != 0) {
	  printf("ERR: Failed to Link SD.\r\n");
	  ErrorHandler();
	}

	// check if SD card is in slot
	if(HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) != GPIO_PIN_RESET) {
	  printf("SD Card detected in slot.\r\n");
	} else {
	  printf("SD Card was not detected in slot.\r\n");
	}
	//mount sd card if exists
	FRESULT mount_success = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
	if(mount_success != FR_OK)
	{
		printf("ERR: Failed to mount SD Card. Error code: %i\r\n",mount_success);
		Error_Handler();
	}
}


FRESULT createLogFile() {
	FRESULT log_res;
	int log_num = 0;
	do {
		sprintf(log_path,"log%i.txt\0",log_num);
		//printf("Trying %s\r\n",log_path);
		log_res = f_open(&LogFile, log_path, FA_CREATE_NEW | FA_WRITE);
		log_num++;

	} while (log_res == FR_EXIST); // continue until file doesnt exist
	if (log_res != FR_OK) {
		printf("ERR: Failed to create log - error code: %i\r\n",log_res);
		ErrorHandler();
	}
	f_close(&LogFile);
	printf("Using log at %s\r\n",log_path);
}

FRESULT createDataFile() {
	FRESULT data_res;
	int data_num = 0;
	do {
		sprintf(data_path,"data%i.bin\0",data_num);
		//printf("Trying %s\r\n",log_path);
		data_res = f_open(&DataFile, data_path, FA_CREATE_NEW | FA_WRITE);
		data_num++;

	} while (data_res == FR_EXIST); // continue until file doesnt exist
	if (data_res != FR_OK) {
		log_printf("ERR: Failed to create data file - error code: %i\r\n",data_res);
		ErrorHandler();
	}
	char* firstLine = "i";
	f_write(&DataFile, firstLine, strlen(firstLine), NULL);
	f_close(&DataFile);
	printf("Made data file: %s\r\n",data_path);
}

void waitForInterruptAndWakeup() {
	//set accelerometer for sleep interrupts
	setupAccSleep();
	//disable power. setting high causes sensors to turn off
	HAL_GPIO_WritePin(Power_Enable_GPIO_Port, Power_Enable_Pin, GPIO_PIN_SET);
	__HAL_GPIO_EXTI_CLEAR_IT(Push_Button_Pin);
	__HAL_GPIO_EXTI_CLEAR_IT(Acc_Int_Pin);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	__enable_irq();
	HAL_SuspendTick();
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

	SystemClock_Config();
	PeriphCommonClock_Config();
	HAL_ResumeTick();

	//initSDCard();
	printf("SD mount: %p\r\n",&SDFatFS);

	FRESULT log_res = f_open(&LogFile, log_path, FA_WRITE | FA_OPEN_APPEND);
	if (log_res != FR_OK) {
		printf("ERR: Failed to create log - error code: %i\r\n",log_res);
		ErrorHandler();
	}
	f_close(&LogFile);
	printf("Reopened log file.\r\n");

	FRESULT data_res = f_open(&DataFile, data_path, FA_WRITE | FA_OPEN_APPEND);
	if (data_res != FR_OK) {
		log_printf("ERR: Failed to create data file - error code: %i\r\n",data_res);
		ErrorHandler();
	}
	f_close(&DataFile);
	printf("Reopened data file.\r\n");

	HAL_GPIO_WritePin(Power_Enable_GPIO_Port, Power_Enable_Pin, GPIO_PIN_RESET);
	start_ms = HAL_GetTick();
	//set accelerometer to track motion when awake
	setupAccWake();
	//setup temperature sensor
	setupTPSens();
}

void ErrorHandler()
{
	HAL_GPIO_WritePin(GPIOA, Error_LED_Pin, GPIO_PIN_SET);
	while (1)
	{
	  blink(1,500);
	}
}

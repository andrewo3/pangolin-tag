/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
//#define ENABLE_SCRATCH_BUFFER
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "helpers.h"
#include "gps.h"
#include <math.h>
#include <string.h>
#include "datetime.h"
#include <stdarg.h>
#include "bmi270.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
DMA_HandleTypeDef hdma_sdmmc1;
uint8_t on = 0;
uint32_t start_ms;
uint8_t state_change = 0;
long last_button_press = 0;
uint8_t button_wake = 0;
uint8_t imu_dt = 0; // in ms

RTC_TimeTypeDef ti;
RTC_DateTypeDef da;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* Private function prototypes -----------------------------------------------*/

void blink(int count, int dur) {
	for (int i = 0; i < count; i++) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(dur);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(dur);
	}
}

void log_printf(const char* fmt, ...) {
	char buf[1024];

	va_list args;

	va_start(args,fmt);
	vsprintf(buf,fmt,args);
	va_end(args);

	FRESULT open_success = f_open(&LogFile, log_path, FA_OPEN_APPEND|FA_WRITE);
	f_write(&LogFile, buf, strlen(buf), NULL);
	f_close(&LogFile);

	printf("%s",buf);
}

void log_raw(uint8_t** buffer, int status) {
	write_buf(buffer,&status,4);
}

HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd);

uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
	uint8_t sd_state = MSD_OK;
  /* Invalidate the dma tx handle*/
  hsd1.hdmatx = NULL;

  /* Prepare the dma channel for a read operation */
  sd_state = SD_DMAConfigRx(&hsd1);

  if(sd_state == HAL_OK)
  {
	   /* Read block(s) in DMA transfer mode */
		sd_state = HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks);
  }

  if( sd_state == HAL_OK)
  {
	return MSD_OK;
  }
  else
  {
	return MSD_ERROR;
  }
}

uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
	uint8_t sd_state = MSD_OK;

	  // Invalidate the dma rx handle
	  hsd1.hdmarx = NULL;

	  // Prepare the dma channel for a read operation
	  sd_state = SD_DMAConfigTx(&hsd1);

	  if(sd_state == HAL_OK)
	  {
		/* Write block(s) in DMA transfer mode */
		sd_state = HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks);
	  }

	  if( sd_state == HAL_OK)
	  {
		return MSD_OK;
	  }
	  else
	  {
		return MSD_ERROR;
	  }

	  return sd_state;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	printf("Interrupt Triggered\r\n");
	if (GPIO_Pin == Push_Button_Pin && HAL_GetTick() - last_button_press > 300) {
	    last_button_press = HAL_GetTick();
	    on ^= 1;
	    state_change = 1;
	    button_wake = 1;
	}

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Read_I2C_Reg(uint8_t addr, uint8_t reg, uint8_t dev) {
	I2C_HandleTypeDef* i2cdev;
	if (dev == 1) {
		i2cdev = &hi2c1;
	} else if (dev == 2) {
		i2cdev = &hi2c2;
	} else {
		return HAL_ERROR; // wrong device
	}
	uint8_t out;
	uint8_t res = HAL_I2C_Master_Transmit(i2cdev,addr << 1,&reg,1,10000);
	res = HAL_I2C_Master_Receive(i2cdev,addr << 1,&out,1,10000);
	if (res != HAL_OK) {
		log_printf("ERR: Failed to receive data from I2C module at address %02x - err code %i\r\n",addr, (*i2cdev).ErrorCode);
		Error_Handler();
	}
	return out;
}

uint8_t Read_I2C_Reg_NoStop(uint8_t addr, uint8_t reg, uint8_t dev) {
	I2C_HandleTypeDef* i2cdev;
	if (dev == 1) {
		i2cdev = &hi2c1;
	} else if (dev == 2) {
		i2cdev = &hi2c2;
	} else {
		return HAL_ERROR; // wrong device
	}
	uint8_t out;
	uint8_t res = HAL_I2C_Mem_Read(i2cdev,addr << 1, reg, 1, &out, 1, 10000);
	if (res != HAL_OK) {
		log_printf("ERR: Failed to read memory from I2C module at address %02x - err code %i\r\n",addr, (*i2cdev).ErrorCode);
		Error_Handler();
	}
	return out;
}

void Write_I2C_Reg(uint8_t addr, uint8_t reg, uint8_t data, uint8_t dev) {
	I2C_HandleTypeDef* i2cdev;
	if (dev == 1) {
		i2cdev = &hi2c1;
	} else if (dev == 2) {
		i2cdev = &hi2c2;
	} else {
		return; // wrong device
	}
	uint8_t wr[2] = {reg,data};
	uint8_t res = HAL_I2C_Master_Transmit(i2cdev,addr << 1,&wr,2,10000);
	if (res != HAL_OK) {
		log_printf("ERR: Failed to write data to I2C module at address %02x - err code %i\r\n",addr, (*i2cdev).ErrorCode);
		Error_Handler();
	}
}

int8_t bmi2_i2c_write(uint8_t dev_id,
                      uint8_t reg_addr,
                      const uint8_t *data,
                      uint16_t len)
{
    if (HAL_I2C_Mem_Write(&hi2c1,
                          dev_id << 1,
                          reg_addr,
                          I2C_MEMADD_SIZE_8BIT,
                          (uint8_t*)data,
                          len,
                          HAL_MAX_DELAY) == HAL_OK)
    {
        return BMI2_OK;
    }
    return BMI2_E_COM_FAIL;
}

int8_t bmi2_i2c_read(uint8_t dev_id,
                     uint8_t reg_addr,
                     uint8_t *data,
                     uint16_t len)
{
    if (HAL_I2C_Mem_Read(&hi2c1,
                         dev_id << 1,
                         reg_addr,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         len,
                         HAL_MAX_DELAY) == HAL_OK)
    {
        return BMI2_OK;
    }
    return BMI2_E_COM_FAIL;
}

void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    // Bosch uses microseconds, STM32 HAL delays in ms.
    // For small values you should use a timer-based delay if needed.
    HAL_Delay(period / 1000);
}


uint8_t alt_set = 0;
float alt_ref;


void Get_TPSens(float* tmp_ret, float* prs_ret) {
	uint8_t addr = 0x76;
	uint8_t PSR_B2 = 0;
	uint8_t TMP_B2 = 3;

	uint8_t PRS_CFG = 6;
	uint8_t TMP_CFG = 7;
	uint8_t TMP_COEF_SRCE = 0x28;
	uint8_t MEAS_CFG = 8;
	uint8_t CFG_REG = 9;
	uint32_t prec_table[8] = {
			524288,
			1572864,
			3670016,
			7864320,
			253952,
			516096,
			1040384,
			2088960
	};

	//get coeffs
	uint8_t coef_src = Read_I2C_Reg(addr,TMP_COEF_SRCE,1) & 0x80;
	uint8_t coeffs[18] = {0};
	for (int i = 0; i < 18; i++) {
		coeffs[i] = Read_I2C_Reg(addr,i+0x10,1);
	}
	int16_t c0 = ((coeffs[1] & 0xf0) >> 4) | ((int16_t)coeffs[0] << 4);
	if (c0 & 0x800) {
		c0 |= 0xF000;
	}
	int16_t c1 = coeffs[2] | ((coeffs[1] & 0xf) << 8);
	if (c1 & 0x800) {
		c1 |= 0xF000;
	}
	int32_t c00 = ((coeffs[5] & 0xf0) >> 4) | (coeffs[4] << 4) | (coeffs[3] << 12);
	if (c00 & 0x80000) {
		c00 |= 0xFFF00000;
	}
	int32_t c10 = coeffs[7] | (coeffs[6] << 8) | ((coeffs[5] & 0xf) << 16);
	if (c10 & 0x80000) {
		c10 |= 0xFFF00000;
	}
	int16_t c01 = coeffs[9] | (coeffs[8] << 8);
	int16_t c11 = coeffs[11] | (coeffs[10] << 8);
	int16_t c20 = coeffs[13] | (coeffs[12] << 8);
	int16_t c21 = coeffs[15] | (coeffs[14] << 8);
	int16_t c30 = coeffs[17] | (coeffs[16] << 8);

	//enable pressure and temp
	Write_I2C_Reg(addr,TMP_CFG,coef_src,1); // set correct temp measurement source
	Write_I2C_Reg(addr,MEAS_CFG,7,1);
	uint8_t meas = 0;

	// wait until everything returns as ready
	while (meas & 0xf0 != 0xf0) {
		meas = Read_I2C_Reg(addr,MEAS_CFG,1);
	}

	uint8_t cfg = Read_I2C_Reg(addr,CFG_REG,1);
	uint8_t tshift = (cfg & 0x8) ? 1 : 0;
	uint8_t pshift = (cfg & 0x4) ? 1 : 0;

	uint32_t kT = prec_table[tshift];
	uint32_t kP = prec_table[pshift];

	uint8_t tprec = Read_I2C_Reg(addr,TMP_CFG,1) & 0x7;
	uint8_t pprec = Read_I2C_Reg(addr,PRS_CFG,1) & 0x7;

	//get pressure measurement
	int32_t psr = 0;
	for (int i = 0; i < 3; i++) {
		uint8_t b = Read_I2C_Reg(addr,PSR_B2+i,1);
		psr |= b << (2 - i) * 8;
	}

	//sign extend 2s complement
	if (psr & 0x800000) {
		psr |= 0xFF000000;
	}

	psr >>= pshift;

	//log_printf("Read %i (psr) from pressure sensor.\r\n",psr);

	int32_t tmp = 0;
	for (int i = 0; i < 3; i++) {
		uint8_t b = Read_I2C_Reg(addr,TMP_B2+i,1);
		tmp |=  b << (2 - i) * 8;
	}

	//sign extend 2s complement
	if (tmp & 0x800000) {
		tmp |= 0xFF000000;
	}

	tmp >>= tshift;

	//log_printf("Read %i (tmp) from pressure sensor (shift=%i).\r\n",tmp,tshift);

	float pscaled = (float)psr / kP;
	float tscaled = (float)tmp / kT;

	float Pcomp = c00 +
			pscaled * (c10 + pscaled * (c20 + pscaled * c30)) +
			tscaled * c01 +
			tscaled * pscaled * (c11 + pscaled * c21);
	float Tcomp = c0*0.5 + c1*tscaled;

	*prs_ret = Pcomp;
	*tmp_ret = Tcomp;

}

void Get_Acc(float* vacc) {
	uint8_t ACC_ADDR = 0b0011001;
	uint8_t x_reg = 0x28;
	uint8_t y_reg = 0x2A;
	uint8_t z_reg = 0x2C;

	float sens = 0.001; // g/digit * m/s^2/g

	uint8_t CTRL_REG1 = 0x20;
	uint8_t CTRL_REG4 = 0x23;

	int16_t x_out = (Read_I2C_Reg(ACC_ADDR,x_reg,2) | (Read_I2C_Reg(ACC_ADDR,x_reg + 1,2) << 8)) >> 4;
	if (x_out & 0x800) {
		x_out |= 0xF000;
	}

	int16_t y_out = (Read_I2C_Reg(ACC_ADDR,y_reg,2) | (Read_I2C_Reg(ACC_ADDR,y_reg + 1,2) << 8)) >> 4;
	if (y_out & 0x800) {
		y_out |= 0xF000;
	}

	int16_t z_out = (Read_I2C_Reg(ACC_ADDR,z_reg,2) | (Read_I2C_Reg(ACC_ADDR,z_reg + 1,2) << 8)) >> 4;
	if (z_out & 0x800) {
		z_out |= 0xF000;
	}

	vacc[0] = x_out * sens;
	vacc[1] = y_out * sens;
	vacc[2] = z_out * sens;

}

void Raw_Acc(uint8_t** buf) {
	uint8_t ACC_ADDR = 0b0011001;
	uint8_t x_reg = 0x28;
	uint8_t out[6];

	for (int i = 0; i < 6; i++) {
		out[i] = Read_I2C_Reg(ACC_ADDR,x_reg+i,2);
	}
	enum PREFIX tp_type = M_ACC;
	write_buf(buf,&tp_type,sizeof(tp_type));
	write_buf(buf,out,6);
}
void Setup_IMU() {
	//write bmi270 config file
	struct bmi2_dev dev = {0};

	uint8_t i2c_addr = 0x68;
	dev.intf = BMI2_I2C_INTF;
	dev.read = bmi2_i2c_read;
	dev.write = bmi2_i2c_write;
	dev.delay_us = bmi2_delay_us;
	dev.intf_ptr = &i2c_addr;
	int8_t rslt;

	rslt = bmi270_init(&dev);
	if (rslt != BMI2_OK) {
		log_printf("LOG: Failed to initialize BMI270.\r\n");
	    Error_Handler();
	}

	rslt = bmi270_load_config(&dev);
	if (rslt != BMI2_OK) {
		log_printf("LOG: Failed to load BMI270 config.\r\n");
	    Error_Handler();
	}

	uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
	rslt = bmi270_sensor_enable(sens_list, 2, &dev);

}

void Raw_IMU(uint8_t** buf) {
	uint8_t IMU_ADDR = 0x68;
	uint8_t DATA_8 = 0x0C; // start of actual data
	uint8_t SENSORTIME = 0x18;
	uint8_t imu_out[12];
	uint8_t sensor_time[3];

	for (int i = 0; i < 6; i++) {
		imu_out[i] = Read_I2C_Reg(IMU_ADDR,DATA_8+i,1);
	}
	for (int i = 0; i < 3; i++) {
		sensor_time[i] = Read_I2C_Reg(IMU_ADDR,SENSORTIME+i,1);
	}



}

void Get_TPSens2(float* tmp_ret, float* prs_ret) {
	uint8_t addr = 0x60;
	uint8_t out_bytes[5];
	uint8_t CTRL_REG1 = 0x26;
	uint8_t PT_DATA_CFG = 0x13;
	uint8_t STATUS = 0x00;
	//log_printf("Status: %02x\r\n",Read_I2C_Reg_NoStop(addr,CTRL_REG1));
	while ((Read_I2C_Reg(addr,STATUS,1) & 0x0e) != 0x0e) {
		// wait until data ready
	}
	// read data
	for (int i = 0; i < 5; i++) {
		out_bytes[i] = Read_I2C_Reg_NoStop(addr, i+1,1);
	}

	float pasc = (((out_bytes[2] >> 6) & 0x3) | (out_bytes[1] << 2) | (out_bytes[0] << 10)) + (((out_bytes[2] >> 4) & 0x3)/4.0);
	*prs_ret = pasc;

	int8_t itemp = out_bytes[3];

	float temp = (float)itemp + ((out_bytes[4]>>4) & 0xf)/16.0;
	*tmp_ret = temp;

}

void Raw_TPSens(uint8_t** buf) {
	uint8_t addr = 0x60;
	uint8_t out_bytes[5];
	uint8_t CTRL_REG1 = 0x26;
	uint8_t PT_DATA_CFG = 0x13;
	uint8_t STATUS = 0x00;
	//log_printf("Status: %02x\r\n",Read_I2C_Reg_NoStop(addr,CTRL_REG1));
	while ((Read_I2C_Reg(addr,STATUS,1) & 0x0e) != 0x0e) {
		// wait until data ready
	}
	// read data
	for (int i = 0; i < 5; i++) {
		out_bytes[i] = Read_I2C_Reg_NoStop(addr, i+1,1);
	}
	enum PREFIX tp_type = M_TP;
	write_buf(buf,&tp_type,sizeof(tp_type));
	write_buf(buf,out_bytes,5);
}

void init_everything() {
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SDMMC1_SD_Init();
	MX_FATFS_Init();
	MX_RTC_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  /*
   * Initialize Processes
   */
  //disable power to non-essential peripherals
  HAL_GPIO_WritePin(Power_Enable_GPIO_Port, Power_Enable_Pin, GPIO_PIN_SET);
  initSDCard();
  printf("LOG: Mounted SD Card at %s\r\n",SDPath);

  createLogFile();
  uint32_t end_ms = start_ms;

  uint32_t elapsed_s = 0;
  uint32_t last_elapsed_s = 0;
  char write_str[256];

  char* AmPm[2] = {"AM","PM"};
  char* days[7] = {"Mon","Tue","Wed","Thur","Fri","Sat","Sun"};

  uint8_t recv[83];
  const uint16_t TEMP_SENS_ADDR = 0b1001000;
  uint8_t RTR = 0;
  uint8_t file_open = 0;

  uint8_t name_changed = 0;

  uint8_t led_on = 1;
  long count = 0;
  float ADC_off = 0.15;
  float MOS_drop = 0.12;

  //reset RTC to epoch
  DateTime epoch = fromepoch(0);
  HAL_RTC_SetDate(&hrtc,&epoch.da,RTC_FORMAT_BCD);


  //create file
  createDataFile();

  //disable power to non-essential peripherals
  HAL_GPIO_WritePin(Power_Enable_GPIO_Port, Power_Enable_Pin, GPIO_PIN_SET);

  //

  waitForInterruptAndWakeup();
  Setup_IMU();
  //initSDCard();
  led_on = 1;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, led_on);
  uint8_t* SD_writebuf = SD_buffer;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if (button_wake) { // if button was pressed to wake back up, flush buffer into SD card.
		 printf("Button wakeup: flushing buffer...\r\n");
		 flush_buf(&SD_writebuf);
		 button_wake = 0;
	 }
	 end_ms = HAL_GetTick();
	 last_elapsed_s = elapsed_s;
	 elapsed_s = (end_ms - start_ms) / 1000;
	 //check battery
	 //float V = pollBatteryVoltage();
	 //printf("Battery Voltage: %.2f V\r\n",V);
	 //determine LED
	 if (elapsed_s != last_elapsed_s) {
		 pollADC(&SD_writebuf);

		 count++;

		 float temp3;
		 float pascals2;
		 log_printf("LOG: Reading Temp + Barometer IC #2\r\n");
		 //Get_TPSens2(&temp3, &pascals2);
		 Raw_TPSens(&SD_writebuf);

		 float acc[3];
		 log_printf("LOG: Reading Accelerometer\r\n");
		 //Get_Acc(acc);
		 Raw_Acc(&SD_writebuf);
     
		 int uart_res = 0;
		 int msg_len = 0;

		 log_printf("LOG: Querying GPS Module\r\n");
		 char* current = recv;
		 current++;
		 uint8_t end = 0;
		 //continue until RMC message found
		 while (!end || strncmp(recv,"$GNRMC",6)) {

			 //receive new character from GPS
			 __HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
			  uart_res = HAL_UART_Receive(&huart1, current, 1, 1);
			  //reset message length when end found
			  if (current == recv) {
				  msg_len = 0;
			  }
			  //if successful
			  if (uart_res == 0) {
				  current++;
				  msg_len++;
				  //if we reach end of message
				  if ((*(current-1) == '\r' || *(current-1) == '\n') && current != recv) {
					  *current = 0;
					  current = recv;
					  //log_printf("Found one GPS message: %s\r",recv);
					  end = 1;
				  } else {
					  end = 0;
				  }
			  }
		 }
		 char* RMC = recv;
		 RMC[msg_len] = 0;
		 char* placeholder = "$GNRMC,151227.40,A,4723.54036,N,00826.88672,E,0.0,81.6,111022,,,R*7C";
		 strcpy(RMC,placeholder);
		 log_printf("LOG: %s\r\n", RMC);

		 GPSData gpsOut;
		 int gps_res = parseNMEA(RMC,&gpsOut);

		 ti = gpsOut.ti;
		 da = gpsOut.da;

		 if (gps_res == G_OK) {
			 enum PREFIX gps_type = M_GPS;
			 write_buf(&SD_writebuf,&gps_type,sizeof(gps_type));
			 write_buf(&SD_writebuf,&recv,83);
		 }

		 blink(1,10);
		 //blink every second to indicate that it is functioning if the SD card was detected
		 if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10) == 1) {
			 blink(1,10);
		 }
		 //HAL_Delay(1000);

		 //if gps was found, or more than 5 minutes passed, go back to sleep and wait for interrupt.
		 if (gps_res == G_OK || elapsed_s >= 10) {
			 led_on = 0;
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, led_on);
			 if (gps_res != G_OK) {
				 log_printf("Timeout reached.");
			 } else {
				 log_printf("GPS Fix acquired.");
			 }
			 log_printf(" Going back to sleep...\r\n");
			 waitForInterruptAndWakeup();
			 led_on = 1;
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, led_on);
		 }
	 }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  return;
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x1;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x30;
  sTime.TimeFormat = RTC_HOURFORMAT12_PM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_JUNE;
  sDate.Date = 0x10;
  sDate.Year = 0x25;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */
  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Battery_LED_GPIO_Port, Battery_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Power_Enable_GPIO_Port, Power_Enable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Error_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Push_Button_Pin */
  GPIO_InitStruct.Pin = Push_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Push_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Battery_LED_Pin */
  GPIO_InitStruct.Pin = Battery_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Battery_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Acc_Int_Pin */
  GPIO_InitStruct.Pin = Acc_Int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Acc_Int_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Power_Enable_Pin */
  GPIO_InitStruct.Pin = Power_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Power_Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Error_LED_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Error_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(GPIOA, Error_LED_Pin, GPIO_PIN_SET);
  while (1)
  {
	  blink(1,500);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: log_printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

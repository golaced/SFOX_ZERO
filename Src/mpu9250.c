#include "mpu9250.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "flash_on_chip.h"

#define MAG_READ_DELAY 30

#define MPU9250_Byte16(Type, ByteH, ByteL)  ((Type)((((uint16_t)(ByteH))<<8) | ((uint16_t)(ByteL))))
#define MPU9250_ON  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)
#define MPU9250_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)
#define CALIBRATING_GYRO_CYCLES             2048

extern osSemaphoreId myBinarySem01MPU9250GyroAccCalibrateOffsetHandle;
extern osSemaphoreId myBinarySem02LED1Handle;

SPI_HandleTypeDef *MPU9250_Handler;
char mpu_data_ok;
uint8_t MPU9250_data_buffer[28];//9250原始数据

float accx_raw_mps, accy_raw_mps, accz_raw_mps;
float accx_raw_bias_mps, accy_raw_bias_mps, accz_raw_bias_mps;
float gyrox_raw_dps, gyroy_raw_dps, gyroz_raw_dps;
float gyrox_raw_bias_dps, gyroy_raw_bias_dps, gyroz_raw_bias_dps;
float magx_raw_uT, magy_raw_uT, magz_raw_uT;
//float accx, accy, accz;
//float gyrox, gyroy, gyroz;
//float magx, magy, magz;
int MPU9250_ERROR;


static void simpdelay(void) {
	for (int i = 0; i < 84000; i++) {
		asm("nop");
	}
}

uint8_t MPU9250_ReadReg(uint8_t ReadAddr) {
	MPU9250_ON;
	uint8_t ReadData = 0;
	uint8_t tx = ReadAddr | 0x80;
	HAL_SPI_Transmit(MPU9250_Handler, &tx, 1, 100);
	HAL_SPI_Receive(MPU9250_Handler, &ReadData, 1, 100);
	MPU9250_OFF;
	return ReadData;
}

void MPU9250_WriteReg(uint8_t WriteAddr, uint8_t WriteData) {
        HAL_StatusTypeDef status;
	MPU9250_ON;
	status = HAL_SPI_Transmit(MPU9250_Handler, &WriteAddr, 1, 100);
	if (status != HAL_OK)
	{
		asm("nop");
		return;
	}
	status = HAL_SPI_Transmit(MPU9250_Handler, &WriteData, 1, 100);
        if (status != HAL_OK)
	{
		asm("nop");
		return;
	}
	MPU9250_OFF;
}

void MPU9250_ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes) {
	MPU9250_ON;
	uint8_t tx = ReadAddr | 0x80;
	HAL_SPI_Transmit(MPU9250_Handler, &tx, 1, 100);
	HAL_SPI_Receive(MPU9250_Handler, ReadBuf, Bytes, 100);
	MPU9250_OFF;
}

void MPU9250_Mag_WriteReg(uint8_t writeAddr, uint8_t writeData) {
	uint8_t  status = 0;
	int timeout = MAG_READ_DELAY;

	MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, writeAddr);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_DO, writeData);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
	simpdelay();

	do {
		status = MPU9250_ReadReg(MPU6500_I2C_MST_STATUS);
		simpdelay();
	} while (((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));
	simpdelay();
}

uint8_t MPU9250_Mag_ReadReg(uint8_t readAddr) {
	uint8_t status = 0;
	uint8_t readData = 0;
	int timeout = MAG_READ_DELAY;

	MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, readAddr);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
	simpdelay();

	do {
		status = MPU9250_ReadReg(MPU6500_I2C_MST_STATUS);
		simpdelay();
	} while (((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));

	readData = MPU9250_ReadReg(MPU6500_I2C_SLV4_DI);
	simpdelay();
	return readData;
}

void MPU9250_Mag_ReadRegs(uint8_t readAddr, uint8_t *readData, uint8_t lens) {
	for (uint8_t i = 0; i < lens; i++) {
		readData[i] = MPU9250_Mag_ReadReg(readAddr + i);
		simpdelay();
	}
}

#define AK8963_CNTL1_Value 0x16
uint8_t MPU9250_Mag_Init(void) {
	uint8_t buf;
	while (1) {
		MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, 0x0C);  //write I2C addr
		simpdelay();
		MPU9250_WriteReg(MPU6500_I2C_SLV0_DO, AK8963_CNTL1_Value);
		MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_CNTL1);     // Set Write Reg
		MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x81);          // Start Write, 1 bytes
		// read back to check
		simpdelay();
		MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, 0x8C);//read I2C addr
		MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_CNTL1);     // Set Write Reg
		MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x81);          // Start Read, 6 bytes
		simpdelay();
		buf = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_00);   // Read Data
		if (buf != AK8963_CNTL1_Value) {
			asm("nop");
			return ERROR;
		}
		else
			return SUCCESS;
	}
}

#define MPU9250_InitRegNum 10
void MPU9250_Init(SPI_HandleTypeDef *hspi) {
	MPU9250_Handler = hspi;
        gyrox_raw_bias_dps = 0; 
        gyroy_raw_bias_dps = 0, 
        gyroz_raw_bias_dps = 0;
        accx_raw_bias_mps = 0;
        accy_raw_bias_mps = 0;
        accz_raw_bias_mps = 0;
	uint8_t i = 0;
	uint8_t MPU6500_InitData[MPU9250_InitRegNum][2] = {
		//{ 0x80, MPU6500_PWR_MGMT_1 },     // Reset Device
		{ 0x04, MPU6500_PWR_MGMT_1 },     // Clock Source
		{ 0x10, MPU6500_INT_PIN_CFG },    // Set INT_ANYRD_2CLEAR
		{ 0x01, MPU6500_INT_ENABLE },     // Set RAW_RDY_EN
		{ 0x00, MPU6500_PWR_MGMT_2 },     // Enable Acc & Gyro
		{ 0x13, MPU6500_GYRO_CONFIG },    // default : +-1000dps
		{ 0x08, MPU6500_ACCEL_CONFIG },   // default : +-4G
		{ 0x07, MPU6500_CONFIG },         // default : LPS_3600Hz
		{ 0x08, MPU6500_ACCEL_CONFIG_2 }, // default : LPS_1.13KHz
		{ 0x40, MPU6500_I2C_MST_CTRL },
		{ 0x35, MPU6500_USER_CTRL }, // Set I2C_MST_EN, I2C_IF_DIS
	};
	for (i = 0; i < MPU9250_InitRegNum; i++) {
		MPU9250_WriteReg(MPU6500_InitData[i][1], MPU6500_InitData[i][0]);
		simpdelay();
	}
	if(MPU9250_Check() != SUCCESS)
	{
		MPU9250_ERROR = 1;
	}
	else
	{
		MPU9250_ERROR = 0;
	}
//	while (MPU9250_Check() != SUCCESS)
//	{
//		asm("nop");
//	}
}

uint8_t MPU9250_Check(void) {
	uint8_t DeviceID = 0x00;
	/* MPU6500 Check*/
	DeviceID = 0x00;
	DeviceID = MPU9250_ReadReg(MPU6500_WHO_AM_I);
	if (DeviceID != MPU6500_Device_ID)
		return ERROR;
	/* AK8975 Check */
	DeviceID = 0x00;
	DeviceID = MPU9250_Mag_ReadReg(AK8963_WIA);
	if (DeviceID != AK8963_Device_ID) {
		return ERROR;
	}
	uint8_t tmpRead[3];

	MPU9250_Mag_WriteReg(AK8963_CNTL2, 0x01);       // Reset Device
	simpdelay();

	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x10);       // Power-down mode
	simpdelay();

	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x1F);       // Fuse ROM access mode

	MPU9250_Mag_ReadRegs(AK8963_ASAX, tmpRead, 3);  // Read sensitivity adjustment values
	simpdelay();
	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x10);       // Power-down mode
	simpdelay();

	if (tmpRead[0] == 0x00 || tmpRead[1] == 0x00 || tmpRead[2] == 0x00) {
		return ERROR;
	}
	simpdelay();
	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x16);       // 连续测量模式2
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, 0x09); //关闭slv4,陀螺仪加速度计odr=1000,延迟9个周期,磁力计50Hz(磁力计ODR=8Hz)
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_MST_DELAY_CTRL, 0x81); //开启延迟
	simpdelay();

	MPU9250_WriteReg(MPU6500_I2C_MST_CTRL, 0x5D);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_ST1);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, MPU6500_I2C_SLVx_EN | 8);//从st1开始读8个字节,中间六个为磁场数据,最后是st2
	simpdelay();

	return SUCCESS;
}

void MPU9250_read_raw_data(void)
{
	HAL_StatusTypeDef status;
	uint8_t tx = MPU6500_ACCEL_XOUT_H | 0x80;
	MPU9250_ON;
	status = HAL_SPI_Transmit(MPU9250_Handler, &tx, 1, 100);
	if (status == HAL_BUSY)
	{
		HAL_SPI_DMAStop(MPU9250_Handler);
		asm("nop");
	}
	if (status != HAL_OK)
	{
		asm("nop");
		return;
	}
	if (mpu_data_ok == 0)
		status = HAL_SPI_Receive_DMA(MPU9250_Handler, MPU9250_data_buffer, 22);

	if (status != HAL_OK) {
		asm("nop");
		return;
	}
	return;
}

void MPU9250_data_ready_to_read(void)
{
	mpu_data_ok = 1;
	MPU9250_OFF;
}

 void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
 {
    if (hspi->Instance == SPI1)
    {
		MPU9250_data_ready_to_read();
    }
 }

void MPU9250_calibrate_offset_func(float gyrox, float gyroy, float gyroz, float accx, float accy, float accz)
{
  static float tempGyro[3];
  static float tempAcc[3];
  static uint16_t cnt_g = 0;
  static float gyrox_low  = 0;
  static float gyrox_high = 0;
  
  if(cnt_g==0)
  {
          cnt_g = 1;
          tempGyro[0]=0;
          tempGyro[1]=0;
          tempGyro[2]=0;
          tempAcc[0]=0;
          tempAcc[1]=0;
          tempAcc[2]=0;
          gyrox_low  = (float)gyrox;
          gyrox_high = (float)gyrox;
          osSemaphoreRelease(myBinarySem01MPU9250GyroAccCalibrateOffsetHandle);// 释放信号量
          return;
  }
  if(cnt_g == CALIBRATING_GYRO_CYCLES)
  {
          cnt_g = 0;
          //Gyro_CALIBRATED = 0;
          //reset_flag=64;

          gyrox_raw_bias_dps  = tempGyro[0]/(float)CALIBRATING_GYRO_CYCLES;
          gyroy_raw_bias_dps  = tempGyro[1]/(float)CALIBRATING_GYRO_CYCLES;
          gyroz_raw_bias_dps  = tempGyro[2]/(float)CALIBRATING_GYRO_CYCLES;
          
          accx_raw_bias_mps = tempAcc[0]/(float)CALIBRATING_GYRO_CYCLES;
          accy_raw_bias_mps = tempAcc[1]/(float)CALIBRATING_GYRO_CYCLES;
          accz_raw_bias_mps = tempAcc[2]/(float)CALIBRATING_GYRO_CYCLES;
          
          flash_save_parameters();

          return;
  }
  tempGyro[0]+=(float)gyrox;
  tempGyro[1]+=(float)gyroy;
  tempGyro[2]+=(float)gyroz;
  
  tempAcc[0]+=(float)accx;
  tempAcc[1]+=(float)accy;
  tempAcc[2]+=(float)accz;

  if(gyrox_low  > gyrox)	gyrox_low  = gyrox;
  if(gyrox_high < gyrox)	gyrox_high = gyrox;
  
  if((gyrox_high-gyrox_low)>250)
  {
          cnt_g = 0;
          return;
  }
  cnt_g++;
  
  gyrox_raw_bias_dps = 0;
  gyroy_raw_bias_dps = 0;
  gyroz_raw_bias_dps = 0;
  
  accx_raw_bias_mps = 0;
  accy_raw_bias_mps = 0;
  accz_raw_bias_mps = 0;
  
  osSemaphoreRelease(myBinarySem01MPU9250GyroAccCalibrateOffsetHandle);// 释放信号量
}

void MPU9250_data_push(void)
{
	//+-4g m/s^2
	accx_raw_mps = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[0], MPU9250_data_buffer[1]))*MPU9250A_4g - accx_raw_bias_mps;
	accy_raw_mps = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[2], MPU9250_data_buffer[3]))*MPU9250A_4g - accy_raw_bias_mps;
	accz_raw_mps = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[4], MPU9250_data_buffer[5]))*MPU9250A_4g;

	//+-1000dps  
	gyrox_raw_dps = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[8], MPU9250_data_buffer[9]))*MPU9250G_1000dps - gyrox_raw_bias_dps;
	gyroy_raw_dps = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[10], MPU9250_data_buffer[11]))*MPU9250G_1000dps - gyroy_raw_bias_dps;
	gyroz_raw_dps = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[12], MPU9250_data_buffer[13]))*MPU9250G_1000dps - gyroz_raw_bias_dps;

	//NOTE:mag data's order is Y - X - Z
	magy_raw_uT = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[16], MPU9250_data_buffer[15]))*MPU9250M_4800uT;
	magx_raw_uT = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[18], MPU9250_data_buffer[17]))*MPU9250M_4800uT;
	magz_raw_uT = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[20], MPU9250_data_buffer[19]))*MPU9250M_4800uT;
}

void MPU9250_process(void)
{
	if (mpu_data_ok == 1)
	{
		mpu_data_ok = 0;
		MPU9250_data_push();
	}
	MPU9250_read_raw_data();
        if(osOK == osSemaphoreWait(myBinarySem01MPU9250GyroAccCalibrateOffsetHandle,0))
        {
                MPU9250_calibrate_offset_func(gyrox_raw_dps,gyroy_raw_dps,gyroz_raw_dps,accx_raw_mps, accy_raw_mps, accz_raw_mps);
                osSemaphoreRelease(myBinarySem02LED1Handle);// 释放信号量LED1
        }
}



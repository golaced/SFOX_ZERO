
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
extern osSemaphoreId myBinarySem02LED1ONHandle;
extern osSemaphoreId myBinarySem03LED2ONHandle;
extern osSemaphoreId myBinarySem04MPU9250MagCalibrateHandle;

SPI_HandleTypeDef *MPU9250_Handler;
char mpu_data_ok;
uint8_t MPU9250_data_buffer[28];//9250ԭʼ����

float accx_raw_mps, accy_raw_mps, accz_raw_mps;
float accx_raw_bias_mps, accy_raw_bias_mps, accz_raw_bias_mps;
float gyrox_raw_dps, gyroy_raw_dps, gyroz_raw_dps;
float gyrox_raw_bias_dps, gyroy_raw_bias_dps, gyroz_raw_bias_dps;
float magx_raw_uT[110], magy_raw_uT[110], magz_raw_uT[110];
float magx_fir_uT[1],magy_fir_uT[1],magz_fir_uT[1];
float magx,magy,magz;

float magx_offset,magy_offset,magz_offset;
float magx_gain = 1;
float magy_gain = 1;
float magz_gain = 1;

#define LPF_3HZ_100ORDER_LEN	101
static float LPF_3HZ_100ORDER[101] = {
    0.04754435644, 0.004697572906, 0.004919932224, 0.005150574725, 0.005376234651,
   0.005611456465,  0.00583946798, 0.006078177132, 0.006309270393, 0.006546489894,
   0.006777013186,  0.00701350579, 0.007245077752, 0.007482835092,  0.00771403173,
    0.00794617366, 0.008175152354, 0.008409786969, 0.008627979085, 0.008859299123,
   0.009071470238, 0.009296617471, 0.009500969201, 0.009705152363, 0.009914943948,
    0.01010550652,  0.01029173378,  0.01047855895,  0.01066237688,  0.01084415615,
    0.01101681869,  0.01118485257,  0.01134161744,  0.01149071939,  0.01162635349,
    0.01175176259,  0.01187754236,   0.0120053459,  0.01215935219,  0.01221348532,
    0.01231596619,  0.01241615135,  0.01247958653,  0.01255952101,  0.01260889601,
    0.01267067157,  0.01270479523,  0.01274325605,   0.0127606364,  0.01278058346,
    0.01277900953,  0.01278058346,   0.0127606364,  0.01274325605,  0.01270479523,
    0.01267067157,  0.01260889601,  0.01255952101,  0.01247958653,  0.01241615135,
    0.01231596619,  0.01221348532,  0.01215935219,   0.0120053459,  0.01187754236,
    0.01175176259,  0.01162635349,  0.01149071939,  0.01134161744,  0.01118485257,
    0.01101681869,  0.01084415615,  0.01066237688,  0.01047855895,  0.01029173378,
    0.01010550652, 0.009914943948, 0.009705152363, 0.009500969201, 0.009296617471,
   0.009071470238, 0.008859299123, 0.008627979085, 0.008409786969, 0.008175152354,
    0.00794617366,  0.00771403173, 0.007482835092, 0.007245077752,  0.00701350579,
   0.006777013186, 0.006546489894, 0.006309270393, 0.006078177132,  0.00583946798,
   0.005611456465, 0.005376234651, 0.005150574725, 0.004919932224, 0.004697572906,
    0.04754435644
};

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
	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x16);       // ��������ģʽ2
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, 0x09); //�ر�slv4,�����Ǽ��ٶȼ�odr=1000,�ӳ�9������,������50Hz(������ODR=8Hz)
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_MST_DELAY_CTRL, 0x81); //�����ӳ�
	simpdelay();

	MPU9250_WriteReg(MPU6500_I2C_MST_CTRL, 0x5D);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_ST1);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, MPU6500_I2C_SLVx_EN | 8);//��st1��ʼ��8���ֽ�,�м�����Ϊ�ų�����,�����st2
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

static void data_shift_1(float* ddd,int len)
{
  for(int i=len;i>=2;i--)		ddd[i-1] = ddd[i-2];
}

static void filter_fir(float* in, float* out,float* c,int len)
{
  out[0]=0;
  for(int i=0;i<len;i++)
          out[0] += (float)in[i] * c[i];
}

void MPU9250_gyro_acc_calibrate_offset_func(float gyrox, float gyroy, float gyroz, float accx, float accy, float accz)
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
          
          gyrox_raw_bias_dps = 0;
          gyroy_raw_bias_dps = 0;
          gyroz_raw_bias_dps = 0;

          accx_raw_bias_mps = 0;
          accy_raw_bias_mps = 0;
          accz_raw_bias_mps = 0;
          osSemaphoreRelease(myBinarySem01MPU9250GyroAccCalibrateOffsetHandle);// �ͷ��ź���
          return;
  }
  if(cnt_g == CALIBRATING_GYRO_CYCLES)
  {
          cnt_g = 0;

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
  
  osSemaphoreRelease(myBinarySem01MPU9250GyroAccCalibrateOffsetHandle);//
}

void MPU9250_mag_calibrate(int stage, float magx, float magy, float magz)
{
	static float magx_low  = 0;
  	static float magx_high = 0;

	static float magy_low  = 0;
  	static float magy_high = 0;

	static float magz_low  = 0;
  	static float magz_high = 0;

	static int cnt = 0;
	if(cnt++ == 0)
	{
		magx_offset = 0;
		magy_offset = 0;
		magz_offset = 0;
		magx_gain = 1;
		magy_gain = 1;
		magz_gain = 1;
		return;
	}

	if(stage == 1)
	{
		if(magx_low  > magx)	magx_low  = magx;
  		if(magx_high < magx)	magx_high = magx;

		if(magy_low  > magy)	magy_low  = magy;
  		if(magy_high < magy)	magy_high = magy;

		if(magz_low  > magz)	magz_low  = magz;
  		if(magz_high < magz)	magz_high = magz;
	}
	if(stage == 0)
	{
		magx_offset = (magx_low + magx_high) * 0.5f;
		magy_offset = (magy_low + magy_high) * 0.5f;
		magz_offset = (magz_low + magz_high) * 0.5f;

		magx_gain = 1;
		magy_gain = (magx_high - magx_low)/(magy_high - magy_low);
		magz_gain = (magx_high - magx_low)/(magz_high - magz_low);

		flash_save_parameters();

		cnt = 0;
	}
}

void MPU9250_data_push(void)
{
	//机头-X 右侧-Y 下方-Z   NED北东地坐标系
	//+-4g m/s^2
	accx_raw_mps = -(MPU9250_Byte16(int16_t, MPU9250_data_buffer[0], MPU9250_data_buffer[1]))*MPU9250A_4g - accx_raw_bias_mps;
	accy_raw_mps = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[2], MPU9250_data_buffer[3]))*MPU9250A_4g - accy_raw_bias_mps;
	accz_raw_mps = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[4], MPU9250_data_buffer[5]))*MPU9250A_4g;

	//+-1000dps  
	gyrox_raw_dps = -(MPU9250_Byte16(int16_t, MPU9250_data_buffer[8], MPU9250_data_buffer[9]))*MPU9250G_1000dps - gyrox_raw_bias_dps;
	gyroy_raw_dps = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[10], MPU9250_data_buffer[11]))*MPU9250G_1000dps - gyroy_raw_bias_dps;
	gyroz_raw_dps = -(MPU9250_Byte16(int16_t, MPU9250_data_buffer[12], MPU9250_data_buffer[13]))*MPU9250G_1000dps - gyroz_raw_bias_dps;

	//NOTE:mag data's order is Y - X - Z
	magy_raw_uT[0] = -(MPU9250_Byte16(int16_t, MPU9250_data_buffer[16], MPU9250_data_buffer[15]))*MPU9250M_4800uT;
	magx_raw_uT[0] = -(MPU9250_Byte16(int16_t, MPU9250_data_buffer[18], MPU9250_data_buffer[17]))*MPU9250M_4800uT;
	magz_raw_uT[0] = (MPU9250_Byte16(int16_t, MPU9250_data_buffer[20], MPU9250_data_buffer[19]))*MPU9250M_4800uT;

	data_shift_1(magx_raw_uT,LPF_3HZ_100ORDER_LEN);
	data_shift_1(magy_raw_uT,LPF_3HZ_100ORDER_LEN);
	data_shift_1(magz_raw_uT,LPF_3HZ_100ORDER_LEN);

	filter_fir(magx_raw_uT,magx_fir_uT,LPF_3HZ_100ORDER,LPF_3HZ_100ORDER_LEN);
	filter_fir(magy_raw_uT,magy_fir_uT,LPF_3HZ_100ORDER,LPF_3HZ_100ORDER_LEN);
	filter_fir(magz_raw_uT,magz_fir_uT,LPF_3HZ_100ORDER,LPF_3HZ_100ORDER_LEN);

	// magx = magx_fir_uT[0];
	// magy = magy_fir_uT[0];
	// magz = magz_fir_uT[0];

	magx = magx_raw_uT[0];
	magy = magy_raw_uT[0];
	magz = magz_raw_uT[0];

	magx = magx_gain * (magx - magx_offset);
	magy = magy_gain * (magy - magy_offset);
	magz = magz_gain * (magz - magz_offset);

	//方向变换
	// accx_raw_mps = -accx_raw_mps;
	// accy_raw_mps = accy_raw_mps;
	// accz_raw_mps = accz_raw_mps;

	// gyrox_raw_dps = -gyrox_raw_dps;
	// gyroy_raw_dps = gyroy_raw_dps;
	// gyroz_raw_dps = -gyroz_raw_dps;

}

void MPU9250_process(void)
{       
  //读取MPU数据
  if (mpu_data_ok == 1)
  {
    mpu_data_ok = 0;
    MPU9250_data_push();
  }
  MPU9250_read_raw_data();
  //水平静置校准加速度计&磁强计
  if(osOK == osSemaphoreWait(myBinarySem01MPU9250GyroAccCalibrateOffsetHandle,0))
  {
    MPU9250_gyro_acc_calibrate_offset_func(gyrox_raw_dps,gyroy_raw_dps,gyroz_raw_dps,accx_raw_mps, accy_raw_mps, accz_raw_mps);
    osSemaphoreRelease(myBinarySem02LED1ONHandle);// 
  }
  //三轴旋转校准磁强计
  static int MPU9250MagCalibrateStart = 0;
  if(MPU9250MagCalibrateStart==0)
  {
    if(osOK==osSemaphoreWait(myBinarySem04MPU9250MagCalibrateHandle,0))
    {
      MPU9250MagCalibrateStart = 1;
      osSemaphoreRelease(myBinarySem03LED2ONHandle);// 
    }
  }
  if(MPU9250MagCalibrateStart==1)
  {
    MPU9250_mag_calibrate(MPU9250MagCalibrateStart, magx,magy,magz);
    if(osOK==osSemaphoreWait(myBinarySem04MPU9250MagCalibrateHandle,0))
    {
      MPU9250MagCalibrateStart = 0;
      osSemaphoreRelease(myBinarySem02LED1ONHandle);// 
      MPU9250_mag_calibrate(MPU9250MagCalibrateStart, magx,magy,magz);
    }
  }
}


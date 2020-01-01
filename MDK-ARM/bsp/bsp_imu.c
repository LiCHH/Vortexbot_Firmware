#include "stm32f4xx_hal.h"
#include "string.h"

#include "sys_config.h"
#include "bsp_imu.h"
#include "bsp_io.h"
#include "spi.h"
#include "usart.h"
#include "ist8310_reg.h"
#include "mpu6500_reg.h"

#include "test_ctrl.h"

#define MPU_DELAY(x) HAL_Delay(x)

#define MPU_HSPI hspi5
#define IST8310
#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

static uint8_t tx, rx;
static uint8_t tx_buff[14] = {0xff};
static uint8_t mpu_buff[14];
static uint8_t ist_buff[6];
volatile float m_q0 = 1.0f, m_q1 = 0.0f, m_q2 = 0.0f, m_q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
volatile float beta = 0.2f;
uint8_t temp_buf[300];

mpu_data_t mpu_data;
imu_t imu = {0};

float inv_sqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;

	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}

/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval 
  * @usage  call in ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() function
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
	MPU_NSS_LOW;
	tx = reg & 0x7F;
	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	tx = data;
	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	MPU_NSS_HIGH;
	return 0;
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval 
  * @usage  call in ist_reg_read_by_mpu(),         
  *                 mpu_device_init() function
  */
uint8_t mpu_read_byte(uint8_t const reg)
{
	MPU_NSS_LOW;
	tx = reg | 0x80;
	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	MPU_NSS_HIGH;
	return rx;
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval 
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() function
  */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
	MPU_NSS_LOW;
	tx = regAddr | 0x80;
	tx_buff[0] = tx;
	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
	MPU_NSS_HIGH;
	return 0;
}

/**
	* @brief  write IST8310 register through MPU6500's I2C master
  * @param  addr: the address to be written of IST8310's register
  *         data: data to be written
	* @retval   
  * @usage  call in ist8310_init() function
	*/
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
	/* turn off slave 1 at first */
	mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
	MPU_DELAY(2);
	mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
	MPU_DELAY(2);
	mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
	MPU_DELAY(2);
	/* turn on slave 1 with one byte transmitting */
	mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
	/* wait longer to ensure the data is transmitted from slave 1 */
	MPU_DELAY(10);
}

/**
	* @brief  write IST8310 register through MPU6500's I2C Master
	* @param  addr: the address to be read of IST8310's register
	* @retval 
  * @usage  call in ist8310_init() function
	*/
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
	uint8_t retval;
	mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
	MPU_DELAY(10);
	mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
	MPU_DELAY(10);
	retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
	/* turn off slave4 after read */
	mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
	MPU_DELAY(10);
	return retval;
}

/**
	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
  * @param    device_address: slave device address, Address[6:0]
	* @retval   void
	* @note     
	*/
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
	/* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
	mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
	MPU_DELAY(2);
	mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
	MPU_DELAY(2);
	mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
	MPU_DELAY(2);

	/* use slave0,auto read data */
	mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
	MPU_DELAY(2);
	mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
	MPU_DELAY(2);

	/* every eight mpu6500 internal samples one i2c master read */
	mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
	MPU_DELAY(2);
	/* enable slave 0 and 1 access delay */
	mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
	MPU_DELAY(2);
	/* enable slave 1 auto transmit */
	mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
	/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
	MPU_DELAY(6);
	/* enable slave 0 with data_num bytes reading */
	mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
	MPU_DELAY(2);
}

/**
	* @brief  Initializes the IST8310 device
	* @param  
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t ist8310_init()
{
	/* enable iic master mode */
	mpu_write_byte(MPU6500_USER_CTRL, 0x30);
	MPU_DELAY(10);
	/* enable iic 400khz */
	mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d);
	MPU_DELAY(10);

	/* turn on slave 1 for ist write and slave 4 to ist read */
	mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
	MPU_DELAY(10);
	mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
	MPU_DELAY(10);

	/* IST8310_R_CONFB 0x01 = device rst */
	ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
	MPU_DELAY(10);
	if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
		return 1;

	/* soft reset */
	ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
	MPU_DELAY(10);

	/* config as ready mode to access register */
	ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
	if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
		return 2;
	MPU_DELAY(10);

	/* normal state, no int */
	ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
	if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
		return 3;
	MPU_DELAY(10);

	/* config low noise mode, x,y,z axis 16 time 1 avg */
	ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
	if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
		return 4;
	MPU_DELAY(10);

	/* Set/Reset pulse duration setup,normal mode */
	ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
	if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
		return 5;
	MPU_DELAY(10);

	/* turn off slave1 & slave 4 */
	mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
	MPU_DELAY(10);
	mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
	MPU_DELAY(10);

	/* configure and turn on slave 0 */
	mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
	MPU_DELAY(100);
	return 0;
}

/**
	* @brief  get the data of IST8310
  * @param  buff: the buffer to save the data of IST8310
	* @retval 
  * @usage  call in mpu_get_data() function
	*/
void ist8310_get_data(uint8_t *buff)
{
	mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
}

/**
	* @brief  get the data of imu
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_get_data()
{
	mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

	mpu_data.ax = mpu_buff[0] << 8 | mpu_buff[1];
	mpu_data.ay = mpu_buff[2] << 8 | mpu_buff[3];
	mpu_data.az = mpu_buff[4] << 8 | mpu_buff[5];
	mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

	mpu_data.gx = ((mpu_buff[8] << 8 | mpu_buff[9]) - mpu_data.gx_offset);
	mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
	mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

	ist8310_get_data(ist_buff);
	memcpy(&mpu_data.mx, ist_buff, 6);

	memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));

	imu.temp = 21 + mpu_data.temp / 333.87f;
	/* 2000dps -> rad/s */
	imu.wx = mpu_data.gx / 16.384f / 57.3f;
	imu.wy = mpu_data.gy / 16.384f / 57.3f;
	imu.wz = mpu_data.gz / 16.384f / 57.3f;
}

/**
	* @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
	return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}

/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
	return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3);
}

uint8_t id;

/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
uint8_t mpu_device_init(void)
{
	MPU_NSS_HIGH;
	MPU_DELAY(100);

	id = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t i = 0;
	uint8_t MPU6500_Init_Data[10][2] = {
			{MPU6500_PWR_MGMT_1, 0x80},			/* Reset Device */
			{MPU6500_PWR_MGMT_1, 0x03},			/* Clock Source - Gyro-Z */
			{MPU6500_PWR_MGMT_2, 0x00},			/* Enable Acc & Gyro */
			{MPU6500_CONFIG, 0x04},					/* LPF 41Hz */
			{MPU6500_GYRO_CONFIG, 0x18},		/* +-2000dps */
			{MPU6500_ACCEL_CONFIG, 0x10},		/* +-8G */
			{MPU6500_ACCEL_CONFIG_2, 0x02}, /* enable LowPassFilter  Set Acc LPF */
			{MPU6500_USER_CTRL, 0x20},
	}; /* Enable AUX */
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		MPU_DELAY(1);
	}

	mpu_set_gyro_fsr(3);
	mpu_set_accel_fsr(2);

	ist8310_init();
	mpu_offset_call();
	return 0;
}

/**
	* @brief  get the offset data of MPU6500
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_offset_call(void)
{
	int i;
	for (i = 0; i < 300; i++)
	{
		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];

		mpu_data.gx_offset += mpu_buff[8] << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

		MPU_DELAY(5);
	}
	mpu_data.ax_offset = mpu_data.ax_offset / 300;
	mpu_data.ay_offset = mpu_data.ay_offset / 300;
	mpu_data.az_offset = mpu_data.az_offset / 300;
	mpu_data.gx_offset = mpu_data.gx_offset / 300;
	mpu_data.gy_offset = mpu_data.gx_offset / 300;
	mpu_data.gz_offset = mpu_data.gz_offset / 300;
}

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	static uint32_t this_update, last_update;
	float T;

	this_update = HAL_GetTick();
	T = (float)(this_update - last_update) / 1000.f;
	last_update = this_update;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	// if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
	// 	MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
	// 	return;
	// }

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-m_q1 * gx - m_q2 * gy - m_q3 * gz);
	qDot2 = 0.5f * (m_q0 * gx + m_q2 * gz - m_q3 * gy);
	qDot3 = 0.5f * (m_q0 * gy - m_q1 * gz + m_q3 * gx);
	qDot4 = 0.5f * (m_q0 * gz + m_q1 * gy - m_q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement
		recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

// Normalise magnetometer measurement
#ifdef IST8310
		recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;
#else
		mx = 0;
		my = 0;
		mz = 0;
#endif // !IST8310

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * m_q0 * mx;
		_2q0my = 2.0f * m_q0 * my;
		_2q0mz = 2.0f * m_q0 * mz;
		_2q1mx = 2.0f * m_q1 * mx;
		_2q0 = 2.0f * m_q0;
		_2q1 = 2.0f * m_q1;
		_2q2 = 2.0f * m_q2;
		_2q3 = 2.0f * m_q3;
		_2q0q2 = 2.0f * m_q0 * m_q2;
		_2q2q3 = 2.0f * m_q2 * m_q3;
		q0q0 = m_q0 * m_q0;
		q0q1 = m_q0 * m_q1;
		q0q2 = m_q0 * m_q2;
		q0q3 = m_q0 * m_q3;
		q1q1 = m_q1 * m_q1;
		q1q2 = m_q1 * m_q2;
		q1q3 = m_q1 * m_q3;
		q2q2 = m_q2 * m_q2;
		q2q3 = m_q2 * m_q3;
		q3q3 = m_q3 * m_q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * m_q3 + _2q0mz * m_q2 + mx * q1q1 + _2q1 * my * m_q2 + _2q1 * mz * m_q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * m_q3 + my * q0q0 - _2q0mz * m_q1 + _2q1mx * m_q2 - my * q1q1 + my * q2q2 + _2q2 * mz * m_q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * m_q2 + _2q0my * m_q1 + mz * q0q0 + _2q1mx * m_q3 - mz * q1q1 + _2q2 * my * m_q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * m_q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * m_q3 + _2bz * m_q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * m_q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * m_q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * m_q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * m_q2 + _2bz * m_q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * m_q3 - _4bz * m_q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * m_q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * m_q2 - _2bz * m_q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * m_q1 + _2bz * m_q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * m_q0 - _4bz * m_q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * m_q3 + _2bz * m_q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * m_q0 + _2bz * m_q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * m_q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	m_q0 += qDot1 * T;
	m_q1 += qDot2 * T;
	m_q2 += qDot3 * T;
	m_q3 += qDot4 * T;

	// Normalise quaternion
	recipNorm = inv_sqrt(m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3);
	m_q0 *= recipNorm;
	m_q1 *= recipNorm;
	m_q2 *= recipNorm;
	m_q3 *= recipNorm;
}

void MadgwickUpdate(void)
{
	MadgwickAHRSupdate(imu.wx, imu.wy, imu.wz, imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz);
}

static float get_rpy_relative(float pitch)
{
	static int count = 0;
	static float offset = 0;

	// pitch = pitch / 180 * 3.14159;
	float corr_r, corr_p, corr_y;
	corr_r = atan2(2 * (m_q0 * m_q1 + m_q2 * m_q3), 1 - 2 * m_q1 * m_q1 - 2 * m_q2 * m_q2);
	corr_p = asin(2 * (m_q0 * m_q2 - m_q1 * m_q3));
	corr_y = atan2(2 * (m_q0 * m_q3 + m_q1 * m_q2), 1 - 2 * m_q2 * m_q2 - 2 * m_q3 * m_q3);

	float q_res_x, q_res_y, q_res_z, q_res_w;
	// float q_res2_x, q_res2_y, q_res2_z, q_res2_w;
	float q_corr_x, q_corr_y, q_corr_z, q_corr_w;

	// q_corr_x = 0;
	// q_corr_y = 0;
	// q_corr_z = sin(-corr_y / 2);
	// q_corr_w = cos(-corr_y / 2);

	// q_res2_w = q_corr_w * m_q0 - q_corr_x * m_q1 - q_corr_y * m_q2 - q_corr_z * m_q3;
	// q_res2_x = q_corr_w * m_q1 + q_corr_x * m_q0 + q_corr_y * m_q3 - q_corr_z * m_q2;
	// q_res2_y = q_corr_w * m_q2 - q_corr_x * m_q3 + q_corr_y * m_q0 + q_corr_z * m_q1;
	// q_res2_z = q_corr_w * m_q3 + q_corr_x * m_q2 - q_corr_y * m_q1 + q_corr_z * m_q0;

	// q_res_w = q_corr_w * q_res2_w - q_corr_x * q_res2_x - q_corr_y * q_res2_y - q_corr_z * q_res2_z;
	// q_res_x = q_corr_w * q_res2_x + q_corr_x * q_res2_w + q_corr_y * q_res2_z - q_corr_z * q_res2_y;
	// q_res_y = q_corr_w * q_res2_y - q_corr_x * q_res2_z + q_corr_y * q_res2_w + q_corr_z * q_res2_x;
	// q_res_z = q_corr_w * q_res2_z + q_corr_x * q_res2_y - q_corr_y * q_res2_x + q_corr_z * q_res2_w;

	// TODO:
	// q_corr_x = 0;
	// q_corr_y = sin(pitch / 2);
	// q_corr_z = 0;
	// q_corr_w = cos(pitch / 2);
	// q_res_w = q_corr_w * m_q0 - q_corr_x * m_q1 - q_corr_y * m_q2 - q_corr_z * m_q3;
	// q_res_x = q_corr_w * m_q1 + q_corr_x * m_q0 + q_corr_y * m_q3 - q_corr_z * m_q2;
	// q_res_y = q_corr_w * m_q2 - q_corr_x * m_q3 + q_corr_y * m_q0 + q_corr_z * m_q1;
	// q_res_z = q_corr_w * m_q3 + q_corr_x * m_q2 - q_corr_y * m_q1 + q_corr_z * m_q0;
	// corr_r = atan2(2 * (q_res_w * q_res_x + q_res_y * q_res_z), 1 - 2 * q_res_x * q_res_x - 2 * q_res_y * q_res_y);
	// corr_p = asin(2 * (q_res_w * q_res_y - q_res_x * q_res_z));
	// corr_y = atan2(2 * (q_res_w * q_res_z + q_res_x * q_res_y), 1 - 2 * q_res_y * q_res_y - 2 * q_res_z * q_res_z);
	// if(fabs(corr_r) > PI / 2) {
	// 	corr_y = -corr_y;
	// }

	if (count < 3000)
	{
		offset = corr_y;
		++count;
	}
	else if(count == 3000) 
	{
		beep_ctrl(300, 150);
		// HAL_Delay(3000);
		++count;
		// beep_ctrl(0, 0);
	}
	else if(count < 3150){
		++count;
	}
	else if(count == 3150) {
		beep_ctrl(0, 0);
	}
	// sprintf(test_buf, "%.2f %.2f %.2f\r\n", corr_r*RAD_TO_DEG, corr_p*RAD_TO_DEG, corr_y*RAD_TO_DEG);
	// HAL_UART_Transmit(&TEST_HUART, (uint8_t *)test_buf, 40, 10);
	// memset(temp_buf, 0, sizeof(temp_buf));

	return corr_y; // - offset;

}

const float INIT_PITCH = PI / 2;
float get_yaw(void)
{
	// return get_rpy_relative(0);
	return get_rpy_relative(INIT_PITCH);
}

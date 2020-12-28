/*
 * File: mpu6050.c
 * See mpu6050.h description
 */

#include "mpu6050.h"



void mpu6050_send_stop()
{
	hi2c1.Instance->CR1 |= 0x01 << 9;
}


void mpu6050_send_start()
{
	hi2c1.Instance->CR1 |= 0x01 << 8;
}


void mpu6050_clear_start()
{
	hi2c1.Instance->CR1 &= ~(0x01 << 8);
}


void mpu6050_clear_stop()
{
	hi2c1.Instance->CR1 &= ~(0x01 << 9);
}



void mpu6050_write(uint8_t reg, uint8_t data)
{
	mpu6050_clear_stop();
	hi2c1.Instance->CR1 |= MPU6050_START_BIT;
	hi2c1.Instance->DR = MPU6050_ADD_WRITE;
	//Checks ACK
	if(mpu6050_ack_ok()) {
		hi2c1.Instance->DR = reg;
		//Checks ACK
		if(mpu6050_ack_ok()){
			hi2c1.Instance->DR = data;
			if(mpu6050_ack_ok()) {
				hi2c1.Instance->CR1 |= MPU6050_STOP_BIT;
			}
		}
	}
}

/*
 * _Bool mpu6050_ack_ok()
 * Checks if there is acknowledge failure.
 * If returns 0 -> ACK failed
 * If returns 1 -> ACK is OK
 */
_Bool mpu6050_ack_ok(){
	return ~(hi2c1.Instance->SR1 & (0x01 << 10));
}


_Bool mpu6050_rx_not_empty()
{
	return hi2c1.Instance->SR1 & (0x01 << 6);
}


/*
 * void mpu6050_send_nack()
 * Sends a NACK command to the slave.
 */
void mpu6050_send_nack()
{
	hi2c1.Instance->CR1 &= ~(0x01 << 10); //clears ACK bit
}


//void mpu6050_config_clock_source(uint8_t clk_src)
//{
//	mpu6050_write(MPU_REG_PWR_MGMT_1, clk_src);
//}



_Bool mpu6050_config_clock_source(uint8_t clk_src)
{
	_Bool status = 0;
	buffer[0] = MPU_REG_PWR_MGMT_1;
	ret = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADD_8BIT, buffer, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		buffer[0] = clk_src; //0x01
		ret = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADD_8BIT, buffer, 1, HAL_MAX_DELAY);
		if(ret == HAL_OK) {
			status = 1;
		}
		else
			status = 0;
	}
	return status;
}



//void mpu6050_config_sample_rate(uint8_t smplrt_div)
//{
//	/*
//	 * SAMPLE_RATE is the update rate of sensor register. It is calculated as following:
//	 * SAMPLE_RATE = INTERNAL_SAMPLE_RATE/(1 + SMPLRT_DIV)
//	 * where INTERNAL_SAMPLE_RATE = 1 kHz
//	 */
//	mpu6050_write(MPU_REG_SMPLRT_DIV, smplrt_div);
//}


_Bool mpu6050_config_sample_rate(uint8_t smplrt_div)
{
	_Bool status = 0;
	buffer[0] = MPU_REG_SMPLRT_DIV;
	ret = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADD_8BIT, buffer, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		buffer[0] = smplrt_div; //0x07
		ret = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADD_8BIT, buffer, 1, HAL_MAX_DELAY);
		if(ret == HAL_OK) {
			status = 1;
		}
		else
			status = 0;
	}
	return status;
}



//void mpu6050_config_gyro_scale(uint8_t scale)
//{
//	mpu6050_write(MPU_REG_GYRO_CONFIG, scale);
//}


_Bool mpu6050_config_gyro_scale(uint8_t scale)
{
	_Bool status = 0;
	buffer[0] = MPU_REG_GYRO_CONFIG;
	ret = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADD_8BIT, buffer, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		buffer[0] = scale; //0x18
		ret = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADD_8BIT, buffer, 1, HAL_MAX_DELAY);
		if(ret == HAL_OK) {
			status = 1;
		}
		else
			status = 0;
	}
	return status;
}


//void mpu6050_config_accel_scale(uint8_t scale)
//{
//	mpu6050_write(MPU_REG_ACCEL_CONFIG, scale);
//}


_Bool mpu6050_config_accel_scale(uint8_t scale)
{
	_Bool status = 0;
	buffer[0] = MPU_REG_ACCEL_CONFIG;
	ret = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADD_8BIT, buffer, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		buffer[0] = scale; //0x00 -> 2g
		ret = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADD_8BIT, buffer, 1, HAL_MAX_DELAY);
		if(ret == HAL_OK) {
			status = 1;
		}
		else
			status = 0;
	}
	return status;
}


/*
 * void mpu6050_config_dlpf(uint8_t bandwidth)
 * Configures MPU6050 internal Digital Low Pass Filter (DLPF) with the selected bandwidth.
 */
//void mpu6050_config_dlpf(uint8_t bandwidth)
//{
//	mpu6050_write(MPU_REG_CONFIG, bandwidth);
//}


_Bool mpu6050_config_dlpf(uint8_t bandwidth)
{
	_Bool status = 0;
	buffer[0] = MPU_REG_CONFIG;
	ret = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADD_8BIT, buffer, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		buffer[0] = bandwidth;
		ret = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADD_8BIT, buffer, 1, HAL_MAX_DELAY);
		if(ret == HAL_OK) {
			status = 1;
		}
		else
			status = 0;
	}
	return status;
}



void mpu6050_init()
{
	HAL_Delay(100);	//power-up delay
	//mpu6050_config_sample_rate(0x07);	//sample rate = 125 Hz
	mpu6050_config_clock_source(0x00);	//auto selects the best available clock source - PLL if ready
	HAL_Delay(10);
	mpu6050_config_dlpf(MPU6050_BANDWIDTH_260_HZ);	//bandwidth = 260 Hz
	HAL_Delay(10);
	mpu6050_config_accel_scale(MPU6050_ACCEL_SCALE_2G);	//scale = 2g
	HAL_Delay(10);
	mpu6050_config_gyro_scale(MPU6050_GYRO_SCALE_2000DPS << 3);	//scale = 2000°/s
	HAL_Delay(10);
}



/*
 * void mpu6050_begin()
 * Initializes the accelerometer configuring the following parameters:
 * clock source, sample rate, gyroscope scale, accelerometer scale and DLPF bandwidth.
 */
void mpu6050_begin()
{
	mpu6050_config_clock_source(0x01); 	//auto selects the best available clock source - PLL if ready
	mpu6050_config_sample_rate(0x07);	//sample rate = 125 Hz
	mpu6050_config_gyro_scale(MPU6050_GYRO_SCALE_2000DPS << 3);	//scale = 2000°/s
	mpu6050_config_accel_scale(MPU6050_ACCEL_SCALE_2G);		//scale = 2g
	mpu6050_config_dlpf(MPU6050_BANDWIDTH_260_HZ);	//bandwidth = 260 Hz
}


int8_t mpu6050_read(uint8_t reg)
{
	int8_t read_data = 0;
	//hi2c1.Instance->CR1 &= ~(0x01 << 9);
	//hi2c1.Instance->CR1 &= ~(0x01 << 8);
	//If ARLO is set, clears it
	if(hi2c1.Instance->SR1 & (0x01 << 9)) {
		hi2c1.Instance->SR1 &= ~(0x01 << 9);
	}
	hi2c1.Instance->CR1 |= MPU6050_START_BIT;
	hi2c1.Instance->DR = MPU6050_ADD_WRITE;
	hi2c1.Instance->DR = reg;
	hi2c1.Instance->CR1 |= MPU6050_START_BIT;
	hi2c1.Instance->DR = MPU6050_ADD_READ;
	while(!(mpu6050_ack_ok() && mpu6050_rx_not_empty())) {
		;
	}
	read_data = hi2c1.Instance->DR; //the error is here!
	mpu6050_send_nack();
	mpu6050_send_stop();

	return read_data;

}





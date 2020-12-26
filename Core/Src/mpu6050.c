/*
 * File: mpu6050.c
 * See mpu6050.h description
 */

#include "mpu6050.h"


void mpu6050_write(uint8_t reg, uint8_t data)
{
	hi2c1.Instance->CR1 |= MPU6050_START_BIT;
	hi2c1.Instance->DR = MPU6050_ADD_WRITE;
	//Checks if there is acknowledge failure
	if(!(hi2c1.Instance->SR1 & (0x01 << 10))) {
		hi2c1.Instance->DR = reg;
		//If there is not acknowledge failure
		if(!(hi2c1.Instance->SR1 & (0x01 << 10))){
			hi2c1.Instance->DR = data;
			if(!(hi2c1.Instance->SR1 & (0x01 << 10))) {
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
	return ~((hi2c1.Instance->SR1 & (0x01 << 10)));
}

/*
 * void mpu6050_send_nack()
 * Sends a NACK command to the slave.
 */
void mpu6050_send_nack()
{
	hi2c1.Instance->SR1 |= (0x01 << 10); //clears AF bit
}


void mpu6050_config_clock_source(uint8_t clk_src)
{
	mpu6050_write(MPU_REG_PWR_MGMT_1, clk_src);
}


void mpu6050_config_sample_rate(uint8_t smplrt_div)
{
	/*
	 * SAMPLE_RATE is the update rate of sensor register. It is calculated as following:
	 * SAMPLE_REAT = INTERNAL_SAMPLE_RATE/(1 + SMPLRT_DIV)
	 * where INTERNAL_SAMPLE_RATE = 1 kHz
	 */
	mpu6050_write(MPU_REG_SMPLRT_DIV, smplrt_div);
}


void mpu6050_config_gyro_scale(uint8_t scale)
{
	mpu6050_write(MPU_REG_GYRO_CONFIG, scale);
}


void mpu6050_config_accel_scale(uint8_t scale)
{
	mpu6050_write(MPU_REG_ACCEL_CONFIG, scale);
}

/*
 * void mpu6050_config_dlpf(uint8_t bandwidth)
 * Configures MPU6050 internal Digital Low Pass Filter (DLPF) with the selected bandwidth.
 */
void mpu6050_config_dlpf(uint8_t bandwidth)
{
	mpu6050_write(MPU_REG_CONFIG, bandwidth);
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
	hi2c1.Instance->CR1 &= ~(0x01 << 9);
	hi2c1.Instance->CR1 &= ~(0x01 << 8);
	hi2c1.Instance->CR1 |= MPU6050_START_BIT;
	hi2c1.Instance->DR = MPU6050_ADD_WRITE;
	if(mpu6050_ack_ok()) {
		hi2c1.Instance->DR = reg;
		if(mpu6050_ack_ok()) {
			hi2c1.Instance->CR1 |= MPU6050_START_BIT;
			hi2c1.Instance->DR = MPU6050_ADD_READ;
			if(mpu6050_ack_ok()) {
				/*if(hi2c1.Instance->SR1 & (0x01 << 6)) {
					read_data = hi2c1.Instance->DR;
					mpu6050_send_nack();
					hi2c1.Instance->CR1 |= MPU6050_STOP_BIT;
				}*/
				read_data = hi2c1.Instance->DR;
				mpu6050_send_nack();
				hi2c1.Instance->CR1 |= MPU6050_STOP_BIT;
			}
		}
	}
	return read_data;

}



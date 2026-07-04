/*
 * mpu6050.h
 *
 *  Created on: Jul 1, 2026
 *      Author: Jackson
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct { //struct for the raw input of the MPU6050, combines all reads into a single typedef
	int16_t accel_x_raw;
	int16_t accel_y_raw;
	int16_t accel_z_raw;

	int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;

	float accel_x_g;
	float accel_y_g;
	float accel_z_g;

	float gyro_x_dps;
	float gyro_y_dps;
	float gyro_z_dps;
} MPU6050_Data_t; //name of struct

typedef struct { //struct for the raw input of the MPU6050, combines all reads into a single typedef
	int16_t accel_x_bias;
	int16_t accel_y_bias;
	int16_t accel_z_bias;

	int16_t gyro_x_bias;
	int16_t gyro_y_bias;
	int16_t gyro_z_bias;
} MPU6050_Bias_t; //name of struct

//Initialize the mpu using the stm32 i2c1 peripheral, Returns HAL_OK or HAL_ERROR etc
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

//Combines the raw read of gyro/accel functions and uses the Convert functions to get physical units and place in *data struct
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data, const MPU6050_Bias_t *bias);

//Calibrates raw data
HAL_StatusTypeDef MPU6050_Calibrate_All(I2C_HandleTypeDef *hi2c, MPU6050_Bias_t *bias);

//uses i2c peripheral to read 6 bytes from accelerometer and combine into 3 signed 16 bit integers
HAL_StatusTypeDef MPU6050_Read_Accel_Raw(I2C_HandleTypeDef *hi2c, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);

//uses i2c peripheral to read 6 bytes from gyroscope and combine into 3 signed 16 bit integers
HAL_StatusTypeDef MPU6050_Read_Gyro_Raw(I2C_HandleTypeDef *hi2c, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

//Uses accel scale factor to convert raw units to physical units
float MPU6050_Convert_Accel_To_Grav(int16_t raw_accel_data);

//Uses gyro scale factor to convert raw units to physical units
float MPU6050_Convert_Gyro_To_Deg(int16_t raw_gyro_data);

#endif /* INC_MPU6050_H */

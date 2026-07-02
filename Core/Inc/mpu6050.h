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

//Initialize the mpu using the stm32 i2c1 peripheral, Returns HAL_OK or HAL_ERROR etc
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

//Uses i2c1 peripheral to read data from mpu6050 and place in *data struct
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);

#endif /* INC_MPU6050_H */

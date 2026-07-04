/*
 * imu_filter.h
 *
 *  Created on: Jul 4, 2026
 *      Author: Jackson
 */

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "stm32f4xx_hal.h"
#include "mpu6050.h"

typedef struct {

	float pitch;
	float roll;
	float yaw;

	uint32_t current_time_ms;
	uint32_t previous_time_ms;
	float dt;

} IMU_Angles_t; //name of struct

//Calculates dt
HAL_StatusTypeDef IMU_Update_dt(IMU_Angles_t *angles);

//Calculates angles pitch, roll, yaw
HAL_StatusTypeDef IMU_Calculate_Angles(MPU6050_Data_t *data, IMU_Angles_t *angles);

#endif

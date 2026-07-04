/*
 * imu_filter.c
 *
 *  Created on: Jul 4, 2026
 *      Author: Jackson
 */

#include "imu_filter.h"
#include <math.h>
#include "mpu6050.h"

#define IMU_PI 3.1415927f //6-7 decimal float precision of PI
#define IMU_RAD_TO_DEG (180.0f / IMU_PI) //The conversion of radians to degrees is 180/pi

HAL_StatusTypeDef IMU_Calculate_Angles(MPU6050_Data_t *data, IMU_Angles_t *angles){

	if(data == NULL || angles == NULL){ //IF pointers equal NULL
		return HAL_ERROR; //Return HAL_ERROR, invalid pointer
	}
	//Accelerometer based pitch and roll estimates before filtering
	float accel_pitch;
	float accel_roll;

	//Inverse tangent of y over the square root of x^2 + z^2 gives us pitch, atan2f produces quadrant aware data
	accel_pitch = atan2f(data->accel_y_g, sqrtf((data->accel_x_g * data->accel_x_g) + (data->accel_z_g * data->accel_z_g))) * IMU_RAD_TO_DEG;
	accel_roll = atan2f(data->accel_x_g, data->accel_z_g) * IMU_RAD_TO_DEG;

	//Complementary filter with 98% gyroscope integration with 2% accelerometer correction
	angles->pitch = 0.98f * (angles->pitch + data->gyro_x_dps * angles->dt) + 0.02f * accel_pitch;
	angles->roll = 0.98f * (angles->roll + data->gyro_y_dps * angles->dt) + 0.02f * accel_roll;

	//Yaw is the integrated gyro z, with no magnetometer it can't have a true heading
	angles->yaw += data->gyro_z_dps *angles->dt;

	return HAL_OK; //HAL_OK means math has been done
}
HAL_StatusTypeDef IMU_Update_dt(IMU_Angles_t *angles){
	if(angles == NULL){ //IF angles pointer is null
		return HAL_ERROR; // Return HAL_ERROR
	}

	angles->current_time_ms = HAL_GetTick(); //time is based on HAL_GetTick(), in the future this will be hardware based to increase accuracy

	if (angles->previous_time_ms == 0){ //IF the last recorded time was 0, its on startup
		angles->dt = 0.0f; //Delta of time is 0
	}
	else { //ELSE
		angles->dt = (angles->current_time_ms-angles->previous_time_ms) / 1000.0f; //Converts delta time in milliseconds to seconds
	}
	angles->previous_time_ms = angles->current_time_ms; //The current is now previous!

	return HAL_OK; //Return HAL_OK
}

/*
 * system_health.h
 *
 *  Created on: Jul 4, 2026
 *      Author: Jackson
 */

#ifndef SYSTEM_HEALTH_H
#define SYSTEM_HEALTH_H

#include "stm32f4xx_hal.h"

typedef enum {

	SYS_BOOT = 0,
	SYS_CALIBRATING = 1,
	SYS_RUNNING = 2,
	SYS_MPU_READ_ERROR = 3,
	SYS_IMU_CALC_ERROR = 4,
	SYS_TELEMETRY_ERROR = 5,
	SYS_FAULT = 6,

} System_State_t;

typedef struct {

    uint32_t sample_count;
    uint32_t valid_count;

    uint32_t failed_read_count;
    uint32_t failed_dt_count;
    uint32_t failed_angle_count;
    uint32_t failed_telemetry_count;

    HAL_StatusTypeDef last_mpu_status;
    HAL_StatusTypeDef last_imu_status;
    HAL_StatusTypeDef last_telemetry_status;

    uint8_t data_valid;

    float read_error_rate_percent;

    System_State_t state;

} System_Health_t;


#endif

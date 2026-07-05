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
    SYS_IMU_DT_ERROR = 4,
    SYS_IMU_ANGLE_ERROR = 5,
    SYS_TELEMETRY_ERROR = 6,
    SYS_FAULT = 7

} System_State_t;

typedef struct {

    uint32_t sample_count;
    uint32_t valid_count;

    uint32_t total_error_count;
    uint32_t failed_read_count;
    uint32_t failed_dt_count;
    uint32_t failed_angle_count;
    uint32_t failed_telemetry_count;

    HAL_StatusTypeDef last_mpu_status;
    HAL_StatusTypeDef last_imu_status;
    HAL_StatusTypeDef last_telemetry_status;

    uint8_t data_valid;

    float read_error_rate_percent;
    float total_error_rate_percent;

    System_State_t state;

} System_Health_t;

// Initializes all system health fields to boot/default values
void SystemHealth_Init(System_Health_t *health);

// Sets the high-level system state
void SystemHealth_SetState(System_Health_t *health, System_State_t state);

// Counts one attempted sensor sample
void SystemHealth_RecordSample(System_Health_t *health);

// Counts one fully valid sensor/filter update
void SystemHealth_RecordValidSample(System_Health_t *health);

// Records a failed MPU6050 read
void SystemHealth_RecordReadError(System_Health_t *health, HAL_StatusTypeDef status);

// Records a failed dt update
void SystemHealth_RecordDtError(System_Health_t *health, HAL_StatusTypeDef status);

// Records a failed pitch/roll/yaw calculation
void SystemHealth_RecordAngleError(System_Health_t *health, HAL_StatusTypeDef status);

// Records a failed UART telemetry transmission
void SystemHealth_RecordTelemetryError(System_Health_t *health, HAL_StatusTypeDef status);

// Updates calculated read and total error-rate percentages
void SystemHealth_UpdateErrorRates(System_Health_t *health);

#endif

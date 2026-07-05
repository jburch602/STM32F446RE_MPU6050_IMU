/*
 * system_health.c
 *
 *  Created on: Jul 4, 2026
 *      Author: Jackson
 */

#include "system_health.h"

//Initializes the variables in struct for bootup
void SystemHealth_Init(System_Health_t *health)
{
    if (health == NULL) {
        return;
    }

    health->sample_count = 0;
    health->valid_count = 0;

    health->failed_read_count = 0;
    health->failed_dt_count = 0;
    health->failed_angle_count = 0;
    health->failed_telemetry_count = 0;
    health->total_error_count = 0;

    health->last_mpu_status = HAL_OK;
    health->last_imu_status = HAL_OK;
    health->last_telemetry_status = HAL_OK;

    health->data_valid = 0;

    health->read_error_rate_percent = 0.0f;
    health->total_error_rate_percent = 0.0f;

    health->state = SYS_BOOT;
}

//Sets the high-level system state
void SystemHealth_SetState(System_Health_t *health, System_State_t state)
{
    if (health == NULL) {
        return;
    }

    health->state = state;
}

//Counts one attempted sensor sample
void SystemHealth_RecordSample(System_Health_t *health)
{
    if (health == NULL) {
        return;
    }

    health->sample_count++;
}

//Counts one fully valid sensor/filter update
void SystemHealth_RecordValidSample(System_Health_t *health)
{
    if (health == NULL) {
        return;
    }

    health->valid_count++;
    health->last_mpu_status = HAL_OK;
    health->last_imu_status = HAL_OK;
    health->data_valid = 1;

    SystemHealth_SetState(health, SYS_RUNNING);
}

//Records a failed MPU6050 read
void SystemHealth_RecordReadError(System_Health_t *health, HAL_StatusTypeDef status)
{
    if (health == NULL) {
        return;
    }

    health->failed_read_count++;
    health->last_mpu_status = status;
    health->data_valid = 0;

    SystemHealth_SetState(health, SYS_MPU_READ_ERROR);
}

//Records a failed dt update
void SystemHealth_RecordDtError(System_Health_t *health, HAL_StatusTypeDef status)
{
    if (health == NULL) {
        return;
    }

    health->failed_dt_count++;
    health->last_imu_status = status;
    health->data_valid = 0;

    SystemHealth_SetState(health, SYS_IMU_DT_ERROR);
}

//Records a failed pitch/roll/yaw calculation
void SystemHealth_RecordAngleError(System_Health_t *health, HAL_StatusTypeDef status)
{
    if (health == NULL) {
        return;
    }

    health->failed_angle_count++;
    health->last_imu_status = status;
    health->data_valid = 0;

    SystemHealth_SetState(health, SYS_IMU_ANGLE_ERROR);
}

//Records a failed telemetry transmission
void SystemHealth_RecordTelemetryError(System_Health_t *health, HAL_StatusTypeDef status)
{
    if (health == NULL) {
        return;
    }

    health->failed_telemetry_count++;
    health->last_telemetry_status = status;

    SystemHealth_SetState(health, SYS_TELEMETRY_ERROR);
}

//Updates calculated error-rate fields
void SystemHealth_UpdateErrorRates(System_Health_t *health)
{
    if (health == NULL) {
        return;
    }

    health->total_error_count = health->failed_read_count + health->failed_dt_count + health->failed_angle_count;

    if (health->sample_count > 0) {
        health->read_error_rate_percent = 100.0f * ((float)health->failed_read_count / (float)health->sample_count);

        health->total_error_rate_percent = 100.0f * ((float)health->total_error_count / (float)health->sample_count);
    }
    else {
        health->read_error_rate_percent = 0.0f;
        health->total_error_rate_percent = 0.0f;
    }
}

/*
 * telemetry.h
 *
 *  Created on: Jul 4, 2026
 *      Author: Jackson
 */

#ifndef TELEMETRY_H
#define SYSTEM_HEALTH_H

#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "imu_filter.h"

// Sends CSV header over UART
HAL_StatusTypeDef Telemetry_Send_Header(UART_HandleTypeDef *huart);

// Formats and sends one CSV row over UART
HAL_StatusTypeDef Telemetry_Send_CSV(UART_HandleTypeDef *huart,
                                     const MPU6050_Data_t *mpu,
                                     const IMU_Angles_t *angles);

#endif

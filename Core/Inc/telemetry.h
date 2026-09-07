/*
 * telemetry.h
 *
 *  Created on: Jul 4, 2026
 *      Author: Jackson
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H


#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "imu_filter.h"
#include "system_health.h"
#include "gimbal.h"


/* Sends CSV header over UART */
HAL_StatusTypeDef Telemetry_Send_Header(
        UART_HandleTypeDef *huart
);


/* Sends status notice over UART */
HAL_StatusTypeDef Telemetry_Send_Status(
        UART_HandleTypeDef *huart,
        const char *msg,
        HAL_StatusTypeDef status,
        const System_Health_t *health
);


/* Formats and starts one CSV DMA transfer */
HAL_StatusTypeDef Telemetry_Send_CSV_DMA(
        UART_HandleTypeDef *huart,
        const MPU6050_Data_t *mpu,
        const IMU_Angles_t *angles,
        const Gimbal_t *gimbal,
        const System_Health_t *health
);


/* Handles UART DMA transmit completion */
void Telemetry_UART_TxCpltCallback(
        UART_HandleTypeDef *huart
);


/* Handles UART DMA error */
void Telemetry_UART_ErrorCallback(
        UART_HandleTypeDef *huart
);


#endif

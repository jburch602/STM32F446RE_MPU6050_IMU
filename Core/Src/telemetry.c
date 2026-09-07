/*
 * telemetry.c
 *
 *  Created on: Jul 4, 2026
 *      Author: Jackson
 */

#include "telemetry.h"
#include "system_health.h"
#include "gimbal.h"
#include <stdio.h>


/* Persistent DMA transmit buffer */
static char telemetry_dma_buffer[320];


/* Track DMA transmit state */
static volatile uint8_t telemetry_dma_busy = 0U;


/* Track active telemetry UART */
static UART_HandleTypeDef *telemetry_dma_huart = NULL;


/*
 * Sends CSV header over UART.
 */
HAL_StatusTypeDef Telemetry_Send_Header(
        UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }


    /* Header for csv logging */
    const char csv_header[] =
            "Time_ms,dt,AX_g,AY_g,AZ_g,GX_dps,GY_dps,GZ_dps,"
            "Pitch,Roll,Yaw,"
            "PitchError_deg,RollError_deg,PitchCommand_deg,RollCommand_deg,"
            "State,DataValid,SampleCount,ValidCount,FailedRead,FailedDt,FailedAngle,"
            "FailedTelemetry,ReadErrPct,TotalErrPct\r\n";


    return HAL_UART_Transmit(
            huart,
            (uint8_t *)csv_header,
            sizeof(csv_header) - 1,
            HAL_MAX_DELAY
    );
}


/*
 * Formats and starts one CSV DMA transfer.
 */
HAL_StatusTypeDef Telemetry_Send_CSV_DMA(
        UART_HandleTypeDef *huart,
        const MPU6050_Data_t *mpu,
        const IMU_Angles_t *angles,
        const Gimbal_t *gimbal,
        const System_Health_t *health)
{
    if (huart == NULL ||
        mpu == NULL ||
        angles == NULL ||
        gimbal == NULL ||
        health == NULL)
    {
        return HAL_ERROR;
    }


    /* Do not overwrite an active DMA buffer */
    if (telemetry_dma_busy != 0U)
    {
        return HAL_BUSY;
    }


    int uart_length = 0;


    /* Format CSV message */
    uart_length = snprintf(
            telemetry_dma_buffer,
            sizeof(telemetry_dma_buffer),

            "%lu,%.4f,"
            "%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,%.3f,"
            "%d,%u,%lu,%lu,%lu,%lu,%lu,%lu,"
            "%.3f,%.3f\r\n",

            (unsigned long)angles->current_time_ms,
            angles->dt,

            mpu->accel_x_g,
            mpu->accel_y_g,
            mpu->accel_z_g,

            mpu->gyro_x_dps,
            mpu->gyro_y_dps,
            mpu->gyro_z_dps,

            angles->pitch,
            angles->roll,
            angles->yaw,

            gimbal->pitch_error_deg,
            gimbal->roll_error_deg,

            gimbal->pitch_command_deg,
            gimbal->roll_command_deg,

            (int)health->state,
            (unsigned int)health->data_valid,

            (unsigned long)health->sample_count,
            (unsigned long)health->valid_count,

            (unsigned long)health->failed_read_count,
            (unsigned long)health->failed_dt_count,
            (unsigned long)health->failed_angle_count,
            (unsigned long)health->failed_telemetry_count,

            health->read_error_rate_percent,
            health->total_error_rate_percent
    );


    if (uart_length <= 0 ||
        uart_length >= (int)sizeof(telemetry_dma_buffer))
    {
        return HAL_ERROR;
    }


    /* Mark DMA buffer active */
    telemetry_dma_busy = 1U;
    telemetry_dma_huart = huart;


    /* Start UART DMA transmission */
    HAL_StatusTypeDef status =
            HAL_UART_Transmit_DMA(
                    huart,
                    (uint8_t *)telemetry_dma_buffer,
                    (uint16_t)uart_length
            );


    /* Clear state if DMA failed to start */
    if (status != HAL_OK)
    {
        telemetry_dma_busy = 0U;
        telemetry_dma_huart = NULL;
    }


    return status;
}


/*
 * Clears DMA state after transmission completes.
 */
void Telemetry_UART_TxCpltCallback(
        UART_HandleTypeDef *huart)
{
    if (huart == telemetry_dma_huart)
    {
        telemetry_dma_busy = 0U;
        telemetry_dma_huart = NULL;
    }
}


/*
 * Clears DMA state after UART error.
 */
void Telemetry_UART_ErrorCallback(
        UART_HandleTypeDef *huart)
{
    if (huart == telemetry_dma_huart)
    {
        telemetry_dma_busy = 0U;
        telemetry_dma_huart = NULL;
    }
}


/*
 * Sends status notice over UART.
 */
HAL_StatusTypeDef Telemetry_Send_Status(
        UART_HandleTypeDef *huart,
        const char *msg,
        HAL_StatusTypeDef status,
        const System_Health_t *health)
{
    if (huart == NULL ||
        msg == NULL ||
        health == NULL)
    {
        return HAL_ERROR;
    }


    char status_msg[128];


    int length = snprintf(
            status_msg,
            sizeof(status_msg),

            "# STATUS: %s System=%d HAL_Status=%d\r\n",

            msg,
            (int)health->state,
            (int)status
    );


    if (length > 0 &&
        length < (int)sizeof(status_msg))
    {
        return HAL_UART_Transmit(
                huart,
                (uint8_t *)status_msg,
                (uint16_t)length,
                100
        );
    }


    return HAL_ERROR;
}

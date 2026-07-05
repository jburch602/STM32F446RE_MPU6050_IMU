/*
 * telemetry.c
 *
 *  Created on: Jul 4, 2026
 *      Author: Jackson
 */

#include "telemetry.h"
#include "system_health.h"
#include <stdio.h>

HAL_StatusTypeDef Telemetry_Send_Header(UART_HandleTypeDef *huart){

    if (huart == NULL)
    {
        return HAL_ERROR;
    }
    //Header for csv logging
    const char csv_header[] = "Time_ms,dt,AX_g,AY_g,AZ_g,GX_dps,GY_dps,GZ_dps,Pitch,Roll,Yaw,"
    	    "State,DataValid,SampleCount,ValidCount,FailedRead,FailedDt,FailedAngle,"
    	    "FailedTelemetry,ReadErrPct,TotalErrPct\r\n";

    return HAL_UART_Transmit(
            huart, //Send to UART2
            (uint8_t *)csv_header, // Pointer to the first byte of the header
            sizeof(csv_header ) - 1, //Sends the size of the array minus the '\0'
            HAL_MAX_DELAY);
}

HAL_StatusTypeDef Telemetry_Send_CSV(UART_HandleTypeDef *huart, const MPU6050_Data_t *mpu, const IMU_Angles_t *angles, const System_Health_t *health){

    if (huart == NULL || mpu == NULL || angles == NULL || health == NULL)
    {
        return HAL_ERROR;
    }

    char uart_msg[128]; //character array stores the message
    int uart_length = 0; //uart_length is a integer that counts the number of bytes in the message

    uart_length = snprintf(
        uart_msg,
        sizeof(uart_msg),
        "%lu,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%d,%u,%lu,%lu,%lu,%lu,%lu,%lu,%.3f,%.3f\r\n",
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
        (int)health->state,
        (unsigned int)health->data_valid,
        (unsigned long)health->sample_count,
        (unsigned long)health->valid_count,
        (unsigned long)health->failed_read_count,
        (unsigned long)health->failed_dt_count,
        (unsigned long)health->failed_angle_count,
        (unsigned long)health->failed_telemetry_count,
        health->read_error_rate_percent,
        health->total_error_rate_percent);

    if (uart_length > 0 && uart_length < (int)sizeof(uart_msg)){

          return HAL_UART_Transmit( //Uses HAL library UART transmit
                  huart, //Address of UART2
                  (uint8_t *)uart_msg, // Cast the char buffer pointer to uint8_t* because HAL_UART_Transmit sends byte data
                  (uint16_t)uart_length, //Sends this number of bytes, found in snprintf
                  100); //timeout
    }

    return HAL_ERROR;
}
// Sends status notice over UART
HAL_StatusTypeDef Telemetry_Send_Status(UART_HandleTypeDef *huart, const char *msg, HAL_StatusTypeDef status, const System_Health_t *health){
	if (huart == NULL || msg == NULL || health == NULL)
	    {
	        return HAL_ERROR;
	    }
	char status_msg[128];

	int length = snprintf( //uart_length is a integer that counts the number of bytes in the message
	              status_msg, //character array stores the message
	              sizeof(status_msg), //maximum byte size of the message 128
	              "# STATUS: %s System=%d HAL_Status=%d\r\n", msg, (int)health->state, (int)status);

	    if (length > 0 && length < (int)sizeof(status_msg)){

	          return HAL_UART_Transmit( //Uses HAL library UART transmit
	                  huart, //Address of UART2
	                  (uint8_t *)status_msg, // Cast the char buffer pointer to uint8_t* because HAL_UART_Transmit sends byte data
	                  (uint16_t)length, //Sends this number of bytes, found in snprintf
	                  100); //timeout
	    }

	    return HAL_ERROR;
}

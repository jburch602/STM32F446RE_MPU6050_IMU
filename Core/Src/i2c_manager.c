/*
 * i2c_manager.c
 *
 *  Created on: Jul 5, 2026
 *      Author: Jackson
 */

#include "i2c_manager.h"

// Attempts to recover a stuck I2C1 bus by manually clocking SCL
void I2C_Manager_RecoverI2C1Bus(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOB clock for PB8/PB9
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure SCL and SDA as open-drain GPIO outputs with pullups
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9; // PB8=SCL, PB9=SDA
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Release both lines high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // SCL high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // SDA high
    HAL_Delay(1);

    // If SDA is stuck low, pulse SCL up to 9 times
    for (uint8_t i = 0; i < 9; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
        {
            break; // SDA released
        }

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // SCL low
        HAL_Delay(1);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // SCL high
        HAL_Delay(1);
    }

    // Generate a manual STOP condition:
    // SDA low while SCL high, then SDA high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // SDA low
    HAL_Delay(1);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // SCL high
    HAL_Delay(1);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   // SDA high
    HAL_Delay(1);
}

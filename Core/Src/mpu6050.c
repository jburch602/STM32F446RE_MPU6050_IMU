#include "mpu6050.h"

// Defines

#define MPU6050_ADDR (0x68 << 1)
// MPU6050 7-bit address is 0x68.
// STM32 HAL expects the address shifted left by 1.

#define MPU6050_WHO_AM_I_REG 0x75
// Internal WHO_AM_I register for the MPU6050.

#define MPU6050_EXPECTED_ID 0x68
// The MPU6050 should respond with this after a WHO_AM_I read.

// Static function prototypes
static HAL_StatusTypeDef MPU6050_Read_Register(I2C_HandleTypeDef *hi2c,
                                               uint8_t reg,
                                               uint8_t *data,
                                               uint16_t length);

static HAL_StatusTypeDef MPU6050_Write_Register(I2C_HandleTypeDef *hi2c,
                                                uint8_t reg,
                                                uint8_t data);

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i = 0; // Initialize and declare one byte to hold value read from 0x75

    // Read 0x75 to confirm MPU6050 is responding correctly
    HAL_StatusTypeDef status = MPU6050_Read_Register(
        hi2c,                  // Pointer to the I2C peripheral handle
        MPU6050_WHO_AM_I_REG,  // Internal register of the MPU6050, 0x75
        &who_am_i,             // Store the byte read into who_am_i
        1                      // Read only one byte
    );

    if (status != HAL_OK) // If status is not HAL_OK, the I2C/HAL transaction failed
    {
        return status; // Return the actual status
    }

    if (who_am_i != MPU6050_EXPECTED_ID) // If who_am_i is not 0x68, unexpected device ID
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Static helper functions

static HAL_StatusTypeDef MPU6050_Read_Register(I2C_HandleTypeDef *hi2c,
                                               uint8_t reg,
                                               uint8_t *data,
                                               uint16_t length)
{
    return HAL_I2C_Mem_Read(
        hi2c,                 // Pointer to I2C peripheral handle
        MPU6050_ADDR,         // MPU6050 I2C address in STM32 HAL format
        reg,                  // Internal MPU6050 register address
        I2C_MEMADD_SIZE_8BIT, // Register address size is 8 bits
        data,                 // Location to store read data
        length,               // Number of bytes to read
        100                   // Timeout in ms
    );
}

static HAL_StatusTypeDef MPU6050_Write_Register(I2C_HandleTypeDef *hi2c,
                                                uint8_t reg,
                                                uint8_t data)
{
    return HAL_I2C_Mem_Write(
        hi2c,                 // Pointer to I2C peripheral handle
        MPU6050_ADDR,         // MPU6050 I2C address in STM32 HAL format
        reg,                  // Internal MPU6050 register address
        I2C_MEMADD_SIZE_8BIT, // Register address size is 8 bits
        &data,                // Address of byte to write
        1,                    // Write one byte
        100                   // Timeout in ms
    );
}

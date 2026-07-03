#include "mpu6050.h"

// Defines

#define MPU6050_ADDR (0x68 << 1)
// MPU6050 7-bit address is 0x68.
// STM32 HAL expects the address shifted left by 1.

#define MPU6050_WHO_AM_I_REG 0x75
// Internal WHO_AM_I register for the MPU6050.

#define MPU6050_EXPECTED_ID 0x68
// The MPU6050 should respond with this after a WHO_AM_I read.
#define MPU6050_PWR_MGMT_1_REG 0x6B
// This is the registry of the sleep/wake state
#define MPU6050_ACCEL_XOUT_H_REG 0x3B
// This is the registry of the first accelerometer byte
#define MPU6050_GYRO_XOUT_H_REG 0x43
// This is the registry of the first gyroscope byte

//The scale factors are both in their highest sensitivity modes for now
#define MPU6050_ACCEL_SCALE_FACTOR 16384.0f
// This is the factor to convert raw accel data to physical units (+/-)2g
#define MPU6050_GYRO_SCALE_FACTOR 131.0f
// This is the factor to convert raw gyro data to physical units (+/-)250 degrees per second

// Static function prototypes
static HAL_StatusTypeDef MPU6050_Read_Register(I2C_HandleTypeDef *hi2c,
                                               uint8_t reg,
                                               uint8_t *data,
                                               uint16_t length);

static HAL_StatusTypeDef MPU6050_Write_Register(I2C_HandleTypeDef *hi2c,
                                                uint8_t reg,
                                                uint8_t data);

//Init and read gyro/accel functions

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
        return status; // Return the status
    }

    if (who_am_i != MPU6050_EXPECTED_ID) // If who_am_i is not 0x68, unexpected device ID
    {
        return HAL_ERROR; //Return HAL_ERROR
    }
    //Write to reg 0x6B to equal 0x00. This clears the sleep mode bit
    status = MPU6050_Write_Register(hi2c, MPU6050_PWR_MGMT_1_REG, 0x00);

    if (status != HAL_OK) // IF status is not HAL_OK, the I2C/HAL transaction failed
       {
           return status; // Return the status
       }
    return HAL_OK; // Return HAL_OK, the mpu is awake and has the correct address
}
HAL_StatusTypeDef MPU6050_Read_Accel_Raw(I2C_HandleTypeDef *hi2c, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z){ //function returns a HAL status and combines the 6 bytes from the accelerometer into 3 signed 16 bit integers

	if (hi2c == NULL || accel_x == NULL || accel_y == NULL || accel_z == NULL){ //IF any of the needed pointers are NULL
			return HAL_ERROR; //Return HAL_ERROR
		}
	uint8_t data[6]; //unsigned int array data holds 6, 8 bit values

	HAL_StatusTypeDef status = MPU6050_Read_Register(hi2c, MPU6050_ACCEL_XOUT_H_REG, data, 6); //reads the 6 bytes starting at the address of MPU6050_ACCEL_XOUT_H_REG


	if (status != HAL_OK) // IF status is not HAL_OK, the I2C/HAL transaction failed
	    {
	        return status; // Return the status
	    }

	//so these line shifts take data from the high byte and shift it left 8 bits "<< 8"
	//and then combine the low byte by using bitwise OR operator " | "
	//then it places that 16 bit value in the the value of the pointers address
	*accel_x = (int16_t)(data[0] << 8 | data[1]);
	*accel_y = (int16_t)(data[2] << 8 | data[3]);
	*accel_z = (int16_t)(data[4] << 8 | data[5]);

	return HAL_OK; // Return HAL_OK, does not validate data
}
HAL_StatusTypeDef MPU6050_Read_Gyro_Raw(I2C_HandleTypeDef *hi2c, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z){ //function returns a HAL status and combines the 6 bytes from the gyroscope into 3 signed 16 bit integers

	if (hi2c == NULL || gyro_x == NULL || gyro_y == NULL || gyro_z == NULL){ //IF any of the needed pointers are NULL
			return HAL_ERROR; //Return HAL_ERROR
		}
	uint8_t data[6]; //unsigned int array data holds 6, 8 bit values

	HAL_StatusTypeDef status = MPU6050_Read_Register(hi2c, MPU6050_GYRO_XOUT_H_REG, data, 6); //reads the 6 bytes starting at the address of MPU6050_GYRO_XOUT_H_REG


	if (status != HAL_OK) // IF status is not HAL_OK, the I2C/HAL transaction failed
	    {
	        return status; // Return the status
	    }

	//so these line shifts take data from the high byte and shift it left 8 bits "<< 8"
	//and then combine the low byte by using bitwise OR operator " | "
	//then it places that 16 bit value in the the value of the pointers address
	*gyro_x = (int16_t)(data[0] << 8 | data[1]);
	*gyro_y = (int16_t)(data[2] << 8 | data[3]);
	*gyro_z = (int16_t)(data[4] << 8 | data[5]);

	return HAL_OK; // Return HAL_OK, does not validate data
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

//Unit conversion with scale factor

//Uses accel scale factor to convert raw units to physical units
float MPU6050_Convert_Accel_To_Grav(int16_t raw_accel_data){
	return raw_accel_data / MPU6050_ACCEL_SCALE_FACTOR;
}

//Uses gyro scale factor to convert raw units to physical units
float MPU6050_Convert_Gyro_To_Deg(int16_t raw_gyro_data){
	return raw_gyro_data / MPU6050_GYRO_SCALE_FACTOR;
}

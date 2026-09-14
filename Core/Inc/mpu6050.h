/*
 * mpu6050.h
 *
 * MPU6050 6-axis IMU driver.
 *
 * Sensor configuration:
 *   Accelerometer: +/-2 g
 *   Gyroscope:     +/-1000 deg/s
 *
 * Calibration strategy:
 *   - Accelerometer offset and scale are characterized offline and stored
 *     as permanent calibration constants in mpu6050.c.
 *   - Gyroscope zero-rate bias is measured at startup because it can vary
 *     with temperature and operating conditions.
 *
 * Accelerometer calibration is intentionally independent of startup
 * orientation. The sensor therefore does not need to be level when the
 * system powers on.
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include <stdint.h>


/*
 * Raw and converted MPU6050 measurements.
 */
typedef struct
{
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

} MPU6050_Data_t;


/*
 * Runtime sensor bias values.
 *
 * Gyroscope biases are measured at startup while the sensor is stationary.
 *
 * Accelerometer bias fields are retained for compatibility with existing
 * application code. They are not used by MPU6050_Read_All(); accelerometer
 * correction uses the permanent offset and scale calibration stored in
 * mpu6050.c.
 */
typedef struct
{
    int16_t accel_x_bias;
    int16_t accel_y_bias;
    int16_t accel_z_bias;

    int16_t gyro_x_bias;
    int16_t gyro_y_bias;
    int16_t gyro_z_bias;

} MPU6050_Bias_t;


/*
 * Initialize and verify the MPU6050.
 *
 * Configures:
 *   Accelerometer: +/-2 g
 *   Gyroscope:     +/-1000 deg/s
 */
HAL_StatusTypeDef MPU6050_Init(
        I2C_HandleTypeDef *hi2c
);


/*
 * Read all accelerometer and gyroscope axes.
 *
 * Accelerometer output:
 *   Permanent per-axis offset and scale calibration is applied.
 *
 * Gyroscope output:
 *   Runtime zero-rate bias is removed.
 */
HAL_StatusTypeDef MPU6050_Read_All(
        I2C_HandleTypeDef *hi2c,
        MPU6050_Data_t *data,
        const MPU6050_Bias_t *bias
);


/*
 * Measure zero-rate gyroscope bias.
 *
 * The sensor must remain stationary during calibration but does not
 * need to be level.
 */
HAL_StatusTypeDef MPU6050_Calibrate_Gyro(
        I2C_HandleTypeDef *hi2c,
        MPU6050_Bias_t *bias
);


/*
 * Compatibility wrapper for existing application code.
 *
 * This function now performs gyroscope calibration only. Accelerometer
 * calibration is no longer calculated from the startup orientation.
 */
HAL_StatusTypeDef MPU6050_Calibrate_All(
        I2C_HandleTypeDef *hi2c,
        MPU6050_Bias_t *bias
);


/*
 * Read raw signed 16-bit accelerometer measurements.
 */
HAL_StatusTypeDef MPU6050_Read_Accel_Raw(
        I2C_HandleTypeDef *hi2c,
        int16_t *accel_x,
        int16_t *accel_y,
        int16_t *accel_z
);


/*
 * Read raw signed 16-bit gyroscope measurements.
 */
HAL_StatusTypeDef MPU6050_Read_Gyro_Raw(
        I2C_HandleTypeDef *hi2c,
        int16_t *gyro_x,
        int16_t *gyro_y,
        int16_t *gyro_z
);


/*
 * Convert a raw accelerometer measurement using the nominal +/-2 g
 * sensitivity of 16384 counts/g.
 *
 * Retained for compatibility and diagnostic use. MPU6050_Read_All()
 * uses the measured per-axis calibration instead.
 */
float MPU6050_Convert_Accel_To_Grav(
        int16_t raw_accel_data
);


/*
 * Convert a bias-corrected gyroscope measurement to deg/s using the
 * configured +/-1000 deg/s sensitivity.
 *
 * The input is int32_t so raw - bias cannot overflow a signed 16-bit
 * intermediate value.
 */
float MPU6050_Convert_Gyro_To_Deg(
        int32_t calibrated_gyro_data
);


#endif /* MPU6050_H */

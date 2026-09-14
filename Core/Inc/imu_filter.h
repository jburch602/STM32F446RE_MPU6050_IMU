/*
 * imu_filter.h
 *
 * MPU6050 attitude estimation using an adaptive complementary filter.
 *
 * Pitch and roll combine:
 *   - Short-term gyroscope integration
 *   - Long-term gravity reference from the accelerometer
 *
 * Accelerometer influence is reduced when the IMU is undergoing significant
 * linear acceleration or angular motion. This prevents transient motion from
 * being interpreted as a change in the gravity vector.
 *
 * Current physical axis convention:
 *   Pitch rate = +GY
 *   Roll rate  = -GX
 *
 * The MPU6050 is mounted upside down on the gimbal.
 */

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "stm32f4xx_hal.h"
#include "mpu6050.h"


typedef struct
{
    /* Estimated attitude */
    float pitch;
    float roll;
    float yaw;

    /* Loop timing */
    uint32_t current_time_ms;
    uint32_t previous_time_ms;
    float dt;

    /*
     * Adaptive-filter diagnostics.
     *
     * These values are exposed so filter behavior can be logged through
     * telemetry during tuning and validation.
     */
    float accel_magnitude_g;
    float gyro_magnitude_dps;

    float accel_magnitude_trust;
    float gyro_rate_trust;
    float accel_trust;

    float complementary_alpha;

} IMU_Angles_t;


/*
 * Update the estimator time step using HAL_GetTick().
 */
HAL_StatusTypeDef IMU_Update_dt(
        IMU_Angles_t *angles
);


/*
 * Update pitch, roll, and yaw using the adaptive complementary filter.
 *
 * Accelerometer correction is automatically reduced during dynamic motion.
 */
HAL_StatusTypeDef IMU_Calculate_Angles(
        MPU6050_Data_t *data,
        IMU_Angles_t *angles
);


#endif /* IMU_FILTER_H */

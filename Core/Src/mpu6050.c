/*
 * mpu6050.c
 *
 * MPU6050 6-axis IMU driver.
 *
 * Configuration:
 *   Accelerometer: +/-2 g
 *   Gyroscope:     +/-1000 deg/s
 *
 * Calibration:
 *   Accelerometer:
 *     Permanent per-axis offset and scale correction derived from a
 *     multi-orientation static calibration.
 *
 *   Gyroscope:
 *     Zero-rate bias measured at startup while the sensor is stationary.
 *
 * This separates intrinsic sensor calibration from attitude reference.
 * Startup orientation is never treated as accelerometer bias.
 */

#include "mpu6050.h"


/* --------------------------------------------------------------------------
 * Device address
 * -------------------------------------------------------------------------- */

#define MPU6050_ADDR                         (0x68U << 1)


/* --------------------------------------------------------------------------
 * Register addresses
 * -------------------------------------------------------------------------- */

#define MPU6050_WHO_AM_I_REG                 0x75U
#define MPU6050_PWR_MGMT_1_REG               0x6BU

#define MPU6050_GYRO_CONFIG_REG              0x1BU
#define MPU6050_ACCEL_CONFIG_REG             0x1CU

#define MPU6050_ACCEL_XOUT_H_REG             0x3BU
#define MPU6050_GYRO_XOUT_H_REG              0x43U

#define MPU6050_EXPECTED_ID                  0x68U


/* --------------------------------------------------------------------------
 * Full-scale configuration
 * -------------------------------------------------------------------------- */

/*
 * AFS_SEL = 0
 *
 * Accelerometer range: +/-2 g
 * Nominal sensitivity: 16384 counts/g
 */
#define MPU6050_ACCEL_CONFIG_VALUE           0x00U
#define MPU6050_ACCEL_NOMINAL_COUNTS_PER_G   16384.0f


/*
 * FS_SEL = 2
 *
 * Gyroscope range: +/-1000 deg/s
 * Sensitivity:     32.8 counts/(deg/s)
 */
#define MPU6050_GYRO_CONFIG_VALUE            0x10U
#define MPU6050_GYRO_SCALE_FACTOR            32.8f


/* --------------------------------------------------------------------------
 * Permanent accelerometer calibration
 * --------------------------------------------------------------------------
 *
 * Derived from the 30-position arbitrary-orientation calibration.
 *
 * Each axis is corrected using:
 *
 *              raw - offset
 *   accel_g =  ------------
 *              counts/g
 *
 * These constants characterize the accelerometer itself and are not
 * dependent on gimbal orientation at startup.
 */

#define MPU6050_ACCEL_X_OFFSET_RAW            822.561f
#define MPU6050_ACCEL_Y_OFFSET_RAW           -220.519f
#define MPU6050_ACCEL_Z_OFFSET_RAW           1642.138f

#define MPU6050_ACCEL_X_COUNTS_PER_G        16397.160f
#define MPU6050_ACCEL_Y_COUNTS_PER_G        16388.686f
#define MPU6050_ACCEL_Z_COUNTS_PER_G        16586.842f


/* --------------------------------------------------------------------------
 * Gyroscope startup calibration
 * -------------------------------------------------------------------------- */

#define MPU6050_GYRO_CALIBRATION_SAMPLES       1000U
#define MPU6050_GYRO_CALIBRATION_MAX_ATTEMPTS  5000U
#define MPU6050_GYRO_CALIBRATION_DELAY_MS         2U


/* --------------------------------------------------------------------------
 * Private function prototypes
 * -------------------------------------------------------------------------- */

static HAL_StatusTypeDef MPU6050_Read_Register(
        I2C_HandleTypeDef *hi2c,
        uint8_t reg,
        uint8_t *data,
        uint16_t length
);


static HAL_StatusTypeDef MPU6050_Write_Register(
        I2C_HandleTypeDef *hi2c,
        uint8_t reg,
        uint8_t data
);


/* --------------------------------------------------------------------------
 * Register access
 * -------------------------------------------------------------------------- */

static HAL_StatusTypeDef MPU6050_Read_Register(
        I2C_HandleTypeDef *hi2c,
        uint8_t reg,
        uint8_t *data,
        uint16_t length)
{
    if (hi2c == NULL ||
        data == NULL ||
        length == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(
            hi2c,
            MPU6050_ADDR,
            reg,
            I2C_MEMADD_SIZE_8BIT,
            data,
            length,
            100U
    );
}


static HAL_StatusTypeDef MPU6050_Write_Register(
        I2C_HandleTypeDef *hi2c,
        uint8_t reg,
        uint8_t data)
{
    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(
            hi2c,
            MPU6050_ADDR,
            reg,
            I2C_MEMADD_SIZE_8BIT,
            &data,
            1U,
            100U
    );
}


/* --------------------------------------------------------------------------
 * Initialization
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU6050_Init(
        I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }


    HAL_StatusTypeDef status;
    uint8_t register_value = 0U;


    /* Verify that the expected MPU6050 is present on the I2C bus. */
    status = MPU6050_Read_Register(
            hi2c,
            MPU6050_WHO_AM_I_REG,
            &register_value,
            1U
    );

    if (status != HAL_OK)
    {
        return status;
    }

    if (register_value != MPU6050_EXPECTED_ID)
    {
        return HAL_ERROR;
    }


    /* Wake the device from sleep mode. */
    status = MPU6050_Write_Register(
            hi2c,
            MPU6050_PWR_MGMT_1_REG,
            0x00U
    );

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(100U);


    /* Configure accelerometer full-scale range to +/-2 g. */
    status = MPU6050_Write_Register(
            hi2c,
            MPU6050_ACCEL_CONFIG_REG,
            MPU6050_ACCEL_CONFIG_VALUE
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /*
     * Verify AFS_SEL bits [4:3].
     *
     * Reading the configuration back catches failed or incomplete writes
     * before sensor data is interpreted using the wrong scale factor.
     */
    status = MPU6050_Read_Register(
            hi2c,
            MPU6050_ACCEL_CONFIG_REG,
            &register_value,
            1U
    );

    if (status != HAL_OK)
    {
        return status;
    }

    if ((register_value & 0x18U) !=
        MPU6050_ACCEL_CONFIG_VALUE)
    {
        return HAL_ERROR;
    }


    /* Configure gyroscope full-scale range to +/-1000 deg/s. */
    status = MPU6050_Write_Register(
            hi2c,
            MPU6050_GYRO_CONFIG_REG,
            MPU6050_GYRO_CONFIG_VALUE
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Verify FS_SEL bits [4:3]. */
    status = MPU6050_Read_Register(
            hi2c,
            MPU6050_GYRO_CONFIG_REG,
            &register_value,
            1U
    );

    if (status != HAL_OK)
    {
        return status;
    }

    if ((register_value & 0x18U) !=
        MPU6050_GYRO_CONFIG_VALUE)
    {
        return HAL_ERROR;
    }


    return HAL_OK;
}


/* --------------------------------------------------------------------------
 * Raw sensor reads
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU6050_Read_Accel_Raw(
        I2C_HandleTypeDef *hi2c,
        int16_t *accel_x,
        int16_t *accel_y,
        int16_t *accel_z)
{
    if (hi2c == NULL ||
        accel_x == NULL ||
        accel_y == NULL ||
        accel_z == NULL)
    {
        return HAL_ERROR;
    }


    uint8_t data[6];


    HAL_StatusTypeDef status =
            MPU6050_Read_Register(
                    hi2c,
                    MPU6050_ACCEL_XOUT_H_REG,
                    data,
                    6U
            );

    if (status != HAL_OK)
    {
        return status;
    }


    /*
     * MPU6050 output registers store each signed 16-bit measurement
     * in big-endian format: high byte followed by low byte.
     */
    *accel_x =
            (int16_t)(
                ((uint16_t)data[0] << 8) |
                data[1]
            );

    *accel_y =
            (int16_t)(
                ((uint16_t)data[2] << 8) |
                data[3]
            );

    *accel_z =
            (int16_t)(
                ((uint16_t)data[4] << 8) |
                data[5]
            );


    return HAL_OK;
}


HAL_StatusTypeDef MPU6050_Read_Gyro_Raw(
        I2C_HandleTypeDef *hi2c,
        int16_t *gyro_x,
        int16_t *gyro_y,
        int16_t *gyro_z)
{
    if (hi2c == NULL ||
        gyro_x == NULL ||
        gyro_y == NULL ||
        gyro_z == NULL)
    {
        return HAL_ERROR;
    }


    uint8_t data[6];


    HAL_StatusTypeDef status =
            MPU6050_Read_Register(
                    hi2c,
                    MPU6050_GYRO_XOUT_H_REG,
                    data,
                    6U
            );

    if (status != HAL_OK)
    {
        return status;
    }


    *gyro_x =
            (int16_t)(
                ((uint16_t)data[0] << 8) |
                data[1]
            );

    *gyro_y =
            (int16_t)(
                ((uint16_t)data[2] << 8) |
                data[3]
            );

    *gyro_z =
            (int16_t)(
                ((uint16_t)data[4] << 8) |
                data[5]
            );


    return HAL_OK;
}


/* --------------------------------------------------------------------------
 * Unit conversion
 * -------------------------------------------------------------------------- */

/*
 * Convert raw accelerometer counts using the nominal +/-2 g sensitivity.
 *
 * This function is retained for compatibility and diagnostics. Production
 * accelerometer values returned by MPU6050_Read_All() use the measured
 * per-axis offset and scale constants instead.
 */
float MPU6050_Convert_Accel_To_Grav(
        int16_t raw_accel_data)
{
    return (float)raw_accel_data /
            MPU6050_ACCEL_NOMINAL_COUNTS_PER_G;
}


/*
 * Convert bias-corrected gyro counts to degrees per second.
 */
float MPU6050_Convert_Gyro_To_Deg(
        int32_t calibrated_gyro_data)
{
    return (float)calibrated_gyro_data /
            MPU6050_GYRO_SCALE_FACTOR;
}


/* --------------------------------------------------------------------------
 * Calibrated sensor read
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU6050_Read_All(
        I2C_HandleTypeDef *hi2c,
        MPU6050_Data_t *data,
        const MPU6050_Bias_t *bias)
{
    if (hi2c == NULL ||
        data == NULL ||
        bias == NULL)
    {
        return HAL_ERROR;
    }


    HAL_StatusTypeDef status;


    status = MPU6050_Read_Accel_Raw(
            hi2c,
            &data->accel_x_raw,
            &data->accel_y_raw,
            &data->accel_z_raw
    );

    if (status != HAL_OK)
    {
        return status;
    }


    status = MPU6050_Read_Gyro_Raw(
            hi2c,
            &data->gyro_x_raw,
            &data->gyro_y_raw,
            &data->gyro_z_raw
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /*
     * Apply permanent accelerometer calibration.
     *
     * Unlike the previous startup calibration method, these corrections
     * represent intrinsic sensor offset and scale error rather than the
     * gravity vector present when the system was powered on.
     */
    data->accel_x_g =
            (
                (float)data->accel_x_raw -
                MPU6050_ACCEL_X_OFFSET_RAW
            )
            /
            MPU6050_ACCEL_X_COUNTS_PER_G;


    data->accel_y_g =
            (
                (float)data->accel_y_raw -
                MPU6050_ACCEL_Y_OFFSET_RAW
            )
            /
            MPU6050_ACCEL_Y_COUNTS_PER_G;


    data->accel_z_g =
            (
                (float)data->accel_z_raw -
                MPU6050_ACCEL_Z_OFFSET_RAW
            )
            /
            MPU6050_ACCEL_Z_COUNTS_PER_G;


    /*
     * Remove startup zero-rate gyro bias before converting to deg/s.
     *
     * The intermediate values are int32_t because subtracting two int16_t
     * values can mathematically exceed the int16_t range.
     */
    int32_t gyro_x_calibrated =
            (int32_t)data->gyro_x_raw -
            (int32_t)bias->gyro_x_bias;

    int32_t gyro_y_calibrated =
            (int32_t)data->gyro_y_raw -
            (int32_t)bias->gyro_y_bias;

    int32_t gyro_z_calibrated =
            (int32_t)data->gyro_z_raw -
            (int32_t)bias->gyro_z_bias;


    data->gyro_x_dps =
            MPU6050_Convert_Gyro_To_Deg(
                    gyro_x_calibrated
            );

    data->gyro_y_dps =
            MPU6050_Convert_Gyro_To_Deg(
                    gyro_y_calibrated
            );

    data->gyro_z_dps =
            MPU6050_Convert_Gyro_To_Deg(
                    gyro_z_calibrated
            );


    return HAL_OK;
}


/* --------------------------------------------------------------------------
 * Gyroscope calibration
 * -------------------------------------------------------------------------- */

/*
 * Determine zero-rate gyro bias by averaging stationary raw measurements.
 *
 * Orientation does not matter because a stationary gyroscope should report
 * zero angular velocity regardless of the direction of gravity.
 *
 * Accelerometer runtime bias fields are explicitly cleared because
 * accelerometer calibration is handled by the permanent offset/scale
 * constants above.
 */
HAL_StatusTypeDef MPU6050_Calibrate_Gyro(
        I2C_HandleTypeDef *hi2c,
        MPU6050_Bias_t *bias)
{
    if (hi2c == NULL ||
        bias == NULL)
    {
        return HAL_ERROR;
    }


    bias->accel_x_bias = 0;
    bias->accel_y_bias = 0;
    bias->accel_z_bias = 0;


    int64_t gyro_x_sum = 0;
    int64_t gyro_y_sum = 0;
    int64_t gyro_z_sum = 0;

    uint32_t valid_samples = 0U;
    uint32_t total_attempts = 0U;

    int16_t gyro_x = 0;
    int16_t gyro_y = 0;
    int16_t gyro_z = 0;


    /*
     * Continue until the requested number of successful readings has been
     * collected or the retry limit is reached.
     */
    while (
        valid_samples <
            MPU6050_GYRO_CALIBRATION_SAMPLES
        &&
        total_attempts <
            MPU6050_GYRO_CALIBRATION_MAX_ATTEMPTS
    )
    {
        HAL_StatusTypeDef status =
                MPU6050_Read_Gyro_Raw(
                        hi2c,
                        &gyro_x,
                        &gyro_y,
                        &gyro_z
                );


        total_attempts++;


        if (status == HAL_OK)
        {
            gyro_x_sum +=
                    (int64_t)gyro_x;

            gyro_y_sum +=
                    (int64_t)gyro_y;

            gyro_z_sum +=
                    (int64_t)gyro_z;

            valid_samples++;
        }


        HAL_Delay(
                MPU6050_GYRO_CALIBRATION_DELAY_MS
        );
    }


    if (valid_samples <
        MPU6050_GYRO_CALIBRATION_SAMPLES)
    {
        return HAL_ERROR;
    }


    bias->gyro_x_bias =
            (int16_t)(
                gyro_x_sum /
                (int64_t)valid_samples
            );

    bias->gyro_y_bias =
            (int16_t)(
                gyro_y_sum /
                (int64_t)valid_samples
            );

    bias->gyro_z_bias =
            (int16_t)(
                gyro_z_sum /
                (int64_t)valid_samples
            );


    return HAL_OK;
}


/*
 * Backward-compatible calibration entry point.
 *
 * Historical application code calls MPU6050_Calibrate_All(). Keeping the
 * function avoids unnecessary changes elsewhere while enforcing the new
 * calibration architecture: only gyro bias is estimated at startup.
 */
HAL_StatusTypeDef MPU6050_Calibrate_All(
        I2C_HandleTypeDef *hi2c,
        MPU6050_Bias_t *bias)
{
    return MPU6050_Calibrate_Gyro(
            hi2c,
            bias
    );
}

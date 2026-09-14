/*
 * imu_filter.c
 *
 * Adaptive complementary attitude filter for the MPU6050 gimbal.
 *
 * The estimator relies primarily on the gyroscope during rapid motion and
 * gradually restores accelerometer correction as the system becomes
 * quasi-static.
 *
 * Accelerometer trust is determined from:
 *
 *   1. Gravity-vector magnitude
 *      A stationary accelerometer should measure approximately 1 g.
 *
 *   2. Total angular rate
 *      High angular velocity indicates dynamic motion, during which the
 *      accelerometer is more likely to contain centripetal, tangential, or
 *      other non-gravitational acceleration.
 *
 * The lower of these two trust values determines accelerometer authority.
 *
 * Filter behavior:
 *
 *   Trust = 1.0  -> alpha = 0.98
 *   Trust = 0.5  -> alpha = 0.99
 *   Trust = 0.0  -> alpha = 1.00 (gyro only)
 *
 * Axis convention:
 *
 *   Pitch angular rate = +GY
 *   Roll angular rate  = -GX
 *
 * The MPU6050 is physically mounted upside down on the gimbal.
 */

#include "imu_filter.h"

#include <math.h>


/* --------------------------------------------------------------------------
 * Angle conversion
 * -------------------------------------------------------------------------- */

#define IMU_PI                              3.1415927f
#define IMU_RAD_TO_DEG                      (180.0f / IMU_PI)


/* --------------------------------------------------------------------------
 * Complementary-filter configuration
 * --------------------------------------------------------------------------
 *
 * IMU_ALPHA_STATIC is used when the accelerometer is fully trusted.
 *
 * At a 100 Hz update rate, alpha = 0.98 gives approximately a 0.5 second
 * complementary-filter time constant.
 *
 * IMU_ALPHA_GYRO_ONLY completely disables accelerometer correction.
 */

#define IMU_ALPHA_STATIC                    0.98f
#define IMU_ALPHA_GYRO_ONLY                 1.00f


/* --------------------------------------------------------------------------
 * Acceleration-magnitude trust thresholds
 * --------------------------------------------------------------------------
 *
 * When stationary or moving without significant linear acceleration:
 *
 *      |a| ~= 1 g
 *
 * Full accelerometer trust:
 *
 *      ||a| - 1| <= 0.05 g
 *
 * Zero accelerometer trust:
 *
 *      ||a| - 1| >= 0.15 g
 *
 * Trust decreases linearly between these thresholds.
 */

#define IMU_ACCEL_FULL_TRUST_ERROR_G        0.05f
#define IMU_ACCEL_ZERO_TRUST_ERROR_G        0.15f


/* --------------------------------------------------------------------------
 * Angular-rate trust thresholds
 * --------------------------------------------------------------------------
 *
 * Below 10 deg/s, angular motion alone does not reduce accelerometer trust.
 *
 * Above 40 deg/s, accelerometer attitude correction is disabled until the
 * angular rate falls again.
 *
 * The recent servo validation produced angular rates above 300 deg/s, so
 * rapid gimbal motion will clearly enter gyro-only operation.
 */

#define IMU_GYRO_FULL_TRUST_DPS             10.0f
#define IMU_GYRO_ZERO_TRUST_DPS             40.0f


/* --------------------------------------------------------------------------
 * Private function prototypes
 * -------------------------------------------------------------------------- */

static float IMU_Clamp01(
        float value
);


static float IMU_CalculateTrust(
        float value,
        float full_trust_threshold,
        float zero_trust_threshold
);


static float IMU_CalculateAdaptiveAlpha(
        float accel_trust
);


/* --------------------------------------------------------------------------
 * Private helper functions
 * -------------------------------------------------------------------------- */

/*
 * Clamp a floating-point value to the range [0, 1].
 */
static float IMU_Clamp01(
        float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }

    if (value > 1.0f)
    {
        return 1.0f;
    }

    return value;
}


/*
 * Convert a measurement magnitude into a continuous trust value.
 *
 *               1.0  if value <= full_trust_threshold
 *
 *   trust =     linear transition between thresholds
 *
 *               0.0  if value >= zero_trust_threshold
 *
 * Lower values therefore represent more trustworthy conditions.
 */
static float IMU_CalculateTrust(
        float value,
        float full_trust_threshold,
        float zero_trust_threshold)
{
    if (value <= full_trust_threshold)
    {
        return 1.0f;
    }

    if (value >= zero_trust_threshold)
    {
        return 0.0f;
    }


    float trust =
            1.0f -
            (
                (value - full_trust_threshold)
                /
                (zero_trust_threshold -
                 full_trust_threshold)
            );


    return IMU_Clamp01(
            trust
    );
}


/*
 * Convert accelerometer trust into complementary-filter alpha.
 *
 * Full trust:
 *
 *      alpha = 0.98
 *
 * No trust:
 *
 *      alpha = 1.00
 *
 * Intermediate trust values smoothly interpolate between the two.
 */
static float IMU_CalculateAdaptiveAlpha(
        float accel_trust)
{
    accel_trust =
            IMU_Clamp01(
                accel_trust
            );


    return IMU_ALPHA_GYRO_ONLY -
            accel_trust *
            (
                IMU_ALPHA_GYRO_ONLY -
                IMU_ALPHA_STATIC
            );
}


/* --------------------------------------------------------------------------
 * Attitude estimation
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef IMU_Calculate_Angles(
        MPU6050_Data_t *data,
        IMU_Angles_t *angles)
{
    if (data == NULL ||
        angles == NULL)
    {
        return HAL_ERROR;
    }


    /*
     * ------------------------------------------------------
     * Accelerometer attitude
     * ------------------------------------------------------
     *
     * The MPU6050 is mounted upside down.
     *
     * Pitch convention was validated against independent +GY gyro
     * integration:
     *
     *      pitch = atan2(AX, -AZ)
     *
     * Roll retains the established physical sign convention.
     */

    float accel_pitch =
            atan2f(
                data->accel_x_g,
                -data->accel_z_g
            )
            *
            IMU_RAD_TO_DEG;


    float accel_roll =
            atan2f(
                data->accel_y_g,

                sqrtf(
                    data->accel_x_g *
                    data->accel_x_g
                    +
                    data->accel_z_g *
                    data->accel_z_g
                )
            )
            *
            IMU_RAD_TO_DEG;


    /*
     * ------------------------------------------------------
     * Accelerometer magnitude
     * ------------------------------------------------------
     *
     * A calibrated stationary accelerometer should produce approximately
     * one g regardless of sensor orientation.
     */

    angles->accel_magnitude_g =
            sqrtf(
                data->accel_x_g *
                data->accel_x_g
                +
                data->accel_y_g *
                data->accel_y_g
                +
                data->accel_z_g *
                data->accel_z_g
            );


    float accel_magnitude_error_g =
            fabsf(
                angles->accel_magnitude_g -
                1.0f
            );


    /*
     * ------------------------------------------------------
     * Gyroscope magnitude
     * ------------------------------------------------------
     *
     * Total angular velocity is used rather than looking only at pitch or
     * roll. Motion on any axis can accompany acceleration that corrupts the
     * gravity estimate.
     */

    angles->gyro_magnitude_dps =
            sqrtf(
                data->gyro_x_dps *
                data->gyro_x_dps
                +
                data->gyro_y_dps *
                data->gyro_y_dps
                +
                data->gyro_z_dps *
                data->gyro_z_dps
            );


    /*
     * ------------------------------------------------------
     * Accelerometer trust
     * ------------------------------------------------------
     */


    /*
     * Trust based on how closely acceleration magnitude matches gravity.
     *
     * 1.0 = magnitude looks gravitational
     * 0.0 = magnitude clearly contains significant dynamic acceleration
     */
    angles->accel_magnitude_trust =
            IMU_CalculateTrust(
                accel_magnitude_error_g,
                IMU_ACCEL_FULL_TRUST_ERROR_G,
                IMU_ACCEL_ZERO_TRUST_ERROR_G
            );


    /*
     * Trust based on total angular velocity.
     *
     * Large angular rates cause accelerometer correction to be reduced even
     * when acceleration magnitude happens to pass near 1 g.
     */
    angles->gyro_rate_trust =
            IMU_CalculateTrust(
                angles->gyro_magnitude_dps,
                IMU_GYRO_FULL_TRUST_DPS,
                IMU_GYRO_ZERO_TRUST_DPS
            );


    /*
     * Both conditions must support accelerometer use.
     *
     * Choosing the smaller trust value makes either significant linear
     * acceleration OR significant angular motion sufficient to suppress
     * accelerometer correction.
     */
    angles->accel_trust =
            fminf(
                angles->accel_magnitude_trust,
                angles->gyro_rate_trust
            );


    /*
     * Convert trust to the complementary-filter coefficient.
     *
     * alpha approaches 1.0 as accelerometer trust falls.
     */
    angles->complementary_alpha =
            IMU_CalculateAdaptiveAlpha(
                angles->accel_trust
            );


    float accelerometer_weight =
            1.0f -
            angles->complementary_alpha;


    /*
     * ------------------------------------------------------
     * Pitch
     * ------------------------------------------------------
     *
     * Physical pitch rate = +GY.
     */

    float pitch_gyro_prediction =
            angles->pitch +
            data->gyro_y_dps *
            angles->dt;


    angles->pitch =
            angles->complementary_alpha *
            pitch_gyro_prediction
            +
            accelerometer_weight *
            accel_pitch;


    /*
     * ------------------------------------------------------
     * Roll
     * ------------------------------------------------------
     *
     * Physical roll rate = -GX.
     */

    float roll_gyro_prediction =
            angles->roll -
            data->gyro_x_dps *
            angles->dt;


    angles->roll =
            angles->complementary_alpha *
            roll_gyro_prediction
            +
            accelerometer_weight *
            accel_roll;


    /*
     * ------------------------------------------------------
     * Yaw
     * ------------------------------------------------------
     *
     * The MPU6050 has no magnetometer, so yaw has no absolute reference.
     * It is therefore gyro-integrated and will accumulate drift over time.
     */

    angles->yaw +=
            data->gyro_z_dps *
            angles->dt;


    return HAL_OK;
}


/* --------------------------------------------------------------------------
 * Timing
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef IMU_Update_dt(
        IMU_Angles_t *angles)
{
    if (angles == NULL)
    {
        return HAL_ERROR;
    }


    angles->current_time_ms =
            HAL_GetTick();


    /*
     * No previous timestamp exists on the first update.
     */
    if (angles->previous_time_ms == 0U)
    {
        angles->dt =
                0.0f;
    }
    else
    {
        angles->dt =
                (
                    angles->current_time_ms -
                    angles->previous_time_ms
                )
                /
                1000.0f;
    }


    angles->previous_time_ms =
            angles->current_time_ms;


    return HAL_OK;
}

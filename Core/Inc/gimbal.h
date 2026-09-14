/*
 * gimbal.h
 *
 * Two-axis gimbal controller:
 *
 *   P + I + direct gyro-rate damping
 *
 * Integral behavior:
 *   - Normal accumulation: 1x
 *   - Unwinding old integral bias: 3x
 *
 * Direct gyro-rate feedback must match IMU_Calculate_Angles():
 *
 *   physical pitch rate = +GY
 *   physical roll rate  = -GX
 */

#ifndef INC_GIMBAL_H_
#define INC_GIMBAL_H_

#include "stm32f4xx_hal.h"
#include "servo.h"
#include "imu_filter.h"
#include "mpu6050.h"


/* --------------------------------------------------------------------------
 * Servo calibration
 * -------------------------------------------------------------------------- */

#define GIMBAL_PITCH_MIN_US              500U
#define GIMBAL_PITCH_CENTER_US          1400U
#define GIMBAL_PITCH_MAX_US             2500U

#define GIMBAL_ROLL_MIN_US               500U
#define GIMBAL_ROLL_CENTER_US           1550U
#define GIMBAL_ROLL_MAX_US              2500U


/* Current mechanical direction */
#define GIMBAL_PITCH_SERVO_SIGN          -1.0f
#define GIMBAL_ROLL_SERVO_SIGN           -1.0f


/* Mechanical protection */
#define GIMBAL_MAX_ANGLE_DEG             60.0f


/* --------------------------------------------------------------------------
 * Angle controller gains
 * -------------------------------------------------------------------------- */

#define GIMBAL_PITCH_KP                   1.25f
#define GIMBAL_PITCH_KI                   1.00f
#define GIMBAL_PITCH_KD                   0.00f

#define GIMBAL_ROLL_KP                    1.25f
#define GIMBAL_ROLL_KI                    1.00f
#define GIMBAL_ROLL_KD                    0.00f


/* --------------------------------------------------------------------------
 * Integral unwind
 * --------------------------------------------------------------------------
 *
 * Normal integral accumulation:
 *
 *      integral += error * dt
 *
 * When the current error opposes the stored integral state:
 *
 *      integral += error * dt * 3
 *
 * This lets the controller establish and hold a large equilibrium servo
 * offset normally, but removes that old equilibrium faster when the base
 * is moved back in the opposite direction.
 */

#define GIMBAL_INTEGRAL_UNWIND_MULTIPLIER 3.00f


/* --------------------------------------------------------------------------
 * Direct gyro-rate feedback
 * --------------------------------------------------------------------------
 *
 * rate_term_deg = -KRATE * physical_rate_dps
 *
 * KRATE = 0.05:
 *
 *      20 dps  -> 1 deg
 *      100 dps -> 5 deg
 *      200 dps -> 10 deg
 */

#define GIMBAL_GYRO_RATE_ENABLED           1U

#define GIMBAL_PITCH_KRATE                 0.05f
#define GIMBAL_ROLL_KRATE                  0.05f


/* Ignore tiny residual gyro bias/noise near rest. */
#define GIMBAL_PITCH_RATE_DEADBAND_DPS     1.0f
#define GIMBAL_ROLL_RATE_DEADBAND_DPS      1.0f


/* Safety clamp for instantaneous gyro contribution. */
#define GIMBAL_RATE_TERM_MAX_DEG           15.0f


/* --------------------------------------------------------------------------
 * Anti-windup
 * -------------------------------------------------------------------------- */

#define GIMBAL_ANTI_WINDUP_ENABLED         1U


/* --------------------------------------------------------------------------
 * Angle deadband
 * -------------------------------------------------------------------------- */

#define GIMBAL_PITCH_DEADBAND_DEG          0.5f
#define GIMBAL_ROLL_DEADBAND_DEG           0.5f


/* --------------------------------------------------------------------------
 * Servo command slew rate
 * -------------------------------------------------------------------------- */

#define GIMBAL_PITCH_MAX_RATE_DPS        250.0f
#define GIMBAL_ROLL_MAX_RATE_DPS         250.0f


/* --------------------------------------------------------------------------
 * Controller state
 * -------------------------------------------------------------------------- */

typedef struct
{
    Servo_t pitch_servo;
    Servo_t roll_servo;


    /* Angle error after deadband. */
    float pitch_error_deg;
    float roll_error_deg;


    /* Integral accumulator. */
    float pitch_integral_deg_s;
    float roll_integral_deg_s;


    /* Controller terms. */
    float pitch_p_term_deg;
    float roll_p_term_deg;

    float pitch_i_term_deg;
    float roll_i_term_deg;


    /*
     * Numerical derivative retained for compatibility / telemetry.
     * KD currently remains zero.
     */
    float pitch_previous_error_deg;
    float roll_previous_error_deg;

    float pitch_derivative_dps;
    float roll_derivative_dps;

    float pitch_d_term_deg;
    float roll_d_term_deg;

    uint8_t derivative_initialized;


    /* Direct physical gyro-rate feedback. */
    float pitch_rate_dps;
    float roll_rate_dps;

    float pitch_rate_term_deg;
    float roll_rate_term_deg;


    /* Final servo offset commands. */
    float pitch_command_deg;
    float roll_command_deg;

} Gimbal_t;


/* --------------------------------------------------------------------------
 * Public functions
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef Gimbal_Init(
        Gimbal_t *gimbal,
        TIM_HandleTypeDef *pitch_htim,
        uint32_t pitch_channel,
        TIM_HandleTypeDef *roll_htim,
        uint32_t roll_channel
);


HAL_StatusTypeDef Gimbal_Center(
        Gimbal_t *gimbal
);


/*
 * MPU sample is passed directly so the controller can use calibrated gyro
 * rates matching the attitude estimator:
 *
 *      pitch rate = +gyro_y_dps
 *      roll rate  = -gyro_x_dps
 */
HAL_StatusTypeDef Gimbal_Update(
        Gimbal_t *gimbal,
        const IMU_Angles_t *angles,
        const MPU6050_Data_t *mpu
);


#endif /* INC_GIMBAL_H_ */

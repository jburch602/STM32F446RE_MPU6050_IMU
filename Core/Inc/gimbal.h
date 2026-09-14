/*
 * gimbal.h
 *
 * Two-axis gimbal controller interface.
 *
 * The controller combines angle feedback with direct gyroscope-rate
 * feedback. With KD currently set to zero, the active control law is
 * PI control plus gyro-rate damping.
 *
 * IMU axis convention:
 *   Pitch angular rate = +GY
 *   Roll angular rate  = -GX
 *
 * These signs must remain consistent with the attitude estimator.
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
#define GIMBAL_PITCH_CENTER_US          1380U
#define GIMBAL_PITCH_MAX_US             2500U

#define GIMBAL_ROLL_MIN_US               500U
#define GIMBAL_ROLL_CENTER_US           1550U
#define GIMBAL_ROLL_MAX_US              2500U


/*
 * Servo direction relative to the controller coordinate system.
 *
 * The controller performs all calculations using the IMU/gimbal sign
 * convention. The servo direction sign is applied once to the final
 * controller output before commanding the servo.
 */
#define GIMBAL_PITCH_SERVO_SIGN         -1.0f
#define GIMBAL_ROLL_SERVO_SIGN          -1.0f


/*
 * Maximum commanded servo offset from center.
 *
 * This limits the controller output before it is passed to Servo_SetOffset().
 */
#define GIMBAL_MAX_ANGLE_DEG             90.0f


/* --------------------------------------------------------------------------
 * Angle controller gains
 * --------------------------------------------------------------------------
 *
 * Control law:
 *
 *   u = P + I + D + rate_feedback
 *
 * KD is currently zero, so the numerical derivative is retained for
 * telemetry/experimentation but does not affect the servo command.
 */

#define GIMBAL_PITCH_KP                   1.1f
#define GIMBAL_PITCH_KI                   1.00f
#define GIMBAL_PITCH_KD                   0.00f

#define GIMBAL_ROLL_KP                    1.1f
#define GIMBAL_ROLL_KI                    1.00f
#define GIMBAL_ROLL_KD                    0.00f


/* --------------------------------------------------------------------------
 * Direct gyroscope-rate feedback
 * --------------------------------------------------------------------------
 *
 * The gyroscope supplies angular velocity directly rather than estimating
 * rate by differentiating the filtered attitude.
 *
 *   rate_term_deg = -KRATE * physical_rate_dps
 *
 * KRATE therefore has units of seconds:
 *
 *   (deg/s) * s = deg
 *
 * The negative sign provides damping because the rate contribution opposes
 * the measured angular motion.
 */

#define GIMBAL_GYRO_RATE_ENABLED           1U

#define GIMBAL_PITCH_KRATE                 0.08f
#define GIMBAL_ROLL_KRATE                  0.08f


/*
 * Ignore small gyro measurements near rest to prevent sensor noise and
 * residual gyro bias from continuously affecting the servo command.
 */
#define GIMBAL_PITCH_RATE_DEADBAND_DPS     1.0f
#define GIMBAL_ROLL_RATE_DEADBAND_DPS      1.0f


/*
 * Limit the maximum instantaneous contribution from gyro-rate feedback.
 */
#define GIMBAL_RATE_TERM_MAX_DEG           15.0f


/* --------------------------------------------------------------------------
 * Integral control and angle deadband
 * -------------------------------------------------------------------------- */

#define GIMBAL_ANTI_WINDUP_ENABLED         1U

#define GIMBAL_PITCH_DEADBAND_DEG          0.5f
#define GIMBAL_ROLL_DEADBAND_DEG           0.5f


/* --------------------------------------------------------------------------
 * Command slew-rate limits
 * --------------------------------------------------------------------------
 *
 * Limits how quickly the commanded servo offset is allowed to change.
 * At the 100 Hz control rate, 250 deg/s permits a maximum command change
 * of approximately 2.5 degrees per control cycle.
 */

#define GIMBAL_PITCH_MAX_RATE_DPS          250.0f
#define GIMBAL_ROLL_MAX_RATE_DPS           250.0f


/* --------------------------------------------------------------------------
 * Gimbal state
 * -------------------------------------------------------------------------- */

typedef struct
{
    /* Servo interfaces */
    Servo_t pitch_servo;
    Servo_t roll_servo;

    /* Current angle errors after deadband */
    float pitch_error_deg;
    float roll_error_deg;

    /* Integral accumulator state */
    float pitch_integral_deg_s;
    float roll_integral_deg_s;

    /* Proportional contribution */
    float pitch_p_term_deg;
    float roll_p_term_deg;

    /* Integral contribution */
    float pitch_i_term_deg;
    float roll_i_term_deg;

    /*
     * Numerical error derivative.
     *
     * Retained for telemetry and optional PID operation. These fields do not
     * affect the controller while KD is zero.
     */
    float pitch_previous_error_deg;
    float roll_previous_error_deg;

    float pitch_derivative_dps;
    float roll_derivative_dps;

    float pitch_d_term_deg;
    float roll_d_term_deg;

    uint8_t derivative_initialized;

    /*
     * Direct physical gyro-rate feedback.
     *
     * pitch_rate_dps uses +GY.
     * roll_rate_dps  uses -GX.
     */
    float pitch_rate_dps;
    float roll_rate_dps;

    float pitch_rate_term_deg;
    float roll_rate_term_deg;

    /* Final slew-limited servo offset commands */
    float pitch_command_deg;
    float roll_command_deg;

} Gimbal_t;


/*
 * Initialize the pitch and roll servos and reset all controller state.
 */
HAL_StatusTypeDef Gimbal_Init(
        Gimbal_t *gimbal,
        TIM_HandleTypeDef *pitch_htim,
        uint32_t pitch_channel,
        TIM_HandleTypeDef *roll_htim,
        uint32_t roll_channel
);


/*
 * Command both servos to their calibrated centers and reset controller state.
 */
HAL_StatusTypeDef Gimbal_Center(
        Gimbal_t *gimbal
);


/*
 * Update the gimbal controller.
 *
 * angles:
 *   Supplies pitch, roll, and loop dt from the attitude estimator.
 *   The controller target is zero pitch and zero roll. The meaning of that
 *   zero reference is determined by the attitude estimator upstream.
 *
 * mpu:
 *   Supplies calibrated gyroscope rates for direct rate feedback.
 *
 * Axis convention:
 *   Pitch angular rate = +gyro_y_dps
 *   Roll angular rate  = -gyro_x_dps
 */
HAL_StatusTypeDef Gimbal_Update(
        Gimbal_t *gimbal,
        const IMU_Angles_t *angles,
        const MPU6050_Data_t *mpu
);

#endif /* INC_GIMBAL_H_ */

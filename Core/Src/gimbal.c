/*
 * gimbal.c
 *
 * Two-axis gimbal controller.
 *
 * Controller structure:
 *
 *   angle error
 *       |
 *       +-- proportional feedback
 *       +-- integral feedback
 *       +-- optional numerical derivative feedback
 *       +-- direct gyroscope-rate feedback
 *       |
 *       v
 *   servo direction correction
 *       |
 *   command clamp
 *       |
 *   slew-rate limit
 *       |
 *   servo command
 *
 * The direct gyroscope-rate term provides damping from measured angular
 * velocity without requiring differentiation of the filtered attitude.
 */

#include "gimbal.h"


/*
 * Clamp a signed value to +/-limit.
 */
static float Gimbal_ClampSymmetric(
        float value,
        float limit)
{
    if (value > limit)
    {
        value = limit;
    }

    if (value < -limit)
    {
        value = -limit;
    }

    return value;
}


/*
 * Limit a servo offset command to the configured gimbal range.
 */
static float Gimbal_ClampAngle(
        float angle_deg)
{
    return Gimbal_ClampSymmetric(
            angle_deg,
            GIMBAL_MAX_ANGLE_DEG
    );
}


/*
 * Suppress values whose magnitude is inside the specified deadband.
 */
static float Gimbal_ApplyDeadband(
        float value,
        float deadband)
{
    if (value > -deadband &&
        value < deadband)
    {
        return 0.0f;
    }

    return value;
}


/*
 * Limit the rate at which a servo command can change.
 *
 * max_step_deg = max_rate_dps * dt
 *
 * The returned command can therefore move toward the target by no more
 * than max_step_deg during the current control cycle.
 */
static float Gimbal_ApplySlewRate(
        float current_command_deg,
        float target_command_deg,
        float max_rate_dps,
        float dt)
{
    if (dt <= 0.0f)
    {
        return current_command_deg;
    }

    float max_step_deg =
            max_rate_dps * dt;

    float command_change_deg =
            target_command_deg -
            current_command_deg;

    if (command_change_deg > max_step_deg)
    {
        command_change_deg = max_step_deg;
    }

    if (command_change_deg < -max_step_deg)
    {
        command_change_deg = -max_step_deg;
    }

    return current_command_deg +
            command_change_deg;
}


/*
 * Update the integral accumulator with conditional anti-windup.
 *
 * Integration proceeds normally while the controller is unsaturated.
 * When saturated, integration is allowed only when the error would drive
 * the controller back toward the valid output range.
 */
static float Gimbal_UpdateIntegral(
        float integral_deg_s,
        float error_deg,
        float ki,
        float control_deg,
        float output_limit_deg,
        float dt)
{
    if (ki <= 0.0f || dt <= 0.0f)
    {
        return 0.0f;
    }

#if GIMBAL_ANTI_WINDUP_ENABLED

    if ((control_deg < output_limit_deg &&
         control_deg > -output_limit_deg) ||
        (control_deg >= output_limit_deg &&
         error_deg < 0.0f) ||
        (control_deg <= -output_limit_deg &&
         error_deg > 0.0f))
    {
        return integral_deg_s +
                (error_deg * dt);
    }

    return integral_deg_s;

#else

    return integral_deg_s +
            (error_deg * dt);

#endif
}


/*
 * Calculate the numerical derivative of angle error.
 *
 * This state is retained for telemetry and optional PID operation.
 * It does not affect the controller while KD is zero.
 */
static float Gimbal_CalculateDerivative(
        float error_deg,
        float previous_error_deg,
        float dt)
{
    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    return (error_deg - previous_error_deg) /
            dt;
}


/*
 * Calculate the direct gyro-rate damping term.
 *
 * The attitude error for a zero-degree target is:
 *
 *   error = -angle
 *
 * Therefore:
 *
 *   d(error)/dt = -d(angle)/dt
 *
 * The gyroscope directly measures d(angle)/dt, so the stabilizing rate
 * contribution has the opposite sign:
 *
 *   rate_term = -KRATE * physical_rate
 *
 * Small rates are rejected by the gyro deadband and the resulting command
 * contribution is limited to GIMBAL_RATE_TERM_MAX_DEG.
 */
static float Gimbal_CalculateRateTerm(
        float physical_rate_dps,
        float krate,
        float deadband_dps)
{
#if GIMBAL_GYRO_RATE_ENABLED

    float rate_dps =
            Gimbal_ApplyDeadband(
                    physical_rate_dps,
                    deadband_dps
            );

    float rate_term_deg =
            -krate * rate_dps;

    return Gimbal_ClampSymmetric(
            rate_term_deg,
            GIMBAL_RATE_TERM_MAX_DEG
    );

#else

    (void)physical_rate_dps;
    (void)krate;
    (void)deadband_dps;

    return 0.0f;

#endif
}


/*
 * Initialize both servos, apply their calibration values, start PWM at
 * the calibrated center positions, and clear all controller state.
 */
HAL_StatusTypeDef Gimbal_Init(
        Gimbal_t *gimbal,
        TIM_HandleTypeDef *pitch_htim,
        uint32_t pitch_channel,
        TIM_HandleTypeDef *roll_htim,
        uint32_t roll_channel)
{
    if (gimbal == NULL ||
        pitch_htim == NULL ||
        roll_htim == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;


    /* Initialize and configure the pitch servo. */
    status = Servo_Init(
            &gimbal->pitch_servo,
            pitch_htim,
            pitch_channel
    );

    if (status != HAL_OK)
    {
        return status;
    }

    gimbal->pitch_servo.pulse_min_us =
            GIMBAL_PITCH_MIN_US;

    gimbal->pitch_servo.pulse_center_us =
            GIMBAL_PITCH_CENTER_US;

    gimbal->pitch_servo.pulse_max_us =
            GIMBAL_PITCH_MAX_US;


    /* Initialize and configure the roll servo. */
    status = Servo_Init(
            &gimbal->roll_servo,
            roll_htim,
            roll_channel
    );

    if (status != HAL_OK)
    {
        return status;
    }

    gimbal->roll_servo.pulse_min_us =
            GIMBAL_ROLL_MIN_US;

    gimbal->roll_servo.pulse_center_us =
            GIMBAL_ROLL_CENTER_US;

    gimbal->roll_servo.pulse_max_us =
            GIMBAL_ROLL_MAX_US;


    /* Start both PWM outputs at their calibrated center positions. */
    status = Servo_Start(
            &gimbal->pitch_servo,
            GIMBAL_PITCH_CENTER_US
    );

    if (status != HAL_OK)
    {
        return status;
    }

    status = Servo_Start(
            &gimbal->roll_servo,
            GIMBAL_ROLL_CENTER_US
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Reset angle-controller state. */
    gimbal->pitch_error_deg = 0.0f;
    gimbal->roll_error_deg = 0.0f;

    gimbal->pitch_integral_deg_s = 0.0f;
    gimbal->roll_integral_deg_s = 0.0f;

    gimbal->pitch_p_term_deg = 0.0f;
    gimbal->roll_p_term_deg = 0.0f;

    gimbal->pitch_i_term_deg = 0.0f;
    gimbal->roll_i_term_deg = 0.0f;


    /* Reset numerical derivative state. */
    gimbal->pitch_previous_error_deg = 0.0f;
    gimbal->roll_previous_error_deg = 0.0f;

    gimbal->pitch_derivative_dps = 0.0f;
    gimbal->roll_derivative_dps = 0.0f;

    gimbal->pitch_d_term_deg = 0.0f;
    gimbal->roll_d_term_deg = 0.0f;

    gimbal->derivative_initialized = 0U;


    /* Reset direct gyro-rate feedback state. */
    gimbal->pitch_rate_dps = 0.0f;
    gimbal->roll_rate_dps = 0.0f;

    gimbal->pitch_rate_term_deg = 0.0f;
    gimbal->roll_rate_term_deg = 0.0f;


    /* Reset final servo commands. */
    gimbal->pitch_command_deg = 0.0f;
    gimbal->roll_command_deg = 0.0f;

    return HAL_OK;
}


/*
 * Return both servos to their calibrated centers and clear all controller
 * state so the next update begins without residual integral, derivative,
 * rate-feedback, or command history.
 */
HAL_StatusTypeDef Gimbal_Center(
        Gimbal_t *gimbal)
{
    if (gimbal == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;


    status = Servo_SetPulse(
            &gimbal->pitch_servo,
            GIMBAL_PITCH_CENTER_US
    );

    if (status != HAL_OK)
    {
        return status;
    }


    status = Servo_SetPulse(
            &gimbal->roll_servo,
            GIMBAL_ROLL_CENTER_US
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Reset angle-controller state. */
    gimbal->pitch_error_deg = 0.0f;
    gimbal->roll_error_deg = 0.0f;

    gimbal->pitch_integral_deg_s = 0.0f;
    gimbal->roll_integral_deg_s = 0.0f;

    gimbal->pitch_p_term_deg = 0.0f;
    gimbal->roll_p_term_deg = 0.0f;

    gimbal->pitch_i_term_deg = 0.0f;
    gimbal->roll_i_term_deg = 0.0f;


    /* Reset numerical derivative state. */
    gimbal->pitch_previous_error_deg = 0.0f;
    gimbal->roll_previous_error_deg = 0.0f;

    gimbal->pitch_derivative_dps = 0.0f;
    gimbal->roll_derivative_dps = 0.0f;

    gimbal->pitch_d_term_deg = 0.0f;
    gimbal->roll_d_term_deg = 0.0f;

    gimbal->derivative_initialized = 0U;


    /* Reset gyro-rate feedback state. */
    gimbal->pitch_rate_dps = 0.0f;
    gimbal->roll_rate_dps = 0.0f;

    gimbal->pitch_rate_term_deg = 0.0f;
    gimbal->roll_rate_term_deg = 0.0f;


    /* Reset final servo commands. */
    gimbal->pitch_command_deg = 0.0f;
    gimbal->roll_command_deg = 0.0f;

    return HAL_OK;
}


/*
 * Execute one gimbal control update.
 *
 * Controller target:
 *
 *   pitch = 0 deg
 *   roll  = 0 deg
 *
 * The attitude estimator determines what zero represents. This allows the
 * controller itself to remain unchanged if the estimator later uses a
 * startup-relative attitude reference.
 */
HAL_StatusTypeDef Gimbal_Update(
        Gimbal_t *gimbal,
        const IMU_Angles_t *angles,
        const MPU6050_Data_t *mpu)
{
    if (gimbal == NULL ||
        angles == NULL ||
        mpu == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    float pitch_raw_error_deg;
    float roll_raw_error_deg;

    float pitch_control_deg;
    float roll_control_deg;

    float pitch_target_command_deg;
    float roll_target_command_deg;


    /*
     * Angle error.
     *
     * The controller target is zero on both axes.
     */
    pitch_raw_error_deg =
            0.0f - angles->pitch;

    roll_raw_error_deg =
            0.0f - angles->roll;


    /*
     * Numerical error derivative.
     *
     * Retained for telemetry and optional PID operation. With KD = 0,
     * these values currently have no effect on the servo command.
     */
    if (gimbal->derivative_initialized == 0U)
    {
        gimbal->pitch_derivative_dps = 0.0f;
        gimbal->roll_derivative_dps = 0.0f;

        gimbal->derivative_initialized = 1U;
    }
    else
    {
        gimbal->pitch_derivative_dps =
                Gimbal_CalculateDerivative(
                        pitch_raw_error_deg,
                        gimbal->pitch_previous_error_deg,
                        angles->dt
                );

        gimbal->roll_derivative_dps =
                Gimbal_CalculateDerivative(
                        roll_raw_error_deg,
                        gimbal->roll_previous_error_deg,
                        angles->dt
                );
    }

    gimbal->pitch_previous_error_deg =
            pitch_raw_error_deg;

    gimbal->roll_previous_error_deg =
            roll_raw_error_deg;


    /*
     * Convert MPU gyro measurements into the same physical axis convention
     * used by the attitude estimator.
     *
     *   Pitch rate = +GY
     *   Roll rate  = -GX
     */
    gimbal->pitch_rate_dps =
            mpu->gyro_y_dps;

    gimbal->roll_rate_dps =
            -mpu->gyro_x_dps;


    /*
     * Apply angle-error deadband to reduce unnecessary servo activity near
     * the target attitude.
     */
    gimbal->pitch_error_deg =
            Gimbal_ApplyDeadband(
                    pitch_raw_error_deg,
                    GIMBAL_PITCH_DEADBAND_DEG
            );

    gimbal->roll_error_deg =
            Gimbal_ApplyDeadband(
                    roll_raw_error_deg,
                    GIMBAL_ROLL_DEADBAND_DEG
            );


    /* Proportional terms. */
    gimbal->pitch_p_term_deg =
            GIMBAL_PITCH_KP *
            gimbal->pitch_error_deg;

    gimbal->roll_p_term_deg =
            GIMBAL_ROLL_KP *
            gimbal->roll_error_deg;


    /*
     * Integral terms from the accumulator state at the beginning of this
     * control cycle.
     */
    gimbal->pitch_i_term_deg =
            GIMBAL_PITCH_KI *
            gimbal->pitch_integral_deg_s;

    gimbal->roll_i_term_deg =
            GIMBAL_ROLL_KI *
            gimbal->roll_integral_deg_s;


    /*
     * Numerical derivative terms.
     *
     * These remain zero while KD is configured as zero.
     */
    gimbal->pitch_d_term_deg =
            GIMBAL_PITCH_KD *
            gimbal->pitch_derivative_dps;

    gimbal->roll_d_term_deg =
            GIMBAL_ROLL_KD *
            gimbal->roll_derivative_dps;


    /*
     * Direct gyro-rate feedback.
     *
     * This contribution reacts immediately to angular velocity and provides
     * damping before a large angle error has accumulated.
     */
    gimbal->pitch_rate_term_deg =
            Gimbal_CalculateRateTerm(
                    gimbal->pitch_rate_dps,
                    GIMBAL_PITCH_KRATE,
                    GIMBAL_PITCH_RATE_DEADBAND_DPS
            );

    gimbal->roll_rate_term_deg =
            Gimbal_CalculateRateTerm(
                    gimbal->roll_rate_dps,
                    GIMBAL_ROLL_KRATE,
                    GIMBAL_ROLL_RATE_DEADBAND_DPS
            );


    /*
     * Preliminary controller output used by the anti-windup logic.
     *
     * Servo mounting direction is intentionally not applied here. All
     * controller calculations remain in the common gimbal coordinate system.
     */
    pitch_control_deg =
            gimbal->pitch_p_term_deg +
            gimbal->pitch_i_term_deg +
            gimbal->pitch_d_term_deg +
            gimbal->pitch_rate_term_deg;

    roll_control_deg =
            gimbal->roll_p_term_deg +
            gimbal->roll_i_term_deg +
            gimbal->roll_d_term_deg +
            gimbal->roll_rate_term_deg;


    /*
     * Update the integral accumulators.
     *
     * Conditional anti-windup prevents the integral term from continuing
     * to grow when the controller output is saturated in the same direction
     * as the current error.
     */
    gimbal->pitch_integral_deg_s =
            Gimbal_UpdateIntegral(
                    gimbal->pitch_integral_deg_s,
                    gimbal->pitch_error_deg,
                    GIMBAL_PITCH_KI,
                    pitch_control_deg,
                    GIMBAL_MAX_ANGLE_DEG,
                    angles->dt
            );

    gimbal->roll_integral_deg_s =
            Gimbal_UpdateIntegral(
                    gimbal->roll_integral_deg_s,
                    gimbal->roll_error_deg,
                    GIMBAL_ROLL_KI,
                    roll_control_deg,
                    GIMBAL_MAX_ANGLE_DEG,
                    angles->dt
            );


    /*
     * Recalculate the integral contributions after updating the accumulator
     * so the current sample is reflected in the final command.
     */
    gimbal->pitch_i_term_deg =
            GIMBAL_PITCH_KI *
            gimbal->pitch_integral_deg_s;

    gimbal->roll_i_term_deg =
            GIMBAL_ROLL_KI *
            gimbal->roll_integral_deg_s;


    /*
     * Build the final target commands.
     *
     * Servo mounting direction is applied exactly once at this stage.
     */
    pitch_target_command_deg =
            (
                gimbal->pitch_p_term_deg +
                gimbal->pitch_i_term_deg +
                gimbal->pitch_d_term_deg +
                gimbal->pitch_rate_term_deg
            ) *
            GIMBAL_PITCH_SERVO_SIGN;

    roll_target_command_deg =
            (
                gimbal->roll_p_term_deg +
                gimbal->roll_i_term_deg +
                gimbal->roll_d_term_deg +
                gimbal->roll_rate_term_deg
            ) *
            GIMBAL_ROLL_SERVO_SIGN;


    /*
     * Enforce the configured command range.
     */
    pitch_target_command_deg =
            Gimbal_ClampAngle(
                    pitch_target_command_deg
            );

    roll_target_command_deg =
            Gimbal_ClampAngle(
                    roll_target_command_deg
            );


    /*
     * Slew-limit the command before sending it to the servos.
     *
     * With the current 250 deg/s limit and a 100 Hz control loop, each
     * command can change by at most approximately 2.5 degrees per cycle.
     */
    gimbal->pitch_command_deg =
            Gimbal_ApplySlewRate(
                    gimbal->pitch_command_deg,
                    pitch_target_command_deg,
                    GIMBAL_PITCH_MAX_RATE_DPS,
                    angles->dt
            );

    gimbal->roll_command_deg =
            Gimbal_ApplySlewRate(
                    gimbal->roll_command_deg,
                    roll_target_command_deg,
                    GIMBAL_ROLL_MAX_RATE_DPS,
                    angles->dt
            );


    /*
     * Apply the final servo offset commands.
     */
    status = Servo_SetOffset(
            &gimbal->pitch_servo,
            gimbal->pitch_command_deg
    );

    if (status != HAL_OK)
    {
        return status;
    }


    status = Servo_SetOffset(
            &gimbal->roll_servo,
            gimbal->roll_command_deg
    );

    if (status != HAL_OK)
    {
        return status;
    }


    return HAL_OK;
}

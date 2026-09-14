/*
 * gimbal.c
 *
 * Two-axis gimbal controller:
 *
 *      output = P + I + D + gyro-rate
 *
 * Current controller:
 *
 *      Kp     = 1.25
 *      Ki     = 1.00
 *      Kd     = 0.00
 *      Krate  = 0.05
 *
 * Integral behavior:
 *
 *      Normal accumulation = 1x
 *      Integral unwind     = 3x
 *
 * The accelerated unwind is used only when the current attitude error
 * opposes the sign of the stored integral state.
 *
 * This preserves the ability of the integral controller to maintain large
 * equilibrium servo offsets while allowing old equilibrium bias to disappear
 * faster when the base moves back toward its previous orientation.
 */

#include "gimbal.h"


/* --------------------------------------------------------------------------
 * Clamp helpers
 * -------------------------------------------------------------------------- */

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


static float Gimbal_ClampAngle(
        float angle_deg)
{
    return Gimbal_ClampSymmetric(
            angle_deg,
            GIMBAL_MAX_ANGLE_DEG
    );
}


/* --------------------------------------------------------------------------
 * Deadband
 * -------------------------------------------------------------------------- */

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


/* --------------------------------------------------------------------------
 * Servo command slew rate
 * -------------------------------------------------------------------------- */

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
            max_rate_dps *
            dt;


    float command_change_deg =
            target_command_deg -
            current_command_deg;


    if (command_change_deg >
        max_step_deg)
    {
        command_change_deg =
                max_step_deg;
    }


    if (command_change_deg <
        -max_step_deg)
    {
        command_change_deg =
                -max_step_deg;
    }


    return current_command_deg +
            command_change_deg;
}


/* --------------------------------------------------------------------------
 * Integral controller
 * -------------------------------------------------------------------------- */

/*
 * Update integral state.
 *
 * Normal operation:
 *
 *      integral += error * dt
 *
 * Fast unwind:
 *
 * If the stored integral and current error have opposite signs, the
 * controller is trying to remove an existing equilibrium bias.
 *
 * In that situation:
 *
 *      integral += error * dt * UNWIND_MULTIPLIER
 *
 * Example:
 *
 *      integral = -54 deg*s
 *      error    = +10 deg
 *
 * The positive error is trying to remove the old negative integral.
 * Therefore the update occurs at 3x speed.
 *
 * Once the integral crosses zero, the signs are no longer opposite and
 * integration automatically returns to normal 1x speed.
 *
 * This is intentionally NOT an integral clamp.
 */
static float Gimbal_UpdateIntegral(
        float integral_deg_s,
        float error_deg,
        float ki,
        float control_deg,
        float output_limit_deg,
        float dt)
{
    if (ki <= 0.0f ||
        dt <= 0.0f)
    {
        return 0.0f;
    }


    /*
     * Default:
     *
     * Normal integral accumulation.
     */
    float integral_multiplier =
            1.0f;


    /*
     * If error opposes stored integral state, we are unwinding an old
     * equilibrium command.
     *
     * Only accelerate the removal of existing I.
     */
    if ((integral_deg_s > 0.0f &&
         error_deg < 0.0f) ||

        (integral_deg_s < 0.0f &&
         error_deg > 0.0f))
    {
        integral_multiplier =
                GIMBAL_INTEGRAL_UNWIND_MULTIPLIER;
    }


#if GIMBAL_ANTI_WINDUP_ENABLED

    /*
     * Existing conditional anti-windup behavior.
     *
     * Integrate normally while controller output is unsaturated.
     *
     * While positively saturated:
     *      only negative error may integrate.
     *
     * While negatively saturated:
     *      only positive error may integrate.
     *
     * This prevents the integral state from driving farther into actuator
     * saturation while still allowing it to unwind out of saturation.
     */
    if ((control_deg < output_limit_deg &&
         control_deg > -output_limit_deg) ||

        (control_deg >= output_limit_deg &&
         error_deg < 0.0f) ||

        (control_deg <= -output_limit_deg &&
         error_deg > 0.0f))
    {
        return integral_deg_s +
                (
                    error_deg *
                    dt *
                    integral_multiplier
                );
    }


    return integral_deg_s;

#else

    return integral_deg_s +
            (
                error_deg *
                dt *
                integral_multiplier
            );

#endif
}


/* --------------------------------------------------------------------------
 * Numerical derivative
 * -------------------------------------------------------------------------- */

static float Gimbal_CalculateDerivative(
        float error_deg,
        float previous_error_deg,
        float dt)
{
    if (dt <= 0.0f)
    {
        return 0.0f;
    }


    return (error_deg -
            previous_error_deg)
            /
            dt;
}


/* --------------------------------------------------------------------------
 * Direct gyro-rate damping
 * -------------------------------------------------------------------------- */

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


    /*
     * For startup-relative attitude:
     *
     *      error = target - angle
     *
     * Target is zero:
     *
     *      error = -angle
     *
     * Therefore:
     *
     *      d(error)/dt = -d(angle)/dt
     *
     * The physical gyro measures:
     *
     *      d(angle)/dt
     *
     * so stabilizing rate feedback has the opposite sign:
     *
     *      rate_term = -Krate * physical_rate
     */
    float rate_term_deg =
            -krate *
            rate_dps;


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


/* --------------------------------------------------------------------------
 * Initialization
 * -------------------------------------------------------------------------- */

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


    /* ----------------------------------------------------------------------
     * Pitch servo
     * ---------------------------------------------------------------------- */

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


    /* ----------------------------------------------------------------------
     * Roll servo
     * ---------------------------------------------------------------------- */

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


    /* ----------------------------------------------------------------------
     * Start servos at calibrated centers
     * ---------------------------------------------------------------------- */

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


    /* ----------------------------------------------------------------------
     * Reset controller state
     * ---------------------------------------------------------------------- */

    gimbal->pitch_error_deg =
            0.0f;

    gimbal->roll_error_deg =
            0.0f;


    gimbal->pitch_integral_deg_s =
            0.0f;

    gimbal->roll_integral_deg_s =
            0.0f;


    gimbal->pitch_p_term_deg =
            0.0f;

    gimbal->roll_p_term_deg =
            0.0f;


    gimbal->pitch_i_term_deg =
            0.0f;

    gimbal->roll_i_term_deg =
            0.0f;


    gimbal->pitch_previous_error_deg =
            0.0f;

    gimbal->roll_previous_error_deg =
            0.0f;


    gimbal->pitch_derivative_dps =
            0.0f;

    gimbal->roll_derivative_dps =
            0.0f;


    gimbal->pitch_d_term_deg =
            0.0f;

    gimbal->roll_d_term_deg =
            0.0f;


    gimbal->pitch_rate_dps =
            0.0f;

    gimbal->roll_rate_dps =
            0.0f;


    gimbal->pitch_rate_term_deg =
            0.0f;

    gimbal->roll_rate_term_deg =
            0.0f;


    gimbal->pitch_command_deg =
            0.0f;

    gimbal->roll_command_deg =
            0.0f;


    gimbal->derivative_initialized =
            0U;


    return HAL_OK;
}


/* --------------------------------------------------------------------------
 * Center / controller reset
 * -------------------------------------------------------------------------- */

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


    /* Reset controller state. */

    gimbal->pitch_error_deg =
            0.0f;

    gimbal->roll_error_deg =
            0.0f;


    gimbal->pitch_integral_deg_s =
            0.0f;

    gimbal->roll_integral_deg_s =
            0.0f;


    gimbal->pitch_p_term_deg =
            0.0f;

    gimbal->roll_p_term_deg =
            0.0f;


    gimbal->pitch_i_term_deg =
            0.0f;

    gimbal->roll_i_term_deg =
            0.0f;


    gimbal->pitch_previous_error_deg =
            0.0f;

    gimbal->roll_previous_error_deg =
            0.0f;


    gimbal->pitch_derivative_dps =
            0.0f;

    gimbal->roll_derivative_dps =
            0.0f;


    gimbal->pitch_d_term_deg =
            0.0f;

    gimbal->roll_d_term_deg =
            0.0f;


    gimbal->pitch_rate_dps =
            0.0f;

    gimbal->roll_rate_dps =
            0.0f;


    gimbal->pitch_rate_term_deg =
            0.0f;

    gimbal->roll_rate_term_deg =
            0.0f;


    gimbal->pitch_command_deg =
            0.0f;

    gimbal->roll_command_deg =
            0.0f;


    gimbal->derivative_initialized =
            0U;


    return HAL_OK;
}


/* --------------------------------------------------------------------------
 * Closed-loop control update
 * -------------------------------------------------------------------------- */

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


    /* ----------------------------------------------------------------------
     * Attitude error
     * ----------------------------------------------------------------------
     *
     * main.c supplies startup-relative angles.
     *
     * Therefore target attitude is:
     *
     *      pitch = 0 deg
     *      roll  = 0 deg
     */

    pitch_raw_error_deg =
            0.0f -
            angles->pitch;


    roll_raw_error_deg =
            0.0f -
            angles->roll;


    /* ----------------------------------------------------------------------
     * Numerical derivative
     * ----------------------------------------------------------------------
     *
     * Retained for telemetry and future PID experimentation.
     *
     * KD currently remains zero.
     */

    if (gimbal->derivative_initialized ==
        0U)
    {
        gimbal->pitch_derivative_dps =
                0.0f;

        gimbal->roll_derivative_dps =
                0.0f;

        gimbal->derivative_initialized =
                1U;
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


    /* ----------------------------------------------------------------------
     * Physical gyro rates
     * ----------------------------------------------------------------------
     *
     * These MUST match the attitude estimator:
     *
     *      pitch = +GY
     *      roll  = -GX
     */

    gimbal->pitch_rate_dps =
            mpu->gyro_y_dps;


    gimbal->roll_rate_dps =
            -mpu->gyro_x_dps;


    /* ----------------------------------------------------------------------
     * Angle deadband
     * ---------------------------------------------------------------------- */

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


    /* ----------------------------------------------------------------------
     * P terms
     * ---------------------------------------------------------------------- */

    gimbal->pitch_p_term_deg =
            GIMBAL_PITCH_KP *
            gimbal->pitch_error_deg;


    gimbal->roll_p_term_deg =
            GIMBAL_ROLL_KP *
            gimbal->roll_error_deg;


    /* ----------------------------------------------------------------------
     * Existing I terms
     * ---------------------------------------------------------------------- */

    gimbal->pitch_i_term_deg =
            GIMBAL_PITCH_KI *
            gimbal->pitch_integral_deg_s;


    gimbal->roll_i_term_deg =
            GIMBAL_ROLL_KI *
            gimbal->roll_integral_deg_s;


    /* ----------------------------------------------------------------------
     * Numerical D terms
     * ----------------------------------------------------------------------
     *
     * Currently zero because KD = 0.
     */

    gimbal->pitch_d_term_deg =
            GIMBAL_PITCH_KD *
            gimbal->pitch_derivative_dps;


    gimbal->roll_d_term_deg =
            GIMBAL_ROLL_KD *
            gimbal->roll_derivative_dps;


    /* ----------------------------------------------------------------------
     * Direct gyro-rate damping
     * ---------------------------------------------------------------------- */

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


    /* ----------------------------------------------------------------------
     * Preliminary controller output
     * ----------------------------------------------------------------------
     *
     * Controller coordinates:
     *
     *      u = P + I + D + rate
     *
     * This is used for anti-windup before the servo mounting sign is applied.
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


    /* ----------------------------------------------------------------------
     * Integral update
     * ----------------------------------------------------------------------
     *
     * This is the ONLY meaningful controller behavior changed from the
     * previous stable version.
     *
     * Gimbal_UpdateIntegral() automatically chooses:
     *
     *      1x when building/maintaining integral
     *      3x when current error opposes existing integral
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


    /* ----------------------------------------------------------------------
     * Recalculate I after current sample
     * ---------------------------------------------------------------------- */

    gimbal->pitch_i_term_deg =
            GIMBAL_PITCH_KI *
            gimbal->pitch_integral_deg_s;


    gimbal->roll_i_term_deg =
            GIMBAL_ROLL_KI *
            gimbal->roll_integral_deg_s;


    /* ----------------------------------------------------------------------
     * Final servo target
     * ----------------------------------------------------------------------
     *
     * Servo mounting direction is applied exactly once here.
     */

    pitch_target_command_deg =
            (
                gimbal->pitch_p_term_deg +
                gimbal->pitch_i_term_deg +
                gimbal->pitch_d_term_deg +
                gimbal->pitch_rate_term_deg
            )
            *
            GIMBAL_PITCH_SERVO_SIGN;


    roll_target_command_deg =
            (
                gimbal->roll_p_term_deg +
                gimbal->roll_i_term_deg +
                gimbal->roll_d_term_deg +
                gimbal->roll_rate_term_deg
            )
            *
            GIMBAL_ROLL_SERVO_SIGN;


    /* ----------------------------------------------------------------------
     * Mechanical protection
     * ---------------------------------------------------------------------- */

    pitch_target_command_deg =
            Gimbal_ClampAngle(
                    pitch_target_command_deg
            );


    roll_target_command_deg =
            Gimbal_ClampAngle(
                    roll_target_command_deg
            );


    /* ----------------------------------------------------------------------
     * Servo command slew limiting
     * ---------------------------------------------------------------------- */

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


    /* ----------------------------------------------------------------------
     * Servo output
     * ---------------------------------------------------------------------- */

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

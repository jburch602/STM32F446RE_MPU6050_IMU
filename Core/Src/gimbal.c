/*
 * gimbal.c
 *
 * Created on: Sep 6, 2026
 * Author: Jackson
 */

#include "gimbal.h"


/*
 * Clamps a gimbal command to the configured range.
 */
static float Gimbal_ClampAngle(float angle_deg)
{
    if (angle_deg > GIMBAL_MAX_ANGLE_DEG)
    {
        angle_deg = GIMBAL_MAX_ANGLE_DEG;
    }

    if (angle_deg < -GIMBAL_MAX_ANGLE_DEG)
    {
        angle_deg = -GIMBAL_MAX_ANGLE_DEG;
    }

    return angle_deg;
}


/*
 * Applies deadband around zero error.
 */
static float Gimbal_ApplyDeadband(
        float error_deg,
        float deadband_deg)
{
    if (error_deg > -deadband_deg &&
        error_deg < deadband_deg)
    {
        return 0.0f;
    }

    return error_deg;
}


/*
 * Limits the rate of command change.
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
 * Updates integral state.
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

    /* Apply conditional integration */
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

    /* Update integral without anti-windup */
    return integral_deg_s +
            (error_deg * dt);

#endif
}


/*
 * Initializes both gimbal axes.
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


    /* Initialize pitch servo */
    status = Servo_Init(
            &gimbal->pitch_servo,
            pitch_htim,
            pitch_channel
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Apply pitch servo calibration */
    gimbal->pitch_servo.pulse_min_us =
            GIMBAL_PITCH_MIN_US;

    gimbal->pitch_servo.pulse_center_us =
            GIMBAL_PITCH_CENTER_US;

    gimbal->pitch_servo.pulse_max_us =
            GIMBAL_PITCH_MAX_US;


    /* Initialize roll servo */
    status = Servo_Init(
            &gimbal->roll_servo,
            roll_htim,
            roll_channel
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Apply roll servo calibration */
    gimbal->roll_servo.pulse_min_us =
            GIMBAL_ROLL_MIN_US;

    gimbal->roll_servo.pulse_center_us =
            GIMBAL_ROLL_CENTER_US;

    gimbal->roll_servo.pulse_max_us =
            GIMBAL_ROLL_MAX_US;


    /* Start pitch servo at center */
    status = Servo_Start(
            &gimbal->pitch_servo,
            GIMBAL_PITCH_CENTER_US
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Start roll servo at center */
    status = Servo_Start(
            &gimbal->roll_servo,
            GIMBAL_ROLL_CENTER_US
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Initialize controller state */
    gimbal->pitch_error_deg = 0.0f;
    gimbal->roll_error_deg = 0.0f;

    gimbal->pitch_integral_deg_s = 0.0f;
    gimbal->roll_integral_deg_s = 0.0f;

    gimbal->pitch_p_term_deg = 0.0f;
    gimbal->roll_p_term_deg = 0.0f;

    gimbal->pitch_i_term_deg = 0.0f;
    gimbal->roll_i_term_deg = 0.0f;

    gimbal->pitch_command_deg = 0.0f;
    gimbal->roll_command_deg = 0.0f;


    return HAL_OK;
}


/*
 * Centers both gimbal axes.
 */
HAL_StatusTypeDef Gimbal_Center(Gimbal_t *gimbal)
{
    if (gimbal == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;


    /* Center pitch servo */
    status = Servo_SetPulse(
            &gimbal->pitch_servo,
            GIMBAL_PITCH_CENTER_US
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Center roll servo */
    status = Servo_SetPulse(
            &gimbal->roll_servo,
            GIMBAL_ROLL_CENTER_US
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Reset controller state */
    gimbal->pitch_error_deg = 0.0f;
    gimbal->roll_error_deg = 0.0f;

    gimbal->pitch_integral_deg_s = 0.0f;
    gimbal->roll_integral_deg_s = 0.0f;

    gimbal->pitch_p_term_deg = 0.0f;
    gimbal->roll_p_term_deg = 0.0f;

    gimbal->pitch_i_term_deg = 0.0f;
    gimbal->roll_i_term_deg = 0.0f;

    gimbal->pitch_command_deg = 0.0f;
    gimbal->roll_command_deg = 0.0f;


    return HAL_OK;
}


/*
 * Updates pitch and roll gimbal commands.
 */
HAL_StatusTypeDef Gimbal_Update(
        Gimbal_t *gimbal,
        const IMU_Angles_t *angles)
{
    if (gimbal == NULL || angles == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    float pitch_control_deg;
    float roll_control_deg;

    float pitch_target_command_deg;
    float roll_target_command_deg;


    /* Calculate control error */
    gimbal->pitch_error_deg =
            0.0f - angles->pitch;

    gimbal->roll_error_deg =
            0.0f - angles->roll;


    /* Apply control deadband */
    gimbal->pitch_error_deg =
            Gimbal_ApplyDeadband(
                    gimbal->pitch_error_deg,
                    GIMBAL_PITCH_DEADBAND_DEG
            );

    gimbal->roll_error_deg =
            Gimbal_ApplyDeadband(
                    gimbal->roll_error_deg,
                    GIMBAL_ROLL_DEADBAND_DEG
            );


    /* Calculate proportional terms */
    gimbal->pitch_p_term_deg =
            GIMBAL_PITCH_KP *
            gimbal->pitch_error_deg;

    gimbal->roll_p_term_deg =
            GIMBAL_ROLL_KP *
            gimbal->roll_error_deg;


    /* Calculate current integral terms */
    gimbal->pitch_i_term_deg =
            GIMBAL_PITCH_KI *
            gimbal->pitch_integral_deg_s;

    gimbal->roll_i_term_deg =
            GIMBAL_ROLL_KI *
            gimbal->roll_integral_deg_s;


    /* Calculate controller outputs */
    pitch_control_deg =
            gimbal->pitch_p_term_deg +
            gimbal->pitch_i_term_deg;

    roll_control_deg =
            gimbal->roll_p_term_deg +
            gimbal->roll_i_term_deg;


    /* Update integral state */
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


    /* Update integral terms */
    gimbal->pitch_i_term_deg =
            GIMBAL_PITCH_KI *
            gimbal->pitch_integral_deg_s;

    gimbal->roll_i_term_deg =
            GIMBAL_ROLL_KI *
            gimbal->roll_integral_deg_s;


    /* Calculate target commands */
    pitch_target_command_deg =
            (
                gimbal->pitch_p_term_deg +
                gimbal->pitch_i_term_deg
            ) *
            GIMBAL_PITCH_SERVO_SIGN;

    roll_target_command_deg =
            (
                gimbal->roll_p_term_deg +
                gimbal->roll_i_term_deg
            ) *
            GIMBAL_ROLL_SERVO_SIGN;


    /* Clamp target commands */
    pitch_target_command_deg =
            Gimbal_ClampAngle(
                    pitch_target_command_deg
            );

    roll_target_command_deg =
            Gimbal_ClampAngle(
                    roll_target_command_deg
            );


    /* Apply slew rate limits */
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


    /* Update pitch servo */
    status = Servo_SetOffset(
            &gimbal->pitch_servo,
            gimbal->pitch_command_deg
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Update roll servo */
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

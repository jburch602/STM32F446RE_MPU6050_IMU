/*
 * gimbal.c
 *
 * Created on: Sep 6, 2026
 * Author: Jackson
 */

#include "gimbal.h"


/*
 * Limits a requested gimbal angle to the
 * configured mechanical range.
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
 * Initializes both gimbal servos, applies their
 * calibrated ranges, and starts PWM at mechanical center.
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


    /* Initialize pitch servo object */
    status = Servo_Init(
            &gimbal->pitch_servo,
            pitch_htim,
            pitch_channel
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Apply calibrated pitch servo range */
    gimbal->pitch_servo.pulse_min_us =
            GIMBAL_PITCH_MIN_US;

    gimbal->pitch_servo.pulse_center_us =
            GIMBAL_PITCH_CENTER_US;

    gimbal->pitch_servo.pulse_max_us =
            GIMBAL_PITCH_MAX_US;


    /* Initialize roll servo object */
    status = Servo_Init(
            &gimbal->roll_servo,
            roll_htim,
            roll_channel
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Apply calibrated roll servo range */
    gimbal->roll_servo.pulse_min_us =
            GIMBAL_ROLL_MIN_US;

    gimbal->roll_servo.pulse_center_us =
            GIMBAL_ROLL_CENTER_US;

    gimbal->roll_servo.pulse_max_us =
            GIMBAL_ROLL_MAX_US;


    /*
     * Start PWM at the actual calibrated centers.
     * This prevents movement to the generic servo center first.
     */
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


    return HAL_OK;
}


/*
 * Moves both gimbal axes to their calibrated
 * mechanical center positions.
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


    return HAL_OK;
}


/*
 * Updates both gimbal axes using the filtered
 * physical pitch and roll angles.
 *
 * Current controller:
 *   Proportional control
 *   Deadband around level
 *   +/-30 degree mechanical limit
 */
/*
 * Updates both gimbal axes using the filtered
 * physical pitch and roll angles.
 *
 * Current controller:
 *   Proportional control
 *   Deadband around level
 *   +/-30 degree mechanical limit
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


    /*
     * Control error = target - measured.
     *
     * Target orientation is level at 0 degrees.
     */
    gimbal->pitch_error_deg =
            0.0f - angles->pitch;

    gimbal->roll_error_deg =
            0.0f - angles->roll;


    /* Apply pitch deadband */
    if (gimbal->pitch_error_deg > -GIMBAL_PITCH_DEADBAND_DEG &&
        gimbal->pitch_error_deg <  GIMBAL_PITCH_DEADBAND_DEG)
    {
        gimbal->pitch_error_deg = 0.0f;
    }


    /* Apply roll deadband */
    if (gimbal->roll_error_deg > -GIMBAL_ROLL_DEADBAND_DEG &&
        gimbal->roll_error_deg <  GIMBAL_ROLL_DEADBAND_DEG)
    {
        gimbal->roll_error_deg = 0.0f;
    }


    /*
     * Calculate proportional pitch correction.
     * Servo sign accounts for mechanical mounting direction.
     */
    gimbal->pitch_command_deg =
            gimbal->pitch_error_deg *
            GIMBAL_PITCH_KP *
            GIMBAL_PITCH_SERVO_SIGN;


    /*
     * Calculate proportional roll correction.
     */
    gimbal->roll_command_deg =
            gimbal->roll_error_deg *
            GIMBAL_ROLL_KP *
            GIMBAL_ROLL_SERVO_SIGN;


    /* Limit movement to safe mechanical range */
    gimbal->pitch_command_deg =
            Gimbal_ClampAngle(
                    gimbal->pitch_command_deg
            );

    gimbal->roll_command_deg =
            Gimbal_ClampAngle(
                    gimbal->roll_command_deg
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

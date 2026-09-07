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
 * Initializes the pitch and roll servos with
 * their calibrated pulse ranges.
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

    /* Apply calibrated pitch servo range */
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

    /* Apply calibrated roll servo range */
    gimbal->roll_servo.pulse_min_us =
            GIMBAL_ROLL_MIN_US;

    gimbal->roll_servo.pulse_center_us =
            GIMBAL_ROLL_CENTER_US;

    gimbal->roll_servo.pulse_max_us =
            GIMBAL_ROLL_MAX_US;


    /* Start both axes at calibrated center */
    return Gimbal_Center(gimbal);
}


/*
 * Moves both gimbal axes to their calibrated
 * center positions.
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
 * The current controller is proportional only.
 * Servo movement is limited to +/-30 degrees.
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

    float pitch_offset;
    float roll_offset;


    /*
     * Calculate pitch correction.
     *
     * The negative angle opposes platform movement.
     * The servo sign accounts for physical mounting direction.
     */
    pitch_offset =
            -angles->pitch *
            GIMBAL_PITCH_KP *
            GIMBAL_PITCH_SERVO_SIGN;


    /*
     * Calculate roll correction.
     */
    roll_offset =
            -angles->roll *
            GIMBAL_ROLL_KP *
            GIMBAL_ROLL_SERVO_SIGN;


    /* Limit movement to safe mechanical range */
    pitch_offset = Gimbal_ClampAngle(pitch_offset);
    roll_offset = Gimbal_ClampAngle(roll_offset);


    /* Update pitch servo */
    status = Servo_SetOffset(
            &gimbal->pitch_servo,
            pitch_offset
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Update roll servo */
    status = Servo_SetOffset(
            &gimbal->roll_servo,
            roll_offset
    );

    if (status != HAL_OK)
    {
        return status;
    }


    return HAL_OK;
}

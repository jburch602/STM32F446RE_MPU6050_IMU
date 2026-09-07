/*
 * servo.c
 *
 * Created on: Sep 5, 2026
 * Author: Jackson
 */

#include "servo.h"


/*
 * Initializes a servo object and starts PWM output
 * at the default center pulse.
 */
HAL_StatusTypeDef Servo_Init(
        Servo_t *servo,
        TIM_HandleTypeDef *htim,
        uint32_t channel)
{
    if (servo == NULL || htim == NULL)
    {
        return HAL_ERROR;
    }

    servo->htim = htim;
    servo->channel = channel;

    servo->pulse_min_us = SERVO_DEFAULT_MIN_US;
    servo->pulse_center_us = SERVO_DEFAULT_CENTER_US;
    servo->pulse_max_us = SERVO_DEFAULT_MAX_US;

    /* Set initial PWM pulse to servo center */
    __HAL_TIM_SET_COMPARE(
            servo->htim,
            servo->channel,
            servo->pulse_center_us
    );

    return HAL_TIM_PWM_Start(
            servo->htim,
            servo->channel
    );
}


/*
 * Sets the servo PWM pulse width in microseconds.
 * The requested pulse is limited to the configured servo range.
 */
HAL_StatusTypeDef Servo_SetPulse(
        Servo_t *servo,
        uint16_t pulse_us)
{
    if (servo == NULL || servo->htim == NULL)
    {
        return HAL_ERROR;
    }

    /* Clamp pulse to configured servo limits */
    if (pulse_us < servo->pulse_min_us)
    {
        pulse_us = servo->pulse_min_us;
    }

    if (pulse_us > servo->pulse_max_us)
    {
        pulse_us = servo->pulse_max_us;
    }

    __HAL_TIM_SET_COMPARE(
            servo->htim,
            servo->channel,
            pulse_us
    );

    return HAL_OK;
}


/*
 * Sets servo position from 0 to 180 degrees.
 *
 * 0 deg   -> minimum pulse
 * 90 deg  -> center pulse
 * 180 deg -> maximum pulse
 *
 * The two halves are calculated separately so an
 * off-center calibrated pulse can be used correctly.
 */
HAL_StatusTypeDef Servo_SetAngle(
        Servo_t *servo,
        float angle_deg)
{
    if (servo == NULL)
    {
        return HAL_ERROR;
    }

    /* Clamp requested angle to servo range */
    if (angle_deg < 0.0f)
    {
        angle_deg = 0.0f;
    }

    if (angle_deg > 180.0f)
    {
        angle_deg = 180.0f;
    }

    float pulse;

    if (angle_deg <= 90.0f)
    {
        /* Map 0-90 degrees between minimum and center */
        pulse =
                (float)servo->pulse_min_us +
                ((angle_deg / 90.0f) *
                ((float)servo->pulse_center_us -
                 (float)servo->pulse_min_us));
    }
    else
    {
        /* Map 90-180 degrees between center and maximum */
        pulse =
                (float)servo->pulse_center_us +
                (((angle_deg - 90.0f) / 90.0f) *
                ((float)servo->pulse_max_us -
                 (float)servo->pulse_center_us));
    }

    return Servo_SetPulse(
            servo,
            (uint16_t)pulse
    );
}


/*
 * Sets servo position relative to its calibrated center.
 *
 * Example:
 *   0 deg   = center
 *   +20 deg = 20 degrees above center
 *   -20 deg = 20 degrees below center
 *
 * Final limits are enforced by Servo_SetAngle().
 */
HAL_StatusTypeDef Servo_SetOffset(
        Servo_t *servo,
        float offset_deg)
{
    if (servo == NULL)
    {
        return HAL_ERROR;
    }

    return Servo_SetAngle(
            servo,
            90.0f + offset_deg
    );
}

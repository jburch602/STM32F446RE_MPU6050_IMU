/*
 * servo.h
 *
 * Created on: Sep 5, 2026
 * Author: Jackson
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f4xx_hal.h"


/*
 * Default servo pulse calibration.
 *
 * These values are used during Servo_Init().
 * Application-specific calibration can override
 * them after initialization.
 */
#define SERVO_DEFAULT_MIN_US       600U
#define SERVO_DEFAULT_CENTER_US   1500U
#define SERVO_DEFAULT_MAX_US      2400U


/*
 * Servo configuration and hardware interface.
 *
 * Pulse widths are expressed in microseconds.
 */
typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;

    uint16_t pulse_min_us;
    uint16_t pulse_center_us;
    uint16_t pulse_max_us;

} Servo_t;


/*
 * Initializes the servo object and starts PWM
 * at the default center position.
 */
HAL_StatusTypeDef Servo_Init(
        Servo_t *servo,
        TIM_HandleTypeDef *htim,
        uint32_t channel
);


/*
 * Sets servo pulse width directly in microseconds.
 * The pulse is clamped to the configured servo limits.
 */
HAL_StatusTypeDef Servo_SetPulse(
        Servo_t *servo,
        uint16_t pulse_us
);


/*
 * Sets an absolute servo position from 0 to 180 degrees.
 *
 * 0 deg   = minimum pulse
 * 90 deg  = center pulse
 * 180 deg = maximum pulse
 */
HAL_StatusTypeDef Servo_SetAngle(
        Servo_t *servo,
        float angle_deg
);


/*
 * Sets servo position relative to its calibrated center.
 *
 * 0 deg = center position
 * Positive and negative values move on either side of center.
 */
HAL_StatusTypeDef Servo_SetOffset(
        Servo_t *servo,
        float offset_deg
);


#endif /* INC_SERVO_H_ */

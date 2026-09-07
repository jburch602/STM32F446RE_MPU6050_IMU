/*
 * gimbal.h
 *
 * Created on: Sep 6, 2026
 * Author: Jackson
 */

#ifndef INC_GIMBAL_H_
#define INC_GIMBAL_H_

#include "stm32f4xx_hal.h"
#include "servo.h"
#include "imu_filter.h"


/*
 * ---------------------------------------------------------
 * Gimbal servo calibration
 * ---------------------------------------------------------
 *
 * These values were determined experimentally for the
 * servos installed in the gimbal.
 */

/* Pitch servo - TIM8_CH2 */
#define GIMBAL_PITCH_MIN_US        500U
#define GIMBAL_PITCH_CENTER_US    1530U
#define GIMBAL_PITCH_MAX_US       2500U

/* Roll servo - TIM4_CH1 */
#define GIMBAL_ROLL_MIN_US         500U
#define GIMBAL_ROLL_CENTER_US     1540U
#define GIMBAL_ROLL_MAX_US        2500U


/*
 * ---------------------------------------------------------
 * Servo direction
 * ---------------------------------------------------------
 *
 * -1.0f reverses the servo correction direction.
 *
 * Both axes were experimentally determined to require
 * reversed correction for the current mechanical mounting.
 */
#define GIMBAL_PITCH_SERVO_SIGN   -1.0f
#define GIMBAL_ROLL_SERVO_SIGN    -1.0f


/*
 * ---------------------------------------------------------
 * Gimbal movement limits
 * ---------------------------------------------------------
 *
 * Maximum servo movement from calibrated center.
 * This protects the current mechanical assembly while
 * the controller is being developed and tested.
 */
#define GIMBAL_MAX_ANGLE_DEG       60.0f


/*
 * ---------------------------------------------------------
 * Controller configuration
 * ---------------------------------------------------------
 *
 * Initial proportional gains.
 * Pitch and roll are kept separate because the two axes
 * have different mechanical loads and geometry.
 */
#define GIMBAL_PITCH_KP             1.0f
#define GIMBAL_ROLL_KP              1.0f
//Defines for deadbands
#define GIMBAL_PITCH_DEADBAND_DEG 0.5f
#define GIMBAL_ROLL_DEADBAND_DEG 0.5f
//Defines for slew rate control
#define GIMBAL_PITCH_MAX_RATE_DPS   120.0f
#define GIMBAL_ROLL_MAX_RATE_DPS    120.0f

/*
 * Gimbal object containing both servo axes.
 */
typedef struct
{
    Servo_t pitch_servo;
    Servo_t roll_servo;

    float pitch_error_deg;
    float roll_error_deg;

    float pitch_target_command_deg;
    float roll_target_command_deg;

    float pitch_command_deg;
    float roll_command_deg;

} Gimbal_t;


/*
 * Initializes both gimbal servos using their calibrated
 * pulse ranges and starts them at center.
 */
HAL_StatusTypeDef Gimbal_Init(
        Gimbal_t *gimbal,
        TIM_HandleTypeDef *pitch_htim,
        uint32_t pitch_channel,
        TIM_HandleTypeDef *roll_htim,
        uint32_t roll_channel
);


/*
 * Moves both gimbal axes to their calibrated
 * center positions.
 */
HAL_StatusTypeDef Gimbal_Center(
        Gimbal_t *gimbal
);


/*
 * Updates pitch and roll servo positions using
 * the current filtered IMU angles.
 */
HAL_StatusTypeDef Gimbal_Update(
        Gimbal_t *gimbal,
        const IMU_Angles_t *angles
);


#endif /* INC_GIMBAL_H_ */

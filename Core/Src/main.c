/* USER CODE BEGIN Header */

/**
 ******************************************************************************
 * @file    main.c
 * @brief   Two-axis closed-loop gimbal application.
 *
 * Hardware:
 *   - STM32F446RE
 *   - MPU6050 six-axis IMU
 *   - Two MG90S servos
 *
 * Data path:
 *   MPU6050 -> calibrated sensor data -> adaptive complementary filter
 *           -> absolute pitch/roll -> startup-relative pitch/roll
 *           -> gimbal controller -> servo commands
 *
 * The attitude measured at startup is captured as the zero reference for
 * closed-loop stabilization. The controller therefore operates on:
 *
 *   relative attitude = current attitude - startup attitude
 *
 * With a zero-degree setpoint, the resulting control error is equivalent to:
 *
 *   error = startup attitude - current attitude
 *
 * Validated sensor-axis mapping for this installation:
 *   Pitch angle: atan2(AX, -AZ)          Pitch rate: +GY
 *   Roll angle:  atan2(AY, sqrt(AX^2 + AZ^2))  Roll rate: -GX
 *
 * TIM6 schedules the control loop at approximately 100 Hz. Telemetry is
 * decimated to approximately 20 Hz so UART logging does not run every control
 * iteration.
 *
 * Controller tuning and servo-specific behavior are implemented in the gimbal
 * and servo modules; main.c owns system startup, reference capture, scheduling,
 * estimator updates, and telemetry.
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "imu_filter.h"
#include "i2c_manager.h"
#include "gimbal.h"
#include "servo.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STARTUP_REFERENCE_SAMPLES        200U
#define STARTUP_REFERENCE_DELAY_MS         2U
/* Log one sample for every five ~100 Hz control updates (~20 Hz). */
#define TELEMETRY_DECIMATION_TICKS         5U
#define MAIN_PI                     3.1415927f
#define MAIN_RAD_TO_DEG             (180.0f / MAIN_PI)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Control-loop scheduler tick incremented by the TIM6 ISR. */
static volatile uint32_t control_tick_count = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Main_SendText(
        const char *text
);

static float Main_Wrap180(
        float angle_deg
);

static HAL_StatusTypeDef Main_CaptureStartupReference(
        MPU6050_Data_t *mpu,
        const MPU6050_Bias_t *bias,
        float *reference_pitch_deg,
        float *reference_roll_deg,
        float *reference_magnitude_g
);

static void Main_SendTelemetry(
        uint32_t time_ms,
        const MPU6050_Data_t *mpu,
        const IMU_Angles_t *angles,
        const IMU_Angles_t *control_angles,
        const Gimbal_t *gimbal
);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* Blocking USART2 helper used for startup/status messages. */
static void Main_SendText(
        const char *text)
{
    if (text == NULL)
    {
        return;
    }
    HAL_UART_Transmit(
            &huart2,
            (uint8_t *)text,
            (uint16_t)strlen(text),
            HAL_MAX_DELAY
    );
}
/* Normalize an angle to the controller's [-180, +180] degree range. */
static float Main_Wrap180(
        float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}
/*
 * Average the stationary accelerometer vector at startup and convert it to
 * pitch/roll. These angles become the closed-loop zero reference.
 */
static HAL_StatusTypeDef Main_CaptureStartupReference(
        MPU6050_Data_t *mpu,
        const MPU6050_Bias_t *bias,
        float *reference_pitch_deg,
        float *reference_roll_deg,
        float *reference_magnitude_g)
{
    if (mpu == NULL ||
        bias == NULL ||
        reference_pitch_deg == NULL ||
        reference_roll_deg == NULL ||
        reference_magnitude_g == NULL)
    {
        return HAL_ERROR;
    }
    float sum_ax = 0.0f;
    float sum_ay = 0.0f;
    float sum_az = 0.0f;
    uint32_t valid_samples = 0U;
    for (uint32_t i = 0U;
         i < STARTUP_REFERENCE_SAMPLES;
         i++)
    {
        HAL_StatusTypeDef status =
                MPU6050_Read_All(
                        &hi2c1,
                        mpu,
                        bias
                );
        if (status == HAL_OK)
        {
            sum_ax +=
                    mpu->accel_x_g;
            sum_ay +=
                    mpu->accel_y_g;
            sum_az +=
                    mpu->accel_z_g;
            valid_samples++;
        }
        HAL_Delay(
                STARTUP_REFERENCE_DELAY_MS
        );
    }
    if (valid_samples == 0U)
    {
        return HAL_ERROR;
    }
    float reference_ax =
            sum_ax /
            (float)valid_samples;
    float reference_ay =
            sum_ay /
            (float)valid_samples;
    float reference_az =
            sum_az /
            (float)valid_samples;
    /*
     * Calibrated stationary acceleration should have magnitude near 1 g.
     */
    *reference_magnitude_g =
            sqrtf(
                reference_ax *
                reference_ax
                +
                reference_ay *
                reference_ay
                +
                reference_az *
                reference_az
            );
    /*
     * Validated pitch mapping for the upside-down MPU mounting.
     */
    *reference_pitch_deg =
            atan2f(
                reference_ax,
                -reference_az
            )
            *
            MAIN_RAD_TO_DEG;
    /*
     * Validated roll mapping.
     */
    *reference_roll_deg =
            atan2f(
                reference_ay,
                sqrtf(
                    reference_ax *
                    reference_ax
                    +
                    reference_az *
                    reference_az
                )
            )
            *
            MAIN_RAD_TO_DEG;
    return HAL_OK;
}
/*
 * Emit one CSV telemetry row. This function is called at the decimated
 * telemetry rate rather than on every control iteration.
 */
static void Main_SendTelemetry(
        uint32_t time_ms,
        const MPU6050_Data_t *mpu,
        const IMU_Angles_t *angles,
        const IMU_Angles_t *control_angles,
        const Gimbal_t *gimbal)
{
    if (mpu == NULL ||
        angles == NULL ||
        control_angles == NULL ||
        gimbal == NULL)
    {
        return;
    }
    char buffer[420];
    int length = snprintf(
            buffer,
            sizeof(buffer),
            "%lu,"
            "%.4f,"
            "%.3f,"
            "%.3f,"
            "%.3f,"
            "%.3f,"
            "%.2f,"
            "%.2f,"
            "%.3f,"
            "%.2f,"
            "%.3f,"
            "%.4f,"
            "%.3f,"
            "%.3f,"
            "%.3f,"
            "%.3f,"
            "%.3f,"
            "%.3f,"
            "%.3f,"
            "%.3f,"
            "%.3f,"
            "%.3f\r\n",
            (unsigned long)time_ms,
            angles->dt,
            /* Absolute attitude */
            angles->pitch,
            angles->roll,
            /* Startup-relative attitude */
            control_angles->pitch,
            control_angles->roll,
            /* Physical angular rate */
            mpu->gyro_y_dps,
            -mpu->gyro_x_dps,
            /* Adaptive-filter diagnostics */
            angles->accel_magnitude_g,
            angles->gyro_magnitude_dps,
            angles->accel_trust,
            angles->complementary_alpha,
            /* Controller error */
            gimbal->pitch_error_deg,
            gimbal->roll_error_deg,
            /* P terms */
            gimbal->pitch_p_term_deg,
            gimbal->roll_p_term_deg,
            /* I terms */
            gimbal->pitch_i_term_deg,
            gimbal->roll_i_term_deg,
            /* Direct gyro-rate damping */
            gimbal->pitch_rate_term_deg,
            gimbal->roll_rate_term_deg,
            /* Final servo-offset commands */
            gimbal->pitch_command_deg,
            gimbal->roll_command_deg
    );
    if (length <= 0)
    {
        return;
    }
    if (length >= (int)sizeof(buffer))
    {
        length =
                (int)sizeof(buffer) - 1;
    }
    HAL_UART_Transmit(
            &huart2,
            (uint8_t *)buffer,
            (uint16_t)length,
            HAL_MAX_DELAY
    );
}
/* USER CODE END 0 */

/**
 * @brief Application entry point.
 */

int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */
    HAL_Init();
    /* USER CODE BEGIN Init */
    /* USER CODE END Init */
    SystemClock_Config();
    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */
    /* Initialize board peripherals generated by STM32CubeMX. */
    MX_GPIO_Init();
    /*
     * Recover a potentially stuck I2C bus before enabling I2C1. This handles
     * the observed case where rapid resets can leave the MPU6050/bus busy.
     */
    I2C_Manager_RecoverI2C1Bus();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM8_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    /* USER CODE BEGIN 2 */
    MPU6050_Data_t mpu =
            {0};
    MPU6050_Bias_t bias =
            {0};
    /* Gravity-referenced absolute attitude produced by the IMU filter. */
    IMU_Angles_t angles =
            {0};
    /* Startup-relative attitude presented to the zero-setpoint controller. */
    IMU_Angles_t control_angles =
            {0};
    Gimbal_t gimbal =
            {0};
    HAL_StatusTypeDef status;
    /* --- Gimbal initialization ----------------------------------------- */
    status = Gimbal_Init(
            &gimbal,
            &htim8,
            TIM_CHANNEL_2,
            &htim4,
            TIM_CHANNEL_1
    );
    if (status != HAL_OK)
    {
        Main_SendText(
            "# ERROR: Gimbal initialization failed\r\n"
        );
        Error_Handler();
    }
    /* Center both servos before sensor calibration/reference capture. */
    status = Servo_SetOffset(
            &gimbal.pitch_servo,
            0.0f
    );
    if (status != HAL_OK)
    {
        Error_Handler();
    }
    status = Servo_SetOffset(
            &gimbal.roll_servo,
            0.0f
    );
    if (status != HAL_OK)
    {
        Error_Handler();
    }
    /* Let the mechanism settle before calibrating the stationary IMU. */
    HAL_Delay(1000U);
    /*
     * --- MPU6050 initialization -----------------------------------------
     * Driver configuration: accelerometer +/-2 g, gyroscope +/-1000 deg/s.
     */
    status = MPU6050_Init(
            &hi2c1
    );
    if (status != HAL_OK)
    {
        Main_SendText(
            "# ERROR: MPU6050 initialization failed\r\n"
        );
        Error_Handler();
    }
    Main_SendText(
        "# MPU6050 initialized\r\n"
    );
    /*
     * --- Gyroscope zero-rate calibration --------------------------------
     * The mechanism must remain stationary, but it does not need to be level.
     */
    Main_SendText(
        "# Calibrating gyro - KEEP GIMBAL STATIONARY\r\n"
    );
    status = MPU6050_Calibrate_Gyro(
            &hi2c1,
            &bias
    );
    if (status != HAL_OK)
    {
        Main_SendText(
            "# ERROR: gyro calibration failed\r\n"
        );
        Error_Handler();
    }
    /* Report measured raw gyro bias for startup diagnostics. */
    {
        char message[160];
        snprintf(
                message,
                sizeof(message),
                "# GYRO_BIAS_RAW,"
                "GX=%d,"
                "GY=%d,"
                "GZ=%d\r\n",
                (int)bias.gyro_x_bias,
                (int)bias.gyro_y_bias,
                (int)bias.gyro_z_bias
        );
        Main_SendText(
            message
        );
    }
    /* Short settling delay before capturing the attitude reference. */
    HAL_Delay(500U);
    /* --- Capture startup attitude reference --------------------------- */
    float reference_pitch_deg =
            0.0f;
    float reference_roll_deg =
            0.0f;
    float reference_magnitude_g =
            0.0f;
    status = Main_CaptureStartupReference(
            &mpu,
            &bias,
            &reference_pitch_deg,
            &reference_roll_deg,
            &reference_magnitude_g
    );
    if (status != HAL_OK)
    {
        Main_SendText(
            "# ERROR: startup attitude capture failed\r\n"
        );
        Error_Handler();
    }
    /*
     * --- Initialize absolute estimator ----------------------------------
     * Seed the filter from the measured gravity attitude to avoid an
     * artificial convergence transient at startup.
     */
    angles.pitch =
            reference_pitch_deg;
    angles.roll =
            reference_roll_deg;
    angles.yaw =
            0.0f;
    angles.current_time_ms =
            0U;
    angles.previous_time_ms =
            0U;
    angles.dt =
            0.0f;
    angles.accel_magnitude_g =
            reference_magnitude_g;
    angles.gyro_magnitude_dps =
            0.0f;
    angles.accel_magnitude_trust =
            1.0f;
    angles.gyro_rate_trust =
            1.0f;
    angles.accel_trust =
            1.0f;
    angles.complementary_alpha =
            0.98f;
    /* Initialize the controller-facing attitude at the zero reference. */
    control_angles =
            angles;
    control_angles.pitch =
            0.0f;
    control_angles.roll =
            0.0f;
    control_angles.yaw =
            0.0f;
    /* Report the captured reference before closed-loop operation begins. */
    {
        char message[260];
        snprintf(
                message,
                sizeof(message),
                "# STARTUP_REFERENCE\r\n"
                "# Pitch=%.3f deg\r\n"
                "# Roll=%.3f deg\r\n"
                "# AccelMag=%.5f g\r\n"
                "# Startup attitude is now control target 0,0\r\n",
                reference_pitch_deg,
                reference_roll_deg,
                reference_magnitude_g
        );
        Main_SendText(
            message
        );
    }
    /* --- Telemetry ----------------------------------------------------- */
    Main_SendText(
        "# TWO-AXIS CLOSED-LOOP STABILIZATION ARMED\r\n"
        "# RelativePitch/RelativeRoll are the values sent to Gimbal_Update\r\n"
        "# PitchRate = +GY\r\n"
        "# RollRate = -GX\r\n"
        "\r\n"
        "Time_ms,"
        "dt,"
        "AbsPitch_deg,"
        "AbsRoll_deg,"
        "RelativePitch_deg,"
        "RelativeRoll_deg,"
        "PitchRate_dps,"
        "RollRate_dps,"
        "AccelMag_g,"
        "GyroMag_dps,"
        "AccelTrust,"
        "Alpha,"
        "PitchError_deg,"
        "RollError_deg,"
        "PitchP_deg,"
        "RollP_deg,"
        "PitchI_deg,"
        "RollI_deg,"
        "PitchRateTerm_deg,"
        "RollRateTerm_deg,"
        "PitchCommand_deg,"
        "RollCommand_deg\r\n"
    );
    /* --- Start ~100 Hz control scheduler ------------------------------ */
    control_tick_count =
            0U;
    uint32_t last_control_tick =
            0U;
    uint32_t telemetry_counter =
            0U;
    uint32_t system_start_ms =
            HAL_GetTick();
    status = HAL_TIM_Base_Start_IT(
            &htim6
    );
    if (status != HAL_OK)
    {
        Main_SendText(
            "# ERROR: TIM6 failed to start\r\n"
        );
        Error_Handler();
    }
    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* Run one control iteration for each newly observed TIM6 tick. */
        uint32_t tick_snapshot =
                control_tick_count;
        if (tick_snapshot !=
            last_control_tick)
        {
            last_control_tick =
                    tick_snapshot;
            /* Read one calibrated IMU sample. */
            status = MPU6050_Read_All(
                    &hi2c1,
                    &mpu,
                    &bias
            );
            if (status != HAL_OK)
            {
                continue;
            }
            /* Update estimator timing from the current sample interval. */
            status = IMU_Update_dt(
                    &angles
            );
            if (status != HAL_OK)
            {
                continue;
            }
            /* The first estimator update may not yet contain a valid dt. */
            if (angles.dt <= 0.0f)
            {
                continue;
            }
            /*
             * Update absolute pitch/roll with the adaptive complementary
             * filter. The filter increases gyro weighting during motion and
             * restores accelerometer correction as the system becomes quiet.
             */
            status = IMU_Calculate_Angles(
                    &mpu,
                    &angles
            );
            if (status != HAL_OK)
            {
                continue;
            }
            /*
             * Convert gravity-referenced attitude into the startup-relative
             * frame expected by the zero-setpoint gimbal controller:
             *
             *   relative = current - startup
             *   error    = 0 - relative = startup - current
             */
            control_angles =
                    angles;
            control_angles.pitch =
                    Main_Wrap180(
                        angles.pitch -
                        reference_pitch_deg
                    );
            control_angles.roll =
                    Main_Wrap180(
                        angles.roll -
                        reference_roll_deg
                    );
            /* The current mechanism stabilizes pitch and roll only. */
            control_angles.yaw =
                    0.0f;
            /*
             * Execute the gimbal controller using startup-relative attitude
             * and calibrated gyro data. Controller details remain encapsulated
             * in the gimbal module so main.c does not duplicate tuning logic.
             */
            status = Gimbal_Update(
                    &gimbal,
                    &control_angles,
                    &mpu
            );
            if (status != HAL_OK)
            {
                Main_SendText(
                    "# ERROR: Gimbal_Update failed\r\n"
                );
                Error_Handler();
            }
            /* Publish telemetry at ~20 Hz instead of every control cycle. */
            telemetry_counter++;
            if (telemetry_counter >=
                TELEMETRY_DECIMATION_TICKS)
            {
                telemetry_counter =
                        0U;
                uint32_t elapsed_ms =
                        HAL_GetTick() -
                        system_start_ms;
                Main_SendTelemetry(
                        elapsed_ms,
                        &mpu,
                        &angles,
                        &control_angles,
                        &gimbal
                );
            }
        }
    }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */
}
/* TIM6 ISR callback: advance the scheduler only; control runs in main(). */
void HAL_TIM_PeriodElapsedCallback(
        TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        control_tick_count++;
    }
}

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct =
            {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
            {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(
            PWR_REGULATOR_VOLTAGE_SCALE3
    );
    RCC_OscInitStruct.OscillatorType =
            RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState =
            RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue =
            RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState =
            RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource =
            RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM =
            16;
    RCC_OscInitStruct.PLL.PLLN =
            336;
    RCC_OscInitStruct.PLL.PLLP =
            RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ =
            2;
    RCC_OscInitStruct.PLL.PLLR =
            2;
    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct
        ) != HAL_OK)
    {
        Error_Handler();
    }
    RCC_ClkInitStruct.ClockType =
            RCC_CLOCKTYPE_HCLK |
            RCC_CLOCKTYPE_SYSCLK |
            RCC_CLOCKTYPE_PCLK1 |
            RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource =
            RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider =
            RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider =
            RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider =
            RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2
        ) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief Error handler.
 */

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(
        uint8_t *file,
        uint32_t line)
{
}
#endif

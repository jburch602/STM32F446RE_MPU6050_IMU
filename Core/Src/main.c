/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * Programmed by Jackson Burch
 *
 * STM32F446RE + MPU6050 IMU
 *
 * TIM8_CH2 = Pitch Servo
 *   Center = 1530 us
 *
 * TIM4_CH1 = Roll Servo
 *   Center = 1540 us
 *
 * Physical Pitch:
 *   Accelerometer = +AX
 *   Gyroscope     = +GY
 *
 * Physical Roll:
 *   Accelerometer = +AY
 *   Gyroscope     = -GX
 ******************************************************************************
 */
/* USER CODE END Header */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mpu6050.h"
#include "imu_filter.h"
#include "telemetry.h"
#include "system_health.h"
#include "i2c_manager.h"
#include "servo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PITCH_CENTER_US   1530U
#define ROLL_CENTER_US    1540U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */


    /* MCU Configuration--------------------------------------------------------*/

    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */


    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    /* Recover I2C bus before initializing I2C peripheral */
    I2C_Manager_RecoverI2C1Bus();

    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_TIM8_Init();
    MX_TIM4_Init();


    /* USER CODE BEGIN 2 */

    MPU6050_Data_t mpu = {0};
    MPU6050_Bias_t bias = {0};
    IMU_Angles_t angles = {0};

    Servo_t pitch_servo = {0};
    Servo_t roll_servo = {0};

    HAL_StatusTypeDef wake_status;
    HAL_StatusTypeDef mpu_status;
    HAL_StatusTypeDef imu_status;
    HAL_StatusTypeDef cal_status;
    HAL_StatusTypeDef tel_status;
    HAL_StatusTypeDef pitch_servo_status;
    HAL_StatusTypeDef roll_servo_status;

    System_Health_t health = {0};

    SystemHealth_Init(&health);

    Telemetry_Send_Status(
            &huart2,
            "===== BOOT START =====",
            HAL_OK,
            &health
    );


    /* Initialize MPU6050 */
    wake_status = MPU6050_Init(&hi2c1);

    if (wake_status != HAL_OK)
    {
        SystemHealth_SetState(&health, SYS_FAULT);

        Telemetry_Send_Status(
                &huart2,
                "The MPU failed to initialize",
                wake_status,
                &health
        );

        Error_Handler();
    }


    /* Calibrate MPU6050 */
    SystemHealth_SetState(&health, SYS_CALIBRATING);

    Telemetry_Send_Status(
            &huart2,
            "The system is calibrating, keep it still",
            HAL_OK,
            &health
    );

    cal_status = MPU6050_Calibrate_All(
            &hi2c1,
            &bias
    );

    if (cal_status != HAL_OK)
    {
        SystemHealth_SetState(&health, SYS_FAULT);

        Telemetry_Send_Status(
                &huart2,
                "The calibration failed",
                cal_status,
                &health
        );

        Error_Handler();
    }


    SystemHealth_SetState(&health, SYS_RUNNING);

    Telemetry_Send_Status(
            &huart2,
            "Calibration complete",
            HAL_OK,
            &health
    );


    /* Send csv header */
    tel_status = Telemetry_Send_Header(&huart2);

    if (tel_status != HAL_OK)
    {
        SystemHealth_SetState(&health, SYS_FAULT);

        Telemetry_Send_Status(
                &huart2,
                "The csv header failed to send",
                tel_status,
                &health
        );

        Error_Handler();
    }


    /* Initialize pitch servo */
    pitch_servo_status = Servo_Init(
            &pitch_servo,
            &htim8,
            TIM_CHANNEL_2
    );

    if (pitch_servo_status != HAL_OK)
    {
        Error_Handler();
    }


    /* Initialize roll servo */
    roll_servo_status = Servo_Init(
            &roll_servo,
            &htim4,
            TIM_CHANNEL_1
    );

    if (roll_servo_status != HAL_OK)
    {
        Error_Handler();
    }


    /* Set servos to calibrated center positions */
    Servo_SetPulse(
            &pitch_servo,
            PITCH_CENTER_US
    );

    Servo_SetPulse(
            &roll_servo,
            ROLL_CENTER_US
    );


    /* USER CODE END 2 */


    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1)
    {
        /* USER CODE END WHILE */


        SystemHealth_RecordSample(&health);


        /* Read MPU data */
        mpu_status = MPU6050_Read_All(
                &hi2c1,
                &mpu,
                &bias
        );


        if (mpu_status == HAL_OK)
        {
            /* Update delta time */
            imu_status = IMU_Update_dt(&angles);


            if (imu_status == HAL_OK)
            {
                /* Calculate pitch, roll, and yaw */
                imu_status = IMU_Calculate_Angles(
                        &mpu,
                        &angles
                );


                if (imu_status == HAL_OK)
                {
                    SystemHealth_RecordValidSample(&health);
                }

                else
                {
                    SystemHealth_RecordAngleError(
                            &health,
                            imu_status
                    );
                }
            }

            else
            {
                SystemHealth_RecordDtError(
                        &health,
                        imu_status
                );
            }
        }

        else
        {
            SystemHealth_RecordReadError(
                    &health,
                    mpu_status
            );
        }


        /* Update system error rates */
        SystemHealth_UpdateErrorRates(&health);


        /* Send data over UART */
        tel_status = Telemetry_Send_CSV(
                &huart2,
                &mpu,
                &angles,
                &health
        );


        if (tel_status != HAL_OK)
        {
            SystemHealth_RecordTelemetryError(
                    &health,
                    tel_status
            );
        }


        HAL_Delay(10);


        /* USER CODE BEGIN 3 */
    }

    /* USER CODE END 3 */
}


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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

    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;

    RCC_OscInitStruct.PLL.PLLP =
            RCC_PLLP_DIV4;

    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;


    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
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
            FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */

    __disable_irq();

    while (1)
    {
    }

    /* USER CODE END Error_Handler_Debug */
}


#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error occurred.
 */
void assert_failed(
        uint8_t *file,
        uint32_t line)
{
    /* USER CODE BEGIN 6 */

    /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */

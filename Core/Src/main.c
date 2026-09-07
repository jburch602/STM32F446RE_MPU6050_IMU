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
#include "dma.h"
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
#include "gimbal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* Count TIM6 control ticks */
static volatile uint32_t control_tick_count = 0U;

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

    MX_DMA_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_TIM8_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();


    /* USER CODE BEGIN 2 */

    MPU6050_Data_t mpu = {0};
    MPU6050_Bias_t bias = {0};
    IMU_Angles_t angles = {0};
    Gimbal_t gimbal = {0};

    HAL_StatusTypeDef wake_status;
    HAL_StatusTypeDef mpu_status;
    HAL_StatusTypeDef imu_status;
    HAL_StatusTypeDef cal_status;
    HAL_StatusTypeDef tel_status;
    HAL_StatusTypeDef gimbal_status;

    System_Health_t health = {0};

    uint32_t last_control_tick = 0U;
    uint32_t missed_control_ticks = 0U;
    uint8_t telemetry_divider = 0U;


    /* Initialize system health */
    SystemHealth_Init(&health);


    /* Send boot status */
    Telemetry_Send_Status(
            &huart2,
            "===== BOOT START =====",
            HAL_OK,
            &health
    );


    /* Initialize gimbal */
    gimbal_status = Gimbal_Init(
            &gimbal,
            &htim8,
            TIM_CHANNEL_2,
            &htim4,
            TIM_CHANNEL_1
    );

    if (gimbal_status != HAL_OK)
    {
        Error_Handler();
    }


    HAL_Delay(1000);


    /* Initialize MPU6050 */
    wake_status = MPU6050_Init(
            &hi2c1
    );

    if (wake_status != HAL_OK)
    {
        SystemHealth_SetState(
                &health,
                SYS_FAULT
        );

        Telemetry_Send_Status(
                &huart2,
                "The MPU failed to initialize",
                wake_status,
                &health
        );

        Error_Handler();
    }


    /* Calibrate MPU6050 */
    SystemHealth_SetState(
            &health,
            SYS_CALIBRATING
    );

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
        SystemHealth_SetState(
                &health,
                SYS_FAULT
        );

        Telemetry_Send_Status(
                &huart2,
                "The calibration failed",
                cal_status,
                &health
        );

        Error_Handler();
    }


    /* Enter running state */
    SystemHealth_SetState(
            &health,
            SYS_RUNNING
    );

    Telemetry_Send_Status(
            &huart2,
            "Calibration complete",
            HAL_OK,
            &health
    );


    /* Send csv header */
    tel_status = Telemetry_Send_Header(
            &huart2
    );

    if (tel_status != HAL_OK)
    {
        SystemHealth_SetState(
                &health,
                SYS_FAULT
        );

        Telemetry_Send_Status(
                &huart2,
                "The csv header failed to send",
                tel_status,
                &health
        );

        Error_Handler();
    }
    /* Start 100 Hz control timer */
    control_tick_count = 0U;

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE END 2 */


    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1)
    {
        /* USER CODE END WHILE */


        /* USER CODE BEGIN 3 */

        uint32_t tick_snapshot =
                control_tick_count;


        /* Run control update on new TIM6 tick */
        if (tick_snapshot != last_control_tick)
        {
            /*
             * Record missed control periods instead
             * of running multiple catch-up updates.
             */
            uint32_t elapsed_ticks =
                    tick_snapshot -
                    last_control_tick;

            if (elapsed_ticks > 1U)
            {
                missed_control_ticks +=
                        elapsed_ticks - 1U;
            }

            last_control_tick =
                    tick_snapshot;


            /* Record system sample */
            SystemHealth_RecordSample(
                    &health
            );


            /* Read MPU data */
            mpu_status = MPU6050_Read_All(
                    &hi2c1,
                    &mpu,
                    &bias
            );


            if (mpu_status == HAL_OK)
            {
                /* Update delta time */
                imu_status = IMU_Update_dt(
                        &angles
                );


                if (imu_status == HAL_OK)
                {
                    /* Calculate pitch, roll, and yaw */
                    imu_status = IMU_Calculate_Angles(
                            &mpu,
                            &angles
                    );


                    if (imu_status == HAL_OK)
                    {
                        /* Update gimbal */
                        gimbal_status = Gimbal_Update(
                                &gimbal,
                                &angles
                        );


                        if (gimbal_status != HAL_OK)
                        {
                            Error_Handler();
                        }


                        SystemHealth_RecordValidSample(
                                &health
                        );
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
            SystemHealth_UpdateErrorRates(
                    &health
            );


            /* Update telemetry divider */
            telemetry_divider++;


            /* Send telemetry every fifth control tick */
            if (telemetry_divider >= 5U)
            {
                telemetry_divider = 0U;


                tel_status = Telemetry_Send_CSV_DMA(
                        &huart2,
                        &mpu,
                        &angles,
                        &gimbal,
                        &health
                );


                /*
                 * HAL_BUSY only means the previous
                 * DMA transfer is still active.
                 */
                if (tel_status != HAL_OK &&
                    tel_status != HAL_BUSY)
                {
                    SystemHealth_RecordTelemetryError(
                            &health,
                            tel_status
                    );
                }
            }
        }
    }

    /* USER CODE END 3 */

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

/*
 * Record TIM6 control tick.
 */
void HAL_TIM_PeriodElapsedCallback(
        TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        control_tick_count++;
    }
}

/*
 * Handle UART DMA transmit completion.
 */
void HAL_UART_TxCpltCallback(
        UART_HandleTypeDef *huart)
{
    Telemetry_UART_TxCpltCallback(
            huart
    );
}


/*
 * Handle UART DMA errors.
 */
void HAL_UART_ErrorCallback(
        UART_HandleTypeDef *huart)
{
    Telemetry_UART_ErrorCallback(
            huart
    );
}

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

void assert_failed(
        uint8_t *file,
        uint32_t line)
{
    /* USER CODE BEGIN 6 */

    /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */

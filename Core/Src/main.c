/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * Programmed by Jackson Burch
  * Real-time IMU visualizer and system monitor
  ******************************************************************************
  * MPU6050 WHO_AM_I
  *
  *
  *
  *
  *
  *
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "imu_filter.h"
#include "telemetry.h"
#include "system_health.h"
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  MPU6050_Data_t mpu = {0}; //Initializes the struct
  MPU6050_Bias_t bias = {0};
  IMU_Angles_t angles = {0};

  HAL_StatusTypeDef wake_status; //status of MPU address/wake
  HAL_StatusTypeDef mpu_status; //status of mpu reads/conversion
  HAL_StatusTypeDef imu_status; //status of IMU timing and angle calculations
  HAL_StatusTypeDef cal_status; //status of calibration
  HAL_StatusTypeDef tel_status; //status of telemetry transmission

  System_Health_t health = {0}; //struct
  SystemHealth_Init(&health); //Initializes system health monitor

  wake_status = MPU6050_Init(&hi2c1); //Call MPU6050_Init and store wake/init status
  if (wake_status != HAL_OK){ //IF mpu_status is not okay
      Error_Handler(); //Send to error handler
  }

  SystemHealth_SetState(&health, SYS_CALIBRATING);
  cal_status = MPU6050_Calibrate_All(&hi2c1, &bias);
  if (cal_status != HAL_OK){ //IF cal_status is not okay
      Error_Handler(); //Send to error handler
  }

  tel_status = Telemetry_Send_Header(&huart2); //Send the header to csv for logging
  if (tel_status != HAL_OK){
      Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* USER CODE END WHILE */

      //Counts one attempted sample cycle
      SystemHealth_RecordSample(&health);

      //Gets raw values from mpu, subtracts bias, and converts to physical units
      mpu_status = MPU6050_Read_All(&hi2c1, &mpu, &bias);

      if (mpu_status == HAL_OK) { //IF mpu is ok

          //Gets delta time through HAL_GetTick()
          imu_status = IMU_Update_dt(&angles);

          if (imu_status == HAL_OK) { //IF imu is ok

              //Calculates pitch, roll, and yaw
              imu_status = IMU_Calculate_Angles(&mpu, &angles);

              if (imu_status == HAL_OK) { //IF imu is ok
                  SystemHealth_RecordValidSample(&health); //Record full valid sample
              }
              else { //ELSE
                  SystemHealth_RecordAngleError(&health, imu_status); //Record angle error, sends the hal code
              }
          }
          else { //ELSE
              SystemHealth_RecordDtError(&health, imu_status); //Record dt error, sends the hal code
          }
      }
      else { //ELSE
          SystemHealth_RecordReadError(&health, mpu_status); //Record read error, sends the hal code
      }

      //Updates calculated error-rate fields
      SystemHealth_UpdateErrorRates(&health);

      //Send data to csv for logging
      tel_status = Telemetry_Send_CSV(&huart2, &mpu, &angles);

      if (tel_status != HAL_OK) { //IF telemetry is not HAL_OK
          SystemHealth_RecordTelemetryError(&health, tel_status); //Record telemetry error, sends the hal code
      }
      //Updates total and read error rates, does not include telemetry errors
      SystemHealth_UpdateErrorRates(&health);

      HAL_Delay(10); //temporary 10ms delay for testing CSV output

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

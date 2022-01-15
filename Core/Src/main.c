/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define MPU6050_ADDRESS_AD0_LOW 0x68
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_GYRO_CONFIG  0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_PWR_MGMT_1   0x6B
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
void PeriphCommonClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
int I2C_WriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer, uint8_t TXBUFFERSIZE);
int I2C_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE);
int MPU6050_Init(void);
void MPU6050_GetAllData(int16_t *Data);
int print_DataToTerminal(int16_t *Data);

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  MPU6050_Init();

  int16_t Data;
  int err;
  char dbg_msg[BUF_SIZE];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    // blinks green
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    HAL_Delay(500);

    MPU6050_GetAllData(&Data);

    err = print_DataToTerminal(&Data);
    if (err != 0) {
    	sprintf(dbg_msg, "Error has occurred: %d\n", err);
    	HAL_UART_Transmit(&huart2, (uint8_t *) dbg_msg, strlen(dbg_msg), HAL_MAX_DELAY);
    	// Error_Handler();
    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    	continue;
    } else {
    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 8;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int I2C_WriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer, uint8_t TXBUFFERSIZE) {
    while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)I2C_ADDRESS<<1, (uint8_t*)aTxBuffer, (uint16_t)TXBUFFERSIZE, (uint32_t)1000)!= HAL_OK){
        if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF){
            // _Error_Handler(__FILE__, aTxBuffer[0]);
        	return 1;
        }

    }

      while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

      return 0;
}

int I2C_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE){

    I2C_WriteBuffer(I2C_ADDRESS, &RegAddr, 1);

    while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)I2C_ADDRESS<<1, aRxBuffer, (uint16_t)RXBUFFERSIZE, (uint32_t)1000) != HAL_OK){
        if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF){
            // _Error_Handler(__FILE__, __LINE__);
        	return 1;
        }
    }

    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

    return 0;
}

int MPU6050_Init(void){

    uint8_t buffer[7];
    int res = 0;

    buffer[0] = MPU6050_RA_PWR_MGMT_1;
    buffer[1] = 0x00;
    res = I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);
    if (res != 0) return res;

    buffer[0] = MPU6050_RA_GYRO_CONFIG;
    buffer[1] = 0x8;
    res = I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);
    if (res != 0) return res;

    buffer[0] = MPU6050_RA_ACCEL_CONFIG;
    buffer[1] = 0x10;
    res = I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);

    return res;
}


void MPU6050_GetAllData(int16_t *Data){

  uint8_t accelbuffer[14];

  I2C_ReadBuffer(MPU6050_ADDRESS_AD0_LOW,MPU6050_RA_ACCEL_XOUT_H, accelbuffer, 14);

  /* Registers 59 to 64 – Accelerometer Measurements */
  for (int i = 0; i< 3; i++)
      Data[i] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);

  /* Registers 65 and 66 – Temperature Measurement */

  /* Registers 67 to 72 – Gyroscope Measurements */
  for (int i = 4; i < 7; i++)
      Data[i - 1] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);

}


int print_DataToTerminal(int16_t *Data) {
	char msg[BUF_SIZE];
	sprintf(msg, "A: %d, %d, %d; G: %d, %d, %d\n",
			Data[0], Data[1], Data[2],
			Data[3], Data[4], Data[5]);

	// remembet that this works as sizeof char == 1
	int err = HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
	if (err != HAL_OK) return err;

	return 0;
}


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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

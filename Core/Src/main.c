/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//	address of registers with auto-increment for multi-bytes

#define HTS221_ADDR 0xBE
#define HTS221_ADDR_CTRL1 0x20
#define HTS221_ADDR_T_OUT (0x2A | 0x80)
#define HTS221_ADDR_T0T1_OUT (0x3C | 0x80)
#define HTS221_ADDR_T0T1_degC_x8_LSB (0x32 | 0x80)
#define HTS221_ADDR_T0T1_MSB 0x35

#define HTS221_ADDR_H0H1_RH_X2 (0x30 | 0x80)
#define HTS221_ADDR_H_OUT (0x28 | 0x80)
#define HTS221_ADDR_H0_T0_OUT (0x36 | 0x80)
#define HTS221_ADDR_H1_T0_OUT (0x3A | 0x80)
//---------------------------------State----------------------------------------
#define POWER_DOWN 0x00
#define POWER_UP_CTRL1_1HZ 0x85

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int16_t T0_degC, T1_degC, T1_OUT, T0_OUT;
int16_t H0_rh, H1_rh, H0_T0_out, H1_T0_out;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef HTS221_Read_Reg(uint8_t reg_addr, uint16_t size, uint8_t *data) {
	HAL_StatusTypeDef res;

	res = HAL_I2C_Master_Transmit(&hi2c1, HTS221_ADDR, &reg_addr, 1, HAL_MAX_DELAY);
	if (res != HAL_OK) return res;

	res = HAL_I2C_Master_Receive(&hi2c1, HTS221_ADDR, data, size, HAL_MAX_DELAY);
	if (res != HAL_OK) return res;

	return res;
}

HAL_StatusTypeDef HTS221_Write_Reg(uint8_t reg_addr, uint8_t data) {
	HAL_StatusTypeDef res;

	uint8_t buf[2] = {reg_addr, data};

	res = HAL_I2C_Master_Transmit(&hi2c1, HTS221_ADDR, buf, 2, HAL_MAX_DELAY);
	if (res != HAL_OK) return res;

	return res;
}

HAL_StatusTypeDef HTS221_Read_Temp_Coef(void) {
	HAL_StatusTypeDef res;
	uint8_t buf[4], T1T0_msb;
	int16_t T0_degC_x8, T1_degC_x8;

	//	reading fabric coefficient

	res = HTS221_Read_Reg(HTS221_ADDR_T0T1_degC_x8_LSB, 2, buf);
	if (res != HAL_OK) return res;

	res = HTS221_Read_Reg(HTS221_ADDR_T0T1_MSB, 1, &T1T0_msb);
	if (res != HAL_OK) return res;

	T0_degC_x8 = (((uint16_t)T1T0_msb & 0x03) << 8) | ((uint16_t)buf[0]);
	T1_degC_x8 = (((uint16_t)T1T0_msb & 0x0C) << 6) | ((uint16_t)buf[1]);

	T0_degC = T0_degC_x8 >> 3;
	T1_degC = T1_degC_x8 >> 3;

	res = HTS221_Read_Reg(HTS221_ADDR_T0T1_OUT, 4, buf);
	if (res != HAL_OK) return res;

	T0_OUT = (((uint16_t)buf[1]) << 8) | (uint16_t)buf[0];
	T1_OUT = (((uint16_t)buf[3]) << 8) | (uint16_t)buf[2];

	return res;
}

HAL_StatusTypeDef HTS221_Read_Humi_Coef(void) {
	HAL_StatusTypeDef res;
	uint8_t buf[2];

	//	reading fabric coefficient

	res = HTS221_Read_Reg(HTS221_ADDR_H0H1_RH_X2, 2, buf);
	if (res != HAL_OK) return res;

	H0_rh = buf[0] >> 1;
	H1_rh = buf[1] >> 1;

	res = HTS221_Read_Reg(HTS221_ADDR_H0_T0_OUT, 2, buf);
	if (res != HAL_OK) return res;

	H0_T0_out = (uint16_t)(buf[1] << 8) | (uint16_t)buf[0];

	res = HTS221_Read_Reg(HTS221_ADDR_H1_T0_OUT, 2, buf);
	if (res != HAL_OK) return res;

	H1_T0_out = (uint16_t)(buf[1] << 8) | (uint16_t)buf[0];

	return res;

}

int16_t getTemperature(void) {
	HAL_StatusTypeDef res;
	uint8_t buf[2];
	int16_t T_OUT;
	int32_t T_degC;
	int16_t temp;

	//	read raw data

	res = HTS221_Read_Reg(HTS221_ADDR_T_OUT, 2, buf);
	if (res != HAL_OK) return res;

	T_OUT = (((uint16_t)buf[1]) << 8) | (uint16_t)buf[0];

	//	counting temperature

	T_degC = ((int32_t)(T_OUT - T0_OUT)) * ((int32_t)(T1_degC - T0_degC) * 10);
	temp = (T_degC / (T1_OUT - T0_OUT) + T0_degC * 10) / 10;

	return temp;
}

int16_t getHumidity(void) {
	HAL_StatusTypeDef res;
	uint8_t buf[2];
	int32_t H_rh;
	int16_t hum;
	int16_t H_T_out;

	//	read raw data

	res = HTS221_Read_Reg(HTS221_ADDR_H_OUT, 2, buf);
	if (res != HAL_OK) return res;

	H_T_out = (((uint16_t)buf[1]) << 8) | (uint16_t)buf[0];

	//	counting humidity

	H_rh = ((int32_t)(H_T_out - H0_T0_out)) * ((int32_t)(H1_rh - H0_rh) * 10);
	hum = ((uint16_t)(H_rh / (H1_T0_out - H0_T0_out) + H0_rh * 10)) / 10;

	//	in case of overflow

	if(hum > 100) hum = 100;

	return hum;
}

HAL_StatusTypeDef HTS221_Init(void) {
	HAL_StatusTypeDef res;

	res =  HTS221_Write_Reg(HTS221_ADDR_CTRL1, POWER_DOWN);
	if (res != HAL_OK) return res;

	res = HTS221_Write_Reg(HTS221_ADDR_CTRL1, POWER_UP_CTRL1_1HZ);
	if (res != HAL_OK) return res;

	return res;
}
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef res;
  uint8_t buf[50];

  //	HTS221 initialization

  res = HTS221_Init();
  if (res != HAL_OK) return res;

  //	reading fabric coefficient

  res = HTS221_Read_Temp_Coef();
  if (res != HAL_OK) return res;

  res = HTS221_Read_Humi_Coef();
  if (res != HAL_OK) return res;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  sprintf((char *)buf, "Temperature: %dC\r\nHumidity: %d%%\r\n", getTemperature(), getHumidity());
	  HAL_UART_Transmit(&huart1, buf, strlen((char *)buf), HAL_MAX_DELAY);

	  HAL_Delay(500);
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000608;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 *
 *
 */
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FlightComputer.h"
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
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

FlightComputer flight_computer;

uint32_t time_buff;
//uint32_t time_diff;

uint8_t read_data[128];
uint8_t received_data[128];
uint8_t bytesRecv = 0;

#define GPS_BUF_SIZE 128
uint8_t gps_raw_data[GPS_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  	uint8_t isSend = 65;
  	HAL_Delay(5000);
  	HAL_UART_Transmit(&huart1, &isSend, 1, 100);
  	HAL_Delay(1000);
  	FlightComputer_init(&flight_computer, &hspi1, CS_Lora_GPIO_Port, CS_Lora_Pin, &hi2c1);
  	/*flight_computer.LoRa = newLoRa();

  		flight_computer.LoRa.hSPIx                 = &hspi1;
  		flight_computer.LoRa.CS_port               = CS_Lora_GPIO_Port;
  		flight_computer.LoRa.CS_pin                = CS_Lora_Pin;

  		flight_computer.LoRa.frequency             = 433;							  // default = 433 MHz
  		flight_computer.LoRa.spredingFactor        = SF_7;							// default = SF_7
  		flight_computer.LoRa.bandWidth			       = BW_125KHz;				  // default = BW_125KHz
  		flight_computer.LoRa.crcRate				       = CR_4_5;						// default = CR_4_5
  		flight_computer.LoRa.power					       = POWER_20db;				// default = 20db
  		flight_computer.LoRa.overCurrentProtection = 120; 							// default = 100 mA
  		flight_computer.LoRa.preamble				 = 10;		  					// default = 8;

  		LoRa_reset(&(flight_computer.LoRa));
  		LoRa_init(&(flight_computer.LoRa));
  		LoRa_startReceiving(&(flight_computer.LoRa));

  		// TELEMETRIA
  		flight_computer.telemetry_frame[0] = 0x24; // $
  		flight_computer.telemetry_frame[59] = 0x0A; // \n
  		flight_computer.telemetry_frame[60] = 0x0D; // \r
  		flight_computer.telemetry_frame[61] = 0x00; // \0*/

  	//---------------------------------------------------------------
  	//HAL_UART_Transmit(&huart1, "Start Task!\n\r", 13, HAL_MAX_DELAY);

  	/*bmp280_init_default_params(&bmp280.params);
  	bmp280.addr = BMP280_I2C_ADDRESS_0;
  	bmp280.i2c = &hi2c1;

  	while (!bmp280_init(&bmp280, &bmp280.params)) {
  		size = sprintf((char *)Data, "BMP280 initialization failed\n");
  		HAL_UART_Transmit(&huart1, Data, size, 1000);
  		HAL_Delay(2000);
  	}

  	bool bme280p = bmp280.id == BME280_CHIP_ID;
  	size = sprintf((char *)Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
  	HAL_UART_Transmit(&huart1, Data, size, 1000);

  	uint8_t settings = 0x08; // Włącz tryb pomiaru (Measure bit)
  	HAL_I2C_Mem_Write(&hi2c1, (0x53 << 1), 0x2D, I2C_MEMADD_SIZE_8BIT, &settings, 1, 100);

  	uint8_t data[6];
  	int16_t x, y, z;*/

  	// Startujemy odbiór przez DMA
  	//HAL_UART_Receive_DMA(&huart2, gps_raw_data, GPS_BUF_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  time_buff = HAL_GetTick();
	  flight_computer.telemetry_frame[1] = (uint8_t)(time_buff >> 24);
	  flight_computer.telemetry_frame[2] = (uint8_t)(time_buff >> 16);
	  flight_computer.telemetry_frame[3] = (uint8_t)(time_buff >> 8);
	  flight_computer.telemetry_frame[4] = (uint8_t)time_buff;

	  //Odczyty z IMU
	  HAL_I2C_Mem_Read(&hi2c1, 0x53 << 1, 0x32, 1, read_data, 6, 100);
	  flight_computer.telemetry_frame[17] = read_data[1];
	  flight_computer.telemetry_frame[18] = read_data[0];
	  flight_computer.telemetry_frame[19] = read_data[3];
	  flight_computer.telemetry_frame[20] = read_data[2];
	  flight_computer.telemetry_frame[21] = read_data[5];
	  flight_computer.telemetry_frame[22] = read_data[4];
	  HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x1D, 1, read_data, 6, 100);
	  flight_computer.telemetry_frame[23] = read_data[0];
	  flight_computer.telemetry_frame[24] = read_data[1];
	  flight_computer.telemetry_frame[25] = read_data[2];
	  flight_computer.telemetry_frame[26] = read_data[3];
	  flight_computer.telemetry_frame[27] = read_data[4];
	  flight_computer.telemetry_frame[28] = read_data[5];
	  HAL_I2C_Mem_Read(&hi2c1, 0x1E << 1, 0x03, 1, read_data, 6, 100);
	  flight_computer.telemetry_frame[11] = read_data[0];
	  flight_computer.telemetry_frame[12] = read_data[1];
	  flight_computer.telemetry_frame[13] = read_data[4];
	  flight_computer.telemetry_frame[14] = read_data[5];
	  flight_computer.telemetry_frame[15] = read_data[2];
	  flight_computer.telemetry_frame[16] = read_data[3];

	  isSend = LoRa_transmit(&(flight_computer.LoRa), &(flight_computer.telemetry_frame[0]), 62, 500) + 48;
	  /*time_diff = HAL_GetTick() - time_buff;
	  flight_computer.telemetry_frame[1] = 0x01;//(uint8_t)(time_buff >> 24);
	  flight_computer.telemetry_frame[2] = (uint8_t)(time_diff >> 16);
	  flight_computer.telemetry_frame[3] = (uint8_t)(time_diff >> 8);
	  flight_computer.telemetry_frame[4] = (uint8_t)time_diff;
	  isSend = LoRa_transmit(&(flight_computer.LoRa), &(flight_computer.telemetry_frame[0]), 62, 500) + 48;*/
	  HAL_UART_Transmit(&huart1,&(flight_computer.telemetry_frame[0]), 62, 100);
	  //HAL_UART_Transmit(&huart1,&isSend, 1, 100);

	  bytesRecv = LoRa_receive(&(flight_computer.LoRa), received_data, 128);
	  if(bytesRecv > 0){
		  if(received_data[0] == 8){
			  received_data[1] = 0x0A; // \n
			  received_data[2] = 0x0D; // \r
			  received_data[3] = 0x00; // \0
			  HAL_UART_Transmit(&huart1,received_data, 4, 100);
		  }
	  }

	  HAL_Delay(1000);

	  // Prosty test - przekierowanie na inny UART (jeśli masz podpięty ST-Link)
	 /*HAL_UART_Transmit(&huart1, gps_raw_data, GPS_BUF_SIZE, 100);
	  HAL_Delay(1000);*/

	  // SENDING DATA - - - - - - - - - - - - - - - - - - - - - - - - -
	  /*send_data[0] = 0x44;
	  send_data[1] = 0x7A;
	  send_data[2] = 0x69;
	  send_data[3] = 0x65;
	  send_data[4] = 0x6B;
	  send_data[5] = 0x61;
	  send_data[6] = 0x6E;
	  send_data[7] = 0x0D;
	  send_data[8] = 0x0A;

	  isSend = LoRa_transmit(&(flight_computer.LoRa), send_data, 9, 500) + 48;
	  HAL_UART_Transmit(&huart1, send_data, 3, 100);*/

	  // RECEIVING DATA - - - - - - - - - - - - - - - - - - - - - - - -
	  //LoRa_receive(&myLoRa, read_data, 128);

	  /*HAL_Delay(100);
	  while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
		  size = sprintf((char *)Data, "Temperature/pressure reading failed\n");
		  HAL_UART_Transmit(&huart1, Data, size, 1000);
		  HAL_Delay(2000);
	  }

	  size = sprintf((char *)Data,"Pressure: %.2f Pa\n\r",	pressure);
	  isSend = LoRa_transmit(&myLoRa, Data, size, 500) + 48;
//	  HAL_UART_Transmit(&huart1, &isSend, 1, 100);
	  HAL_Delay(500);

	  // Przykład dla ADXL345 (rejestr początkowy 0x32)
	  HAL_I2C_Mem_Read(&hi2c1, 0xA6, 0x32, I2C_MEMADD_SIZE_8BIT, data, 6, 100);

	  x = (int16_t)(data[1] << 8 | data[0]); // Kolejność bajtów zależy od sensora
	  y = (int16_t)(data[3] << 8 | data[2]);
	  z = (int16_t)(data[5] << 8 | data[4]);

	  size = sprintf((char *)Data,"Z: %hd\n\r", z);
	  isSend = LoRa_transmit(&myLoRa, Data, size, 500) + 48;
	  HAL_UART_Transmit(&huart1, "Data Transmit!\n\r", 16, 1000);

	  HAL_Delay(2000);*/
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_Lora_GPIO_Port, CS_Lora_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CS_Lora_Pin */
  GPIO_InitStruct.Pin = CS_Lora_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_Lora_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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

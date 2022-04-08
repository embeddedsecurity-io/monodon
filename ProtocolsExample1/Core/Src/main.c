/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define _GNU_SOURCE
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t uart2_rx_buffer[16];
uint8_t i2c1_rx_buffer[3];

uint32_t command = 0;
uint32_t storage_id = 0;
uint32_t offset = 0;
uint32_t size = 0;

uint8_t * i2c_current;

void uart1_log(char * message, size_t len){
	// opportunistic log - if the HAL is busy, we throw away the message.
	HAL_StatusTypeDef status;
	status = HAL_UART_Transmit_IT(&huart1, (uint8_t *) message, len);
	if (status != HAL_OK)
		free(message);
}

void parse_uart2_rx_buffer(){
	/* The receive buffer is 16 byte. Internally we treat those 16 byte as 4 * 32bit:
	 *
	 * [uint32_t command] [uint32_t storage_id] [uint32_t offset] [uint32_t size]
	 *
	 * The receive buffer is an array of uint8_t and we just cast it, so we can interpret it as
	 * a uint32_t[4].
	 */
	uint32_t * casted_buffer = (uint32_t *) uart2_rx_buffer;
	command = (uint32_t) casted_buffer[0];
	storage_id = (uint32_t) casted_buffer[1];
	offset = (uint32_t) casted_buffer[2];
	size = (uint32_t) casted_buffer[3];
}
/* This is called when TIM2 (every 5 seconds) overflows.
 * We use this to print "Hello World! on UART1
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	char* buffer;
	int len;
	len = asprintf(&buffer, "Hello world!\r\n");
	uart1_log(buffer, len);
}

void uart2_reset(){
	 command = 0;
	 storage_id = 0;
	 offset = 0;
	 size = 0;
}

/* Here we execute the UART "READ FLASH" command (0x2c) */
void uart2_read(){
	HAL_StatusTypeDef status;
	status = HAL_UART_Transmit(&huart2, (uint8_t *)FLASH_BASE + offset, size, 500);

	char* buffer;
	int len;
	if(status == HAL_OK){
		len = asprintf(&buffer, "UART2: Transmitted!\r\n");
	}else {
		len = asprintf(&buffer, "UART2: Transmission Error!\r\n");
	}
	uart1_log(buffer, len);
}

void uart2_command_parse(){
	char * buffer;
 	int len;

	switch(command){
		// 2c is the READ FLASH command.
		case 0x2c:
			uart2_read();
			break;
		default:
		 	len = asprintf(&buffer, "UART2: Invalid command received: 0x%lX\r\n", command);
			uart1_log(buffer, len);
	}
	uart2_reset();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	 if(huart == &huart1){
		 // Transmits via huart1 are our log. These are done via Interrupt.
		 // Messages are allocated on the Heap. We need to free it here after successful transmission.
		 // The transmit buffer is moved byte by byte during transmission.
		 // We calculate the original pointer to free it.
		 uint8_t * buff = (huart->pTxBuffPtr) - huart->TxXferSize;
		 free(buff);
	 }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

 if(huart == &huart2){
	 // We received data on UART! get the command, offset and size from the RX buffer.
	 parse_uart2_rx_buffer();

	 char * buffer;
 	 int len = asprintf(&buffer, "UART2: READ FLASH command received! Command: 0x%lX, Offset:%lX, Size: %lX\r\n", command, offset, size);
	 uart1_log(buffer, len);

	 // Now execute whatever was requested.
	 uart2_command_parse();

 }
 // Enable UART RX...
 HAL_UART_Receive_IT(&huart2, uart2_rx_buffer, 16);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c1){
	// we received 3 bytes of data!

	/* We have only 128 kByte of flash, from 3 bytes received, we only need 17 bits.
	 * make sure the upper 7 bits are not set.
	 */

	if(!(i2c1_rx_buffer[0] && 0xFE)){
			// we have a valid address
			uint32_t received_address = 0x00000000 | (i2c1_rx_buffer[0] << 16);
			received_address = received_address | (i2c1_rx_buffer[1] << 8);
			received_address = received_address | (i2c1_rx_buffer[2]);

			i2c_current = (uint8_t *) (FLASH_BASE + received_address);
	}

	char * buffer;
	int len = asprintf(&buffer, "I2C1: READ FLASH offset set: %p\r\n", i2c_current);
	uart1_log(buffer, len);

}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c1, uint8_t TransferDirection, uint16_t AddrMatchCode){

	/* Called when our I2C address was requested on the bus.
	 * Depending on the transfer direction we will either start a Transmit or Receive.
	 */

	// TransferDirection is seen from the Master, but we are on a Slave

	// We send data to the master
	if(TransferDirection == I2C_DIRECTION_RECEIVE){
		// We do not know how many bytes the master wants. This means we transmit everythin.
		// The controller will stop receiving bytes by sending a NACK. This will result in a Error and
		// subsequently into a callback to HAL_I2C_ErrorCallback.
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c1, i2c_current, (uint16_t)0xFFFFF, I2C_FIRST_FRAME);
	  }

	// We receive the address from the master.
	// This is normally a command to set the flash pointer offset. When the transfer is done,
	// HAL_I2C_SlaveRxCpltCallback is called. There we will parse the received data.
	if(TransferDirection == I2C_DIRECTION_TRANSMIT){
		HAL_I2C_Slave_Seq_Receive_IT(hi2c1, i2c1_rx_buffer, 3, I2C_FIRST_AND_LAST_FRAME);
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
	// This is a hack. The current_address pointer is always shifted one byte too far.
	// If the error was HAL_I2C_ERROR_AF (ACK Failed), we substract one to correct that.
	if(hi2c->ErrorCode == HAL_I2C_ERROR_AF){
		uint32_t bytes_transferred = ((hi2c->XferSize - hi2c->XferCount) - 1);
		i2c_current = i2c_current + bytes_transferred;
		char * buffer;
		int len;
		len = asprintf(&buffer, "I2C1: READ FLASH ended! %lu bytes transfered\r\n", bytes_transferred);
		uart1_log(buffer, len);
	}
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
  i2c_current = (uint8_t *) FLASH_BASE;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_UART_StateTypeDef uart2_status = HAL_UART_GetState(&huart2);
	  if (uart2_status == HAL_UART_STATE_READY)
		  HAL_UART_Receive_IT(&huart2, uart2_rx_buffer, 16);
	  HAL_I2C_StateTypeDef i2c1_status = HAL_I2C_GetState(&hi2c1);
	  if (i2c1_status == HAL_I2C_STATE_READY)
		  HAL_I2C_EnableListen_IT(&hi2c1);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hi2c1.Init.OwnAddress1 = 66;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 36000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

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

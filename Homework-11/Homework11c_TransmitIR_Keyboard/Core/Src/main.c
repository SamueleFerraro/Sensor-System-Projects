/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROW_0 GPIOC,GPIO_PIN_3
#define ROW_1 GPIOC,GPIO_PIN_2
#define ROW_2 GPIOC,GPIO_PIN_13
#define ROW_3 GPIOC,GPIO_PIN_12

#define COL_0 GPIOC,GPIO_PIN_11
#define COL_1 GPIOC,GPIO_PIN_10
#define COL_2 GPIOC,GPIO_PIN_9
#define COL_3 GPIOC,GPIO_PIN_8

#define NUM_OF_SAMPLES 2
#define ROW_LENGTH 4
#define COLUMN_LENGTH 4

#define NUM_OF_BITS 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
char c_tx;
char c_rx;

/*This index is used for the UART protocol*/
uint8_t bit_index = 0;

uint8_t sample_index = 0;	//index of the samples collected in order to debounce
uint8_t col_index = 0;		//index of the number of the column
uint8_t row_index = 0;		//index of the number of the row

uint8_t btn = 0;	//button state (used also for debouncing) - if btn = 0 -> button is pressed

uint8_t i = 0;
uint8_t j = 0;
uint8_t k = 0;


uint8_t rx_data[NUM_OF_SAMPLES][COLUMN_LENGTH][ROW_LENGTH];		//buttons state - 3rd dimension stores the different samples to debounce

char out_matrix[COLUMN_LENGTH][ROW_LENGTH] = {
		{'F', 'E', 'D', 'C'},
		{'B', 'A', '9', '8'},
		{'7', '6', '5', '4'},
		{'3', '2', '1', '0'}
};

uint8_t old_btn[COLUMN_LENGTH][ROW_LENGTH] = {	//previous state of the buttons
		{1, 1, 1, 1},
		{1, 1, 1, 1},
		{1, 1, 1, 1},
		{1, 1, 1, 1},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//start new Byte transmission using UART protocol
/*Maybe this function could be implemented with a parameter instead of a global variable...*/
void IR_UART_Transmit(){
	bit_index=0;

	//clear interrupt request BEFORE enabling tim interrupt
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

	/*TIM3 have been set as prescaler = 0 and ARR = 35000-1,in order to set a baudrate of
	 * 2400 bps, which means having a 1/2400s as a timer*/
	HAL_TIM_Base_Start_IT(&htim3);

	/*This is the Start bit: communication starts when Start bit = 0, and it is 0 when PWM on*/
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

}

//received char through IR (UART)
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart){
	if(huart == &huart1){

		/*Transmission of the char via UART*/
		HAL_UART_Transmit_DMA(&huart2, &c_rx, sizeof(c_rx));
	}
	/*Waiting for the next char via USART1 (IR receiver).
	 * -Parameters: (which port, where to save, size of the data)
	 * -NOTES: UART protocol waits for start bit, so until transmission does
	 * not start, the receiver waits */
	HAL_UART_Receive_DMA(&huart1, &c_rx, sizeof(c_rx));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	/*Every 1/2400s I send new bit*/
	if (htim == &htim3){

		/*Here I implement the transmission of the bits*/

		/*At the last bit I send the Stop bit, which is equal to 1. */
		if(bit_index == NUM_OF_BITS){
			/*The packet is over so I stop TIM3*/
			HAL_TIM_Base_Stop_IT(&htim3);

			/*Stop bit*/
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

		}else{
			/*Here I check what to send (0 or 1):
			 * c_tx is composed by 8 bit: the first bit is the LSB and the last one is the MSB
			 * I compare the char with a byte which is composed as "00000100" where the 1 is the one moving
			 * depending on the bit_index. What I get is a bit-wise AND whose result is 0b0 or 0b100
			 * depending on the value of c_tx third bit. After that the if operator gives 0 if the argument is 0,
			 * while gives 1 if the argument is != from zero*/
			if(c_tx & (1<<bit_index)){
				/*If the char bit at bit_index = 1*/
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

			}else{

				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

			}
			bit_index++;
		}
	}


	if(htim == &htim4){

		/*LOOK HOMEWORK-10: this method have some minor differences:
		 * - Transmission is done every row reading (as soon as it is read)
		 * - No print[][] matrix
		 *
		 * PRO:
		 * - less memory
		 * CONS:
		 * - More overhead in the transmission
		 * 	 (in particular for non-custom transmission functions)*/
		row_index = 0;

		//ROW_0

		//save the btn state
		rx_data[sample_index][row_index][col_index] = HAL_GPIO_ReadPin(ROW_0);	//debouncing: check if the btn is stably zero (check if many samples are = to 0)
		for (i = 0, btn = 0; i < NUM_OF_SAMPLES; i++)
			btn = btn || rx_data[i][row_index][col_index];

		//check if button already pressed: print only if previously not pressed, now pressed
		if (!btn && old_btn[row_index][col_index]){
			c_tx = out_matrix[row_index][col_index];

			/*Instead of the UART transmission of homework-10 I use our custom function*/
			IR_UART_Transmit();
		}

		old_btn[row_index][col_index] = btn; //save the state for the next check

		//ROW_1
		row_index++;
		rx_data[sample_index][row_index][col_index] = HAL_GPIO_ReadPin(ROW_1);
		for (i = 0, btn = 0; i < NUM_OF_SAMPLES; i++)
			btn = btn || rx_data[i][row_index][col_index];

		if (!btn && old_btn[row_index][col_index]){
			c_tx = out_matrix[row_index][col_index];
			IR_UART_Transmit();
		}

		old_btn[row_index][col_index] = btn;

		//ROW_2
		row_index++;

		rx_data[sample_index][row_index][col_index] = HAL_GPIO_ReadPin(ROW_2);
		for (i = 0, btn = 0; i < NUM_OF_SAMPLES; i++)
			btn = btn || rx_data[i][row_index][col_index];

		if (!btn && old_btn[row_index][col_index]){
			c_tx = out_matrix[row_index][col_index];
			IR_UART_Transmit();
		}

		old_btn[row_index][col_index] = btn;

		//ROW_3
		row_index++;
		rx_data[sample_index][row_index][col_index] = HAL_GPIO_ReadPin(ROW_3);
			for (i = 0, btn = 0; i < NUM_OF_SAMPLES; i++)
				btn = btn || rx_data[i][row_index][col_index];

		if (!btn && old_btn[row_index][col_index]){
			c_tx = out_matrix[row_index][col_index];
			IR_UART_Transmit();
		}

		old_btn[row_index][col_index] = btn;


		col_index++;
		if (col_index == ROW_LENGTH) {
			col_index = 0;
			sample_index++;	//need to acquire new sample
			if (sample_index == NUM_OF_SAMPLES)
				sample_index = 0;
		}

		//columns drive manager
		HAL_GPIO_WritePin(COL_0, (col_index == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(COL_1, (col_index == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(COL_2, (col_index == 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(COL_3, (col_index == 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  for (i = 0; i < NUM_OF_SAMPLES; i++) {
       for (j = 0; j < COLUMN_LENGTH; j++) {
           for (k = 0; k < ROW_LENGTH; k++) {
               rx_data[i][j][k] = 1;
           }
       }
   }

  /*Very first receiving*/
  HAL_UART_Receive_DMA(&huart1, &c_rx, sizeof(c_rx));

  HAL_GPIO_WritePin(COL_0, GPIO_PIN_SET); //drive the first column

  __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);	//clear interrupt request BEFORE enabling tim interrupt
  HAL_TIM_Base_Start_IT(&htim4); 				//start TIM2 in interrupt mode

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2210-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1105-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 35000-1;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 2400;
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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC2 PC3 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

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
  *
  * GOAL:
  * Scan each column and read the keyboard using a timer interrupt. Letters have
  * to be transmitted to the PC via UART
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/*NUM_OF_SAMPLES is choosen in order to debounce*/
#define NUM_OF_SAMPLES 2
#define ROW_LENGTH 4
#define COLUMN_LENGTH 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

//index of the samples collected in order to debounce
uint8_t sample_index = NUM_OF_SAMPLES-1;

//index of the number of the column
uint8_t col_index = ROW_LENGTH-1;
//index of the number of the row
uint8_t row_index = 0;
//button state (used also for debouncing) - if zero btn is pressed
uint8_t btn = 0;

uint8_t i = 0;
uint8_t j = 0;
uint8_t k = 0;

//buttons state - 3rd dimension stores the different samples to debounce
uint8_t rx_data[NUM_OF_SAMPLES][COLUMN_LENGTH][ROW_LENGTH];

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

uint8_t print[COLUMN_LENGTH][ROW_LENGTH] = {	//if 1 -> print the corresponding button
		{0, 0, 0, 0},
		{0, 0, 0, 0},
		{0, 0, 0, 0},
		{0, 0, 0, 0},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*Every 5 ms*/
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {	//5ms elapsed
	if (htim == &htim2) {
		row_index = 0;

		/*Getting the data from ROW 0
		 * Here I save the state of the button*/
		rx_data[sample_index][row_index][col_index] = HAL_GPIO_ReadPin(ROW_0);

		/*Debouncing: check if the btn is stable at 0 (2 times in a row)*/
		btn = 0;
		for (i= 0; i < NUM_OF_SAMPLES; i++) {

			/*Here I update the state of the button between new and old value*/
			btn = btn || rx_data[i][row_index][col_index];
		}

		/*If the the button was already pressed, do no print anything
		 * else (if the button was not pressed (old_btn[][] = 1) then print the char out
		 * (by setting print matrix)*/
		if (!btn && old_btn[row_index][col_index]){
			print[row_index][col_index] = 1;
		}
		/*Here I update the button state*/
		old_btn[row_index][col_index] = btn;

		/*Row 1: same procedure as before*/

		row_index++; //update of the index

		/*Getting the btn state*/
		rx_data[sample_index][row_index][col_index] = HAL_GPIO_ReadPin(ROW_1);

		/*Debouncing*/
		btn = 0;
		for (i= 0; i < NUM_OF_SAMPLES; i++) {
			btn = btn || rx_data[i][row_index][col_index];
		}

		/*Same control as before*/
		if (!btn && old_btn[row_index][col_index]){
			print[row_index][col_index] = 1;
		}

		old_btn[row_index][col_index] = btn;

		/*ROW 3 */

		row_index++; //update of the index

		/*Getting the btn state*/
		rx_data[sample_index][row_index][col_index] = HAL_GPIO_ReadPin(ROW_2);

		/*Debouncing*/
		btn = 0;
		for (i= 0; i < NUM_OF_SAMPLES; i++) {
			btn = btn || rx_data[i][row_index][col_index];
		}

		if (!btn && old_btn[row_index][col_index]){
			print[row_index][col_index] = 1;
		}

		old_btn[row_index][col_index] = btn;


		/*ROW 3*/

		row_index++; //update of the index

		/*Getting the btn state*/
		rx_data[sample_index][row_index][col_index] = HAL_GPIO_ReadPin(ROW_3);

		btn = 0;
		for (i= 0; i < NUM_OF_SAMPLES; i++) {
			btn = btn || rx_data[i][row_index][col_index];
		}

		if (!btn && old_btn[row_index][col_index])
			print[row_index][col_index] = 1;

		old_btn[row_index][col_index] = btn;


		/*Here I transmit to the PC via uart all the */
		for (i = 0; i < ROW_LENGTH; i++) {
			if(print[i][col_index]){
				HAL_UART_Transmit_DMA(&huart2, &out_matrix[i][col_index], 1);
				print[i][col_index]=0;
		    }
		}

		/*Update the column which have to be turned on: this have to be done at the very end
		 * in order to allow a stable "turned on" situation for the next iteration
		 * in this way overlap is avoided*/
		col_index++;

		if (col_index == ROW_LENGTH){
			col_index = 0;
		}

		//Resetting every column
		HAL_GPIO_WritePin(COL_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(COL_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(COL_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(COL_3, GPIO_PIN_RESET);

		/*Turning on the right column depending on the index*/
		switch (col_index) {
		case 0:
			/*every time I start from the first column, I check the state of
			 * the sample index: for every iteration I acquire a new sample*/
			sample_index++;	//need to acquire new sample
			if (sample_index == NUM_OF_SAMPLES){
				sample_index = 0;
			}
			HAL_GPIO_WritePin(COL_0, GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(COL_1, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(COL_2, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(COL_3, GPIO_PIN_SET);
			break;
		default:
			break;
		}
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
  /* USER CODE BEGIN 2 */

  /*Reset*/
  for (i = 0; i < NUM_OF_SAMPLES; i++) {
      for (j = 0; j < COLUMN_LENGTH; j++) {
          for (k = 0; k < ROW_LENGTH; k++) {
              rx_data[i][j][k] = 1;
          }
      }
  }

  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);	//clear interrupt request BEFORE enabling tim interrupt
    HAL_TIM_Base_Start_IT(&htim2); 				//start TIM2 in interrupt mode

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define WELCOME_MSG "Welcome to PWM Manager\r\n"
#define MAIN_MENU   "Enter duty Cycle (000 -> 999):\r\n"
#define PROMPT "\r\n> "

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim4;
char readBuf[1];


__IO ITStatus UartReady = SET;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
extern void MX_GPIO_Init(void);
extern void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
void performCriticalTasks(void);
void printWelcomeMessage(void);
void transmitResponse(uint8_t);
void transmitString(char *);
uint16_t processUserInput(int8_t, uint16_t, uint16_t);
int8_t readByte(void);

int main(void) {

  /* Reset of all peripherals, Initializes the Flash interface and the SysTick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();

  /* Enable USART2 interrupt */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  uint16_t dutyCycle = 0;

  /* strt PWM */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  uint16_t maxDuty = __HAL_TIM_GET_AUTORELOAD(&htim4) - 1;

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, dutyCycle);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, dutyCycle);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, dutyCycle);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, dutyCycle);

  /* set motor to forward state*/
  HAL_GPIO_WritePin(Forward_Reverse_GPIO_Port, Forward_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Forward_Reverse_GPIO_Port, Reverse_Pin, GPIO_PIN_RESET);


	printWelcomeMessage();

	int8_t opt = -1;
	uint16_t tmpDuty = dutyCycle;
	char msg[30];

	while (1)  {
		tmpDuty = dutyCycle;
		opt = readByte();
		if (opt > -1) {
			tmpDuty = processUserInput(opt, dutyCycle, maxDuty);

		}
		else if (tmpDuty == dutyCycle){
			if ((tmpDuty - 25) > 0) {
				tmpDuty = dutyCycle - 25;

				sprintf(msg, "Reducing speed, dutyCycle: %d", tmpDuty);
				transmitString(msg);
			}
			else {
				tmpDuty = 0;
				sprintf(msg, "minimum speed, dutyCycle: %d", tmpDuty);
				transmitString(msg);
			}

		}
		if (tmpDuty != dutyCycle) {
			dutyCycle = tmpDuty;
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, dutyCycle);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, dutyCycle);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, dutyCycle);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, dutyCycle);
		}

		performCriticalTasks();
	}
}

int8_t readByte(void) {
  int8_t retVal = -1;

  if(UartReady == SET) {
    UartReady = RESET;
    HAL_UART_Receive_IT(&huart2, (uint8_t*)readBuf, 1);

    retVal = (int8_t)readBuf[0];
  }
  return retVal;
}


uint16_t processUserInput(int8_t opt, uint16_t duty, uint16_t maxDuty) {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_Delay(5);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    transmitResponse(opt);

    if(!(opt == 'w' || opt == 's')) {
      return duty;
    }
    char msg[30];

    switch(opt) {
    case 'w':
    	HAL_GPIO_WritePin(Forward_Reverse_GPIO_Port, Forward_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(Forward_Reverse_GPIO_Port, Reverse_Pin, GPIO_PIN_RESET);

    	if ((duty + 25) <= maxDuty) {
    		duty += 25;
    		sprintf(msg, "Increasing forward speed, dutyCycle: %d", duty);
    		transmitString(msg);
    	}
    	else {
    		duty = maxDuty;
			sprintf(msg, "Max Forward speed, dutyCycle: %d", duty);
			transmitString(msg);
		}
    	break;

    case 's':
    	HAL_GPIO_WritePin(Forward_Reverse_GPIO_Port, Forward_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Forward_Reverse_GPIO_Port, Reverse_Pin, GPIO_PIN_SET);

		if ((duty + 25) <= maxDuty) {
			duty += 25;
			sprintf(msg, "Increasing reverse speed, dutyCycle: %d", duty);
			transmitString(msg);
		}
		else {
			duty = maxDuty;
			sprintf(msg, "Max reverse speed, dutyCycle: %d", duty);
			transmitString(msg);
		}
		break;

    default:
    	break;
    }

	return duty;
}
void transmitResponse(uint8_t resp){
	char msg[30];
	sprintf(msg, "\nYou entered: %c", resp);
	transmitString(msg);
}

void transmitString(char * msg) {
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)PROMPT, strlen(PROMPT), HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
 /* Set transmission flag: transfer complete*/
 UartReady = SET;
}

void performCriticalTasks(void) {
  HAL_Delay(100);
}

void printWelcomeMessage(void) {
  char *strings[] = {"\033[0;0H", "\033[2J", WELCOME_MSG, MAIN_MENU, PROMPT};

  for (uint8_t i = 0; i < 5; i++) {
    HAL_UART_Transmit_IT(&huart2, (uint8_t*)strings[i], strlen(strings[i]));
    while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX_RX);
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();


  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Forward_Pin | Reverse_Pin */
  GPIO_InitStruct.Pin   = Forward_Pin | Reverse_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(Forward_Reverse_GPIO_Port, &GPIO_InitStruct);


  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

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

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

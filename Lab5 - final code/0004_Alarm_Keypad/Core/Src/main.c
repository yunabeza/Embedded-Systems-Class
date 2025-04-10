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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Keypad4X4.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "timers.h"
#include "fonts.h"
#include "ssd1306.h"

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
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;

/* Definitions for keypadTask */
osThreadId_t keypadTaskHandle;
const osThreadAttr_t keypadTask_attributes = {
  .name = "keypadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 2,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 2,
};
/* Definitions for motionDetectTas */
osThreadId_t motionDetectTasHandle;
const osThreadAttr_t motionDetectTas_attributes = {
  .name = "motionDetectTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 2,
};
/* Definitions for TimerManagement */
osThreadId_t TimerManagementHandle;
const osThreadAttr_t TimerManagement_attributes = {
  .name = "TimerManagement",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 2,
};
/* USER CODE BEGIN PV */
extern char key;
char hold[4];
char code[8];
char savedCode[8];
uint8_t codeIndex = 0;
uint8_t configMode = 1;
uint8_t armed = 0;
uint32_t countcheck = 0;
volatile uint8_t secondes = 0; // Compteur de secondes
TimerHandle_t xTimer;
char msg[20];
int motion = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM12_Init(void);
void StartKeypadTask(void *argument);
void StartOledTask(void *argument);
void StartmotionDetectTask(void *argument);
void StartTask04(void *argument);
/* Callback du Timer */
// Global flag to indicate the timer should be deleted
volatile uint8_t deleteTimerFlag = 0;
volatile uint8_t buzerFlag = 0;
volatile uint8_t sameCode = 1;
void vTimerCallback(TimerHandle_t xTimer)
{

    secondes++;

	// When 60 seconds have elapsed and the system is armed,
	// set the flag to indicate that the timer should be deleted.
	if (armed == 1 && secondes >= 60 && countcheck == 1)
	{

		deleteTimerFlag = 1;
		buzerFlag=1;
	}

}

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
	/* Callback du Timer */
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	/* USER CODE BEGIN 2 */
	SSD1306_Init();
	SSD1306_GotoXY(0, 0);
	//SSD1306_Puts ("Voltage:", &Font_11x18, 1);
	SSD1306_Puts("Enter Code:", &Font_11x18, 1);
	SSD1306_GotoXY(0, 30);
	SSD1306_UpdateScreen();
	SSD1306_UpdateScreen();
	HAL_Delay(500);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of keypadTask */
  keypadTaskHandle = osThreadNew(StartKeypadTask, NULL, &keypadTask_attributes);

  /* creation of oledTask */
  oledTaskHandle = osThreadNew(StartOledTask, NULL, &oledTask_attributes);

  /* creation of motionDetectTas */
  motionDetectTasHandle = osThreadNew(StartmotionDetectTask, NULL, &motionDetectTas_attributes);

  /* creation of TimerManagement */
  TimerManagementHandle = osThreadNew(StartTask04, NULL, &TimerManagement_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  hi2c1.Init.ClockSpeed = 400000;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8400-1;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 180-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KC0_Pin|KC3_Pin|KC1_Pin|KC2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KC0_Pin KC3_Pin KC1_Pin KC2_Pin */
  GPIO_InitStruct.Pin = KC0_Pin|KC3_Pin|KC1_Pin|KC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KR1_Pin */
  GPIO_InitStruct.Pin = KR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KR3_Pin KR2_Pin */
  GPIO_InitStruct.Pin = KR3_Pin|KR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KR0_Pin */
  GPIO_InitStruct.Pin = KR0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR0_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void setNStars(char *str, int n) {
	for (int i = 0; i < n; i++) {
		str[i] = '*';
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartKeypadTask */
/**
 * @brief  Function implementing the keypadTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartKeypadTask */
void StartKeypadTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		/* D10 to D7 as input pins for row 0 to row 3. D6 to D3 as output for column pins C1 to C3*/
		key = Get_Key();
		sprintf(hold, "%c", key);
		code[codeIndex] = key;

		// Save the code if the config mode is activated
		if (configMode == 1 && codeIndex > 4) {
			configMode = 0;
			for (int i = 0; i < strlen(code); i++) {
				savedCode[i] = code[i];
			}
			codeIndex = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		} else if (configMode == 0 && codeIndex > 4) {

			for (int i = 0; i < strlen(code); i++) {
				// Verify if the code corresponds to the saved code
				if (code[i] != savedCode[i]) {
					sameCode = 0;
				}
			}
			// If the entered code matches arm or disarm
			if (sameCode == 1) {
				if (armed == 1) {
					//Flags
					armed = 0;
					countcheck = 0;
					buzerFlag = 0;

					// change flag to delete the timer if exist
					if(xTimer != NULL){
						deleteTimerFlag = 1;
					}

					//Led management
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // Green led
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //Red led

					//open the door
					__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2, 750);
					HAL_Delay(1000);
				} else {
					armed = 1;
					// Launch Timer
					if(countcheck == 0){
						xTimer = xTimerCreate("Timer1Sec", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, vTimerCallback);
						  if (xTimer == NULL)
						  {
								  sprintf(msg, "%s \r\n","Erreur Timer");
								  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						  }else{
							xTimerStart(xTimer, 0);
							countcheck =1;
						}
					}

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
					//locking door
					__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2, 250);
					HAL_Delay(1000);

				}
			}
			codeIndex = 0;
		} else {
			codeIndex = codeIndex + 1;
		}

		HAL_UART_Transmit(&huart2, (uint8_t*) hold, strlen(hold), 100);
		HAL_Delay(500);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
 * @brief Function implementing the oledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartOledTask */
void StartOledTask(void *argument)
{
  /* USER CODE BEGIN StartOledTask */
	/* Infinite loop */

	for (;;) {
		if (codeIndex > 0) {
			if (codeIndex == 1) {
				SSD1306_GotoXY(0, 30);
				SSD1306_UpdateScreen();
				SSD1306_Puts("           ", &Font_16x26, 1);
			}
			char str[20] = "";
			char str2[8] = "";
			setNStars(str2, codeIndex);
			snprintf(str, sizeof(str2), str2);
			SSD1306_GotoXY(0, 30);
			SSD1306_UpdateScreen();
			SSD1306_Puts(str2, &Font_16x26, 1);
		} else if (configMode == 0) {
			char str[10] = "";
			if (armed == 1) {
				char msg[20] = "";
				// Show the timer update on the screen
				if(secondes > 0){
				    snprintf(msg, sizeof(msg), "%d", 60 - (int)secondes);
				    SSD1306_Clear();
					// Set cursor position
					SSD1306_GotoXY(0, 30);
					// Write the counter on the display
					SSD1306_Puts(msg, &Font_11x18, 1);
					// Refresh the screen to show the update
					SSD1306_UpdateScreen();
					// Wait for 1 second before next count
					osDelay(1000);}
				//Just to make sure that will display Armed after the timer is delete since when it's happen the secondes variable goes to 0
					if(secondes == 0){
						if(sameCode == 0){
							snprintf(str, sizeof(str), "Incorrect");
							// Clear previous
							SSD1306_Clear();
							// Set cursor position
							SSD1306_GotoXY(0, 30);
							// Write the counter on the display
							SSD1306_Puts(str, &Font_11x18, 1);
							// Refresh the screen to show the update
							SSD1306_UpdateScreen();
							// Wait for 1 second before next count
							osDelay(1000);
							sameCode = 1;
						}
						snprintf(str, sizeof(str), "ARMED");
						// Clear previous
						SSD1306_Clear();
						// Set cursor position
						SSD1306_GotoXY(0, 30);
						// Write the counter on the display
						SSD1306_Puts(str, &Font_11x18, 1);
						// Refresh the screen to show the update
						SSD1306_UpdateScreen();
						// Wait for 1 second before next count
						osDelay(1000);
					}
			} else {
				snprintf(str, sizeof(str), "NOT ARMED");
				// Clear previous content (optional, if you want a clean screen)
				SSD1306_Clear();
				// Set cursor position
				SSD1306_GotoXY(0, 30);
				// Write the counter on the display
				SSD1306_Puts(str, &Font_11x18, 1);
				// Refresh the screen to show the update
				SSD1306_UpdateScreen();
				// Wait for 1 second before next count
				osDelay(1000);
			}
			SSD1306_GotoXY(0, 30);
			SSD1306_UpdateScreen();
			SSD1306_Puts(str, &Font_11x18, 1);
		}
		SSD1306_UpdateScreen();
		osDelay(200);
	}
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartmotionDetectTask */
/**
* @brief Function implementing the motionDetectTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartmotionDetectTask */
void StartmotionDetectTask(void *argument)
{
  /* USER CODE BEGIN StartmotionDetectTask */
  /* Infinite loop */
  for(;;)
  {
	if (armed == 1) {
		motion = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		if (buzerFlag ==1 && motion == 1) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_UART_Transmit(&huart2, (uint8_t*) "motion\n",
					strlen("motion\n"), 100);
			osDelay(6000); // Buzzer goes on for 6 seconds if motion is sensed
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // buzzer off
			osDelay(6000); // Buzzer goes off for 6 seconds after before checking if there's still motion sensed
		}
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // buzzer off
	}
    osDelay(1);
  }
  /* USER CODE END StartmotionDetectTask */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the TimerManagement thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {

	  // Check if the timer should be deleted
	          if (deleteTimerFlag == 1)
	          {
	        	  vTimerSetReloadMode(xTimer, pdFALSE);
	        	  if (armed == 1)
				  {
	        		  // Optionally update display before deletion
					  char msg[20];
					  snprintf(msg, sizeof(msg), "ARMED");
					  SSD1306_Clear();
					  SSD1306_GotoXY(0, 30);
					  SSD1306_Puts(msg, &Font_11x18, 1);
					  SSD1306_UpdateScreen();
					  osDelay(1000);


				  }else{
					  // Optionally update display before deletion
					  char msg[20];
					  snprintf(msg, sizeof(msg), "NOT ARMED");
					  SSD1306_Clear();
					  SSD1306_GotoXY(0, 30);
					  SSD1306_Puts(msg, &Font_11x18, 1);
					  SSD1306_UpdateScreen();
					  osDelay(1000);
				  }

	              // Delete the timer to stop further callbacks
	              if (xTimerDelete(xTimer, 0) == pdPASS)
	              {
	                  // Reset counter and flag
	                  secondes = 0;
	                  deleteTimerFlag = 0;
	                  xTimer = NULL;  // Mark the timer as deleted

	              }
	          }
	          // Sleep for a short period before checking again
	          osDelay(100);
	      }
  /* USER CODE END StartTask04 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

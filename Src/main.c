/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

RTC_TimeTypeDef realTime = {0};
RTC_DateTypeDef realDate = {0};

uint8_t lastSendHour = 80;

uint32_t lastReqTime;

uint32_t lastInteractionTime;
const uint32_t interactionAbsenceMaxTime = 10000;

double lastEnergy = 0;

const char MenuElements[4][16] = {{"Output Now:"}, {"Energy today:"}, {"Week mid. value:"}, {"Element 4"}};
const char ElementsUnits[4][6] = {{" KW"}, {" KWhr"}, {" KWhr"}, {" U4"}};

char Data[4][13] = {"0.0", "0.0", "0.0", "0.0"}; 
char EnergyData[7][12] = {"0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0"};

char ExceptionArr[15] = {0};
uint8_t ExceptionNum = 0;

uint8_t selectedElemIndex = 0;  
uint8_t buttonIsPressed = 0;
uint8_t daysScroll = 0;
uint8_t scrollCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Set up timer for Encoder  
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
	HAL_Delay(1000);

  // Initialize lcd display  
	lcd_init();
	lcd_clear();
	lcd_put_cur(0,0);
	lcd_send_string(MenuElements[selectedElemIndex]);
	lcd_put_cur(1,0);
  char DataStr[16] = {0}; 
	strcpy(DataStr, Data[selectedElemIndex]);   
	strcat(DataStr, ElementsUnits[selectedElemIndex]); 
	lcd_send_string(DataStr);

	//  Staring Modbus
	StartMB(&huart1);

  // Get data from Modbus
  DataRequest();
  lastReqTime = lastInteractionTime;
	
	//  Get real time from the server
  InitRealTime(&huart3, &hrtc, &realTime);
  // char commandToSend[2] = "1";
	// char* responseRx = TxESP(&huart3, commandToSend);
	// if(strcmp(responseRx, "Error 2") != 0 && strcmp(responseRx, "0") != 0) {
	// 	DelException('2');
	// 	realTime.Hours = atoi(strtok(responseRx,":"));
	// 	realTime.Minutes = atoi(strtok(NULL,":"));
	// 	realTime.Seconds = 0;
	// 	if (HAL_RTC_SetTime(&hrtc, &realTime, RTC_FORMAT_BIN) != HAL_OK)
	// 	{
	// 		Error_Handler();
	// 	}
	// }else
	// 	AddException('2');

	// free(responseRx);

  // Initialize lastInteractionTime
	lastInteractionTime = HAL_GetTick();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	 
    if(ExceptionNum != 0)
      HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);

    HAL_RTC_GetTime(&hrtc, &realTime, RTC_FORMAT_BIN);

    if(ExceptionNum == 0 && ((realTime.Hours > 4 && realTime.Hours <= 22) || (realTime.Hours == 4 && realTime.Minutes > 0)))
    {
      if(realTime.Minutes<=50 && lastSendHour != realTime.Hours)
      {
        sendEnergyDelta();
        lastSendHour = realTime.Hours;
        
        char mess[25] = {0};
        sprintf(mess, "Data sent, Time: %u:%u", realTime.Hours, realTime.Minutes);
        HAL_UART_Transmit(&huart2, (uint8_t *) mess, 25, 0xFFFF);
      }
    }
	
    scrollCount = __HAL_TIM_GET_COUNTER(&htim2);
    if(scrollCount == 1)
    {
      lastInteractionTime = HAL_GetTick();
      if(buttonIsPressed != 1)
      {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) == 0)
        {	
          if(selectedElemIndex !=0)
          {
            selectedElemIndex--;
          }else {
            selectedElemIndex = 3;
          }
          RefreshLCDAnim(0);
        }
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) == 1)
        {				
          if(selectedElemIndex != 3)
          {
            selectedElemIndex++;
          }else {
            selectedElemIndex = 0;
          }
          RefreshLCDAnim(1);	
          
        }
      }else
      {
        lcd_put_cur(0,7);
        if(((TIM2->CR1) & 0x10) == 0x00)
        {	
          if(daysScroll !=0 && daysScroll != 1)
          {
            daysScroll--;
            char daysStr[7];
            sprintf(daysStr, "%u", daysScroll); 
            strcat(daysStr, "d bef:");
            lcd_send_string(daysStr);	
          }else if(daysScroll ==1)
          {
            daysScroll = 0;
            lcd_send_string("today: ");
          }else
          {
            daysScroll = 6;
            lcd_send_string("6d bef:");	
          }
        }
        if(((TIM2->CR1) & 0x10) == 0x10)
        {
          if(daysScroll !=6)
          {
            daysScroll++;
            char daysStr[7];
            sprintf(daysStr, "%u", daysScroll); 
            strcat(daysStr, "d bef:");
            lcd_send_string(daysStr);	
          }else
          {
            daysScroll = 0;
            lcd_send_string("today: ");
          }
        }
      }
      HAL_Delay(200);	
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      DataRequest();
    }
    
    if(HAL_GetTick() - lastReqTime > 5000)
    {
      DataRequest();
    }

    // char hourStr[6] = {0};
    // sprintf(hourStr, "%u", realTime.Hours);
    // char minStr[5] = {0};
    // sprintf(minStr, "%u", realTime.Minutes);
    // strcat(hourStr, minStr);
    // lcd_clear();
    // lcd_put_cur(0,0);
    // lcd_send_string(hourStr);
    // HAL_Delay(200);
    
    // If there is no interection for more then interactionAbsenceMaxTime, go to sleep...
    if(HAL_GetTick() - lastInteractionTime > interactionAbsenceMaxTime)
    {
      sleepBegin(&hrtc, &realTime, &huart3, &selectedElemIndex, &scrollCount, &buttonIsPressed, &lastInteractionTime, &huart2);
    }
		
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 4;
  sTime.Minutes = 55;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_TUESDAY;
  DateToUpdate.Month = RTC_MONTH_SEPTEMBER;
  DateToUpdate.Date = 22;
  DateToUpdate.Year = 20;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A 
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  htim3.Init.Period = 49;
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
  htim4.Init.Prescaler = 3000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
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
  huart1.Init.BaudRate = 9600;
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
  huart2.Init.Mode = UART_MODE_TX;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin|RS485_RTS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP_Reset_GPIO_Port, ESP_Reset_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP_Wakeup_GPIO_Port, ESP_Wakeup_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP_Reset_Pin */
  GPIO_InitStruct.Pin = ESP_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP_Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_RTS_Pin */
  GPIO_InitStruct.Pin = RS485_RTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_RTS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP_Wakeup_Pin */
  GPIO_InitStruct.Pin = ESP_Wakeup_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP_Wakeup_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

uint8_t ExceptionExists(const char exceptionCode)
{
  uint8_t i = 0, pos = 100;
  while(ExceptionArr[i] != '\0'){
    if(ExceptionArr[i] == exceptionCode) {
      pos = i;
      break;
    }
    i++;
  }
  return pos;
}

void AddException(const char exceptionCode)
{
  uint8_t pos = ExceptionExists(exceptionCode);
  if(pos == 100){
    
    char mess[10] = {0};
    strcpy(mess, "Error ");
    mess[6] = exceptionCode;
    HAL_UART_Transmit(&huart2, (uint8_t *) mess, 10, 0xFFFF);

    ExceptionArr[strlen(ExceptionArr)] = exceptionCode;
    ExceptionNum++;
    RefreshLCD();
  }
}

void DelException(const char exceptionCode)
{
  uint8_t pos = ExceptionExists(exceptionCode);
  if(pos != 100){
    while(ExceptionArr[pos] != '\0')
    {
      ExceptionArr[pos] = ExceptionArr[pos+1];
      pos++;
    }
    ExceptionNum--;
    RefreshLCD();
  }
  if(ExceptionNum == 0)
    HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
}

void DelAllExceptions()
{
  uint8_t i = 0;
  while(ExceptionArr[i] != '\0')
  {
    ExceptionArr[i] = '\0';
    i++;
  }
  ExceptionNum = 0;
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
}

void DataRequest()
{
  char* getRX;

	if(selectedElemIndex == 0)
	{
		getRX = MBGetQuery(&huart1, 64250, 1);
    if(strcmp(Data[selectedElemIndex], getRX) != 0)
    {
      strcpy(Data[selectedElemIndex], getRX);
      RefreshLCD();
    }
	}else if(selectedElemIndex == 1){			
		getRX = MBGetQuery(&huart1, 2052+(daysScroll*2), 2);
    if(strcmp(EnergyData[daysScroll], getRX) != 0)
    {
      if(buttonIsPressed != 1)
      {
        strcpy(EnergyData[daysScroll], getRX);
        RefreshLCD();
      }
    }
	}else if(selectedElemIndex == 2)
	{
		double summ = 0; 
		for(int i=0; i<7; i++)
		{
			getRX = MBGetQuery(&huart1, 2052+i*2, 2);
      summ += atof(getRX); 	
		}
		char middVal[12] = {0};
    summ /= 7;
		snprintf(middVal, sizeof middVal, "%.1lf", summ);
    if(strcmp(Data[selectedElemIndex], middVal) != 0)
    {
      strcpy(Data[selectedElemIndex], middVal);
      RefreshLCD();
    }
	}
	lastReqTime = HAL_GetTick();
}

void sendEnergyDelta()
{
  // Get and calculate energy delta
  char* getRX;
	double TotalEnergy = 0.0;

	getRX = MBGetQuery(&huart1, 2050, 2);
  TotalEnergy = atof(getRX);

  // Send energy delta to the server
  char commandToSend[25];
  snprintf(commandToSend, sizeof commandToSend, "2 %.1lf", TotalEnergy);

  char responseRx[15];
  TxESP(&huart3, commandToSend, responseRx, 15);
  if(strcmp(responseRx, "Error 3") != 0 && strcmp(responseRx, "0") != 0) {
    DelException('3');
  } else
    AddException('3');
}

void LoadingScreen()
{
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("Loading...");
}

void RefreshLCD()
{
	lcd_clear();
	lcd_put_cur(0,0);
  if(ExceptionNum == 0) {
    lcd_send_string(MenuElements[selectedElemIndex]);
    if(selectedElemIndex == 1)
    {
      DayChangeLCD();
    }else
    {
      lcd_put_cur(1,0);
      char DataStr[16] = {0}; 
      strcpy(DataStr, Data[selectedElemIndex]);  
      strcat(DataStr, ElementsUnits[selectedElemIndex]); 
      lcd_send_string(DataStr);
    }
  } else {
    char tempStr[10] = "Error #";
    tempStr[7] = ExceptionArr[0];
    lcd_send_string(tempStr);
  }
}

void RefreshLCDAnim(uint8_t Left)
{
	uint8_t steps = 16;
	if(selectedElemIndex==2 && Left==1)
	{
		steps = 25;
	}
	for(int i = 0; i< steps ; i++)
	{
		if(Left == 1)
		{
			scrollDiaplayLeft();
		}else{
			scrollDiaplayRight();
		}
		
		if(i==15)
		{	
			lcd_clear();
		}else
		{	
			HAL_Delay(40);
		}
	}
	for(int i = 0; i<steps ; i++)
	{
		if(Left==1)
		{
			scrollDiaplayRight();
		}else {
			scrollDiaplayLeft();
		}
	}
	HAL_Delay(200);
	lcd_put_cur(0,0);
	lcd_send_string(MenuElements[selectedElemIndex]);
	if(selectedElemIndex == 1)
	{
		DayChangeLCD();
	}else
	{
		lcd_put_cur(1,0);
    char DataStr[16] = {0}; 
		strcpy(DataStr, Data[selectedElemIndex]);  
		strcat(DataStr, ElementsUnits[selectedElemIndex]); 
		lcd_send_string(DataStr);
	}
	
}

void DayChangeLCD(void)
{
	lcd_put_cur(0,7);
	if(daysScroll !=0)
	{
		char daysStr[7];
		sprintf(daysStr, "%u", daysScroll); 
		strcat(daysStr, "d bef:");
		lcd_send_string(daysStr);	
	}else{
		lcd_send_string("today: ");	
	}
	lcd_put_cur(1,0);
  char DataStr[16] = {0}; 
	strcpy(DataStr, EnergyData[daysScroll]);   
	strcat(DataStr, ElementsUnits[selectedElemIndex]); 
	lcd_send_string(DataStr);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
	
	}
	if(htim->Instance == TIM4) 
	{
		HAL_TIM_Base_Stop_IT(&htim4); // ????????????? ??????
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);  // ??????? ??? EXTI_PR (??? ??????????)
		NVIC_ClearPendingIRQ(EXTI4_IRQn); // ??????? ??? NVIC_ICPRx (??? ???????)
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);   // ???????? ??????? ??????????
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Encoder button callback
  if(GPIO_Pin== GPIO_PIN_4) {
		//delayAlarm();
		lastInteractionTime = HAL_GetTick();
    HAL_NVIC_DisableIRQ(EXTI4_IRQn); 
    if(ExceptionNum == 0) {
      RefreshLCD();
      if(selectedElemIndex == 1)
      {
        if(buttonIsPressed == 0)
        {
          buttonIsPressed = 1;
          lcd_put_cur(0,15);
          lcd_send_string("<");
        }else{
          buttonIsPressed = 0;
          
          lcd_clear();
          lcd_put_cur(0,0);
          lcd_send_string(MenuElements[selectedElemIndex]);
          
          DayChangeLCD();
        }
      }
    }
		HAL_TIM_Base_Start_IT(&htim4);
  } 
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

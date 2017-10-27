/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

#include "ServoIface.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

osThreadId MotorCtrlTaskIdHandle;
osThreadId RGBLedTaskIdHandle;
osThreadId AutoTestTaskIdHandle;
osSemaphoreId selfTestSemHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//int zero_is_found = 0;
int speed_done = 0;
int slow_done = 0;
volatile uint32_t pwm=0;
uint32_t debug_var = 0;
//int32_t encoder_counter=0;
//int32_t encoder_0_value=0;
int sens_pwm=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void MotorCtrlTask(void const * argument);
void RGBLedTask(void const * argument);
void AutoTestTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */


  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);

  /*Init PID library*/
  pPID = pid_init(); //position PID
  PID_Set_Coefficient(pPID->PID,50,1,260,0); // KP, KI, KD, Ilimit
  PID_Set_limitation(pPID,1024,0);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of selfTestSem */
  osSemaphoreDef(selfTestSem);
  selfTestSemHandle = osSemaphoreCreate(osSemaphore(selfTestSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of MotorCtrlTaskId */
  osThreadDef(MotorCtrlTaskId, MotorCtrlTask, osPriorityNormal, 0, 128);
  MotorCtrlTaskIdHandle = osThreadCreate(osThread(MotorCtrlTaskId), NULL);

  /* definition and creation of RGBLedTaskId */
  osThreadDef(RGBLedTaskId, RGBLedTask, osPriorityNormal, 0, 128);
  RGBLedTaskIdHandle = osThreadCreate(osThread(RGBLedTaskId), NULL);

  /* definition and creation of AutoTestTaskId */
  osThreadDef(AutoTestTaskId, AutoTestTask, osPriorityIdle, 0, 128);
  AutoTestTaskIdHandle = osThreadCreate(osThread(AutoTestTaskId), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;//0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = (uint16_t) (SystemCoreClock / 21000000) - 1;//0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1024;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 512;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_G_Pin|LED_B_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ENDSTOP_2_Pin */
  GPIO_InitStruct.Pin = ENDSTOP_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENDSTOP_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CMD_IN_Pin */
  GPIO_InitStruct.Pin = CMD_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(CMD_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENDSTOP_1_Pin */
  GPIO_InitStruct.Pin = ENDSTOP_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENDSTOP_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_G_Pin LED_B_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_B_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief Hardware function for setting the motor speed.
  * @param speed: the new motor speed. If positive the motor will move forward, if negative it will move backward.
  */
void HwSetMotorSpeed(int speed) {
	if(speed > 0) {
		TIM3->CCR1 = 0;
		TIM3->CCR2 = speed;
	} else if (speed < 0) {
		TIM3->CCR1 = speed * -1;
		TIM3->CCR2 = 0;
	} else {
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
	}
}

/**
  * @brief Hardware function for getting the current motor position. Unit = ticks as read by the encoder.
  */
int32_t HwGetCurrentPosition() {
	int32_t output = 0;

	output = TIM2->CNT;

	return output;
}

uint8_t HwGetCommand(void)
{
  return (((HAL_GPIO_ReadPin(CMD_IN_GPIO_Port,    CMD_IN_Pin)    == GPIO_PIN_SET?1:0) << 1U) |
          ((HAL_GPIO_ReadPin(ENDSTOP_2_GPIO_Port, ENDSTOP_2_Pin) == GPIO_PIN_SET?1:0) << 0U));
}

/**
  * @brief Hardware function called when the goal is reached.
  */
void HwGoalIsReached() {
	//TODO: code for SPI feedback when goal is reached

}

/**
  * @brief Hardware function called when the goal is set.
  */
void HwGoalIsActive() {
	//TODO: code for SPI feedback when goal is set

}
/* USER CODE END 4 */

/* MotorCtrlTask function */
void MotorCtrlTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

  uint8_t command ;
  uint8_t prev_command;
  bool initialized = false; // prevent over-run if not init

	set_hw_set_motor_speed(HwSetMotorSpeed);
	set_hw_get_current_position(HwGetCurrentPosition);
	set_hw_goal_is_active(HwGoalIsActive);
	set_hw_goal_is_reached(HwGoalIsReached);

	command = CMD_NOP;
	prev_command = CMD_NOP;


  for(;;)
  {
	  LED_YELLOW;
	  osDelay(1000);
	  PID_Set_limitation(pPID,1024,0);
	  TIM2->CNT=0;
	  PID_Set_Ref_Position(pPID,8000);
	  LED_MAGENTA;
	  osDelay(2000);
	  PID_Set_limitation(pPID,150,0);
	  PID_Set_Ref_Position(pPID,0);
	  LED_BLUE;
	  osDelay(2000);
  }
  /* Infinite loop */
  for(;;)
  {

    command = HwGetCommand();

    // Take action only on command changes
    if(prev_command != command)
    {
      switch(command)
      {

        // Goto Left + Init
        case CMD_GOTO_LEFT:
          LED_YELLOW;
          CheckForZero();
          while(ZeroIsFound() == 0); // wait for endstop trigger
          initialized = true;
          LED_GREEN;
          break;

        case CMD_GOTO_MIDDLE:
          if(initialized)
          {
            LED_CYAN;
            SetObjective(POS_MIDDLE);
            GoToObjective();
            LED_GREEN;
          }

          break;

        case CMD_GOTO_RIGHT:
          if(initialized)
          {
            LED_MAGENTA;
            SetObjective(POS_RIGHT);
            GoToObjective();
            LED_GREEN;
          }
          break;

        default:
          case CMD_NOP:
            LED_BLUE;
            break;
      }
    } // if (command change)

    prev_command = command;

    // Delay in ms
    osDelay(10);

  }

  /* USER CODE END 5 */ 
}

/* RGBLedTask function */
void RGBLedTask(void const * argument)
{
  GPIO_PinState LED_R_Pin_state;
  GPIO_PinState LED_G_Pin_state;
  GPIO_PinState LED_B_Pin_state;
  /* USER CODE BEGIN RGBLedTask */
  /* Infinite loop */
  /*Just toggle LEDs*/
  for(;;)
  {
	/*Save LEDs state*/
	LED_R_Pin_state = HAL_GPIO_ReadPin(GPIOA,LED_R_Pin);
	LED_G_Pin_state = HAL_GPIO_ReadPin(GPIOA,LED_G_Pin);
	LED_B_Pin_state = HAL_GPIO_ReadPin(GPIOA,LED_B_Pin);
	/*Shutdown all LEDs*/
	LED_OFF;
    osDelay(200);
    if(LED_R_Pin_state==0)
	{
    	HAL_GPIO_TogglePin(GPIOA,LED_R_Pin);
	}
    if(LED_G_Pin_state==0)
	{
    	HAL_GPIO_TogglePin(GPIOA,LED_G_Pin);
	}
    if(LED_B_Pin_state==0)
	{
    	HAL_GPIO_TogglePin(GPIOA,LED_B_Pin);
	}

    /*Apply old state*/
    HAL_GPIO_WritePin(GPIOA, LED_R_Pin, LED_R_Pin_state);
    /*HAL_GPIO_WritePin(GPIOA, LED_G_Pin, LED_G_Pin_state);
    HAL_GPIO_WritePin(GPIOA, LED_B_Pin, LED_B_Pin_state);*/
    osDelay(200);

    pwm = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);//XXX
  }
  /* USER CODE END RGBLedTask */
}

/* AutoTestTask function */
void AutoTestTask(void const * argument)
{
	uint32_t PreviousWakeTime = osKernelSysTick();
	int32_t i32CurPos=0;
	/* USER CODE BEGIN PIDTask */
	/* Infinite loop */
	for(;;)
	{
		/*Ensure constant time base*/
		osDelayUntil(&PreviousWakeTime,10);
		i32CurPos = GetCurrentPosition();
		PID_Process_Position(pPID, NULL, i32CurPos);

	}
	/* USER CODE END PIDTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* NOTE: This function should not be modified, when the callback is needed,
	  the HAL_GPIO_EXTI_Callback could be implemented in the user file
	*/
	if(ENDSTOP_1_Pin == GPIO_Pin) {
		if(HAL_GPIO_ReadPin(ENDSTOP_1_GPIO_Port, ENDSTOP_1_Pin) == 0) {
			ZeroTriggered();
		}
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
    LED_RED;
  }
  /* USER CODE END Error_Handler */ 
}

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

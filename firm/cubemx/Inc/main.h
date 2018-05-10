/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ENDSTOP_2_Pin GPIO_PIN_8
#define ENDSTOP_2_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOF
#define ENDSTOP_1_Pin GPIO_PIN_1
#define ENDSTOP_1_GPIO_Port GPIOF
#define ENC_A_Pin GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define POT_IN_Pin GPIO_PIN_2
#define POT_IN_GPIO_Port GPIOA
#define CMD_ANALOG_Pin GPIO_PIN_3
#define CMD_ANALOG_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_5
#define LED_B_GPIO_Port GPIOA
#define MOT_IN1_Pin GPIO_PIN_6
#define MOT_IN1_GPIO_Port GPIOA
#define MOT_IN2_Pin GPIO_PIN_7
#define MOT_IN2_GPIO_Port GPIOA
#define MOT_CUR_Pin GPIO_PIN_1
#define MOT_CUR_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_9
#define LED_R_GPIO_Port GPIOA
#define CMD_IN_Pin GPIO_PIN_10
#define CMD_IN_GPIO_Port GPIOA

// Macros

#define LEDR_WR(_v) {HAL_GPIO_WritePin(GPIOA, LED_R_Pin, (_v));}
#define LEDG_WR(_v) {HAL_GPIO_WritePin(GPIOA, LED_G_Pin, (_v));}
#define LEDB_WR(_v) {HAL_GPIO_WritePin(GPIOA, LED_B_Pin, (_v));}

// Leds
#define LEDx_ON   GPIO_PIN_RESET
#define LEDx_OFF  GPIO_PIN_SET

#define LED_WHITE   {LEDR_WR(LEDx_ON);  LEDG_WR(LEDx_ON);   LEDB_WR(LEDx_ON) ;}
#define LED_RED     {LEDR_WR(LEDx_ON);  LEDG_WR(LEDx_OFF);  LEDB_WR(LEDx_OFF);}
#define LED_GREEN   {LEDR_WR(LEDx_OFF); LEDG_WR(LEDx_ON);   LEDB_WR(LEDx_OFF);}
#define LED_BLUE    {LEDR_WR(LEDx_OFF); LEDG_WR(LEDx_OFF);  LEDB_WR(LEDx_ON) ;}
#define LED_CYAN    {LEDR_WR(LEDx_OFF); LEDG_WR(LEDx_ON);   LEDB_WR(LEDx_ON) ;}
#define LED_YELLOW  {LEDR_WR(LEDx_ON);  LEDG_WR(LEDx_ON);   LEDB_WR(LEDx_OFF);}
#define LED_MAGENTA {LEDR_WR(LEDx_ON);  LEDG_WR(LEDx_OFF);  LEDB_WR(LEDx_ON) ;}
#define LED_OFF     {LEDR_WR(LEDx_OFF); LEDG_WR(LEDx_OFF);  LEDB_WR(LEDx_OFF);}

// Commands
#define CMD_WAIT          0U
#define CMD_SHOOT_HIGH    2U
#define CMD_SHOOT_LOW     3U
#define CMD_INIT          1U

// Objective positions
#define POS_RESET     100
#define POS_ONE_TURN  8000

// Objective speeds
#define SPEED_FAST  1023
#define SPEED_SLOW  700
#define SPEED_INIT -200

/* USER CODE BEGIN Private defines */

uint8_t HwGetCommand(void);

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

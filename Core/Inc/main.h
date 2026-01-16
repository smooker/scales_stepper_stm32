/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* rx codes */
typedef enum
{
   RX_CR,           //CR received
   RX_OF,           //overflow
   RX_CPLT,         //mark
   RX_NOTCPLT,      //default
} CDCReceiveCharTypes;

typedef enum
{
   RX_ECHO_ON,      //echo chars. rcs2 semaphore
   RX_ECHO_OFF,     //do not echo chars. rcs2 semaphore
} echoTypes;

typedef enum
{
   RX_VAR1,      // mode for input var1
   RX_VAR2,      // mode for input var2
   RX_VAR3,      // mode for input var3
   RX_VAR4,      // mode for input var4
   RX_NONE,      // mode for input var4
} inputTypes;

uint8_t cdcprintf(const char *format, ... );
void CDCReceiveChar(uint8_t* inchar);

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define BKPT asm("bkpt 255")
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_USER_Pin GPIO_PIN_13
#define LED_USER_GPIO_Port GPIOC
#define BUTT_JOGL_Pin GPIO_PIN_6
#define BUTT_JOGL_GPIO_Port GPIOA
#define BUTT_JOGR_Pin GPIO_PIN_7
#define BUTT_JOGR_GPIO_Port GPIOA
#define BUTT_STEPL_Pin GPIO_PIN_0
#define BUTT_STEPL_GPIO_Port GPIOB
#define BUTT_STEPR_Pin GPIO_PIN_1
#define BUTT_STEPR_GPIO_Port GPIOB
#define HX711_DATA_Pin GPIO_PIN_13
#define HX711_DATA_GPIO_Port GPIOB
#define HX711_CLK_Pin GPIO_PIN_14
#define HX711_CLK_GPIO_Port GPIOB
#define PULSE_Pin GPIO_PIN_8
#define PULSE_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_9
#define DIR_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

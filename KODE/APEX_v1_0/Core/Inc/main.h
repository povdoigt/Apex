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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "utils/scheduler.h"

#include "utils/data_topic.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RESET_GPS_Pin			GPIO_PIN_0
#define RESET_GPS_GPIO_Port		GPIOC
#define CS_FLASH_Pin			GPIO_PIN_1
#define CS_FLASH_GPIO_Port		GPIOC
#define LED0G_Pin				GPIO_PIN_0
#define LED0G_GPIO_Port			GPIOA
#define LED0B_Pin				GPIO_PIN_1
#define LED0B_GPIO_Port			GPIOA
#define LED0R_Pin				GPIO_PIN_2
#define LED0R_GPIO_Port			GPIOA
#define CS_ACC1_Pin				GPIO_PIN_3
#define CS_ACC1_GPIO_Port		GPIOA
#define CS_ACC0_Pin				GPIO_PIN_4
#define CS_ACC0_GPIO_Port		GPIOA
#define CS_LORA_Pin				GPIO_PIN_4
#define CS_LORA_GPIO_Port		GPIOC
#define RESET_LORA_Pin			GPIO_PIN_5
#define RESET_LORA_GPIO_Port	GPIOC
#define SENCE_BAT_Pin			GPIO_PIN_0
#define SENCE_BAT_GPIO_Port		GPIOB
#define BUZZER_Pin				GPIO_PIN_1
#define BUZZER_GPIO_Port		GPIOB
#define CS_GRYO_Pin				GPIO_PIN_2
#define CS_GRYO_GPIO_Port		GPIOB
#define LORA_DIO0_Pin			GPIO_PIN_15
#define LORA_DIO0_GPIO_Port		GPIOA
#define LED1G_Pin				GPIO_PIN_6
#define LED1G_GPIO_Port			GPIOB
#define LED1B_Pin				GPIO_PIN_7
#define LED1B_GPIO_Port			GPIOB
#define LED1R_Pin				GPIO_PIN_8
#define LED1R_GPIO_Port			GPIOB

/* USER CODE BEGIN Private defines */

typedef struct TASK_Program_start_ARGS { } TASK_Program_start_ARGS;

TASK_POOL_CONFIGURE(TASK_Program_start, 1, 1024);

void TASK_Program_start(void *argument);


typedef struct TASK_Data_USB_Transmit_ARGS {
    data_topic_t **dt;
    uint32_t delay;
} TASK_Data_USB_Transmit_ARGS;

TASK_POOL_CONFIGURE(TASK_Data_USB_Transmit, 1, 1024);

void TASK_Data_USB_Transmit(void *argument);



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

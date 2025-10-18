/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include "utils/scheduler.h"

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */


typedef void* UART_CALLBACK_PARAM_PTR;

typedef void (*UART_Callback)(UART_CALLBACK_PARAM_PTR param_ptr, UART_HandleTypeDef *huart, uint16_t size);


typedef struct UART_CALLBACK_STRUCT {
	#ifdef USART1
		UART_Callback uart1_callback;
        UART_CALLBACK_PARAM_PTR uart1_param_ptr;
	#endif
	#ifdef USART2
        UART_Callback uart2_callback;
        UART_CALLBACK_PARAM_PTR uart2_param_ptr;
	#endif
	#ifdef USART6
        UART_Callback uart3_callback;
        UART_CALLBACK_PARAM_PTR uart3_param_ptr;
	#endif
} UART_BCALLBACK_STRUCT;

extern UART_BCALLBACK_STRUCT uart_callback;

void UART_Callback_struct_set(UART_HandleTypeDef *huart, UART_Callback callback, 
                              UART_CALLBACK_PARAM_PTR param_ptr);


#define ASYNC_UART_Tx_DMA_NUMBER 10

typedef struct ASYNC_UART_Tx_DMA_CONTEXT {
	UART_HandleTypeDef *uart;

	bool has_started;

	uint8_t *buf;
	uint16_t size;
} ASYNC_UART_Tx_DMA_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_UART_Tx_DMA);

void ASYNC_UART_Tx_DMA_init(TASK				*self,
							UART_HandleTypeDef	*uart,
							uint8_t			    *buf,
							uint16_t             size);

TASK_RETURN ASYNC_UART_Tx_DMA(SCHEDULER* scheduler, TASK* self);


// =================================================

#define ASYNC_UART_Rx_DMA_NUMBER 10

typedef struct ASYNC_UART_Rx_DMA_CONTEXT {
	UART_HandleTypeDef *uart;

	bool has_started;

	uint8_t *buf;
	uint16_t size;
} ASYNC_UART_Rx_DMA_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_UART_Rx_DMA);

void ASYNC_UART_Rx_DMA_init(TASK				*self,
							UART_HandleTypeDef	*uart,
							uint8_t			    *buf,
							uint16_t             size);

TASK_RETURN ASYNC_UART_Rx_DMA(SCHEDULER* scheduler, TASK* self);


// =================================================

#define ASYNC_UART_RxToIdle_DMA_NUMBER 10

typedef struct ASYNC_UART_RxToIdle_DMA_CONTEXT {
	UART_HandleTypeDef *uart;

	bool has_started;

	uint8_t *buf;
	uint16_t size;
} ASYNC_UART_RxToIdle_DMA_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_UART_RxToIdle_DMA);

void ASYNC_UART_RxToIdle_DMA_init(TASK				    *self,
							      UART_HandleTypeDef	*uart,
							      uint8_t			    *buf,
							      uint16_t               size);

TASK_RETURN ASYNC_UART_RxToIdle_DMA(SCHEDULER* scheduler, TASK* self);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */


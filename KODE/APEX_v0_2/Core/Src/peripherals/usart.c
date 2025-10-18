/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "peripherals/usart.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include "utils/gms.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
//   huart1.Init.BaudRate = 9600;
//   huart1.Init.BaudRate = 115200;
  huart1.Init.BaudRate = 921600;
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

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream5;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

UART_BCALLBACK_STRUCT uart_callback;

void UART_Callback_struct_set(UART_HandleTypeDef *huart, UART_Callback callback, 
                              UART_CALLBACK_PARAM_PTR param_ptr) {
    #ifdef USART1
    if (huart->Instance == USART1) {
        uart_callback.uart1_callback = callback;
        uart_callback.uart1_param_ptr = param_ptr;
    }
    #endif
    #ifdef USART2
    if (huart->Instance == USART2) {
        uart_callback.uart2_callback = callback;
        uart_callback.uart2_param_ptr = param_ptr;
    }
    #endif
    #ifdef USART6
    if (huart->Instance == USART6) {
        uart_callback.uart3_callback = callback;
        uart_callback.uart3_param_ptr = param_ptr;
    }
    #endif
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    #ifdef USART1
    if (huart->Instance == USART1) {
        if (uart_callback.uart1_callback) {
            uart_callback.uart1_callback(uart_callback.uart1_param_ptr, huart, Size);
        }
    }
    #endif
    #ifdef USART2
    if (huart->Instance == USART2) {
        if (uart_callback.uart2_callback) {
            uart_callback.uart2_callback(uart_callback.uart2_param_ptr, huart, Size);
        }
    }
    #endif
    #ifdef USART6
    if (huart->Instance == USART6) {
        if (uart_callback.uart3_callback) {
            uart_callback.uart3_callback(uart_callback.uart3_param_ptr, huart, Size);
        }
    }
    #endif
}


typedef struct UART_BUS {
	#ifdef USART1
		bool uart1_bus_busy;
	#endif
	#ifdef USART2
		bool uart2_bus_busy;
	#endif
	#ifdef USART6
		bool uart3_bus_busy;
	#endif
} UART_BUSY;

UART_BUSY uart_busy = { 0 };

void UART_BUS_set_busy(UART_HandleTypeDef *uart, bool busy) {
	#ifdef USART1
    if (uart->Instance == USART1) {
        uart_busy.uart1_bus_busy = busy;
    }
    #endif
    #ifdef USART2
    if (uart->Instance == USART2) {
        uart_busy.uart2_bus_busy = busy;
    }
    #endif
    #ifdef USART6
    if (uart->Instance == USART6) {
        uart_busy.uart3_bus_busy = busy;
    }
    #endif
}

bool UART_BUS_get_busy(UART_HandleTypeDef *uart) {
	#ifdef USART1
    if (uart->Instance == USART1) {
        return uart_busy.uart1_bus_busy;
    }
    #endif
    #ifdef USART2
    if (uart->Instance == USART2) {
        return uart_busy.uart2_bus_busy;
    }
    #endif
    #ifdef USART6
    if (uart->Instance == USART6) {
        return uart_busy.uart3_bus_busy;
    }
    #endif
    return false;
}


void ASYNC_UART_Tx_DMA_init(TASK				*self,
					    	UART_HandleTypeDef	*uart,
					    	uint8_t			    *buf,
					    	uint16_t             size) {
    ASYNC_UART_Tx_DMA_CONTEXT *context = (ASYNC_UART_Tx_DMA_CONTEXT *)self->context;
    context->uart = uart;
    
    context->has_started = false;
    
    context->buf = GMS_alloc(&GMS_memory, size);
    memcpy(context->buf, buf, size);
    context->size = size;
}

TASK_POOL_CREATE(ASYNC_UART_Tx_DMA);

TASK_RETURN ASYNC_UART_Tx_DMA(SCHEDULER* scheduler, TASK* self) {
	ASYNC_UART_Tx_DMA_CONTEXT* context = (ASYNC_UART_Tx_DMA_CONTEXT*)self->context;

	UNUSED(scheduler);

	if (context->uart->gState == HAL_UART_STATE_READY) {
		if ((!context->has_started) && (!UART_BUS_get_busy(context->uart))) {
			context->has_started = true;
			UART_BUS_set_busy(context->uart, true);                 // Lock SPI bus
            HAL_UART_Transmit_DMA(context->uart, context->buf, context->size);
		} else if ((context->has_started) && (UART_BUS_get_busy(context->uart))) {
			UART_BUS_set_busy(context->uart, false);				// Realease SPI bus
			GMS_free(&GMS_memory, context->buf);
			return TASK_RETURN_STOP;
		}
	}
	return TASK_RETURN_IDLE;
}


void ASYNC_UART_Rx_DMA_init(TASK				*self,
					    	UART_HandleTypeDef	*uart,
					    	uint8_t			    *buf,
					    	uint16_t             size) {
    ASYNC_UART_Rx_DMA_CONTEXT *context = (ASYNC_UART_Rx_DMA_CONTEXT *)self->context;
    context->uart = uart;
    
    context->has_started = false;
    
    context->buf = buf;
    context->size = size;
}

TASK_POOL_CREATE(ASYNC_UART_Rx_DMA);

TASK_RETURN ASYNC_UART_Rx_DMA(SCHEDULER* scheduler, TASK* self) {
	ASYNC_UART_Rx_DMA_CONTEXT* context = (ASYNC_UART_Rx_DMA_CONTEXT*)self->context;

	UNUSED(scheduler);

	if (context->uart->gState == HAL_UART_STATE_READY) {
		if ((!context->has_started) && (!UART_BUS_get_busy(context->uart))) {
			context->has_started = true;
			UART_BUS_set_busy(context->uart, true);                 // Lock SPI bus
            HAL_UART_Receive_DMA(context->uart, context->buf, context->size);
		} else if ((context->has_started) && (UART_BUS_get_busy(context->uart))) {
			UART_BUS_set_busy(context->uart, false);				// Realease SPI bus
			return TASK_RETURN_STOP;
		}
	}
	return TASK_RETURN_IDLE;
}


void ASYNC_UART_RxToIdle_DMA_init(TASK				    *self,
                        	      UART_HandleTypeDef	*uart,
                        	      uint8_t			    *buf,
                        	      uint16_t               size) {
    ASYNC_UART_RxToIdle_DMA_CONTEXT *context = (ASYNC_UART_RxToIdle_DMA_CONTEXT *)self->context;
    context->uart = uart;

    context->has_started = false;

    context->buf = buf;
    context->size = size;
}

TASK_POOL_CREATE(ASYNC_UART_RxToIdle_DMA);

TASK_RETURN ASYNC_UART_RxToIdle_DMA(SCHEDULER* scheduler, TASK* self) {
    ASYNC_UART_RxToIdle_DMA_CONTEXT* context = (ASYNC_UART_RxToIdle_DMA_CONTEXT*)self->context;

    UNUSED(scheduler);

    if (context->uart->gState == HAL_UART_STATE_READY) {
        if ((!context->has_started) && (!UART_BUS_get_busy(context->uart))) {
            context->has_started = true;
            UART_BUS_set_busy(context->uart, true);                 // Lock SPI bus
            HAL_UARTEx_ReceiveToIdle_DMA(context->uart, context->buf, context->size);
        } else if ((context->has_started) && (UART_BUS_get_busy(context->uart))) {
            UART_BUS_set_busy(context->uart, false);				// Realease SPI bus
            return TASK_RETURN_STOP;
        }
    }
    return TASK_RETURN_IDLE;
}



// HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 12);



/* USER CODE END 1 */

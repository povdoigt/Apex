/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "peripherals/i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

/* I2C3 init function */
void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* I2C3 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();

    /* I2C3 DMA Init */
    /* I2C3_RX Init */
    hdma_i2c3_rx.Instance = DMA1_Stream1;
    hdma_i2c3_rx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c3_rx);

    /* I2C3_TX Init */
    hdma_i2c3_tx.Instance = DMA1_Stream5;
    hdma_i2c3_tx.Init.Channel = DMA_CHANNEL_6;
    hdma_i2c3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c3_tx);

  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

    /* I2C3 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);
  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


typedef struct I2C_BUSY {
	#ifdef I2C1
		bool i2c1_bus_busy;
	#endif
	#ifdef I2C2
    bool i2c2_bus_busy;
	#endif
	#ifdef I2C3
    bool i2c3_bus_busy;
	#endif
} I2C_BUSY;

I2C_BUSY i2c_busy = { 0 };

void I2C_BUS_set_busy(I2C_HandleTypeDef *hi2c, bool busy) {
    #ifdef I2C1
    if (hi2c->Instance == I2C1) {
        i2c_busy.i2c1_bus_busy = busy;
    }
    #endif
    #ifdef I2C2
    if (hi2c->Instance == I2C2) {
        i2c_busy.i2c2_bus_busy = busy;
    }
    #endif
    #ifdef I2C3
    if (hi2c->Instance == I2C3) {
        i2c_busy.i2c3_bus_busy = busy;
    }
    #endif
}

bool I2C_BUS_get_busy(I2C_HandleTypeDef *hi2c) {
    #ifdef I2C1
    if (hi2c->Instance == I2C1) {
        return i2c_busy.i2c1_bus_busy;
    }
    #endif
    #ifdef I2C2
    if (hi2c->Instance == I2C2) {
        return i2c_busy.i2c2_bus_busy;
    }
    #endif
    #ifdef I2C3
    if (hi2c->Instance == I2C3) {
        return i2c_busy.i2c3_bus_busy;
    }
    #endif
	return false;
}



TASK_POOL_CREATE(ASYNC_I2C_Tx_or_Rx_DMA);

void ASYNC_I2C_Tx_or_Rx_DMA_init(TASK               *self,
						         I2C_HandleTypeDef  *hi2c,
                                 uint8_t            dev_address,
						         uint8_t		    *buf,
						         uint16_t            size) {
	ASYNC_I2C_Tx_or_Rx_DMA_CONTEXT* context = (ASYNC_I2C_Tx_or_Rx_DMA_CONTEXT*)self->context;

	context->hi2c = hi2c;

	context->has_started = false;

    context->dev_address = dev_address;
	context->buf = buf;
	context->size = size;
}

TASK_RETURN ASYNC_I2C_Tx_DMA(SCHEDULER* scheduler, TASK* self) {
	ASYNC_I2C_Tx_or_Rx_DMA_CONTEXT* context = (ASYNC_I2C_Tx_or_Rx_DMA_CONTEXT*)self->context;

	UNUSED(scheduler);

	if (context->hi2c->State == HAL_I2C_STATE_READY) {
		if ((!context->has_started) && (!I2C_BUS_get_busy(context->hi2c))) {
			context->has_started = true;
			I2C_BUS_set_busy(context->hi2c, true);				// Lock I2C bus
            HAL_I2C_Master_Transmit_DMA(context->hi2c, context->dev_address, context->buf, context->size);
		} else if ((context->has_started) && (I2C_BUS_get_busy(context->hi2c))) {
			I2C_BUS_set_busy(context->hi2c, false);				// Realease SPI bus
			return TASK_RETURN_STOP;
		}
	}
	return TASK_RETURN_IDLE;
}

TASK_RETURN ASYNC_I2C_Rx_DMA(SCHEDULER* scheduler, TASK* self) {
    ASYNC_I2C_Tx_or_Rx_DMA_CONTEXT* context = (ASYNC_I2C_Tx_or_Rx_DMA_CONTEXT*)self->context;

    UNUSED(scheduler);

    if (context->hi2c->State == HAL_I2C_STATE_READY) {
        if ((!context->has_started) && (!I2C_BUS_get_busy(context->hi2c))) {
            context->has_started = true;
            I2C_BUS_set_busy(context->hi2c, true);				// Lock I2C bus
            HAL_I2C_Master_Receive_DMA(context->hi2c, context->dev_address, context->buf, context->size);
        } else if ((context->has_started) && (I2C_BUS_get_busy(context->hi2c))) {
            I2C_BUS_set_busy(context->hi2c, false);				// Realease SPI bus
            return TASK_RETURN_STOP;
        }
    }
    return TASK_RETURN_IDLE;
}



TASK_POOL_CREATE(ASYNC_I2C_Mem_Tx_or_Rx_DMA);


void ASYNC_I2C_Mem_Tx_or_Rx_DMA_init(TASK               *self,
                                     I2C_HandleTypeDef  *hi2c,
                                     uint8_t             dev_address,
                                     uint16_t            mem_address,
                                     uint16_t            mem_address_size,
                                     uint8_t		    *buf,
                                     uint16_t            size) {
    ASYNC_I2C_Mem_Tx_or_Rx_DMA_CONTEXT* context = (ASYNC_I2C_Mem_Tx_or_Rx_DMA_CONTEXT*)self->context;

    context->hi2c = hi2c;

    context->has_started = false;

    context->dev_address = dev_address;
    context->mem_address = mem_address;
    context->mem_address_size = mem_address_size; // I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT
    context->buf = buf;
    context->size = size;
}

TASK_RETURN ASYNC_I2C_Mem_Tx_DMA(SCHEDULER* scheduler, TASK* self) {
    ASYNC_I2C_Mem_Tx_or_Rx_DMA_CONTEXT* context = (ASYNC_I2C_Mem_Tx_or_Rx_DMA_CONTEXT*)self->context;

    UNUSED(scheduler);

    if (context->hi2c->State == HAL_I2C_STATE_READY) {
        if ((!context->has_started) && (!I2C_BUS_get_busy(context->hi2c))) {
            context->has_started = true;
            I2C_BUS_set_busy(context->hi2c, true);				// Lock I2C bus
            HAL_I2C_Mem_Write_DMA(context->hi2c, context->dev_address, context->mem_address, context->mem_address_size, context->buf, context->size);
        } else if ((context->has_started) && (I2C_BUS_get_busy(context->hi2c))) {
            I2C_BUS_set_busy(context->hi2c, false);				// Realease SPI bus
            return TASK_RETURN_STOP;
        }
    }
    return TASK_RETURN_IDLE;
}

TASK_RETURN ASYNC_I2C_Mem_Rx_DMA(SCHEDULER* scheduler, TASK* self) {
    ASYNC_I2C_Mem_Tx_or_Rx_DMA_CONTEXT* context = (ASYNC_I2C_Mem_Tx_or_Rx_DMA_CONTEXT*)self->context;

    UNUSED(scheduler);

    if (context->hi2c->State == HAL_I2C_STATE_READY) {
        if ((!context->has_started) && (!I2C_BUS_get_busy(context->hi2c))) {
            context->has_started = true;
            I2C_BUS_set_busy(context->hi2c, true);				// Lock I2C bus
            HAL_I2C_Mem_Read_DMA(context->hi2c, context->dev_address, context->mem_address, context->mem_address_size, context->buf, context->size);
        } else if ((context->has_started) && (I2C_BUS_get_busy(context->hi2c))) {
            I2C_BUS_set_busy(context->hi2c, false);				// Realease SPI bus
            return TASK_RETURN_STOP;
        }
    }
    return TASK_RETURN_IDLE;
}


/* USER CODE END 1 */

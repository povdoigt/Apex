/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "peripherals/spi.h"
#include <string.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

// SPI_HandleTypeDef_flag hspi1_flag;
// SPI_HandleTypeDef_flag hspi2_flag;

/* SPI1 init function */
void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}
/* SPI2 init function */
void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (spiHandle->Instance == SPI1) {
		/* USER CODE BEGIN SPI1_MspInit 0 */

		/* USER CODE END SPI1_MspInit 0 */
		  /* SPI1 clock enable */
		__HAL_RCC_SPI1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**SPI1 GPIO Configuration
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* SPI1 DMA Init */
		/* SPI1_RX Init */
		hdma_spi1_rx.Instance = DMA2_Stream0;
		hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
		hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_spi1_rx.Init.Mode = DMA_NORMAL;
		hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(spiHandle, hdmarx, hdma_spi1_rx);

		/* SPI1_TX Init */
		hdma_spi1_tx.Instance = DMA2_Stream2;
		hdma_spi1_tx.Init.Channel = DMA_CHANNEL_2;
		hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_spi1_tx.Init.Mode = DMA_NORMAL;
		hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(spiHandle, hdmatx, hdma_spi1_tx);

		/* USER CODE BEGIN SPI1_MspInit 1 */

		/* USER CODE END SPI1_MspInit 1 */
	} else if (spiHandle->Instance == SPI2) {
		/* USER CODE BEGIN SPI2_MspInit 0 */

		/* USER CODE END SPI2_MspInit 0 */
		  /* SPI2 clock enable */
		__HAL_RCC_SPI2_CLK_ENABLE();

		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**SPI2 GPIO Configuration
		PC2     ------> SPI2_MISO
		PC3     ------> SPI2_MOSI
		PB10     ------> SPI2_SCK
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* SPI2 DMA Init */
		/* SPI2_RX Init */
		hdma_spi2_rx.Instance = DMA1_Stream3;
		hdma_spi2_rx.Init.Channel = DMA_CHANNEL_0;
		hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_spi2_rx.Init.Mode = DMA_NORMAL;
		hdma_spi2_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(spiHandle, hdmarx, hdma_spi2_rx);

		/* SPI2_TX Init */
		hdma_spi2_tx.Instance = DMA1_Stream4;
		hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
		hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_spi2_tx.Init.Mode = DMA_NORMAL;
		hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(spiHandle, hdmatx, hdma_spi2_tx);

		/* USER CODE BEGIN SPI2_MspInit 1 */

		/* USER CODE END SPI2_MspInit 1 */
	}
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle) {

	if (spiHandle->Instance == SPI1) {
		/* USER CODE BEGIN SPI1_MspDeInit 0 */

		/* USER CODE END SPI1_MspDeInit 0 */
		  /* Peripheral clock disable */
		__HAL_RCC_SPI1_CLK_DISABLE();

		/**SPI1 GPIO Configuration
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

		/* SPI1 DMA DeInit */
		HAL_DMA_DeInit(spiHandle->hdmarx);
		HAL_DMA_DeInit(spiHandle->hdmatx);
		/* USER CODE BEGIN SPI1_MspDeInit 1 */

		/* USER CODE END SPI1_MspDeInit 1 */
	} else if (spiHandle->Instance == SPI2) {
		/* USER CODE BEGIN SPI2_MspDeInit 0 */

		/* USER CODE END SPI2_MspDeInit 0 */
		  /* Peripheral clock disable */
		__HAL_RCC_SPI2_CLK_DISABLE();

		/**SPI2 GPIO Configuration
		PC2     ------> SPI2_MISO
		PC3     ------> SPI2_MOSI
		PB10     ------> SPI2_SCK
		*/
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2 | GPIO_PIN_3);

		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

		/* SPI2 DMA DeInit */
		HAL_DMA_DeInit(spiHandle->hdmarx);
		HAL_DMA_DeInit(spiHandle->hdmatx);
		/* USER CODE BEGIN SPI2_MspDeInit 1 */

		/* USER CODE END SPI2_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */


typedef struct SPI_BUS {
	#ifdef SPI1
		bool spi1_bus_busy;
	#endif
	#ifdef SPI2
		bool spi2_bus_busy;
	#endif
	#ifdef SPI3
		bool spi3_bus_busy;
	#endif
	#ifdef SPI4
		bool spi4_bus_busy;
	#endif
	#ifdef SPI5
		bool spi5_bus_busy;
	#endif
} SPI_BUSY;

SPI_BUSY spi_busy = { 0 };

void SPI_BUS_set_busy(SPI_HandleTypeDef *hspi, bool busy) {
	#ifdef SPI1
	if (hspi->Instance == SPI1) {
		spi_busy.spi1_bus_busy = busy;
	}
	#endif
	#ifdef SPI2
	if (hspi->Instance == SPI2) {
		spi_busy.spi2_bus_busy = busy;
	}
	#endif
	#ifdef SPI3
	if (hspi->Instance == SPI3) {
		spi_busy.spi3_bus_busy = busy;
	}
	#endif
	#ifdef SPI4
	if (hspi->Instance == SPI4) {
		spi_busy.spi4_bus_busy = busy;
	}
	#endif
	#ifdef SPI5
	if (hspi->Instance == SPI5) {
		spi_busy.spi5_bus_busy = busy;
	}
	#endif
}

bool SPI_BUS_get_busy(SPI_HandleTypeDef *hspi) {
	#ifdef SPI1
	if (hspi->Instance == SPI1) {
		return spi_busy.spi1_bus_busy;
	}
	#endif
	#ifdef SPI2
	if (hspi->Instance == SPI2) {
		return spi_busy.spi2_bus_busy;
	}
	#endif
	#ifdef SPI3
	if (hspi->Instance == SPI3) {
		return spi_busy.spi3_bus_busy;
	}
	#endif
	#ifdef SPI4
	if (hspi->Instance == SPI4) {
		return spi_busy.spi4_bus_busy;
	}
	#endif
	#ifdef SPI5
	if (hspi->Instance == SPI5) {
		return spi_busy.spi5_bus_busy;
	}
	#endif
	return false;
}



TASK_POOL_CREATE(ASYNC_SPI_TxRx_DMA);

void ASYNC_SPI_TxRx_DMA_init(TASK			    *self,
							 SPI_HandleTypeDef	*hspi,
							 GPIO_TypeDef	    *csPinBank,
							 uint16_t            csPin,
							 uint8_t		    *tx_buf,
							 uint8_t		    *rx_buf,
							 uint16_t            tx_size,
							 uint16_t            rx_size,
							 TASK			    *task_waiter) {
	ASYNC_SPI_TxRx_DMA_CONTEXT* context = (ASYNC_SPI_TxRx_DMA_CONTEXT*)self->context;

	if (context == NULL) {
		__NOP();
	}

	context->hspi = hspi;
	context->csPinBank = csPinBank;
	context->csPin = csPin;

	context->has_started = false;

	context->buf = GMS_alloc(&GMS_memory, tx_size + rx_size);

	if (context->buf < 1) {
		Error_Handler();
	}

	context->rx_buf = rx_buf;

	memcpy(context->buf, tx_buf, tx_size);

	context->tx_size = tx_size;
	context->rx_size = rx_size;

	context->task_waiter = task_waiter;
}

TASK_RETURN ASYNC_SPI_TxRx_DMA(SCHEDULER* scheduler, TASK* self) {
	ASYNC_SPI_TxRx_DMA_CONTEXT* context = (ASYNC_SPI_TxRx_DMA_CONTEXT*)self->context;

	UNUSED(scheduler);

	uint16_t size = context->tx_size + context->rx_size;

	if (context->hspi->State == HAL_SPI_STATE_READY) {
		if ((!context->has_started) && (!SPI_BUS_get_busy(context->hspi))) {
			context->has_started = true;

			SPI_BUS_set_busy(context->hspi, true);				// Lock SPI bus
			HAL_GPIO_WritePin(context->csPinBank, context->csPin, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive_DMA(context->hspi,
										context->buf, context->buf,
										size);
		} else if ((context->has_started) && (SPI_BUS_get_busy(context->hspi))) {
			SPI_BUS_set_busy(context->hspi, false);				// Realease SPI bus
			HAL_GPIO_WritePin(context->csPinBank, context->csPin, GPIO_PIN_SET);

			if (context->rx_buf) {								// rx_buf can be NULL
				memcpy(context->rx_buf, context->buf + context->tx_size, context->rx_size);
			}
			GMS_free(&GMS_memory, context->buf);

			return TASK_RETURN_STOP;
		}
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_SPI_TxRx_DMA_static);

void ASYNC_SPI_TxRx_DMA_static_init(TASK				*self,
							        SPI_HandleTypeDef	*hspi,
							        GPIO_TypeDef		*csPinBank,
							        uint16_t             csPin,
							        uint8_t			    *tx_buf,
							        uint8_t			    *rx_buf,
							        uint16_t             tx_size,
							        uint16_t             rx_size,
							        TASK				*task_waiter) {
	ASYNC_SPI_TxRx_DMA_static_CONTEXT* context = (ASYNC_SPI_TxRx_DMA_static_CONTEXT*)self->context;

	context->hspi = hspi;
	context->csPinBank = csPinBank;
	context->csPin = csPin;

	context->has_started = false;

	context->rx_buf = rx_buf;

	memcpy(context->buf, tx_buf, tx_size);

	context->tx_size = tx_size;
	context->rx_size = rx_size;

	context->task_waiter = task_waiter;
}

TASK_RETURN ASYNC_SPI_TxRx_DMA_static(SCHEDULER* scheduler, TASK* self) {
	ASYNC_SPI_TxRx_DMA_static_CONTEXT* context = (ASYNC_SPI_TxRx_DMA_static_CONTEXT*)self->context;

	UNUSED(scheduler);

	uint16_t size = context->tx_size + context->rx_size;

	if (context->hspi->State == HAL_SPI_STATE_READY) {
		if ((!context->has_started) && (!SPI_BUS_get_busy(context->hspi))) {
			context->has_started = true;

			SPI_BUS_set_busy(context->hspi, true);				// Lock SPI bus
			HAL_GPIO_WritePin(context->csPinBank, context->csPin, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive_DMA(context->hspi,
										context->buf, context->buf,
										size);
		} else if ((context->has_started) && (SPI_BUS_get_busy(context->hspi))) {
			SPI_BUS_set_busy(context->hspi, false);				// Realease SPI bus
			HAL_GPIO_WritePin(context->csPinBank, context->csPin, GPIO_PIN_SET);

			if (context->rx_buf) {								// rx_buf can be NULL
				memcpy(context->rx_buf, context->buf + context->tx_size, context->rx_size);
			}

			return TASK_RETURN_STOP;
		}
	}
	return TASK_RETURN_IDLE;
}

/* USER CODE END 1 */

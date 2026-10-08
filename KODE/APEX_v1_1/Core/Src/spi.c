/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

#include "main_config.h"

#if (APEX_CFG_SCHED_RTOS == 1)
#include <stdbool.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#endif

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}
/* SPI2 init function */
void MX_SPI2_Init(void)
{

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
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}
/* SPI3 init function */
void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
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
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
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
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

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
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
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
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
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
    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi2_rx);

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
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi2_tx);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* SPI3 DMA Init */
    /* SPI3_RX Init */
    hdma_spi3_rx.Instance = DMA1_Stream0;
    hdma_spi3_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_rx.Init.Mode = DMA_NORMAL;
    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi3_rx);

    /* SPI3_TX Init */
    hdma_spi3_tx.Instance = DMA1_Stream7;
    hdma_spi3_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_tx.Init.Mode = DMA_NORMAL;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi3_tx);

    /* SPI3 interrupt Init */
    HAL_NVIC_SetPriority(SPI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
    PB10     ------> SPI2_SCK
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    /* SPI2 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);
  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

    /* SPI3 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);

    /* SPI3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI3_IRQn);
  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

#if (APEX_CFG_SCHED_RTOS == 1)

/*
 * DMA transfers under FreeRTOS.
 *
 * Each bus has two binary semaphores:
 *   - use : held from SPI_Begin_DMA_RTOS to SPI_End_DMA_RTOS, so one device at
 *           a time has its CS low. Transfers are only valid in between;
 *   - done: starts empty, given by the complete / error callback, taken by the
 *           thread that started the transfer.
 *
 * A transfer never blocks forever: a start refused by the HAL returns at once,
 * a DMA error returns HAL_ERROR, and a lost interrupt returns HAL_TIMEOUT once
 * twice the expected transfer time (plus a margin) has elapsed.
 */

/* Scheduling margin added to the computed transfer time. */
#define SPI_DMA_RTOS_MARGIN_MS	10u

typedef struct spi_rtos_bus_t {
	SPI_HandleTypeDef	*hspi;
	osSemaphoreId_t		 use_id;
	StaticSemaphore_t	 use_cb;
	osSemaphoreId_t		 done_id;
	StaticSemaphore_t	 done_cb;
	volatile bool		 error;		/* Set by HAL_SPI_ErrorCallback for the transfer in progress */
} spi_rtos_bus_t;

static spi_rtos_bus_t spi_rtos_buses[] = {
	{ .hspi = &hspi1 },
	{ .hspi = &hspi2 },
	{ .hspi = &hspi3 },
};

static spi_rtos_bus_t *spi_rtos_bus_get(const SPI_HandleTypeDef *hspi) {
	for (size_t i = 0; i < sizeof(spi_rtos_buses) / sizeof(spi_rtos_buses[0]); i++) {
		if (spi_rtos_buses[i].hspi->Instance == hspi->Instance) {
			return &spi_rtos_buses[i];
		}
	}
	return NULL;
}

void Init_spi_semaphores(void) {
	for (size_t i = 0; i < sizeof(spi_rtos_buses) / sizeof(spi_rtos_buses[0]); i++) {
		spi_rtos_bus_t *bus = &spi_rtos_buses[i];
		if (bus->use_id == NULL) {
			bus->use_id = osSemaphoreNew(1, 1, &(const osSemaphoreAttr_t){
				.cb_mem  = &bus->use_cb,
				.cb_size = sizeof(bus->use_cb),
			});
		}
		if (bus->done_id == NULL) {
			bus->done_id = osSemaphoreNew(1, 0, &(const osSemaphoreAttr_t){
				.cb_mem  = &bus->done_cb,
				.cb_size = sizeof(bus->done_cb),
			});
		}
		if (bus->use_id == NULL || bus->done_id == NULL) {
			Error_Handler();
		}
	}
}

/* Twice the time `size` bytes take on the wire, plus a scheduling margin. */
static uint32_t spi_rtos_timeout_ms(const SPI_HandleTypeDef *hspi, uint16_t size) {
	/* SPI1 is the only APB2 SPI of this part. */
	uint32_t pclk  = (hspi->Instance == SPI1) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
	uint32_t presc = 2u << (hspi->Init.BaudRatePrescaler >> SPI_CR1_BR_Pos);
	uint32_t ms    = (uint32_t)(((uint64_t)size * 8u * presc * 1000u) / pclk);
	return 2u * ms + SPI_DMA_RTOS_MARGIN_MS;
}

/* Called just before a start: no transfer is in progress, so any token left
   in `done` is stale (a completion that raced an abort). */
static void spi_rtos_prepare(spi_rtos_bus_t *bus) {
	(void)osSemaphoreAcquire(bus->done_id, 0u);
	bus->error = false;
}

static HAL_StatusTypeDef spi_rtos_wait(spi_rtos_bus_t *bus, HAL_StatusTypeDef started, uint16_t size) {
	if (started != HAL_OK) {
		return started;		/* Nothing started: no callback will come */
	}
	if (osSemaphoreAcquire(bus->done_id, spi_rtos_timeout_ms(bus->hspi, size)) != osOK) {
		/* Blocking abort: no callback fires after it returns. */
		(void)HAL_SPI_Abort(bus->hspi);
		return HAL_TIMEOUT;
	}
	return bus->error ? HAL_ERROR : HAL_OK;
}

/* Callbacks of a bus this layer does not drive are ignored. */
static void spi_rtos_signal(SPI_HandleTypeDef *hspi, bool error) {
	spi_rtos_bus_t *bus = spi_rtos_bus_get(hspi);
	if (bus == NULL || bus->done_id == NULL) {
		return;
	}
	bus->error = error;
	osSemaphoreRelease(bus->done_id);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	spi_rtos_signal(hspi, false);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	spi_rtos_signal(hspi, false);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	spi_rtos_signal(hspi, false);
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	spi_rtos_signal(hspi, true);
}

HAL_StatusTypeDef SPI_Begin_DMA_RTOS(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPinBank, uint16_t csPin) {
	spi_rtos_bus_t *bus = spi_rtos_bus_get(hspi);
	if (bus == NULL || bus->use_id == NULL) { return HAL_ERROR; }
	if (osSemaphoreAcquire(bus->use_id, osWaitForever) != osOK) { return HAL_ERROR; }
	HAL_GPIO_WritePin(csPinBank, csPin, GPIO_PIN_RESET);
	return HAL_OK;
}

HAL_StatusTypeDef SPI_Transmit_DMA_RTOS(SPI_HandleTypeDef *hspi, const uint8_t *pData, uint16_t Size) {
	spi_rtos_bus_t *bus = spi_rtos_bus_get(hspi);
	if (bus == NULL || bus->done_id == NULL) { return HAL_ERROR; }
	if (Size == 0u) { return HAL_OK; }
	spi_rtos_prepare(bus);
	return spi_rtos_wait(bus, HAL_SPI_Transmit_DMA(hspi, pData, Size), Size);
}

HAL_StatusTypeDef SPI_TransmitReceive_DMA_RTOS(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size) {
	spi_rtos_bus_t *bus = spi_rtos_bus_get(hspi);
	if (bus == NULL || bus->done_id == NULL) { return HAL_ERROR; }
	if (Size == 0u) { return HAL_OK; }
	spi_rtos_prepare(bus);
	return spi_rtos_wait(bus, HAL_SPI_TransmitReceive_DMA(hspi, pTxData, pRxData, Size), Size);
}

HAL_StatusTypeDef SPI_Receive_DMA_RTOS(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size) {
	spi_rtos_bus_t *bus = spi_rtos_bus_get(hspi);
	if (bus == NULL || bus->done_id == NULL) { return HAL_ERROR; }
	if (Size == 0u) { return HAL_OK; }
	spi_rtos_prepare(bus);
	return spi_rtos_wait(bus, HAL_SPI_Receive_DMA(hspi, pData, Size), Size);
}

HAL_StatusTypeDef SPI_End_DMA_RTOS(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPinBank, uint16_t csPin) {
	spi_rtos_bus_t *bus = spi_rtos_bus_get(hspi);
	if (bus == NULL || bus->use_id == NULL) { return HAL_ERROR; }
	HAL_GPIO_WritePin(csPinBank, csPin, GPIO_PIN_SET);
	return osSemaphoreRelease(bus->use_id) == osOK ? HAL_OK : HAL_ERROR;
}

#endif /* APEX_CFG_SCHED_RTOS */

/* USER CODE END 1 */


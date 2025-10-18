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
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

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

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};
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

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle) {

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

typedef struct spi_semaphores_t {
#ifdef SPI1
	osSemaphoreId_t spi1_use_semaphore_id;
	StaticSemaphore_t spi1_use_semaphore;

	osSemaphoreId_t spi1_dma_semaphore_id;
	StaticSemaphore_t spi1_dma_semaphore;
#endif
#ifdef SPI2
	osSemaphoreId_t spi2_use_semaphore_id;
	StaticSemaphore_t spi2_use_semaphore;

	osSemaphoreId_t spi2_dma_semaphore_id;
	StaticSemaphore_t spi2_dma_semaphore;
#endif
} spi_semaphores_t;

spi_semaphores_t spi_semaphores;

#ifdef SPI1
const char SPI1_use_semaphore_name[19] = "SPI1_use_semaphore";
const char SPI1_dma_semaphore_name[19] = "SPI1_dma_semaphore";
#endif
#ifdef SPI2
const char SPI2_use_semaphore_name[19] = "SPI2_use_semaphore";
const char SPI2_dma_semaphore_name[19] = "SPI2_dma_semaphore";
#endif


void Init_spi_semaphores() {
#ifdef SPI1
  	const osSemaphoreAttr_t spi1_sem_use_attr = {
		.name = SPI1_use_semaphore_name,
		.cb_mem = &spi_semaphores.spi1_use_semaphore,
		.cb_size = sizeof(spi_semaphores.spi1_use_semaphore),
	};
  	const osSemaphoreAttr_t spi1_sem_dma_attr = {
		.name = SPI1_dma_semaphore_name,
		.cb_mem = &spi_semaphores.spi1_dma_semaphore,
		.cb_size = sizeof(spi_semaphores.spi1_dma_semaphore),
	};
	spi_semaphores.spi1_use_semaphore_id = osSemaphoreNew(1, 1, &spi1_sem_use_attr);
	spi_semaphores.spi1_dma_semaphore_id = osSemaphoreNew(1, 1, &spi1_sem_dma_attr);
#endif
#ifdef SPI2
  	const osSemaphoreAttr_t spi2_sem_use_attr = {
		.name = SPI2_use_semaphore_name,
		.cb_mem = &spi_semaphores.spi2_use_semaphore,
		.cb_size = sizeof(spi_semaphores.spi2_use_semaphore),
	};
  	const osSemaphoreAttr_t spi2_sem_dma_attr = {
		.name = SPI2_dma_semaphore_name,
		.cb_mem = &spi_semaphores.spi2_dma_semaphore,
		.cb_size = sizeof(spi_semaphores.spi2_dma_semaphore),
	};
	spi_semaphores.spi2_use_semaphore_id = osSemaphoreNew(1, 1, &spi2_sem_use_attr);
	spi_semaphores.spi2_dma_semaphore_id = osSemaphoreNew(1, 1, &spi2_sem_dma_attr);
#endif
}

osSemaphoreId_t spi_use_semaphore_get(SPI_HandleTypeDef *hspi) {
#ifdef SPI1
	if (hspi->Instance == SPI1) {
		return spi_semaphores.spi1_use_semaphore_id;
	}
#endif
#ifdef SPI2
	if (hspi->Instance == SPI2) {
		return spi_semaphores.spi2_use_semaphore_id;
	}
#endif
	return NULL; // Invalid SPI instance
}

osSemaphoreId_t *spi_dma_semaphore_get(SPI_HandleTypeDef *hspi) {
#ifdef SPI1
	if (hspi->Instance == SPI1) {
		return spi_semaphores.spi1_dma_semaphore_id;
	}
#endif
#ifdef SPI2
	if (hspi->Instance == SPI2) {
		return spi_semaphores.spi2_dma_semaphore_id;
	}
#endif
	return NULL; // Invalid SPI instance
}

void __HAL_SPI_CpltCallback(SPI_HandleTypeDef *hspi) {
	osSemaphoreId_t dma_semaphore = spi_dma_semaphore_get(hspi);
	if (dma_semaphore == NULL) {
		Error_Handler();
	}
	osSemaphoreRelease(dma_semaphore);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	__HAL_SPI_CpltCallback(hspi);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	__HAL_SPI_CpltCallback(hspi);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	__HAL_SPI_CpltCallback(hspi);
}

HAL_StatusTypeDef TASK_HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPinBank, uint16_t csPin,
											const uint8_t *pData, uint16_t Size) {
	osSemaphoreId_t use_semaphore = spi_use_semaphore_get(hspi);
	osSemaphoreId_t dma_semaphore = spi_dma_semaphore_get(hspi);
	if (use_semaphore == NULL || dma_semaphore == NULL) {
		Error_Handler();
	}

	// Step 1: We're waiting for the SPI to be free
	osSemaphoreAcquire(use_semaphore, osWaitForever);

	// Step 2: We're processing the SPI and taking the DMA semaphore
	osSemaphoreAcquire(dma_semaphore, osWaitForever);

	HAL_GPIO_WritePin(csPinBank, csPin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(hspi, pData, Size);

	// Step 3: We're waiting for the DMA to complete
	osSemaphoreAcquire(dma_semaphore, osWaitForever);

	// Step 4: We're done with the SPI, releasing the use semaphores and setting CS high
	HAL_GPIO_WritePin(csPinBank, csPin, GPIO_PIN_SET);
	osSemaphoreRelease(use_semaphore);
	osSemaphoreRelease(dma_semaphore);

	return status;
}

HAL_StatusTypeDef TASK_HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPinBank, uint16_t csPin,
										   uint8_t *pData, uint16_t Size) {
	osSemaphoreId_t use_semaphore = spi_use_semaphore_get(hspi);
	osSemaphoreId_t dma_semaphore = spi_dma_semaphore_get(hspi);
	if (use_semaphore == NULL || dma_semaphore == NULL) {
		Error_Handler();
	}

	// Step 1: We're waiting for the SPI to be free
	osSemaphoreAcquire(use_semaphore, osWaitForever);

	// Step 2: We're processing the SPI and taking the DMA semaphore
	osSemaphoreAcquire(dma_semaphore, osWaitForever);

	HAL_GPIO_WritePin(csPinBank, csPin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Receive_DMA(hspi, pData, Size);

	// Step 3: We're waiting for the DMA to complete
	osSemaphoreAcquire(dma_semaphore, osWaitForever);

	// Step 4: We're done with the SPI, releasing the use semaphores and setting CS high
	HAL_GPIO_WritePin(csPinBank, csPin, GPIO_PIN_SET);
	osSemaphoreRelease(use_semaphore);
	osSemaphoreRelease(dma_semaphore);

	return status;
}

HAL_StatusTypeDef TASK_HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPinBank, uint16_t csPin,
												   const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size) {
	osSemaphoreId_t use_semaphore = spi_use_semaphore_get(hspi);
	osSemaphoreId_t dma_semaphore = spi_dma_semaphore_get(hspi);
	if (use_semaphore == NULL || dma_semaphore == NULL) {
		Error_Handler();
	}

	// Step 1: We're waiting for the SPI to be free
	osSemaphoreAcquire(use_semaphore, osWaitForever);

	// Step 2: We're processing the SPI and taking the DMA semaphore
	osSemaphoreAcquire(dma_semaphore, osWaitForever);

	HAL_GPIO_WritePin(csPinBank, csPin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(hspi, pTxData, pRxData, Size);

	// Step 3: We're waiting for the DMA to complete
	osSemaphoreAcquire(dma_semaphore, osWaitForever);

	// Step 4: We're done with the SPI, releasing the use semaphores and setting CS high
	HAL_GPIO_WritePin(csPinBank, csPin, GPIO_PIN_SET);
	osSemaphoreRelease(use_semaphore);
	osSemaphoreRelease(dma_semaphore);

	return status;
}

/* USER CODE END 1 */

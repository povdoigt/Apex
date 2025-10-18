/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

	/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "utils\gms.h"
#include "utils\scheduler.h"

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */

#define ASYNC_SPI_TxRx_DMA_NUMBER 20

// typedef enum ASYNC_WAITER_STATE {
// 	ASYNC_START,
// 	ASYNC_WAIT,
// 	ASYNC_END
// } ASYNC_WAITER_STATE;

typedef struct ASYNC_SPI_TxRx_DMA_CONTEXT {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *csPinBank;
	uint16_t csPin;

	bool has_started;

	uint8_t *buf;
	uint8_t *rx_buf;

	uint16_t tx_size;
	uint16_t rx_size;

	uint32_t next_time;

	TASK* task_waiter;
} ASYNC_SPI_TxRx_DMA_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_SPI_TxRx_DMA);

void ASYNC_SPI_TxRx_DMA_init(TASK				*self,
							 SPI_HandleTypeDef	*hspi,
							 GPIO_TypeDef		*csPinBank,
							 uint16_t            csPin,
							 uint8_t			*tx_buf,
							 uint8_t			*rx_buf,
							 uint16_t             tx_size,
							 uint16_t             rx_size,
							 TASK				*task_waiter);

TASK_RETURN ASYNC_SPI_TxRx_DMA(SCHEDULER* scheduler, TASK* self);





#define ASYNC_SPI_TxRx_DMA_static_NUMBER 10

typedef struct ASYNC_SPI_TxRx_DMA_static_CONTEXT {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *csPinBank;
	uint16_t csPin;

	bool has_started;

	uint8_t buf[512];
	uint8_t *rx_buf;

	uint16_t tx_size;
	uint16_t rx_size;

	uint32_t next_time;

	TASK* task_waiter;
} ASYNC_SPI_TxRx_DMA_static_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_SPI_TxRx_DMA_static);

void ASYNC_SPI_TxRx_DMA_static_init(TASK				*self,
							        SPI_HandleTypeDef	*hspi,
							        GPIO_TypeDef		*csPinBank,
							        uint16_t             csPin,
							        uint8_t			    *tx_buf,
							        uint8_t			    *rx_buf,
							        uint16_t             tx_size,
							        uint16_t             rx_size,
							        TASK				*task_waiter);

TASK_RETURN ASYNC_SPI_TxRx_DMA_static(SCHEDULER* scheduler, TASK* self);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

